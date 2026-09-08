"""
Throttle ECU Control GUI — C2000 F280049C (CAN bus)
====================================================
Communicates via CAN bus using python-can.

Requirements:
    pip install python-can

Common adapters:
    CANable / slcan  : --interface slcan  --channel COM5
    PCAN             : --interface pcan   --channel PCAN_USBBUS1
    Kvaser           : --interface kvaser
    SocketCAN(Linux) : --interface socketcan --channel can0

CAN protocol (must match throttle_config.h):
  TX 0x100  [4 bytes]  PC → ECU  commands
  RX 0x101  [8 bytes]  ECU → PC  telemetry

Run:
    python throttle_gui.py
"""

import tkinter as tk
from tkinter import ttk
import threading
import time
import struct
import collections

try:
    import can
    _CAN_OK = True
except ImportError:
    _CAN_OK = False

try:
    import serial.tools.list_ports as _list_ports
    _SERIAL_OK = True
except ImportError:
    _SERIAL_OK = False

def _get_com_ports():
    """Return sorted list of available COM port names, e.g. ['COM3', 'COM5']."""
    if not _SERIAL_OK:
        return []
    return sorted(p.device for p in _list_ports.comports())

# ── Protocol constants (must match throttle_config.h) ──────────────────────────
CAN_CMD_ID   = 0x100   # PC → ECU
CAN_TELEM_ID = 0x101   # ECU → PC
CAN_BITRATE  = 500000

FLAG_RELAY = 0x01
FLAG_PID   = 0x02
FLAG_ESTOP = 0x04
FLAG_RESET = 0x08

# ── Angle calibration defaults (must match throttle_config.h) ─────────────────
# Measured 2026-09-06 via raw CAN angle field (post pin-fix):
#   open  (100%, smaller angle): 22.11° = 2211 raw
#   closed  (0%, larger angle): 132.62° = 13262 raw
# Inverted sensor: smaller angle = more open
ANGLE_MIN_RAW = 2211   # physical open  position (22.11°)
ANGLE_MAX_RAW = 13262  # physical closed position (132.62°)
ANGLE_RANGE   = ANGLE_MAX_RAW - ANGLE_MIN_RAW
USABLE_MIN    = ANGLE_MIN_RAW + ANGLE_RANGE * 5  // 100
USABLE_MAX    = ANGLE_MIN_RAW + ANGLE_RANGE * 95 // 100

# ── Fault bit definitions (must match safety.h) ────────────────────────────────
FAULT_BITS = [
    (0x01, "ENC_STALE",   "Encoder stale (>500 ms without update)"),
    (0x02, "ENC_INVALID", "Encoder signal invalid"),
    (0x04, "OVERCURR_L",  "Left motor overcurrent"),
    (0x08, "OVERCURR_R",  "Right motor overcurrent"),
    (0x10, "POWER_LOW",   "Supply voltage below 9 V"),
    (0x20, "WDG_RESET",   "Recovered from watchdog reset"),
    (0x40, "CAN_TIMEOUT", "CAN RX heartbeat lost (>200 ms)"),
    (0x80, "CAN_BUS_OFF", "CAN controller bus-off — wire/termination fault"),
]

MODE_STYLE = {
    "MAN":  ("MANUAL",           "#a6e3a1", "#1e1e2e"),
    "PID":  ("PID ACTIVE",       "#fab387", "#1e1e2e"),
    "SAFE": ("!! SAFE STATE !!", "#f38ba8", "#1e1e2e"),
}
MODE_NAMES = {0: "MAN", 1: "PID", 2: "SAFE"}

CAN_INTERFACES = ["slcan", "pcan", "kvaser", "socketcan", "usb2can", "ixxat", "gs_usb", "serial"]


# ══════════════════════════════════════════════════════════════════════════════
#  CAN thread — heartbeat TX + telemetry RX
# ══════════════════════════════════════════════════════════════════════════════
class CanThread:
    HB_HZ = 10  # heartbeat rate (frames/s)

    def __init__(self):
        self.bus        = None
        self.lock       = threading.Lock()
        self.connected  = False
        self._running   = False
        self._hb_thread = None
        self._rx_thread = None
        self._tx_seq    = 0

        # Command state (what the GUI wants to send)
        self._relay_on     = False
        self._pid_on       = False
        self._throttle_pct = 0

        # Received telemetry (updated by RX thread)
        self.mode        = "MAN"
        self.thr_act     = None   # int 0-100, or None when no encoder
        self.thr_tgt     = 0
        self.err_flags   = 0
        self.motor_cmd   = 0
        self.relay_state = 0
        self.raw_deg     = None   # float degrees direct from encoder, no calibration
        self.last_frame  = ""     # human-readable last decoded frame for log

    # ── Public API (GUI thread) ────────────────────────────────────────────────
    def connect(self, interface, channel=None, bitrate=CAN_BITRATE):
        self.disconnect()
        try:
            kwargs = {"interface": interface, "bitrate": bitrate}
            if channel:
                kwargs["channel"] = channel
            self.bus       = can.interface.Bus(**kwargs)
            self._running  = True
            self.connected = True
            self._hb_thread = threading.Thread(
                target=self._heartbeat_loop, daemon=True)
            self._rx_thread = threading.Thread(
                target=self._rx_loop, daemon=True)
            self._hb_thread.start()
            self._rx_thread.start()
            return True
        except Exception as exc:
            self.bus = None
            return str(exc)

    def disconnect(self):
        self._running  = False
        self.connected = False
        for t in (self._hb_thread, self._rx_thread):
            if t:
                t.join(timeout=1)
        self._hb_thread = None
        self._rx_thread = None
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

    def cmd_relay(self, on):
        with self.lock:
            self._relay_on = bool(on)
            if not on:
                self._pid_on       = False
                self._throttle_pct = 0

    def cmd_throttle(self, pct):
        """Set throttle target and enter PID mode (auto-enables relay)."""
        with self.lock:
            self._throttle_pct = max(0, min(100, int(pct)))
            self._pid_on       = True
            self._relay_on     = True

    def cmd_stop(self):
        with self.lock:
            self._pid_on       = False
            self._throttle_pct = 0

    def cmd_estop(self):
        """Send ESTOP frame immediately, then clear local state."""
        with self.lock:
            self._relay_on     = False
            self._pid_on       = False
            self._throttle_pct = 0
        self._send_frame(estop=True)

    def cmd_reset(self):
        """Send a single RESET frame to clear faults and exit safe state."""
        self._send_frame(reset=True)

    # ── Internal ──────────────────────────────────────────────────────────────
    def _build_payload(self, estop=False, reset=False):
        with self.lock:
            flags = 0
            if self._relay_on: flags |= FLAG_RELAY
            if self._pid_on:   flags |= FLAG_PID
            if estop:          flags |= FLAG_ESTOP
            if reset:          flags |= FLAG_RESET
            thr = max(0, min(100, self._throttle_pct))
            seq = self._tx_seq & 0xFF
            self._tx_seq += 1
        return bytes([flags, thr, 0, seq])

    def _send_frame(self, estop=False, reset=False):
        if self.bus is None:
            return
        try:
            msg = can.Message(
                arbitration_id=CAN_CMD_ID,
                data=self._build_payload(estop=estop, reset=reset),
                is_extended_id=False)
            self.bus.send(msg)
        except can.CanError:
            pass

    def _heartbeat_loop(self):
        interval = 1.0 / self.HB_HZ
        while self._running:
            self._send_frame()
            time.sleep(interval)

    def _rx_loop(self):
        while self._running:
            try:
                msg = self.bus.recv(timeout=0.3)
                if msg is None or msg.arbitration_id != CAN_TELEM_ID:
                    continue
                d = msg.data
                if len(d) < 8:
                    continue

                # [0]: mode bits1:0, relay bit4
                d0       = int(d[0])
                mode_raw = d0 & 0x03
                relay    = (d0 >> 4) & 0x01
                act_pct  = int(d[1])
                tgt_pct  = int(d[2])
                faults   = int(d[3])
                mot_cmd  = struct.unpack_from('<h', bytes(d[4:6]))[0]
                # [6:7]: raw encoder angle in 0.01° units (0xFFFF = no encoder)
                raw_hun  = int(d[6]) | (int(d[7]) << 8)

                mode_str  = MODE_NAMES.get(mode_raw, "?")
                act_val   = None if act_pct == 0xFF else act_pct
                raw_deg   = None if raw_hun == 0xFFFF else raw_hun / 100.0
                fnames    = [n for b, n, _ in FAULT_BITS if faults & b]

                frame_txt = (
                    f"mode={mode_str}  "
                    f"raw={'---' if raw_deg is None else f'{raw_deg:.2f}°'}  "
                    f"pos={'---' if act_val is None else f'{act_val}%'}  "
                    f"tgt={tgt_pct}%  cmd={mot_cmd:+d}  "
                    f"relay={'ON' if relay else 'OFF'}  "
                    f"faults={'OK' if not fnames else ' '.join(fnames)}"
                )

                with self.lock:
                    self.mode        = mode_str
                    self.thr_act     = act_val
                    self.thr_tgt     = tgt_pct
                    self.err_flags   = faults
                    self.motor_cmd   = mot_cmd
                    self.relay_state = relay
                    self.raw_deg     = raw_deg
                    self.last_frame  = frame_txt
            except Exception:
                pass


# ══════════════════════════════════════════════════════════════════════════════
#  GUI
# ══════════════════════════════════════════════════════════════════════════════
class ThrottleGUI:
    # Catppuccin-Mocha palette
    BG      = "#1e1e2e"
    FG      = "#cdd6f4"
    SURFACE = "#313244"
    OVERLAY = "#45475a"
    ACCENT  = "#89b4fa"
    GREEN   = "#a6e3a1"
    RED     = "#f38ba8"
    YELLOW  = "#f9e2af"
    ORANGE  = "#fab387"
    TEAL    = "#94e2d5"

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Throttle ECU — Tasaru V0.0.1 (F280049C) — CAN")
        self.root.configure(bg=self.BG)
        self.root.geometry("1050x860")
        self.root.minsize(900, 720)

        if not _CAN_OK:
            import tkinter.messagebox as mb
            mb.showerror(
                "python-can not installed",
                "Run:  pip install python-can\nThen restart the GUI.")

        self.can       = CanThread()
        self.log_lines = collections.deque(maxlen=300)

        # PID response monitor state
        self._err_history       = collections.deque(maxlen=400)
        self._last_tgt          = None
        self._step_time         = None
        self._step_error        = 0.0
        self._approach_dir      = 0
        self._crossed           = False
        self._peak_over         = 0.0
        self._rise_time         = None
        self._settle_time       = None
        self._settle_band_start = None

        # Angle calibration (mutable — updated by calibration panel)
        self._cal_open_deg   = ANGLE_MIN_RAW / 100.0   # 22.11° (physical open)
        self._cal_closed_deg = ANGLE_MAX_RAW / 100.0   # 132.62° (physical closed)
        self._cal_buffer_deg = 2.0                      # end-stop safety margin (°)
        self._cal_usable_min = USABLE_MIN
        self._cal_usable_max = USABLE_MAX
        self._cal_angle_min  = ANGLE_MIN_RAW
        self._cal_angle_max  = ANGLE_MAX_RAW
        self._apply_calibration()

        self._setup_styles()
        self._build_ui()
        self._poll()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── Styles ─────────────────────────────────────────────────────────────────
    def _setup_styles(self):
        s = ttk.Style()
        s.theme_use("clam")
        s.configure("TFrame",       background=self.BG)
        s.configure("TLabel",       background=self.BG, foreground=self.FG,
                                    font=("Segoe UI", 10))
        s.configure("Title.TLabel", font=("Segoe UI", 11, "bold"),
                                    foreground=self.ACCENT, background=self.BG)
        s.configure("TButton",      font=("Segoe UI", 10))
        s.configure("TCombobox",    fieldbackground=self.SURFACE,
                                    foreground=self.FG)

    # ── Main layout ────────────────────────────────────────────────────────────
    def _build_ui(self):
        # ── Connection bar ──────────────────────────────────────────────────────
        bar = ttk.Frame(self.root)
        bar.pack(fill="x", padx=12, pady=(10, 4))

        ttk.Label(bar, text="Interface:").pack(side="left")
        self.iface_var = tk.StringVar(value="slcan")
        ttk.Combobox(bar, textvariable=self.iface_var, width=10,
                     values=CAN_INTERFACES, state="readonly"
                     ).pack(side="left", padx=(4, 10))

        ttk.Label(bar, text="Channel:").pack(side="left")
        self.chan_var = tk.StringVar(value="COM5")
        ports = _get_com_ports()
        self.chan_combo = ttk.Combobox(bar, textvariable=self.chan_var, width=10,
                                       values=ports if ports else ["COM5"],
                                       font=("Consolas", 10))
        self.chan_combo.pack(side="left", padx=(4, 2))
        tk.Button(bar, text="⟳", bg=self.SURFACE, fg=self.FG,
                  font=("Segoe UI", 10), relief="flat", width=2,
                  command=self._refresh_ports).pack(side="left", padx=(0, 8))

        ttk.Label(bar, text="Bitrate:").pack(side="left")
        self.baud_var = tk.StringVar(value="500000")
        tk.Entry(bar, textvariable=self.baud_var, width=8,
                 bg=self.SURFACE, fg=self.FG,
                 insertbackground=self.FG, font=("Consolas", 10)
                 ).pack(side="left", padx=(4, 10))

        self.btn_conn = ttk.Button(bar, text="Connect",
                                   command=self._toggle_connect)
        self.btn_conn.pack(side="left", padx=2)

        self.lbl_conn = tk.Label(bar, text="● Disconnected",
                                  bg=self.BG, fg=self.RED,
                                  font=("Segoe UI", 10, "bold"))
        self.lbl_conn.pack(side="left", padx=8)

        self.lbl_mode = tk.Label(bar, text="MODE: ---",
                                  bg=self.BG, fg=self.OVERLAY,
                                  font=("Consolas", 12, "bold"))
        self.lbl_mode.pack(side="right", padx=10)

        # ── Body ─────────────────────────────────────────────────────────────────
        body = ttk.Frame(self.root)
        body.pack(fill="both", expand=True, padx=12, pady=4)
        left  = ttk.Frame(body)
        left.pack(side="left", fill="both", expand=True)
        right = ttk.Frame(body)
        right.pack(side="right", fill="y", padx=(10, 0))

        # ── Relay / E-STOP row ──────────────────────────────────────────────────
        ctrl = ttk.Frame(left)
        ctrl.pack(fill="x", pady=(0, 6))
        ttk.Label(ctrl, text="Solenoid (GPIO7 / TPS1H100B)",
                  style="Title.TLabel").pack(side="left")

        self.btn_estop = tk.Button(
            ctrl, text="E-STOP", bg="#c0392b", fg="white",
            font=("Segoe UI", 12, "bold"), width=8, relief="raised",
            activebackground="#e74c3c", command=self._do_estop)
        self.btn_estop.pack(side="right", padx=(6, 0))

        self.btn_reset = tk.Button(
            ctrl, text="CLR FAULTS", bg="#6c3483", fg="white",
            font=("Segoe UI", 10, "bold"), width=10, relief="raised",
            activebackground="#9b59b6", command=self._do_reset)
        self.btn_reset.pack(side="right", padx=(6, 0))

        self.btn_relay_off = tk.Button(
            ctrl, text="OFF", bg="#5a1e1e", fg="white",
            font=("Segoe UI", 11, "bold"), width=5, relief="flat",
            command=lambda: self.can.cmd_relay(False))
        self.btn_relay_off.pack(side="right", padx=2)

        self.btn_relay_on = tk.Button(
            ctrl, text="ON", bg="#2d5016", fg="white",
            font=("Segoe UI", 11, "bold"), width=5, relief="flat",
            command=lambda: self.can.cmd_relay(True))
        self.btn_relay_on.pack(side="right", padx=2)
        ttk.Label(ctrl, text="Relay:").pack(side="right", padx=(0, 4))

        # ── Telemetry cards ─────────────────────────────────────────────────────
        cards = ttk.Frame(left)
        cards.pack(fill="x", pady=(0, 8))
        self.lbl_raw_deg = self._card(cards, "Raw Angle",       "---")
        self.lbl_pos     = self._card(cards, "Cal Position",    "---")
        self.lbl_thr_act = self._card(cards, "Throttle Actual", "---%")
        self.lbl_thr_tgt = self._card(cards, "Throttle Target", "0%")
        self.lbl_duty    = self._card(cards, "Motor Cmd",       "0")
        self.lbl_relay   = self._card(cards, "Relay",           "---")

        # ── Throttle slider ─────────────────────────────────────────────────────
        sl = tk.Frame(left, bg=self.SURFACE, padx=12, pady=10)
        sl.pack(fill="x", pady=(0, 8))
        tk.Label(sl, text="Throttle Command (%)",
                 bg=self.SURFACE, fg=self.ACCENT,
                 font=("Segoe UI", 11, "bold")).pack(anchor="w")

        sl_row = tk.Frame(sl, bg=self.SURFACE)
        sl_row.pack(fill="x", pady=(4, 0))
        self.thr_var = tk.IntVar(value=0)
        tk.Scale(sl_row, from_=0, to=100, orient="horizontal",
                 variable=self.thr_var, bg=self.SURFACE, fg=self.FG,
                 highlightbackground=self.SURFACE, troughcolor=self.OVERLAY,
                 activebackground=self.ACCENT, length=400,
                 command=self._on_slider_move).pack(side="left")
        self.lbl_slider_val = tk.Label(
            sl_row, text="0%", bg=self.SURFACE, fg=self.GREEN,
            font=("Consolas", 14, "bold"), width=5)
        self.lbl_slider_val.pack(side="left", padx=8)

        btn_row = tk.Frame(sl, bg=self.SURFACE)
        btn_row.pack(fill="x", pady=(6, 0))
        for pct in (0, 10, 25, 50, 75, 100):
            p = pct
            tk.Button(btn_row, text=f"{p}%", bg=self.OVERLAY, fg=self.FG,
                      font=("Segoe UI", 9), width=4, relief="flat",
                      command=lambda v=p: (self._set_throttle(v), self._send_throttle())
                      ).pack(side="left", padx=2)
        tk.Button(btn_row, text="STOP", bg=self.RED, fg=self.BG,
                  font=("Segoe UI", 10, "bold"), relief="flat",
                  command=self._do_stop).pack(side="right", padx=(6, 0))
        tk.Button(btn_row, text="Send →", bg=self.ACCENT, fg=self.BG,
                  font=("Segoe UI", 10, "bold"), relief="flat",
                  command=self._send_throttle).pack(side="right")

        # ── Position bar ────────────────────────────────────────────────────────
        self._build_pos_bar(left)

        # ── CAN telemetry log ───────────────────────────────────────────────────
        log_f = ttk.Frame(left)
        log_f.pack(fill="both", expand=True, pady=(4, 0))
        tk.Label(log_f, text="CAN Telemetry Log  (0x101 → PC at 50 Hz)",
                 bg=self.BG, fg=self.ACCENT,
                 font=("Segoe UI", 10, "bold")).pack(anchor="w")
        log_inner = tk.Frame(log_f, bg=self.SURFACE)
        log_inner.pack(fill="both", expand=True)
        self.log_text = tk.Text(
            log_inner, bg=self.SURFACE, fg=self.FG,
            font=("Consolas", 9), state="disabled",
            wrap="none", height=8, insertbackground=self.FG,
            selectbackground=self.OVERLAY)
        sb = tk.Scrollbar(log_inner, command=self.log_text.yview, bg=self.OVERLAY)
        self.log_text.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log_text.pack(fill="both", expand=True)

        # ── Right panel ─────────────────────────────────────────────────────────
        self._build_pid_monitor(right)
        self._build_fault_panel(right)
        self._build_cal_panel(right)

        # ── Bottom quick-send bar ───────────────────────────────────────────────
        self._build_quick_bar()

    # ── Position bar ────────────────────────────────────────────────────────────
    def _build_pos_bar(self, parent):
        f = tk.Frame(parent, bg=self.SURFACE, padx=12, pady=8)
        f.pack(fill="x", pady=(0, 6))
        tk.Label(f, text="Throttle Position",
                 bg=self.SURFACE, fg=self.ACCENT,
                 font=("Segoe UI", 10, "bold")).pack(anchor="w")
        self.pos_canvas = tk.Canvas(f, height=28, bg=self.OVERLAY,
                                     highlightthickness=0)
        self.pos_canvas.pack(fill="x", pady=(4, 0))
        self.tgt_canvas = tk.Canvas(f, height=16, bg=self.SURFACE,
                                     highlightthickness=0)
        self.tgt_canvas.pack(fill="x")
        tk.Label(f, text="actual ●  target ▼",
                 bg=self.SURFACE, fg=self.OVERLAY,
                 font=("Segoe UI", 8)).pack(anchor="w")

    def _draw_pos_bar(self, thr_act, thr_tgt):
        w = self.pos_canvas.winfo_width()
        if w < 10:
            return
        self.pos_canvas.delete("all")
        self.tgt_canvas.delete("all")
        self.pos_canvas.create_rectangle(0, 0, w, 28, fill=self.OVERLAY, outline="")
        if thr_act is not None:
            bw = int(w * thr_act / 100)
            col = (self.GREEN if thr_act < 80 else
                   self.ORANGE if thr_act < 95 else self.RED)
            self.pos_canvas.create_rectangle(0, 4, bw, 24, fill=col, outline="")
            self.pos_canvas.create_text(
                bw + 6 if bw < w - 40 else bw - 6, 14,
                text=f"{thr_act}%",
                fill=self.BG if bw > 30 else self.FG,
                font=("Consolas", 10, "bold"),
                anchor="w" if bw < w - 40 else "e")
        tx = int(w * thr_tgt / 100)
        self.tgt_canvas.create_text(tx, 8, text="▼", fill=self.YELLOW,
                                     font=("Segoe UI", 10, "bold"))

    # ── PID response monitor ─────────────────────────────────────────────────────
    def _build_pid_monitor(self, parent):
        f = tk.Frame(parent, bg=self.SURFACE, padx=10, pady=8)
        f.pack(fill="x", pady=(0, 8))
        tk.Label(f, text="PID Response",
                 bg=self.SURFACE, fg=self.TEAL,
                 font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(0, 4))
        self.err_canvas = tk.Canvas(f, height=120, bg="#11111b",
                                    highlightthickness=1,
                                    highlightbackground=self.OVERLAY)
        self.err_canvas.pack(fill="x", pady=(0, 6))

        stats = tk.Frame(f, bg=self.SURFACE)
        stats.pack(fill="x")

        def _stat(label, init, color=None):
            col = tk.Frame(stats, bg=self.SURFACE)
            col.pack(side="left", expand=True)
            tk.Label(col, text=label, bg=self.SURFACE, fg=self.OVERLAY,
                     font=("Segoe UI", 7)).pack()
            lbl = tk.Label(col, text=init, bg=self.SURFACE,
                           fg=color or self.FG,
                           font=("Consolas", 10, "bold"))
            lbl.pack()
            return lbl

        self.lbl_err_cur    = _stat("Error",       "---")
        self.lbl_err_over   = _stat("Overshoot",   "---", self.ORANGE)
        self.lbl_err_rise   = _stat("Rise Time",   "---", self.TEAL)
        self.lbl_err_settle = _stat("Settle Time", "---", self.GREEN)

        self.lbl_pid_hint = tk.Label(
            f, text="Use throttle slider to begin tracking",
            bg=self.SURFACE, fg=self.OVERLAY,
            font=("Segoe UI", 8), wraplength=240, justify="left")
        self.lbl_pid_hint.pack(anchor="w", pady=(5, 0))

    def _draw_err_chart(self):
        c = self.err_canvas
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10:
            return
        c.delete("all")
        BG_C = "#11111b"; GRID = "#313244"; ZERO = "#585b70"; BAND = "#1a2a1a"
        Y_RNG = 25.0; X_SPAN = 12.0
        c.create_rectangle(0, 0, w, h, fill=BG_C, outline="")
        mid_y = h / 2.0

        def to_y(e):
            return mid_y - (max(-Y_RNG, min(Y_RNG, e)) / Y_RNG) * mid_y

        c.create_rectangle(0, to_y(5), w, to_y(-5), fill=BAND, outline="")
        for pct in (20, 10, 5, -5, -10, -20):
            c.create_line(0, to_y(pct), w, to_y(pct), fill=GRID, dash=(2, 4))
        c.create_line(0, mid_y, w, mid_y, fill=ZERO, width=1)
        now_s = time.time()
        for age in (2, 4, 6, 8, 10):
            x = w * (1.0 - age / X_SPAN)
            if x > 0:
                c.create_line(x, mid_y - 3, x, mid_y + 3, fill=ZERO)
                c.create_text(x, h - 2, text=f"-{age}s", fill=ZERO,
                              font=("Consolas", 7), anchor="s")
        c.create_text(3, 2,     text=f"+{int(Y_RNG)}%", fill=ZERO,
                      font=("Consolas", 7), anchor="nw")
        c.create_text(3, h - 2, text=f"-{int(Y_RNG)}%", fill=ZERO,
                      font=("Consolas", 7), anchor="sw")
        pts = []
        for t, err in self._err_history:
            age = now_s - t
            if age > X_SPAN:
                continue
            pts.append((w * (1.0 - age / X_SPAN), to_y(err), err))
        if len(pts) > 1:
            for i in range(len(pts) - 1):
                x1, y1, e1 = pts[i]
                x2, y2, e2 = pts[i + 1]
                avg = abs((e1 + e2) / 2.0)
                col = ("#a6e3a1" if avg < 5.0 else
                       "#fab387" if avg < 15.0 else "#f38ba8")
                c.create_line(x1, y1, x2, y2, fill=col, width=2)
        if pts:
            x, y, e = pts[-1]
            col = ("#a6e3a1" if abs(e) < 5.0 else
                   "#fab387" if abs(e) < 15.0 else "#f38ba8")
            c.create_oval(x - 3, y - 3, x + 3, y + 3, fill=col, outline="")

    # ── Fault panel ──────────────────────────────────────────────────────────────
    def _build_fault_panel(self, parent):
        outer = tk.Frame(parent, bg=self.SURFACE, padx=12, pady=10)
        outer.pack(fill="x", pady=(0, 8))
        tk.Label(outer, text="Active Faults",
                 bg=self.SURFACE, fg=self.ACCENT,
                 font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(0, 4))
        self.fault_labels = {}
        for bit, short, desc in FAULT_BITS:
            row = tk.Frame(outer, bg=self.SURFACE)
            row.pack(fill="x", pady=1)
            dot = tk.Label(row, text="●", bg=self.SURFACE, fg=self.OVERLAY,
                           font=("Consolas", 12))
            dot.pack(side="left", padx=(0, 4))
            tk.Label(row, text=f"{short:<14}", bg=self.SURFACE, fg=self.FG,
                     font=("Consolas", 10)).pack(side="left")
            tk.Label(row, text=desc, bg=self.SURFACE, fg=self.OVERLAY,
                     font=("Segoe UI", 8)).pack(side="left")
            self.fault_labels[bit] = dot

    # ── Bottom quick-send bar ────────────────────────────────────────────────────
    def _build_quick_bar(self):
        bar = tk.Frame(self.root, bg=self.SURFACE, padx=12, pady=6)
        bar.pack(fill="x", side="bottom", pady=(4, 0))
        tk.Label(bar, text="Quick %:", bg=self.SURFACE, fg=self.FG,
                 font=("Segoe UI", 10)).pack(side="left")
        for pct in (0, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100):
            p = pct
            clr = self.GREEN if p < 70 else self.ORANGE if p < 90 else self.RED
            tk.Button(bar, text=f"{p}%", bg=self.OVERLAY, fg=clr,
                      font=("Segoe UI", 9), relief="flat", width=4,
                      command=lambda v=p: (self._set_throttle(v),
                                           self._send_throttle())
                      ).pack(side="left", padx=1)
        tk.Button(bar, text="ESTOP", bg="#c0392b", fg="white",
                  font=("Segoe UI", 9, "bold"), relief="flat", width=6,
                  command=self._do_estop).pack(side="right", padx=4)
        tk.Button(bar, text="STOP", bg=self.RED, fg=self.BG,
                  font=("Segoe UI", 9, "bold"), relief="flat", width=5,
                  command=self._do_stop).pack(side="right", padx=2)

    # ── Helpers ──────────────────────────────────────────────────────────────────
    def _card(self, parent, title, initial):
        f = tk.Frame(parent, bg=self.SURFACE, padx=10, pady=6)
        f.pack(side="left", padx=4, pady=2)
        tk.Label(f, text=title, bg=self.SURFACE, fg=self.OVERLAY,
                 font=("Segoe UI", 8)).pack()
        lbl = tk.Label(f, text=initial, bg=self.SURFACE, fg=self.GREEN,
                       font=("Consolas", 13, "bold"), width=7)
        lbl.pack()
        return lbl

    def _refresh_ports(self):
        ports = _get_com_ports()
        self.chan_combo["values"] = ports if ports else ["COM5"]
        if ports and self.chan_var.get() not in ports:
            self.chan_var.set(ports[0])

    def _toggle_connect(self):
        if self.can.connected:
            self.can.disconnect()
            self.btn_conn.configure(text="Connect")
            self.lbl_conn.configure(text="● Disconnected", fg=self.RED)
        else:
            iface   = self.iface_var.get().strip()
            channel = self.chan_var.get().strip() or None
            try:
                bitrate = int(self.baud_var.get().strip())
            except ValueError:
                bitrate = CAN_BITRATE
            result = self.can.connect(iface, channel, bitrate)
            if result is True:
                self.btn_conn.configure(text="Disconnect")
                self.lbl_conn.configure(text="● Connected", fg=self.GREEN)
            else:
                self.lbl_conn.configure(
                    text=f"● Error: {str(result)[:45]}", fg=self.RED)

    def _set_throttle(self, val):
        self.thr_var.set(val)
        self.lbl_slider_val.configure(text=f"{val}%")

    def _on_slider_move(self, val):
        self.lbl_slider_val.configure(text=f"{int(float(val))}%")

    def _send_throttle(self):
        self.can.cmd_throttle(self.thr_var.get())

    def _do_stop(self):
        self._set_throttle(0)
        self.can.cmd_stop()

    def _do_estop(self):
        self._set_throttle(0)
        self.can.cmd_estop()

    def _do_reset(self):
        self.can.cmd_reset()

    # ── Poll loop (GUI thread, every 150 ms) ─────────────────────────────────────
    def _poll(self):
        if not self.can.connected:
            self.root.after(200, self._poll)
            return

        with self.can.lock:
            mode       = self.can.mode
            thr_act    = self.can.thr_act
            thr_tgt    = self.can.thr_tgt
            err_flags  = self.can.err_flags
            motor_cmd  = self.can.motor_cmd
            relay      = self.can.relay_state
            raw_deg    = self.can.raw_deg
            last_frame = self.can.last_frame

        # Mode badge
        label, bg_col, fg_col = MODE_STYLE.get(mode, (mode, self.OVERLAY, self.FG))
        self.lbl_mode.configure(text=f"MODE: {label}", bg=bg_col, fg=fg_col)

        # Raw angle card — direct from encoder, zero calibration math
        if raw_deg is not None:
            self.lbl_raw_deg.configure(text=f"{raw_deg:.2f}°", fg=self.TEAL)
        else:
            self.lbl_raw_deg.configure(text="---", fg=self.OVERLAY)

        # Telemetry cards
        if thr_act is not None:
            # Inverted sensor: 100% = open = usable_min angle, 0% = closed = usable_max angle
            pos_deg = (self._cal_usable_max -
                       (self._cal_usable_max - self._cal_usable_min) * thr_act / 100.0) / 100.0
            self.lbl_pos.configure(text=f"{pos_deg:.2f}°", fg=self.GREEN)
        else:
            self.lbl_pos.configure(text="---", fg=self.OVERLAY)

        self.lbl_thr_act.configure(
            text=f"{thr_act}%" if thr_act is not None else "---%",
            fg=self.GREEN if thr_act is not None else self.OVERLAY)
        self.lbl_thr_tgt.configure(text=f"{thr_tgt}%")
        self.lbl_duty.configure(text=str(motor_cmd))
        self.lbl_relay.configure(
            text="ON" if relay else "OFF",
            fg=self.GREEN if relay else self.RED)

        # Position bar
        self._draw_pos_bar(thr_act, thr_tgt)

        # PID response monitor
        now_s = time.time()
        if thr_act is not None and mode == "PID":
            error = float(thr_tgt - thr_act)

            if self._last_tgt != thr_tgt:
                self._last_tgt          = thr_tgt
                self._step_time         = now_s
                self._step_error        = abs(error)
                self._approach_dir      = 1 if error > 0 else (-1 if error < 0 else 0)
                self._crossed           = False
                self._peak_over         = 0.0
                self._rise_time         = None
                self._settle_time       = None
                self._settle_band_start = None
                self._err_history.clear()

            self._err_history.append((now_s, error))

            if self._step_time is not None:
                elapsed = now_s - self._step_time
                if self._rise_time is None and abs(error) < 5.0:
                    self._rise_time = elapsed
                if (not self._crossed and self._approach_dir != 0
                        and self._approach_dir * error < 0):
                    self._crossed = True
                if self._crossed:
                    over = -self._approach_dir * error
                    if over > self._peak_over:
                        self._peak_over = over
                if abs(error) < 2.0:
                    if self._settle_band_start is None:
                        self._settle_band_start = now_s
                    elif (self._settle_time is None
                          and (now_s - self._settle_band_start) >= 0.5):
                        self._settle_time = elapsed
                else:
                    self._settle_band_start = None

            ecol = (self.GREEN if abs(error) < 5.0 else
                    self.ORANGE if abs(error) < 15.0 else self.RED)
            self.lbl_err_cur.configure(text=f"{error:+.1f}%", fg=ecol)
            if self._peak_over > 0.5:
                self.lbl_err_over.configure(
                    text=f"{self._peak_over:.1f}%",
                    fg=self.RED if self._peak_over > 10.0 else self.ORANGE)
            else:
                self.lbl_err_over.configure(text="none", fg=self.GREEN)
            if self._rise_time is not None:
                self.lbl_err_rise.configure(
                    text=f"{self._rise_time * 1000:.0f} ms", fg=self.TEAL)
            else:
                self.lbl_err_rise.configure(text="---", fg=self.OVERLAY)
            if self._settle_time is not None:
                self.lbl_err_settle.configure(
                    text=f"{self._settle_time * 1000:.0f} ms", fg=self.GREEN)
            elif self._step_time is not None:
                self.lbl_err_settle.configure(
                    text=f"{(now_s - self._step_time) * 1000:.0f}...",
                    fg=self.OVERLAY)
            else:
                self.lbl_err_settle.configure(text="---", fg=self.OVERLAY)

            if self._step_error < 10.0:
                hint, hcol = "Send a bigger step to evaluate", self.OVERLAY
            elif (self._step_time and (now_s - self._step_time) > 5.0
                  and self._rise_time is None):
                hint, hcol = "Not converging — check encoder & gains", self.RED
            elif self._peak_over > 20.0:
                hint, hcol = "High overshoot — reduce Kp or increase Kd", self.RED
            elif self._peak_over > 10.0:
                hint, hcol = "Moderate overshoot — try increasing Kd", self.ORANGE
            elif (self._rise_time and self._rise_time > 2.0
                  and self._settle_time is None):
                hint, hcol = "Slow rise — try increasing Kp", self.YELLOW
            elif self._settle_time and self._peak_over < 5.0:
                hint, hcol = "Well tuned — fast settle, low overshoot", self.GREEN
            else:
                hint, hcol = "Tracking...", self.OVERLAY
            self.lbl_pid_hint.configure(text=hint, fg=hcol)

        self._draw_err_chart()

        # Fault indicators
        for bit, dot in self.fault_labels.items():
            dot.configure(fg=self.RED if (err_flags & bit) else self.OVERLAY)

        # CAN log
        if last_frame and (not self.log_lines or self.log_lines[-1] != last_frame):
            self.log_lines.append(last_frame)
            self.log_text.configure(state="normal")
            self.log_text.insert("end", last_frame + "\n")
            self.log_text.see("end")
            lc = int(self.log_text.index("end-1c").split(".")[0])
            if lc > 300:
                self.log_text.delete("1.0", "2.0")
            self.log_text.configure(state="disabled")

        self.root.after(150, self._poll)

    # ── Calibration ──────────────────────────────────────────────────────────────
    def _apply_calibration(self):
        """Recompute usable-range bounds from open/closed positions + buffer."""
        buf        = int(self._cal_buffer_deg * 100)   # 0.01° units
        open_raw   = int(self._cal_open_deg   * 100)
        closed_raw = int(self._cal_closed_deg * 100)
        lo = min(open_raw, closed_raw)
        hi = max(open_raw, closed_raw)
        self._cal_angle_min  = lo + buf
        self._cal_angle_max  = hi - buf
        rng = max(1, self._cal_angle_max - self._cal_angle_min)
        self._cal_usable_min = self._cal_angle_min + rng * 5  // 100
        self._cal_usable_max = self._cal_angle_min + rng * 95 // 100

    def _build_cal_panel(self, parent):
        f = tk.Frame(parent, bg=self.SURFACE, padx=12, pady=10)
        f.pack(fill="x", pady=(0, 8))
        tk.Label(f, text="Angle Calibration",
                 bg=self.SURFACE, fg=self.ACCENT,
                 font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(0, 4))

        self._cal_open_var   = tk.StringVar(value=f"{self._cal_open_deg:.2f}")
        self._cal_closed_var = tk.StringVar(value=f"{self._cal_closed_deg:.2f}")
        self._cal_buffer_var = tk.StringVar(value=f"{self._cal_buffer_deg:.1f}")

        def _row(label, var, btn_text=None, btn_cmd=None):
            row = tk.Frame(f, bg=self.SURFACE)
            row.pack(fill="x", pady=2)
            tk.Label(row, text=label, bg=self.SURFACE, fg=self.FG,
                     font=("Segoe UI", 9), width=15, anchor="w").pack(side="left")
            tk.Entry(row, textvariable=var, width=7,
                     bg=self.OVERLAY, fg=self.GREEN,
                     insertbackground=self.GREEN,
                     font=("Consolas", 10)).pack(side="left")
            tk.Label(row, text="°", bg=self.SURFACE, fg=self.OVERLAY,
                     font=("Segoe UI", 9)).pack(side="left", padx=(2, 0))
            if btn_text and btn_cmd:
                tk.Button(row, text=btn_text, bg=self.OVERLAY, fg=self.FG,
                          font=("Segoe UI", 8), relief="flat",
                          command=btn_cmd).pack(side="left", padx=(6, 0))

        _row("OPEN (100%):",   self._cal_open_var,   "← Capture", self._capture_open)
        _row("CLOSED (0%):",   self._cal_closed_var, "← Capture", self._capture_closed)
        _row("End-stop buffer:", self._cal_buffer_var)

        tk.Label(f, text="Buffer is added inside each physical limit",
                 bg=self.SURFACE, fg=self.OVERLAY,
                 font=("Segoe UI", 7)).pack(anchor="w", pady=(0, 4))

        apply_row = tk.Frame(f, bg=self.SURFACE)
        apply_row.pack(fill="x", pady=(2, 4))
        tk.Button(apply_row, text="Apply Calibration",
                  bg=self.ACCENT, fg=self.BG,
                  font=("Segoe UI", 10, "bold"), relief="flat",
                  command=self._do_apply_cal).pack(side="left")

        out_box = tk.Frame(f, bg=self.BG, padx=8, pady=6)
        out_box.pack(fill="x")
        tk.Label(out_box, text="Copy to throttle_config.h:",
                 bg=self.BG, fg=self.OVERLAY,
                 font=("Segoe UI", 8)).pack(anchor="w")
        self.lbl_cal_out = tk.Label(out_box, text="(press Apply)",
                                     bg=self.BG, fg=self.TEAL,
                                     font=("Consolas", 9), justify="left",
                                     wraplength=260, anchor="w")
        self.lbl_cal_out.pack(anchor="w")
        # Sensor resolution note
        tk.Label(out_box,
                 text="AS5147U: 14-bit  →  360°÷16384 = 0.022°/step  (1.3 arcmin)\n"
                      "Display shows 0.01° — sensor LSB ≈ 2 code units",
                 bg=self.BG, fg=self.FG,
                 font=("Segoe UI", 8), justify="left").pack(anchor="w", pady=(5, 0))

    def _capture_open(self):
        with self.can.lock:
            raw_deg = self.can.raw_deg
        if raw_deg is None:
            return
        self._cal_open_var.set(f"{raw_deg:.2f}")

    def _capture_closed(self):
        with self.can.lock:
            raw_deg = self.can.raw_deg
        if raw_deg is None:
            return
        self._cal_closed_var.set(f"{raw_deg:.2f}")

    def _do_apply_cal(self):
        try:
            self._cal_open_deg   = float(self._cal_open_var.get())
            self._cal_closed_deg = float(self._cal_closed_var.get())
            self._cal_buffer_deg = max(0.0, float(self._cal_buffer_var.get()))
        except ValueError:
            return
        self._apply_calibration()
        self.lbl_cal_out.configure(
            text=(f"#define CFG_ANGLE_MIN  {self._cal_angle_min}\n"
                  f"#define CFG_ANGLE_MAX  {self._cal_angle_max}"))

    def _on_close(self):
        self.can.disconnect()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    ThrottleGUI().run()
