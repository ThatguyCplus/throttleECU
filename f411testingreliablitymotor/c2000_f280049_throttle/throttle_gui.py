"""
Throttle ECU Control GUI — C2000 F280049C
Connects to the LaunchPad XDS110 backchannel UART, sends commands, displays live telemetry.

Requirements:
    pip install pyserial

Run:
    python throttle_gui.py
"""

import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import re
import time
import collections

# ── Angle calibration (must match throttle_config.h) ──────────────────────────
ANGLE_MIN_RAW = 7032    # CFG_ANGLE_MIN  (units of 0.01 deg)
ANGLE_MAX_RAW = 17940   # CFG_ANGLE_MAX
ANGLE_RANGE   = ANGLE_MAX_RAW - ANGLE_MIN_RAW
USABLE_MIN    = ANGLE_MIN_RAW + ANGLE_RANGE * 5  // 100   # 5 %
USABLE_MAX    = ANGLE_MIN_RAW + ANGLE_RANGE * 95 // 100   # 95 %

# ── Telemetry regex ────────────────────────────────────────────────────────────
# Board sends every 200 ms:
#   mode=MAN cmd=0 RIS=0 LIS=0 pos=75.77 thr=0% tgt=0% err=0x00
#   mode=MAN cmd=0 RIS=0 LIS=0 pos=--- thr=---% tgt=0% err=0x00
# cmd is signed (negative = reverse); replaces the old "duty" field
TELEM_RE = re.compile(
    r"mode=(\w+)\s+cmd=(-?\d+)\s+"
    r"RIS=(\d+)\s+LIS=(\d+)\s+"
    r"pos=([\d.]+|---)\s+"
    r"thr=(\d+|---)%\s+"
    r"tgt=(\d+)%\s+"
    r"err=0x([0-9A-Fa-f]+)"
)
SAFE_RE      = re.compile(r"\[SAFE\]\s*(.+)")
WARN_RE      = re.compile(r"\[WARN\]\s*(.+)")

# ── Fault bit definitions (must match safety.h) ────────────────────────────────
FAULT_BITS = [
    (0x01, "ENC_STALE",    "Encoder stale (>500 ms without update)"),
    (0x02, "ENC_INVALID",  "Encoder signal invalid"),
    (0x04, "OVERCURR_L",   "Left motor overcurrent"),
    (0x08, "OVERCURR_R",   "Right motor overcurrent"),
    (0x10, "POWER_LOW",    "Supply voltage below 9 V"),
    (0x20, "WDG_RESET",    "Recovered from watchdog reset"),
    (0x40, "CAN_TIMEOUT",  "CAN RX heartbeat lost (>200 ms)"),
    (0x80, "CAN_BUS_OFF",  "CAN controller bus-off — wire/termination fault"),
]

MODE_STYLE = {
    "MAN":  ("MANUAL",           "#a6e3a1", "#1e1e2e"),
    "PID":  ("PID ACTIVE",       "#fab387", "#1e1e2e"),
    "SAFE": ("!! SAFE STATE !!", "#f38ba8", "#1e1e2e"),
}


# ══════════════════════════════════════════════════════════════════════════════
#  Serial thread — non-blocking read + parse
# ══════════════════════════════════════════════════════════════════════════════
class SerialThread:
    def __init__(self):
        self.ser       = None
        self.running   = False
        self.thread    = None
        self.lock      = threading.Lock()
        self.connected = False

        # Latest parsed telemetry
        self.mode      = "MAN"
        self.dir       = "F"
        self.cmd       = 0   # signed motor command (-4095..4095)
        self.ris       = 0
        self.lis       = 0
        self.pos       = None      # float degrees, or None when invalid
        self.thr_act   = None      # int 0-100 or None
        self.thr_tgt   = 0
        self.err_flags = 0

        self.safe_reason = ""
        self.warn_msg    = ""
        self.last_line   = ""      # most recent raw line received

    def connect(self, port, baud=115200):
        self.disconnect()
        try:
            self.ser       = serial.Serial(port, baud, timeout=0.1)
            self.connected = True
            self.running   = True
            self.thread    = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as exc:
            self.connected = False
            return str(exc)

    def disconnect(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
            self.thread = None
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser       = None
        self.connected = False

    def send(self, cmd):
        with self.lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write((cmd.strip() + "\n").encode())
                except Exception:
                    pass

    def _read_loop(self):
        buf = ""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    buf += self.ser.read(self.ser.in_waiting).decode("utf-8", errors="replace")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        with self.lock:
                            self.last_line = line

                        m = TELEM_RE.search(line)
                        if m:
                            with self.lock:
                                self.mode = m.group(1)
                                self.cmd  = int(m.group(2))
                                self.ris  = int(m.group(3))
                                self.lis  = int(m.group(4))
                                p = m.group(5)
                                self.pos = float(p) if p != "---" else None
                                t = m.group(6)
                                self.thr_act   = int(t) if t != "---" else None
                                self.thr_tgt   = int(m.group(7))
                                self.err_flags = int(m.group(8), 16)
                                if self.mode != "SAFE":
                                    self.safe_reason = ""
                            continue

                        sm = SAFE_RE.search(line)
                        if sm:
                            with self.lock:
                                self.safe_reason = sm.group(1).strip()
                            continue

                        wm = WARN_RE.search(line)
                        if wm:
                            with self.lock:
                                self.warn_msg = wm.group(1).strip()

                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.05)


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
        self.root.title("Throttle ECU — C2000 F280049C")
        self.root.configure(bg=self.BG)
        self.root.geometry("1150x900")
        self.root.minsize(1000, 800)

        self.serial = SerialThread()
        self.log_lines = collections.deque(maxlen=300)

        # PID response monitor state
        self._err_history       = collections.deque(maxlen=400)
        self._last_tgt          = None
        self._step_time         = None   # time.time() when target last changed
        self._step_error        = 0.0    # |error| at the moment of the step
        self._approach_dir      = 0      # +1 or -1
        self._crossed           = False  # has pos crossed setpoint yet?
        self._peak_over         = 0.0    # peak overshoot (%)
        self._rise_time         = None   # seconds from step to first ±5% entry
        self._settle_time       = None   # seconds from step to settled
        self._settle_band_start = None   # time.time() when entered ±2% band

        self._setup_styles()
        self._build_ui()
        self._poll()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── Styles ─────────────────────────────────────────────────────────────────
    def _setup_styles(self):
        s = ttk.Style()
        s.theme_use("clam")
        s.configure("TFrame",      background=self.BG)
        s.configure("TLabel",      background=self.BG, foreground=self.FG,
                                   font=("Segoe UI", 10))
        s.configure("Title.TLabel",font=("Segoe UI", 11, "bold"),
                                   foreground=self.ACCENT, background=self.BG)
        s.configure("Val.TLabel",  font=("Consolas", 14, "bold"),
                                   foreground=self.GREEN, background=self.SURFACE)
        s.configure("Mode.TLabel", font=("Consolas", 12, "bold"),
                                   background=self.BG)
        s.configure("Small.TLabel",font=("Segoe UI", 9),
                                   foreground=self.OVERLAY, background=self.BG)
        s.configure("Card.TFrame", background=self.SURFACE,
                                   relief="flat")
        s.configure("TButton",     font=("Segoe UI", 10))
        s.configure("TScale",      background=self.BG)
        s.configure("TCombobox",   fieldbackground=self.SURFACE,
                                   foreground=self.FG)

    # ── Main UI layout ─────────────────────────────────────────────────────────
    def _build_ui(self):
        # ── Connection bar ──────────────────────────────────────────────────────
        bar = ttk.Frame(self.root)
        bar.pack(fill="x", padx=12, pady=(10, 4))

        ttk.Label(bar, text="Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(bar, textvariable=self.port_var,
                                     width=26, state="readonly")
        self.port_cb.pack(side="left", padx=(4, 2))
        self._refresh_ports()

        ttk.Button(bar, text="⟳ Refresh",
                   command=self._refresh_ports).pack(side="left", padx=2)
        self.btn_conn = ttk.Button(bar, text="Connect",
                                   command=self._toggle_connect)
        self.btn_conn.pack(side="left", padx=2)

        self.lbl_conn = tk.Label(bar, text="● Disconnected",
                                  bg=self.BG, fg=self.RED,
                                  font=("Segoe UI", 10, "bold"))
        self.lbl_conn.pack(side="left", padx=8)

        self.lbl_warn = tk.Label(bar, text="", bg=self.YELLOW, fg=self.BG,
                                  font=("Consolas", 9, "bold"), padx=6)

        self.lbl_mode = tk.Label(bar, text="MODE: ---",
                                  bg=self.BG, fg=self.OVERLAY,
                                  font=("Consolas", 12, "bold"))
        self.lbl_mode.pack(side="right", padx=10)

        # ── Body (left | right) ─────────────────────────────────────────────────
        body = ttk.Frame(self.root)
        body.pack(fill="both", expand=True, padx=12, pady=4)

        left  = ttk.Frame(body)
        left.pack(side="left", fill="both", expand=True)
        right = ttk.Frame(body)
        right.pack(side="right", fill="y", padx=(10, 0))

        # ── Left top: relay + E-STOP ────────────────────────────────────────────
        ctrl_bar = ttk.Frame(left)
        ctrl_bar.pack(fill="x", pady=(0, 6))
        ttk.Label(ctrl_bar, text="Motor Relay", style="Title.TLabel").pack(side="left")

        self.btn_estop = tk.Button(
            ctrl_bar, text="E-STOP", bg="#c0392b", fg="white",
            font=("Segoe UI", 12, "bold"), width=8, relief="raised",
            activebackground="#e74c3c",
            command=lambda: self.serial.send("s"))
        self.btn_estop.pack(side="right", padx=(6, 0))

        self.btn_reset = tk.Button(
            ctrl_bar, text="RESET", bg=self.OVERLAY, fg=self.YELLOW,
            font=("Segoe UI", 10, "bold"), width=7, relief="flat",
            command=lambda: self.serial.send("reset"))
        self.btn_reset.pack(side="right", padx=2)

        self.btn_cfaults = tk.Button(
            ctrl_bar, text="CLR FAULTS", bg=self.OVERLAY, fg=self.TEAL,
            font=("Segoe UI", 10, "bold"), width=10, relief="flat",
            command=lambda: self.serial.send("clearfaults"))
        self.btn_cfaults.pack(side="right", padx=2)

        self.btn_relay_off = tk.Button(
            ctrl_bar, text="OFF", bg="#5a1e1e", fg="white",
            font=("Segoe UI", 11, "bold"), width=5, relief="flat",
            command=lambda: self.serial.send("off"))
        self.btn_relay_off.pack(side="right", padx=(2, 2))

        self.btn_relay_on = tk.Button(
            ctrl_bar, text="ON", bg="#2d5016", fg="white",
            font=("Segoe UI", 11, "bold"), width=5, relief="flat",
            command=lambda: self.serial.send("on"))
        self.btn_relay_on.pack(side="right", padx=2)
        ttk.Label(ctrl_bar, text="Relay:").pack(side="right", padx=(0, 2))

        # ── Live telemetry cards ────────────────────────────────────────────────
        cards = ttk.Frame(left)
        cards.pack(fill="x", pady=(0, 8))

        self.lbl_pos     = self._card(cards, "Position (deg)",     "---")
        self.lbl_thr_act = self._card(cards, "Throttle Actual",    "---%")
        self.lbl_thr_tgt = self._card(cards, "Throttle Target",    "0%")
        self.lbl_duty    = self._card(cards, "Motor Cmd",          "0")
        self.lbl_ris     = self._card(cards, "Current RIS (ADC)",  "0")
        self.lbl_lis     = self._card(cards, "Current LIS (ADC)",  "0")

        # ── Throttle slider ─────────────────────────────────────────────────────
        sl_frame = tk.Frame(left, bg=self.SURFACE, padx=12, pady=10)
        sl_frame.pack(fill="x", pady=(0, 8))

        tk.Label(sl_frame, text="Throttle Command (%)",
                 bg=self.SURFACE, fg=self.ACCENT,
                 font=("Segoe UI", 11, "bold")).pack(anchor="w")

        slider_row = tk.Frame(sl_frame, bg=self.SURFACE)
        slider_row.pack(fill="x", pady=(4, 0))

        self.thr_var = tk.IntVar(value=0)
        self.thr_slider = tk.Scale(
            slider_row, from_=0, to=100, orient="horizontal",
            variable=self.thr_var, bg=self.SURFACE, fg=self.FG,
            highlightbackground=self.SURFACE, troughcolor=self.OVERLAY,
            activebackground=self.ACCENT, length=420,
            command=self._on_slider_move)
        self.thr_slider.pack(side="left")

        self.lbl_slider_val = tk.Label(
            slider_row, text="0%",
            bg=self.SURFACE, fg=self.GREEN,
            font=("Consolas", 14, "bold"), width=5)
        self.lbl_slider_val.pack(side="left", padx=8)

        btn_row = tk.Frame(sl_frame, bg=self.SURFACE)
        btn_row.pack(fill="x", pady=(6, 0))
        for pct in (0, 10, 25, 50, 75, 100):
            p = pct
            tk.Button(btn_row, text=f"{p}%", bg=self.OVERLAY, fg=self.FG,
                      font=("Segoe UI", 9), width=4, relief="flat",
                      command=lambda v=p: self._set_throttle(v)
                      ).pack(side="left", padx=2)

        tk.Button(btn_row, text="Send →", bg=self.ACCENT, fg=self.BG,
                  font=("Segoe UI", 10, "bold"), relief="flat",
                  command=self._send_throttle).pack(side="right")

        # ── Direct PWM duty ─────────────────────────────────────────────────────
        duty_frame = tk.Frame(left, bg=self.SURFACE, padx=12, pady=8)
        duty_frame.pack(fill="x", pady=(0, 8))

        tk.Label(duty_frame, text="Direct PWM Duty (0 – 4095)  [Manual mode]",
                 bg=self.SURFACE, fg=self.ACCENT,
                 font=("Segoe UI", 11, "bold")).pack(anchor="w")

        duty_row = tk.Frame(duty_frame, bg=self.SURFACE)
        duty_row.pack(fill="x", pady=(4, 0))

        self.duty_entry = tk.Entry(duty_row, bg=self.OVERLAY, fg=self.FG,
                                    insertbackground=self.FG,
                                    font=("Consolas", 12), width=8)
        self.duty_entry.insert(0, "0")
        self.duty_entry.pack(side="left")

        tk.Label(duty_row, text="Dir:", bg=self.SURFACE, fg=self.FG,
                 font=("Segoe UI", 10)).pack(side="left", padx=(10, 2))
        self.dir_var = tk.StringVar(value="F")
        tk.Radiobutton(duty_row, text="Forward (f)", variable=self.dir_var,
                       value="F", bg=self.SURFACE, fg=self.GREEN,
                       selectcolor=self.OVERLAY,
                       command=lambda: self.serial.send("f")).pack(side="left")
        tk.Radiobutton(duty_row, text="Reverse (r)", variable=self.dir_var,
                       value="R", bg=self.SURFACE, fg=self.ORANGE,
                       selectcolor=self.OVERLAY,
                       command=lambda: self.serial.send("r")).pack(side="left")

        tk.Button(duty_row, text="Send duty →", bg=self.OVERLAY, fg=self.FG,
                  font=("Segoe UI", 10), relief="flat",
                  command=self._send_duty).pack(side="right")

        # ── Raw log ─────────────────────────────────────────────────────────────
        log_frame = ttk.Frame(left)
        log_frame.pack(fill="both", expand=True)
        tk.Label(log_frame, text="Serial Log",
                 bg=self.BG, fg=self.ACCENT,
                 font=("Segoe UI", 10, "bold")).pack(anchor="w")

        log_inner = tk.Frame(log_frame, bg=self.SURFACE)
        log_inner.pack(fill="both", expand=True)

        self.log_text = tk.Text(
            log_inner, bg=self.SURFACE, fg=self.FG,
            font=("Consolas", 9), state="disabled",
            wrap="none", height=7,
            insertbackground=self.FG,
            selectbackground=self.OVERLAY)
        log_sb = tk.Scrollbar(log_inner, command=self.log_text.yview,
                               bg=self.OVERLAY)
        self.log_text.configure(yscrollcommand=log_sb.set)
        log_sb.pack(side="right", fill="y")
        self.log_text.pack(fill="both", expand=True)

        # ── Right panel: PID + Response Monitor + Faults ───────────────────────
        self._build_pid_panel(right)
        self._build_pid_monitor(right)
        self._build_fault_panel(right)
        self._build_cmd_panel(right)

        # ── Position bar ────────────────────────────────────────────────────────
        self._build_pos_bar(left)

        # ── Bottom quick-send bar ───────────────────────────────────────────────
        self._build_quick_bar()

    # ── Position visual bar ─────────────────────────────────────────────────────
    def _build_pos_bar(self, parent):
        frame = tk.Frame(parent, bg=self.SURFACE, padx=12, pady=8)
        frame.pack(fill="x", pady=(0, 6))

        tk.Label(frame, text="Throttle Position",
                 bg=self.SURFACE, fg=self.ACCENT,
                 font=("Segoe UI", 10, "bold")).pack(anchor="w")

        self.pos_canvas = tk.Canvas(frame, height=28, bg=self.OVERLAY,
                                     highlightthickness=0)
        self.pos_canvas.pack(fill="x", pady=(4, 0))

        # Target marker row
        self.tgt_canvas = tk.Canvas(frame, height=16, bg=self.SURFACE,
                                     highlightthickness=0)
        self.tgt_canvas.pack(fill="x")
        tk.Label(frame, text="◂ usable 5–95 %  |  actual ●  target ▼",
                 bg=self.SURFACE, fg=self.OVERLAY,
                 font=("Segoe UI", 8)).pack(anchor="w")

    def _draw_pos_bar(self, thr_act, thr_tgt):
        w = self.pos_canvas.winfo_width()
        if w < 10:
            return
        self.pos_canvas.delete("all")
        self.tgt_canvas.delete("all")

        # Background track
        self.pos_canvas.create_rectangle(0, 0, w, 28, fill=self.OVERLAY, outline="")

        if thr_act is not None:
            bar_w = int(w * thr_act / 100)
            colour = self.GREEN if thr_act < 80 else self.ORANGE if thr_act < 95 else self.RED
            self.pos_canvas.create_rectangle(0, 4, bar_w, 24, fill=colour, outline="")
            self.pos_canvas.create_text(
                bar_w + 6 if bar_w < w - 40 else bar_w - 6,
                14, text=f"{thr_act}%", fill=self.BG if bar_w > 30 else self.FG,
                font=("Consolas", 10, "bold"), anchor="w" if bar_w < w - 40 else "e")

        # Target marker
        tx = int(w * thr_tgt / 100)
        self.tgt_canvas.create_text(tx, 8, text="▼", fill=self.YELLOW,
                                     font=("Segoe UI", 10, "bold"))

    # ── PID panel ───────────────────────────────────────────────────────────────
    def _build_pid_panel(self, parent):
        frame = tk.Frame(parent, bg=self.SURFACE, padx=12, pady=10)
        frame.pack(fill="x", pady=(0, 8))

        tk.Label(frame, text="PID Gains",
                 bg=self.SURFACE, fg=self.ACCENT,
                 font=("Segoe UI", 11, "bold")).grid(row=0, column=0,
                                                      columnspan=3, sticky="w",
                                                      pady=(0, 6))
        self.pid_entries = {}
        for row, (label, cmd_prefix, default) in enumerate([
            ("Kp (p##)", "p", "12.00"),
            ("Ki (i##)", "i", "0.30"),
            ("Kd (k##)", "k", "1.50"),
        ], start=1):
            tk.Label(frame, text=label, bg=self.SURFACE, fg=self.FG,
                     font=("Segoe UI", 10)).grid(row=row, column=0, sticky="w")
            e = tk.Entry(frame, bg=self.OVERLAY, fg=self.FG,
                         insertbackground=self.FG,
                         font=("Consolas", 11), width=8)
            e.insert(0, default)
            e.grid(row=row, column=1, padx=6, pady=2)
            self.pid_entries[cmd_prefix] = e
            tk.Button(frame, text="Set", bg=self.OVERLAY, fg=self.FG,
                      font=("Segoe UI", 9), relief="flat",
                      command=lambda p=cmd_prefix: self._send_pid(p)
                      ).grid(row=row, column=2, padx=2)

        tk.Button(frame, text="Send All PID", bg=self.ACCENT, fg=self.BG,
                  font=("Segoe UI", 10, "bold"), relief="flat",
                  command=self._send_all_pid
                  ).grid(row=4, column=0, columnspan=3, pady=(8, 0), sticky="ew")

    # ── PID response monitor panel ───────────────────────────────────────────────
    def _build_pid_monitor(self, parent):
        frame = tk.Frame(parent, bg=self.SURFACE, padx=10, pady=8)
        frame.pack(fill="x", pady=(0, 8))

        tk.Label(frame, text="PID Response",
                 bg=self.SURFACE, fg=self.TEAL,
                 font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(0, 4))

        self.err_canvas = tk.Canvas(frame, height=120, bg="#11111b",
                                    highlightthickness=1,
                                    highlightbackground=self.OVERLAY)
        self.err_canvas.pack(fill="x", pady=(0, 6))

        stats = tk.Frame(frame, bg=self.SURFACE)
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
            frame, text="Send tXX to begin tracking",
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

        BG_C   = "#11111b"
        GRID   = "#313244"
        ZERO   = "#585b70"
        BAND   = "#1a2a1a"
        Y_RNG  = 25.0
        X_SPAN = 12.0

        c.create_rectangle(0, 0, w, h, fill=BG_C, outline="")
        mid_y = h / 2.0

        def to_y(err):
            return mid_y - (max(-Y_RNG, min(Y_RNG, err)) / Y_RNG) * mid_y

        # Settle band (±5%)
        c.create_rectangle(0, to_y(5), w, to_y(-5), fill=BAND, outline="")

        # Grid lines
        for pct in (20, 10, 5, -5, -10, -20):
            c.create_line(0, to_y(pct), w, to_y(pct), fill=GRID, dash=(2, 4))

        # Zero line
        c.create_line(0, mid_y, w, mid_y, fill=ZERO, width=1)

        # Time ticks
        now_s = time.time()
        for age in (2, 4, 6, 8, 10):
            x = w * (1.0 - age / X_SPAN)
            if x > 0:
                c.create_line(x, mid_y - 3, x, mid_y + 3, fill=ZERO)
                c.create_text(x, h - 2, text=f"-{age}s", fill=ZERO,
                              font=("Consolas", 7), anchor="s")

        # Axis labels
        c.create_text(3, 2,      text=f"+{int(Y_RNG)}%", fill=ZERO,
                      font=("Consolas", 7), anchor="nw")
        c.create_text(3, h - 2,  text=f"-{int(Y_RNG)}%", fill=ZERO,
                      font=("Consolas", 7), anchor="sw")
        c.create_text(w - 3, mid_y, text="0", fill=ZERO,
                      font=("Consolas", 7), anchor="e")

        # Error trace — colored by magnitude
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
                col = ("#a6e3a1" if avg < 5.0
                       else "#fab387" if avg < 15.0
                       else "#f38ba8")
                c.create_line(x1, y1, x2, y2, fill=col, width=2)

        # Current dot
        if pts:
            x, y, e = pts[-1]
            col = ("#a6e3a1" if abs(e) < 5.0
                   else "#fab387" if abs(e) < 15.0
                   else "#f38ba8")
            c.create_oval(x - 3, y - 3, x + 3, y + 3, fill=col, outline="")

    # ── Fault panel ─────────────────────────────────────────────────────────────
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

        self.lbl_safe_reason = tk.Label(outer, text="",
                                         bg=self.SURFACE, fg=self.RED,
                                         font=("Consolas", 9, "bold"),
                                         wraplength=240)
        self.lbl_safe_reason.pack(anchor="w", pady=(4, 0))

    # ── Quick command panel ──────────────────────────────────────────────────────
    def _build_cmd_panel(self, parent):
        frame = tk.Frame(parent, bg=self.SURFACE, padx=12, pady=10)
        frame.pack(fill="x", pady=(0, 8))

        tk.Label(frame, text="Quick Commands",
                 bg=self.SURFACE, fg=self.ACCENT,
                 font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(0, 6))

        cmds = [
            ("diag",       "Diagnostics",  self.TEAL),
            ("config",     "Config dump",  self.TEAL),
            ("clearfaults","Clear Faults", self.ORANGE),
            ("reset",      "Reset ECU",    self.YELLOW),
            ("s",          "Stop motor",   self.RED),
        ]
        for cmd, label, colour in cmds:
            tk.Button(frame, text=label, bg=self.OVERLAY, fg=colour,
                      font=("Segoe UI", 10), relief="flat", width=16,
                      command=lambda c=cmd: self.serial.send(c)
                      ).pack(fill="x", pady=2)

        # Custom command entry
        tk.Label(frame, text="Custom command:",
                 bg=self.SURFACE, fg=self.FG,
                 font=("Segoe UI", 9)).pack(anchor="w", pady=(8, 2))
        cmd_row = tk.Frame(frame, bg=self.SURFACE)
        cmd_row.pack(fill="x")
        self.custom_entry = tk.Entry(cmd_row, bg=self.OVERLAY, fg=self.FG,
                                      insertbackground=self.FG,
                                      font=("Consolas", 11), width=14)
        self.custom_entry.pack(side="left")
        self.custom_entry.bind("<Return>", self._send_custom)
        tk.Button(cmd_row, text="Send", bg=self.ACCENT, fg=self.BG,
                  font=("Segoe UI", 10, "bold"), relief="flat",
                  command=self._send_custom).pack(side="left", padx=4)

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

    # ── Helpers ─────────────────────────────────────────────────────────────────
    def _card(self, parent, title, initial):
        frame = tk.Frame(parent, bg=self.SURFACE, padx=10, pady=6)
        frame.pack(side="left", padx=4, pady=2)
        tk.Label(frame, text=title, bg=self.SURFACE, fg=self.OVERLAY,
                 font=("Segoe UI", 8)).pack()
        lbl = tk.Label(frame, text=initial, bg=self.SURFACE, fg=self.GREEN,
                       font=("Consolas", 13, "bold"), width=7)
        lbl.pack()
        return lbl

    def _refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        # Put XDS110 Application ports at the top
        app_ports = [p for p in ports if "Application" in (p.description or "")
                     or "XDS110" in (p.description or "")]
        other_ports = [p for p in ports if p not in app_ports]
        ordered = app_ports + other_ports
        names = [f"{p.device}  —  {p.description}" for p in ordered]
        self.port_cb["values"] = names
        if names:
            self.port_cb.current(0)

    def _toggle_connect(self):
        if self.serial.connected:
            self.serial.disconnect()
            self.btn_conn.configure(text="Connect")
            self.lbl_conn.configure(text="● Disconnected", fg=self.RED)
        else:
            sel = self.port_var.get()
            if not sel:
                return
            port = sel.split("  —  ")[0].strip()
            result = self.serial.connect(port)
            if result is True:
                self.btn_conn.configure(text="Disconnect")
                self.lbl_conn.configure(text="● Connected", fg=self.GREEN)
            else:
                self.lbl_conn.configure(
                    text=f"● Error: {result}"[:40], fg=self.RED)

    def _set_throttle(self, val):
        self.thr_var.set(val)
        self.lbl_slider_val.configure(text=f"{val}%")

    def _on_slider_move(self, val):
        self.lbl_slider_val.configure(text=f"{int(float(val))}%")

    def _send_throttle(self):
        self.serial.send(f"t{self.thr_var.get()}")

    def _send_duty(self):
        try:
            v = int(self.duty_entry.get())
            v = max(0, min(4095, v))
        except ValueError:
            return
        self.serial.send(f"d{v}")

    def _send_pid(self, prefix):
        try:
            v = float(self.pid_entries[prefix].get())
        except ValueError:
            return
        self.serial.send(f"{prefix}{v:.3f}")

    def _send_all_pid(self):
        for prefix in ("p", "i", "k"):
            self._send_pid(prefix)

    def _send_custom(self, _event=None):
        cmd = self.custom_entry.get().strip()
        if cmd:
            self.serial.send(cmd)

    # ── Poll loop (runs every 150 ms on GUI thread) ──────────────────────────────
    def _poll(self):
        if not self.serial.connected:
            self.root.after(200, self._poll)
            return

        with self.serial.lock:
            mode      = self.serial.mode
            pos       = self.serial.pos
            thr_act   = self.serial.thr_act
            thr_tgt   = self.serial.thr_tgt
            cmd       = self.serial.cmd
            ris       = self.serial.ris
            lis       = self.serial.lis
            err_flags = self.serial.err_flags
            safe_rsn  = self.serial.safe_reason
            warn_msg  = self.serial.warn_msg
            last_line = self.serial.last_line

        # Mode badge
        label, bg_col, fg_col = MODE_STYLE.get(
            mode, (mode, self.OVERLAY, self.FG))
        self.lbl_mode.configure(text=f"MODE: {label}", bg=bg_col, fg=fg_col)

        # Telemetry cards
        self.lbl_pos.configure(
            text=f"{pos:.2f}°" if pos is not None else "---",
            fg=self.GREEN if pos is not None else self.OVERLAY)
        self.lbl_thr_act.configure(
            text=f"{thr_act}%" if thr_act is not None else "---%",
            fg=self.GREEN if thr_act is not None else self.OVERLAY)
        self.lbl_thr_tgt.configure(text=f"{thr_tgt}%")
        self.lbl_duty.configure(text=str(cmd))
        self.lbl_ris.configure(
            text=str(ris),
            fg=self.RED if ris > 3500 else self.ORANGE if ris > 2500 else self.GREEN)
        self.lbl_lis.configure(
            text=str(lis),
            fg=self.RED if lis > 3500 else self.ORANGE if lis > 2500 else self.GREEN)

        # Position bar
        self._draw_pos_bar(thr_act, thr_tgt)

        # PID response monitor — use raw pos (degrees) for sub-percent resolution
        # Integer thr_act has 1% steps (~0.2°) which hides fine behaviour
        now_s = time.time()
        _usable_deg = (USABLE_MAX - USABLE_MIN) / 100.0   # usable range in degrees
        if pos is not None and mode == "PID":
            target_deg = (USABLE_MIN + (USABLE_MAX - USABLE_MIN) * thr_tgt / 100.0) / 100.0
            error = (target_deg - pos) / _usable_deg * 100.0  # signed %, +ve = undershoot

            # Detect new step command
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

                # Rise time — first entry into ±5% band
                if self._rise_time is None and abs(error) < 5.0:
                    self._rise_time = elapsed

                # Crossing detection
                if (not self._crossed and self._approach_dir != 0
                        and self._approach_dir * error < 0):
                    self._crossed = True

                # Overshoot — peak excursion past target after crossing
                if self._crossed:
                    over = -self._approach_dir * error
                    if over > self._peak_over:
                        self._peak_over = over

                # Settle detection — stay within ±2% for 500 ms
                if abs(error) < 2.0:
                    if self._settle_band_start is None:
                        self._settle_band_start = now_s
                    elif (self._settle_time is None
                          and (now_s - self._settle_band_start) >= 0.5):
                        self._settle_time = elapsed
                else:
                    self._settle_band_start = None

            # Stat labels
            ecol = (self.GREEN if abs(error) < 5.0
                    else self.ORANGE if abs(error) < 15.0
                    else self.RED)
            self.lbl_err_cur.configure(text=f"{error:+.1f}%", fg=ecol)

            if self._peak_over > 0.5:
                ocol = self.RED if self._peak_over > 10.0 else self.ORANGE
                self.lbl_err_over.configure(
                    text=f"{self._peak_over:.1f}%", fg=ocol)
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

            # Tuning hint — only meaningful after a real step (>10% change)
            if self._step_error < 10.0:
                hint = "Send a bigger step to evaluate (e.g. t0 then t60)"
                hcol = self.OVERLAY
            elif (self._step_time is not None
                  and (now_s - self._step_time) > 5.0
                  and self._rise_time is None):
                hint = "Not converging — check encoder & gains"
                hcol = self.RED
            elif self._peak_over > 20.0:
                hint = "High overshoot — reduce Kp or increase Kd"
                hcol = self.RED
            elif self._peak_over > 10.0:
                hint = "Moderate overshoot — try increasing Kd"
                hcol = self.ORANGE
            elif (self._rise_time is not None and self._rise_time > 2.0
                  and self._settle_time is None):
                hint = "Slow rise — try increasing Kp"
                hcol = self.YELLOW
            elif self._settle_time is not None and self._peak_over < 5.0:
                hint = "Well tuned — fast settle, low overshoot"
                hcol = self.GREEN
            else:
                hint = "Tracking..."
                hcol = self.OVERLAY
            self.lbl_pid_hint.configure(text=hint, fg=hcol)

        self._draw_err_chart()

        # Fault indicators
        for bit, dot in self.fault_labels.items():
            dot.configure(fg=self.RED if (err_flags & bit) else self.OVERLAY)

        # Safe reason
        self.lbl_safe_reason.configure(
            text=f"Reason: {safe_rsn}" if safe_rsn else "")

        # Warn banner
        if warn_msg:
            self.lbl_warn.configure(text=f"⚠ {warn_msg}")
            self.lbl_warn.pack(side="left", padx=4)
        else:
            self.lbl_warn.pack_forget()

        # Log
        if last_line and (not self.log_lines or self.log_lines[-1] != last_line):
            self.log_lines.append(last_line)
            self.log_text.configure(state="normal")
            self.log_text.insert("end", last_line + "\n")
            self.log_text.see("end")
            # Keep only last 300 lines visually
            line_count = int(self.log_text.index("end-1c").split(".")[0])
            if line_count > 300:
                self.log_text.delete("1.0", "2.0")
            self.log_text.configure(state="disabled")

        self.root.after(150, self._poll)

    def _on_close(self):
        self.serial.disconnect()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    ThrottleGUI().run()
