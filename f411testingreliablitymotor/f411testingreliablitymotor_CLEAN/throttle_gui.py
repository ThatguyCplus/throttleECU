"""
Throttle ECU Control GUI — PID Position Control
Connects to F411CE over USB serial, sends commands, displays live telemetry.
Requirements:  pip install pyserial
Uses only tkinter (built-in) + pyserial.
"""

import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import re
import time
import math
import collections

# ── Throttle calibration (must match firmware) ──
ANGLE_MIN = 70.32
ANGLE_MAX = 179.40

# ── Serial telemetry regex ──
# mode=MAN dir=F duty=1234 RIS=456 LIS=789 pos=123.45 thr=50% tgt=50% err=0x00
TELEM_RE = re.compile(
    r"mode=(\w+)\s+dir=([FR])\s+duty=(\d+)\s+RIS=(\d+)\s+LIS=(\d+)\s+"
    r"pos=([\d.]+|---)\s+thr=([\d]+|---)%\s+tgt=(\d+)%\s+err=0x([0-9A-Fa-f]+)"
)

# Error flag bit definitions (must match initialsafe.h)
ERR_NAMES = [
    (0x01, "ENC_STALE",  "Encoder stale >500ms"),
    (0x02, "ENC_INVAL",  "Encoder signal invalid"),
    (0x04, "OVERCURR_L", "Left motor overcurrent"),
    (0x08, "OVERCURR_R", "Right motor overcurrent"),
    (0x10, "POWER_LOW",  "Supply voltage low"),
    (0x20, "WDG_RST",    "Watchdog reset recovery"),
    (0x40, "PID_SAT",    "PID output saturated"),
    (0x80, "TEMP_HIGH",  "Temperature high"),
]

MODE_LABELS = {
    "MAN":  ("MANUAL",           "#cdd6f4"),
    "PID":  ("PID ACTIVE",       "#fab387"),
    "SAFE": ("!! SAFE STATE !!", "#f38ba8"),
}


# Regex to capture [SAFE] reason messages from firmware
SAFE_RE = re.compile(r"\[SAFE\]\s*(.+)")


class SerialThread:
    def __init__(self):
        self.ser = None
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        self.mode = "MAN"
        self.dir = "F"
        self.duty = 0
        self.ris = 0
        self.lis = 0
        self.pos = None
        self.thr_actual = None
        self.thr_target = 0
        self.err_flags = 0
        self.raw_line = ""
        self.safe_reason = ""
        self.connected = False

    def connect(self, port, baud=115200):
        self.disconnect()
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.connected = True
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            self.connected = False
            print(f"Serial error: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
            self.thread = None
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.connected = False

    def send(self, cmd):
        with self.lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write((cmd + "\n").encode())
                except Exception:
                    pass

    def _read_loop(self):
        buf = ""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    chunk = self.ser.read(self.ser.in_waiting).decode("utf-8", errors="replace")
                    buf += chunk
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        self.raw_line = line
                        m = TELEM_RE.search(line)
                        if m:
                            with self.lock:
                                self.mode = m.group(1)
                                self.dir = m.group(2)
                                self.duty = int(m.group(3))
                                self.ris = int(m.group(4))
                                self.lis = int(m.group(5))
                                p = m.group(6)
                                self.pos = float(p) if p != "---" else None
                                t = m.group(7)
                                self.thr_actual = int(t) if t != "---" else None
                                self.thr_target = int(m.group(8))
                                self.err_flags = int(m.group(9), 16)
                                # Clear reason when no longer in safe state
                                if self.mode != "SAFE":
                                    self.safe_reason = ""
                        # Capture [SAFE] reason messages
                        sm = SAFE_RE.search(line)
                        if sm:
                            with self.lock:
                                self.safe_reason = sm.group(1).strip()
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.05)


class ThrottleGUI:
    BG = "#1e1e2e"
    FG = "#cdd6f4"
    ACCENT = "#89b4fa"
    GREEN = "#a6e3a1"
    RED = "#f38ba8"
    YELLOW = "#f9e2af"
    ORANGE = "#fab387"
    SURFACE = "#313244"
    OVERLAY = "#45475a"
    DARK_RED = "#45171d"

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Throttle ECU — PID Position Control")
        self.root.configure(bg=self.BG)
        self.root.geometry("1020x800")
        self.root.minsize(960, 750)

        self.serial = SerialThread()
        self.pos_history = collections.deque(maxlen=200)
        self.thr_history = collections.deque(maxlen=200)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background=self.BG)
        style.configure("TLabel", background=self.BG, foreground=self.FG, font=("Segoe UI", 10))
        style.configure("Title.TLabel", font=("Segoe UI", 12, "bold"), foreground=self.ACCENT)
        style.configure("Val.TLabel", font=("Consolas", 13, "bold"), foreground=self.GREEN)
        style.configure("Mode.TLabel", font=("Consolas", 11, "bold"))

        self._build_ui()
        self._poll()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        # ── Top bar: connection + mode ──
        conn_frame = ttk.Frame(self.root)
        conn_frame.pack(fill="x", padx=10, pady=(10, 5))

        ttk.Label(conn_frame, text="Port:", style="TLabel").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.pack(side="left", padx=(5, 5))
        self._refresh_ports()

        self.btn_refresh = ttk.Button(conn_frame, text="Refresh", command=self._refresh_ports)
        self.btn_refresh.pack(side="left", padx=2)

        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self._toggle_connect)
        self.btn_connect.pack(side="left", padx=2)

        self.lbl_status = ttk.Label(conn_frame, text="Disconnected", foreground=self.RED)
        self.lbl_status.pack(side="left", padx=10)

        self.lbl_mode = ttk.Label(conn_frame, text="MODE: ---", style="Mode.TLabel", foreground=self.OVERLAY)
        self.lbl_mode.pack(side="right", padx=10)

        # ── Main content ──
        main = ttk.Frame(self.root)
        main.pack(fill="both", expand=True, padx=10, pady=5)

        left = ttk.Frame(main)
        left.pack(side="left", fill="both", expand=True)

        right = ttk.Frame(main)
        right.pack(side="right", fill="y", padx=(10, 0))

        # ── Relay + E-STOP ──
        top_controls = ttk.Frame(left)
        top_controls.pack(fill="x", pady=(0, 6))

        ttk.Label(top_controls, text="MOSFET Relay", style="Title.TLabel").pack(side="left")

        # E-STOP button (big, red)
        self.btn_estop = tk.Button(top_controls, text="E-STOP", bg="#c0392b", fg="white",
                                    font=("Segoe UI", 12, "bold"), width=8, relief="raised",
                                    activebackground="#e74c3c",
                                    command=lambda: self.serial.send("s"))
        self.btn_estop.pack(side="right", padx=(10, 0))

        # Reset from safe state
        self.btn_reset = tk.Button(top_controls, text="RESET", bg=self.OVERLAY, fg=self.YELLOW,
                                    font=("Segoe UI", 10, "bold"), width=6, relief="flat",
                                    command=lambda: self.serial.send("reset"))
        self.btn_reset.pack(side="right", padx=2)

        self.btn_relay_on = tk.Button(top_controls, text="ON", bg="#2d5016", fg="white",
                                       font=("Segoe UI", 11, "bold"), width=5, relief="flat",
                                       command=lambda: self.serial.send("on"))
        self.btn_relay_on.pack(side="right", padx=2)
        self.btn_relay_off = tk.Button(top_controls, text="OFF", bg="#6e1e1e", fg="white",
                                        font=("Segoe UI", 11, "bold"), width=5, relief="flat",
                                        command=lambda: self.serial.send("off"))
        self.btn_relay_off.pack(side="right", padx=2)

        # ── Direction ──
        dir_frame = ttk.Frame(left)
        dir_frame.pack(fill="x", pady=(0, 6))
        ttk.Label(dir_frame, text="Direction", style="Title.TLabel").pack(side="left")
        self.btn_fwd = tk.Button(dir_frame, text="FWD", bg=self.SURFACE, fg=self.GREEN,
                                  font=("Segoe UI", 11, "bold"), width=6, relief="flat",
                                  command=lambda: self.serial.send("f"))
        self.btn_fwd.pack(side="right", padx=2)
        self.btn_rev = tk.Button(dir_frame, text="REV", bg=self.SURFACE, fg=self.YELLOW,
                                  font=("Segoe UI", 11, "bold"), width=6, relief="flat",
                                  command=lambda: self.serial.send("r"))
        self.btn_rev.pack(side="right", padx=2)
        self.btn_stop = tk.Button(dir_frame, text="STOP", bg=self.SURFACE, fg=self.RED,
                                   font=("Segoe UI", 11, "bold"), width=6, relief="flat",
                                   command=lambda: self.serial.send("s"))
        self.btn_stop.pack(side="right", padx=2)

        # ══════════════════════════════════════════
        # ── THROTTLE POSITION CONTROL ──
        # ══════════════════════════════════════════
        thr_frame = tk.Frame(left, bg=self.SURFACE, highlightbackground=self.ACCENT,
                              highlightthickness=1)
        thr_frame.pack(fill="x", pady=(0, 6), ipady=6, ipadx=6)

        ttk.Label(thr_frame, text="Throttle Position (0-100%)",
                  style="Title.TLabel", background=self.SURFACE).pack(anchor="w", padx=6, pady=(4, 0))

        thr_slider_row = tk.Frame(thr_frame, bg=self.SURFACE)
        thr_slider_row.pack(fill="x", padx=6, pady=4)

        self.thr_var = tk.IntVar(value=0)
        self.thr_slider = tk.Scale(thr_slider_row, from_=0, to=100, orient="horizontal",
                                    variable=self.thr_var, bg=self.SURFACE, fg=self.ORANGE,
                                    troughcolor=self.OVERLAY, highlightthickness=0,
                                    font=("Consolas", 10), length=300,
                                    command=self._on_thr_slider)
        self.thr_slider.pack(side="left", fill="x", expand=True)

        self.thr_entry = tk.Entry(thr_slider_row, width=5, font=("Consolas", 14, "bold"),
                                   bg=self.OVERLAY, fg=self.ORANGE, insertbackground=self.FG,
                                   relief="flat", justify="center")
        self.thr_entry.insert(0, "0")
        self.thr_entry.pack(side="left", padx=(8, 0))
        self.thr_entry.bind("<Return>", self._on_thr_entry)

        ttk.Label(thr_slider_row, text="%", background=self.SURFACE).pack(side="left")

        thr_preset = tk.Frame(thr_frame, bg=self.SURFACE)
        thr_preset.pack(fill="x", padx=6, pady=2)
        for pct in [0, 10, 25, 50, 75, 100]:
            lbl = "WOT" if pct == 100 else f"{pct}%"
            color = self.RED if pct == 0 else (self.ORANGE if pct == 100 else self.FG)
            tk.Button(thr_preset, text=lbl, bg=self.OVERLAY, fg=color,
                       font=("Segoe UI", 9, "bold"), width=5, relief="flat",
                       command=lambda v=pct: self._set_throttle(v)).pack(side="left", padx=2)

        # ── Manual duty override ──
        manual_frame = ttk.Frame(left)
        manual_frame.pack(fill="x", pady=(0, 6))
        ttk.Label(manual_frame, text="Manual Duty Override (d0-d4095)", style="TLabel").pack(anchor="w")

        man_row = ttk.Frame(manual_frame)
        man_row.pack(fill="x", pady=2)

        self.duty_var = tk.IntVar(value=0)
        self.slider = tk.Scale(man_row, from_=0, to=4095, orient="horizontal",
                                variable=self.duty_var, bg=self.SURFACE, fg=self.FG,
                                troughcolor=self.OVERLAY, highlightthickness=0,
                                font=("Consolas", 9), length=260,
                                command=self._on_slider)
        self.slider.pack(side="left", fill="x", expand=True)

        self.duty_entry = tk.Entry(man_row, width=6, font=("Consolas", 11),
                                    bg=self.SURFACE, fg=self.GREEN, insertbackground=self.FG,
                                    relief="flat", justify="center")
        self.duty_entry.insert(0, "0")
        self.duty_entry.pack(side="left", padx=(6, 0))
        self.duty_entry.bind("<Return>", self._on_entry)

        # ══════════════════════════════════════
        # ── SAFETY STATUS PANEL ──
        # ══════════════════════════════════════
        self.err_frame = tk.Frame(left, bg=self.DARK_RED, highlightbackground=self.OVERLAY,
                                   highlightthickness=1)
        self.err_frame.pack(fill="x", pady=(0, 6), ipady=4, ipadx=6)

        err_title_row = tk.Frame(self.err_frame, bg=self.DARK_RED)
        err_title_row.pack(fill="x", padx=6, pady=(4, 0))
        tk.Label(err_title_row, text="Safety Status", bg=self.DARK_RED,
                  fg=self.RED, font=("Segoe UI", 12, "bold")).pack(side="left")
        self.lbl_err_hex = tk.Label(err_title_row, text="0x00", bg=self.DARK_RED,
                                     fg=self.OVERLAY, font=("Consolas", 11))
        self.lbl_err_hex.pack(side="right")

        # Big fault reason label — shows WHY safe state was triggered
        self.lbl_fault_reason = tk.Label(self.err_frame, text="ALL CLEAR",
                                          bg=self.DARK_RED, fg="#2d5016",
                                          font=("Consolas", 14, "bold"),
                                          anchor="w")
        self.lbl_fault_reason.pack(fill="x", padx=6, pady=(4, 2))

        # Fault indicators — each shows short name + description
        self.err_indicators = []
        err_grid = tk.Frame(self.err_frame, bg=self.DARK_RED)
        err_grid.pack(fill="x", padx=6, pady=2)
        for i, (bit, short, desc) in enumerate(ERR_NAMES):
            row_frame = tk.Frame(err_grid, bg=self.DARK_RED)
            row_frame.pack(fill="x", pady=1)
            dot = tk.Label(row_frame, text="\u25cf", bg=self.DARK_RED, fg=self.OVERLAY,
                            font=("Consolas", 10))
            dot.pack(side="left", padx=(0, 4))
            name_lbl = tk.Label(row_frame, text=f"{short}", bg=self.DARK_RED,
                                 fg=self.OVERLAY, font=("Consolas", 9, "bold"),
                                 width=12, anchor="w")
            name_lbl.pack(side="left")
            desc_lbl = tk.Label(row_frame, text=desc, bg=self.DARK_RED,
                                 fg=self.OVERLAY, font=("Segoe UI", 9),
                                 anchor="w")
            desc_lbl.pack(side="left", padx=(4, 0))
            self.err_indicators.append((bit, dot, name_lbl, desc_lbl))

        # Button row
        btn_row = tk.Frame(self.err_frame, bg=self.DARK_RED)
        btn_row.pack(fill="x", padx=6, pady=(4, 2))
        tk.Button(btn_row, text="DIAG", bg=self.OVERLAY, fg=self.FG,
                   font=("Consolas", 9, "bold"), width=6, relief="flat",
                   command=lambda: self.serial.send("diag")).pack(side="left", padx=2)
        tk.Button(btn_row, text="CLEAR FAULTS", bg=self.OVERLAY, fg=self.YELLOW,
                   font=("Consolas", 9, "bold"), width=14, relief="flat",
                   command=lambda: self.serial.send("clearfaults")).pack(side="left", padx=2)

        # ── Live telemetry ──
        telem_frame = ttk.Frame(left)
        telem_frame.pack(fill="x", pady=(4, 4))
        ttk.Label(telem_frame, text="Live Telemetry", style="Title.TLabel").pack(anchor="w")

        grid = ttk.Frame(telem_frame)
        grid.pack(fill="x", pady=2)

        labels = ["Direction", "Duty", "RIS", "LIS", "Position", "Throttle", "Target"]
        self.telem_vals = []
        for i, name in enumerate(labels):
            row = i // 2
            col = (i % 2) * 2
            ttk.Label(grid, text=name + ":", style="TLabel").grid(row=row, column=col, sticky="w", padx=(0, 5))
            v = ttk.Label(grid, text="---", style="Val.TLabel")
            v.grid(row=row, column=col + 1, sticky="w", padx=(0, 15))
            self.telem_vals.append(v)

        # ── Plot ──
        plot_frame = ttk.Frame(left)
        plot_frame.pack(fill="both", expand=True, pady=(4, 0))
        ttk.Label(plot_frame, text="Position & Throttle History", style="Title.TLabel").pack(anchor="w")

        self.plot_canvas = tk.Canvas(plot_frame, bg=self.SURFACE, highlightthickness=0, height=100)
        self.plot_canvas.pack(fill="both", expand=True, pady=4)

        # ════════════════════════════
        # ── Right: DUAL GAUGES ──
        # ════════════════════════════
        ttk.Label(right, text="Shaft Angle", style="Title.TLabel").pack()
        self.gauge = tk.Canvas(right, width=220, height=220, bg=self.BG, highlightthickness=0)
        self.gauge.pack(pady=(4, 8))

        ttk.Label(right, text="Throttle %", style="Title.TLabel").pack()
        self.thr_gauge = tk.Canvas(right, width=220, height=140, bg=self.BG, highlightthickness=0)
        self.thr_gauge.pack(pady=(4, 8))

        ttk.Label(right, text="Serial Log", style="Title.TLabel").pack(anchor="w")
        self.log_text = tk.Text(right, width=36, height=8, bg=self.SURFACE, fg=self.FG,
                                 font=("Consolas", 8), relief="flat", state="disabled",
                                 wrap="word")
        self.log_text.pack(fill="both", expand=True, pady=4)

    # ── Port ──
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.serial.connected:
            self.serial.disconnect()
            self.btn_connect.config(text="Connect")
            self.lbl_status.config(text="Disconnected", foreground=self.RED)
        else:
            port = self.port_var.get()
            if port and self.serial.connect(port):
                self.btn_connect.config(text="Disconnect")
                self.lbl_status.config(text=f"Connected: {port}", foreground=self.GREEN)
            else:
                self.lbl_status.config(text="Failed to connect", foreground=self.RED)

    # ── Throttle controls ──
    def _on_thr_slider(self, val):
        v = int(val)
        self.thr_entry.delete(0, "end")
        self.thr_entry.insert(0, str(v))
        self.serial.send(f"t{v}")

    def _on_thr_entry(self, event):
        try:
            v = int(self.thr_entry.get())
            v = max(0, min(100, v))
            self.thr_var.set(v)
            self.serial.send(f"t{v}")
        except ValueError:
            pass

    def _set_throttle(self, pct):
        self.thr_var.set(pct)
        self.thr_entry.delete(0, "end")
        self.thr_entry.insert(0, str(pct))
        self.serial.send(f"t{pct}")

    # ── Manual duty ──
    def _on_slider(self, val):
        v = int(val)
        self.duty_entry.delete(0, "end")
        self.duty_entry.insert(0, str(v))
        self.serial.send(f"d{v}")

    def _on_entry(self, event):
        try:
            v = int(self.duty_entry.get())
            v = max(0, min(4095, v))
            self.duty_var.set(v)
            self.serial.send(f"d{v}")
        except ValueError:
            pass

    # ── Shaft angle gauge ──
    def _draw_gauge(self, angle_deg):
        c = self.gauge
        c.delete("all")
        cx, cy, r = 110, 110, 90

        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=self.OVERLAY, width=2)

        start_arc = 90 - ANGLE_MAX
        extent_arc = ANGLE_MAX - ANGLE_MIN
        c.create_arc(cx - r + 5, cy - r + 5, cx + r - 5, cy + r - 5,
                      start=start_arc, extent=extent_arc,
                      outline=self.ORANGE, width=4, style="arc")

        for deg in range(0, 360, 30):
            rad = math.radians(deg - 90)
            x1 = cx + (r - 8) * math.cos(rad)
            y1 = cy + (r - 8) * math.sin(rad)
            x2 = cx + r * math.cos(rad)
            y2 = cy + r * math.sin(rad)
            c.create_line(x1, y1, x2, y2, fill=self.OVERLAY, width=2)
            lx = cx + (r - 20) * math.cos(rad)
            ly = cy + (r - 20) * math.sin(rad)
            c.create_text(lx, ly, text=str(deg), fill=self.FG, font=("Consolas", 7))

        if angle_deg is not None:
            rad = math.radians(angle_deg - 90)
            nx = cx + (r - 30) * math.cos(rad)
            ny = cy + (r - 30) * math.sin(rad)
            c.create_line(cx, cy, nx, ny, fill=self.ACCENT, width=3, arrow="last")
            c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill=self.ACCENT, outline="")
            c.create_text(cx, cy + r - 5, text=f"{angle_deg:.2f}\u00b0", fill=self.GREEN,
                           font=("Consolas", 12, "bold"))
        else:
            c.create_text(cx, cy, text="NO\nSIGNAL", fill=self.RED,
                           font=("Consolas", 12, "bold"), justify="center")

    # ── Throttle % gauge ──
    def _draw_thr_gauge(self, thr_pct, tgt_pct):
        c = self.thr_gauge
        c.delete("all")
        cx, cy = 110, 120
        r = 90

        c.create_arc(cx - r, cy - r, cx + r, cy + r,
                      start=0, extent=180, outline=self.OVERLAY, width=8, style="arc")

        zone_colors = [(0, 25, self.GREEN), (25, 75, self.YELLOW), (75, 100, self.RED)]
        for z_start, z_end, color in zone_colors:
            a_start = 180 - z_end * 1.8
            a_extent = (z_end - z_start) * 1.8
            c.create_arc(cx - r, cy - r, cx + r, cy + r,
                          start=a_start, extent=a_extent,
                          outline=color, width=4, style="arc")

        for pct in [0, 25, 50, 75, 100]:
            angle = math.radians(180 - pct * 1.8)
            x1 = cx + (r - 12) * math.cos(angle)
            y1 = cy - (r - 12) * math.sin(angle)
            x2 = cx + (r + 2) * math.cos(angle)
            y2 = cy - (r + 2) * math.sin(angle)
            c.create_line(x1, y1, x2, y2, fill=self.FG, width=2)
            lx = cx + (r - 24) * math.cos(angle)
            ly = cy - (r - 24) * math.sin(angle)
            c.create_text(lx, ly, text=f"{pct}", fill=self.FG, font=("Consolas", 8))

        if tgt_pct is not None:
            tgt_angle = math.radians(180 - tgt_pct * 1.8)
            tx = cx + (r - 30) * math.cos(tgt_angle)
            ty = cy - (r - 30) * math.sin(tgt_angle)
            c.create_line(cx, cy, tx, ty, fill=self.OVERLAY, width=2, dash=(4, 4))

        if thr_pct is not None:
            needle_angle = math.radians(180 - thr_pct * 1.8)
            nx = cx + (r - 30) * math.cos(needle_angle)
            ny = cy - (r - 30) * math.sin(needle_angle)
            if thr_pct < 25:
                needle_color = self.GREEN
            elif thr_pct < 75:
                needle_color = self.YELLOW
            else:
                needle_color = self.RED
            c.create_line(cx, cy, nx, ny, fill=needle_color, width=3, arrow="last")
            c.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill=needle_color, outline="")
            c.create_text(cx, cy + 16, text=f"{thr_pct}%",
                           fill=needle_color, font=("Consolas", 16, "bold"))
        else:
            c.create_text(cx, cy - 20, text="---", fill=self.RED,
                           font=("Consolas", 14, "bold"))

    # ── Plot ──
    def _draw_plot(self):
        c = self.plot_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10:
            return

        n = len(self.pos_history)
        if n < 2:
            c.create_text(w // 2, h // 2, text="Waiting for data...",
                           fill=self.OVERLAY, font=("Segoe UI", 10))
            return

        margin = 5
        plot_h = h - 2 * margin
        plot_w = w - 2 * margin

        for pct in [0, 25, 50, 75, 100]:
            y = margin + plot_h - (pct / 100.0) * plot_h
            c.create_line(margin + 25, y, w - margin, y, fill=self.OVERLAY, dash=(2, 4))
            c.create_text(margin + 2, y, text=f"{pct}%", fill=self.OVERLAY,
                           font=("Consolas", 7), anchor="w")

        thr_data = list(self.thr_history)
        if len(thr_data) >= 2:
            points = []
            for i, val in enumerate(thr_data):
                x = margin + 25 + (i / (len(thr_data) - 1)) * (plot_w - 25)
                if val is not None:
                    y = margin + plot_h - (val / 100.0) * plot_h
                else:
                    y = margin + plot_h
                points.append((x, y))
            flat = [coord for pt in points for coord in pt]
            c.create_line(*flat, fill=self.ORANGE, width=2, smooth=True)

        pos_data = list(self.pos_history)
        if len(pos_data) >= 2:
            points = []
            for i, val in enumerate(pos_data):
                x = margin + 25 + (i / (len(pos_data) - 1)) * (plot_w - 25)
                if val is not None:
                    pct = (val - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN) * 100.0
                    pct = max(0, min(100, pct))
                    y = margin + plot_h - (pct / 100.0) * plot_h
                else:
                    y = margin + plot_h
                points.append((x, y))
            flat = [coord for pt in points for coord in pt]
            c.create_line(*flat, fill=self.ACCENT, width=2, smooth=True)

        c.create_text(w - margin, margin + 4, text="actual", fill=self.ACCENT,
                       font=("Consolas", 8), anchor="ne")
        c.create_text(w - margin, margin + 16, text="target", fill=self.ORANGE,
                       font=("Consolas", 8), anchor="ne")

    # ── Update error indicators ──
    def _update_errors(self, flags, reason=""):
        self.lbl_err_hex.config(text=f"0x{flags:02X}")
        any_active = flags != 0

        # Update fault reason label
        if any_active:
            # Build reason from active flags if no serial reason available
            if reason:
                display_reason = reason
            else:
                active = [short for bit, _, short, _ in
                          [(b, d, s, ds) for b, s, ds in ERR_NAMES for d in [None]]
                          if flags & bit]
                # Simpler approach:
                active = []
                for bit, dot, name_lbl, desc_lbl in self.err_indicators:
                    if flags & bit:
                        active.append(name_lbl.cget("text").strip())
                display_reason = ", ".join(active) if active else f"FAULT 0x{flags:02X}"
            self.lbl_fault_reason.config(text=f"FAULT: {display_reason}",
                                          fg=self.RED)
            self.err_frame.config(highlightbackground=self.RED, highlightthickness=2)
            self.lbl_err_hex.config(fg=self.RED)
        else:
            self.lbl_fault_reason.config(text="ALL CLEAR", fg="#2d5016")
            self.err_frame.config(highlightbackground=self.OVERLAY, highlightthickness=1)
            self.lbl_err_hex.config(fg=self.OVERLAY)

        # Update each indicator row
        for bit, dot, name_lbl, desc_lbl in self.err_indicators:
            if flags & bit:
                dot.config(fg=self.RED)
                name_lbl.config(fg=self.RED, bg="#5c1a1a")
                desc_lbl.config(fg=self.RED, bg="#5c1a1a")
            else:
                dot.config(fg=self.OVERLAY)
                name_lbl.config(fg=self.OVERLAY, bg=self.DARK_RED)
                desc_lbl.config(fg=self.OVERLAY, bg=self.DARK_RED)

    # ── Poll ──
    def _poll(self):
        if self.serial.connected:
            with self.serial.lock:
                mode = self.serial.mode
                d = self.serial.dir
                duty = self.serial.duty
                ris = self.serial.ris
                lis = self.serial.lis
                pos = self.serial.pos
                thr = self.serial.thr_actual
                tgt = self.serial.thr_target
                err = self.serial.err_flags
                raw = self.serial.raw_line
                safe_reason = self.serial.safe_reason

            # Mode indicator
            mode_text, mode_color = MODE_LABELS.get(mode, (f"MODE: {mode}", self.FG))
            self.lbl_mode.config(text=f"MODE: {mode_text}", foreground=mode_color)

            # Safe state visual warning
            if mode == "SAFE":
                self.root.configure(bg="#2d0a0a")
                self.btn_reset.config(bg=self.YELLOW, fg=self.BG)
            else:
                self.root.configure(bg=self.BG)
                self.btn_reset.config(bg=self.OVERLAY, fg=self.YELLOW)

            # Telemetry
            dir_color = self.GREEN if d == "F" else self.YELLOW
            self.telem_vals[0].config(text=("Forward" if d == "F" else "Reverse"), foreground=dir_color)
            self.telem_vals[1].config(text=str(duty))
            self.telem_vals[2].config(text=str(ris))
            self.telem_vals[3].config(text=str(lis))
            if pos is not None:
                self.telem_vals[4].config(text=f"{pos:.2f}\u00b0", foreground=self.ACCENT)
            else:
                self.telem_vals[4].config(text="No signal", foreground=self.RED)
            if thr is not None:
                self.telem_vals[5].config(text=f"{thr}%", foreground=self.ORANGE)
            else:
                self.telem_vals[5].config(text="---", foreground=self.RED)
            self.telem_vals[6].config(text=f"{tgt}%", foreground=self.FG)

            # Direction buttons
            self.btn_fwd.config(bg=(self.GREEN if d == "F" else self.SURFACE),
                                 fg=(self.BG if d == "F" else self.GREEN))
            self.btn_rev.config(bg=(self.YELLOW if d == "R" else self.SURFACE),
                                 fg=(self.BG if d == "R" else self.YELLOW))

            # Error flags with reason
            self._update_errors(err, safe_reason)

            # Gauges
            self._draw_gauge(pos)
            self._draw_thr_gauge(thr, tgt)

            # History
            self.pos_history.append(pos)
            self.thr_history.append(thr)
            self._draw_plot()

            # Log
            if raw:
                self.log_text.config(state="normal")
                self.log_text.insert("end", raw + "\n")
                self.log_text.see("end")
                lines = int(self.log_text.index("end-1c").split(".")[0])
                if lines > 200:
                    self.log_text.delete("1.0", "100.0")
                self.log_text.config(state="disabled")

        self.root.after(100, self._poll)

    def _on_close(self):
        self.serial.disconnect()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    ThrottleGUI().run()
