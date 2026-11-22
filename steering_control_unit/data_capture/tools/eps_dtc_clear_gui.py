"""Step-by-step window for clearing a latched EPS DTC with the eps_bench_steer Feather.

The service manual's erase sequence needs "45 deg left and hold" twice, with the engine stopped.
With the torque sensor cut, the Feather's DACs supply that left torque (firmware clear mode:
`clr`, `L`, `R`, `clr0`). This window walks through the steps, gives one big HOLD LEFT / RELEASE
button (Space bar), and counts the manual's 4 s windows.

Run (needs PySide6 and pyserial, e.g. the Scope1008 venv):
    python eps_dtc_clear_gui.py            # finds the Feather by USB id
    python eps_dtc_clear_gui.py COM11

DISCLAIMER: for research and bench testing only, on a removed EPS unit. Experimental, not
safety-validated, not for use on a road vehicle. Use at your own risk.
"""
from __future__ import annotations

import sys
import time

import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication, QComboBox, QHBoxLayout, QLabel, QPlainTextEdit, QProgressBar, QPushButton,
    QVBoxLayout, QWidget,
)

ADAFRUIT_VID = 0x239A
WINDOW_S = 4.0  # the manual's reaction window

# AUTO: timed sequence from key-on (EPS 12 V on), placed in the middle of each 4 s window of the
# manual. Key-on is detected when the SAS starts sending CAN again (it powers up with the EPS on
# this bench), or by pressing "EPS IS ON NOW". Expected lamp: on 0-6 s; off by ~6 s (or 4 s into
# the hold); on again 4 s after the release; off 4 s after the 2nd hold; two blinks 4 s after the
# 2nd release. Each action below sits inside the 4 s window under both readings.
# Order from people who cleared it on a real car (insightcentral.net "Troubleshooting EPS"): hold
# 45 deg left BEFORE switching the ignition on; the lamp comes on, goes off after ~4 s; release;
# lamp on 4 s later; hold; lamp off after 4 s; release; two blinks. So AUTO holds left while armed
# (before key-on) and the first release comes at 7 s: 3 s after a 4 s lamp-off, 1 s after a 6 s one.
AUTO_PLAN = [  # (seconds after key-on, command, what the lamp should be doing then)
    (7.0, "R", "lamp should have gone OFF (at 4-6 s). Released; it should come back ON at about 11 s"),
    (13.0, "L", "lamp should be back ON. Holding left; it should go OFF at about 17 s"),
    (19.0, "R", "lamp should be OFF. Released; watch for TWO BLINKS at about 23 s"),
]
AUTO_END_S = 26.0
AUTO_REHOLD_S = 10.0  # re-send L while armed: the firmware releases a hold after 15 s
SAS_GAP_S = 1.0  # SAS silent this long = EPS off

# (title, what to do). The big button moves through steps 1-4; step 5 waits for the blinks.
STEPS = [
    ("Get ready",
     "1. EPS 12 V OFF.\n2. Short SCS to ground.\n3. Hands off the wheel.\n\nThen press START CLEAR MODE."),
    ("Switch the EPS on",
     "Switch the EPS 12 V ON. The lamp comes on.\n\nWithin 4 s: press HOLD LEFT."),
    ("Holding left",
     "Wait for the lamp to go OFF.\n\nThen within 4 s: press RELEASE."),
    ("Released",
     "The lamp comes back ON in about 4 s.\n\nWithin 4 s of it coming on: press HOLD LEFT."),
    ("Holding left (2nd time)",
     "The lamp goes OFF after about 4 s.\n\nThen within 4 s: press RELEASE."),
    ("Watch the lamp",
     "About 4 s from now the lamp should BLINK TWICE.\n\nTwo blinks = erased."),
    ("Done",
     "1. EPS 12 V OFF.\n2. Remove the SCS short.\n3. Press EXIT CLEAR MODE.\n\n"
     "Check: EPS on, NEP 900 (normal mode): the lamp should go off."),
]

FAIL_TEXT = ("Didn't work? EPS 12 V OFF, then press TRY AGAIN.\n"
             "If the lamp never went off while holding left, pick a stronger level (-1.00 V) first.")


def find_port() -> str | None:
    for p in list_ports.comports():
        if p.vid == ADAFRUIT_VID:
            return p.device
    return None


class ClearWindow(QWidget):
    def __init__(self, port: str | None):
        super().__init__()
        self.setWindowTitle("EPS DTC clear")
        self.resize(600, 940)
        self.want_port = port
        self.ser: serial.Serial | None = None
        self.buf = b""
        self.clear_mode = False
        self.left = False
        self.changed_at = time.monotonic()
        self.step = 0
        self.rpm = 0
        self.vt = (0.0, 0.0)
        self.sas = None
        self.sas_at = 0.0
        self.auto = "idle"        # idle | wait_off | armed | running | done
        self.auto_t0 = 0.0
        self.auto_next = 0
        self.auto_src = ""
        self.auto_hold_at = 0.0

        big = QFont()
        big.setPointSize(20)
        big.setBold(True)

        self.conn = QLabel("Feather: not connected")
        self.live = QLabel("")
        self.live.setStyleSheet("font-family: Consolas, monospace; font-size: 13px;")
        self.mode = QLabel("NORMAL MODE")
        self.mode.setAlignment(Qt.AlignCenter)
        self.mode.setStyleSheet("font-size: 16px; font-weight: bold; padding: 6px; border-radius: 6px;"
                                "background: #444; color: white;")

        self.step_title = QLabel()
        self.step_title.setStyleSheet("font-size: 22px; font-weight: bold;")
        self.step_text = QLabel()
        self.step_text.setWordWrap(True)
        self.step_text.setStyleSheet("font-size: 16px;")
        self.step_text.setMinimumHeight(130)
        self.step_text.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        self.big = QPushButton("HOLD LEFT")
        self.big.setFont(big)
        self.big.setMinimumHeight(110)
        self.big.clicked.connect(self.toggle)

        self.timer_lbl = QLabel("0.0 s since last press")
        self.timer_lbl.setAlignment(Qt.AlignCenter)
        self.timer_lbl.setStyleSheet("font-size: 18px;")
        self.bar = QProgressBar()
        self.bar.setRange(0, 400)
        self.bar.setTextVisible(False)
        self.bar.setFixedHeight(16)

        self.start_btn = QPushButton("START CLEAR MODE")
        self.start_btn.clicked.connect(lambda: self.send("clr"))
        self.exit_btn = QPushButton("EXIT CLEAR MODE")
        self.exit_btn.clicked.connect(self.exit_clear)
        self.blink_btn = QPushButton("It blinked twice")
        self.blink_btn.clicked.connect(self.blinked)
        self.retry_btn = QPushButton("TRY AGAIN")
        self.retry_btn.clicked.connect(self.retry)
        for b in (self.start_btn, self.exit_btn, self.blink_btn, self.retry_btn):
            b.setMinimumHeight(40)

        self.level = QComboBox()
        for v in ("-0.60", "-0.80", "-1.00", "-1.20", "-1.40"):
            self.level.addItem(f"Left level {v} V", v)
        self.level.setCurrentIndex(3)   # -1.20: -0.60 and -0.80 didn't turn the lamp off (2026-11-21)
        self.level.currentIndexChanged.connect(lambda _: self.send("cd" + self.level.currentData()))

        self.warn = QLabel("")
        self.warn.setWordWrap(True)
        self.warn.setStyleSheet("color: white; background: #b00020; font-size: 15px; padding: 6px;")
        self.warn.hide()

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(300)
        self.log.setStyleSheet("font-family: Consolas, monospace; font-size: 11px;")

        # AUTO panel
        self.auto_btn = QPushButton("AUTO CLEAR")
        self.auto_btn.setFont(big)
        self.auto_btn.setMinimumHeight(70)
        self.auto_btn.setStyleSheet("QPushButton { color: white; background: #6a1b9a; border-radius: 10px; }"
                                    "QPushButton:disabled { background: #999; }")
        self.auto_btn.clicked.connect(self.auto_start)
        self.eps_on_btn = QPushButton("EPS IS ON NOW")
        self.eps_on_btn.setMinimumHeight(70)
        self.eps_on_btn.clicked.connect(lambda: self.auto_go("button"))
        self.abort_btn = QPushButton("ABORT")
        self.abort_btn.setMinimumHeight(70)
        self.abort_btn.setStyleSheet("QPushButton { color: white; background: #b00020; font-weight: bold;"
                                     " border-radius: 10px; } QPushButton:disabled { background: #999; }")
        self.abort_btn.clicked.connect(lambda: self.auto_abort("ABORT pressed"))
        self.auto_lbl = QLabel("AUTO: press AUTO CLEAR, then switch the EPS 12 V on. Hands off.")
        self.auto_lbl.setWordWrap(True)
        self.auto_lbl.setStyleSheet("font-size: 16px; padding: 6px; background: #f3e5f5; color: #222;"
                                    " border-radius: 6px;")
        self.auto_lbl.setMinimumHeight(64)

        top = QHBoxLayout()
        top.addWidget(self.conn, 1)
        top.addWidget(self.level)
        row1 = QHBoxLayout()
        row1.addWidget(self.start_btn)
        row1.addWidget(self.exit_btn)
        row2 = QHBoxLayout()
        row2.addWidget(self.blink_btn)
        row2.addWidget(self.retry_btn)

        lay = QVBoxLayout(self)
        lay.addLayout(top)
        lay.addWidget(self.live)
        lay.addWidget(self.mode)
        lay.addWidget(self.warn)
        arow = QHBoxLayout()
        arow.addWidget(self.auto_btn, 2)
        arow.addWidget(self.eps_on_btn, 1)
        arow.addWidget(self.abort_btn, 1)
        lay.addLayout(arow)
        lay.addWidget(self.auto_lbl)
        manual = QLabel("Manual (step by step)")
        manual.setStyleSheet("color: #888; margin-top: 8px;")
        lay.addWidget(manual)
        lay.addWidget(self.step_title)
        lay.addWidget(self.step_text)
        lay.addWidget(self.big)
        lay.addWidget(self.timer_lbl)
        lay.addWidget(self.bar)
        lay.addLayout(row1)
        lay.addLayout(row2)
        fail = QLabel(FAIL_TEXT)
        fail.setWordWrap(True)
        fail.setStyleSheet("color: #888;")
        lay.addWidget(fail)
        lay.addWidget(QLabel("Feather messages"))
        lay.addWidget(self.log, 1)

        QShortcut(QKeySequence(Qt.Key_Space), self, activated=self.toggle)

        self.poll = QTimer(self, interval=20, timeout=self.read_serial)
        self.poll.start()
        self.tick = QTimer(self, interval=100, timeout=self.refresh)
        self.tick.start()
        self.reconnect = QTimer(self, interval=1500, timeout=self.connect_port)
        self.reconnect.start()
        self.connect_port()
        self.set_step(0)

    # ------------------------------------------------------------------ serial
    def connect_port(self):
        if self.ser:
            return
        port = self.want_port or find_port()
        if not port:
            self.conn.setText("Feather: not found (plug in USB; close Scope1008)")
            return
        try:
            # dsrdtr/rtscts off and DTR left alone so opening the port doesn't reset the board
            self.ser = serial.Serial(port, 115200, timeout=0)
        except serial.SerialException as e:
            self.conn.setText(f"Feather: {port} busy ({e.__class__.__name__}). Close Scope1008 / other terminals.")
            self.ser = None
            return
        self.conn.setText(f"Feather: connected on {port}")
        self.send("cd" + self.level.currentData())   # the Feather boots at -0.80: match the picker
        self.send("?")

    def send(self, cmd: str):
        if not self.ser:
            self.add_log(f"(not connected) {cmd}")
            return
        try:
            self.ser.write((cmd + "\n").encode())
            self.add_log(">> " + cmd)
        except serial.SerialException:
            self.drop()

    def drop(self):
        try:
            if self.ser:
                self.ser.close()
        finally:
            self.ser = None
            self.conn.setText("Feather: disconnected, retrying...")

    def read_serial(self):
        if not self.ser:
            return
        try:
            data = self.ser.read(8192)
        except serial.SerialException:
            self.drop()
            return
        if not data:
            self.auto_run()
            return
        self.buf += data
        *lines, self.buf = self.buf.split(b"\n")
        for raw in lines:
            self.handle(raw.decode(errors="replace").strip())
        self.auto_run()

    def handle(self, line: str):
        if line.startswith("V,"):
            p = line.split(",")
            if len(p) >= 5:
                self.vt = (float(p[2]), float(p[3]))
        elif line.startswith("N,"):
            p = line.split(",")
            if len(p) >= 3:
                self.rpm = int(p[2])
        elif line.startswith("A,"):
            p = line.split(",")
            if len(p) >= 3:
                now = time.monotonic()
                came_back = self.sas is None or now - self.sas_at > SAS_GAP_S
                self.sas = float(p[2])
                self.sas_at = now
                if came_back and self.auto == "armed":
                    self.auto_go("SAS")         # SAS powering up = EPS 12 V just switched on
        elif line.startswith("#"):
            self.add_log(line)
            self.on_comment(line)

    def on_comment(self, line: str):
        if "CLEAR MODE on" in line or "clear mode already on" in line:
            self.clear_mode = True
            if self.step == 0:
                self.set_step(1)
        elif "CLEAR MODE off" in line or "clear mode already off" in line:
            self.clear_mode = False
            self.left = False
        elif "# clear mode ON" in line:          # status line after '?'
            self.clear_mode = True
            self.left = "LEFT" in line
        elif "# clear mode off" in line:
            self.clear_mode = False
            self.left = False
        elif "CLR LEFT HOLD" in line:
            self.left = True
            self.changed_at = time.monotonic()
            if self.step in (1, 3) and self.auto == "idle":
                self.set_step(self.step + 1)
        elif "CLR RELEASE" in line:
            self.left = False
            self.changed_at = time.monotonic()
            if "moved" in line or "limit" in line:
                self.auto_abort("the Feather released by itself")
            if "moved" in line:
                pass                              # the WHEEL MOVED warning is already up
            elif "limit" in line:
                self.show_warn("The Feather released by itself: " + line[2:] +
                               "\nEPS 12 V OFF, then TRY AGAIN.")
            elif self.step in (2, 4) and self.auto == "idle":
                self.set_step(self.step + 1)
        if "WHEEL MOVED" in line:
            self.show_warn("WHEEL MOVED while holding: the EPS is assisting. CUT THE EPS 12 V NOW.")
        if "NEP off (engine stopped)" in line or "needs NEP off" in line:
            self.show_warn("NEP is on. Clear mode needs NEP off: send r0 (or use Scope1008), then START.")

    # ------------------------------------------------------------------ auto sequence
    def sas_fresh(self) -> bool:
        return self.sas is not None and time.monotonic() - self.sas_at < SAS_GAP_S

    def auto_start(self):
        if not self.ser:
            self.add_log("(not connected)")
            return
        self.hide_warn()
        if self.rpm:
            self.send("r0")                 # the erase sequence is engine stopped: NEP off
        if self.left:
            self.send("R")
        if not self.clear_mode:
            self.send("clr")
        self.set_step(5)
        self.step_title.setText("AUTO running the steps")
        self.step_text.setText("Watch the lamp. Space or ABORT stops it.")
        self.send("L")                      # hold left BEFORE key-on, as on a real car
        self.auto_hold_at = time.monotonic()
        if self.sas_fresh():
            self.auto = "wait_off"
            self.auto_lbl.setText("ARMED, already holding LEFT. (The SAS is sending, so it can't see key-on.)\n"
                                  "IG1 OFF, SCS shorted. Flip IG1 ON and press EPS IS ON NOW at the same moment.")
        else:
            self.auto = "armed"
            self.auto_lbl.setText("ARMED, already holding LEFT. SCS shorted, hands off.\n"
                                  "Switch IG1 ON now (or press EPS IS ON NOW as you do).")

    def auto_go(self, src: str):
        if self.auto not in ("armed", "wait_off"):
            return
        if not self.clear_mode:
            self.send("clr")
        self.auto = "running"
        self.auto_src = src
        self.auto_t0 = time.monotonic()
        self.auto_next = 0
        self.send("L")                      # restart the firmware's 15 s hold timer at key-on
        self.add_log(f"AUTO: key-on detected ({src}), sequence started")

    def auto_abort(self, why: str):
        if self.auto == "idle":
            return
        if self.left or self.auto == "running":
            self.send("R")
        self.auto = "idle"
        self.auto_lbl.setText(f"AUTO stopped: {why}.\nEPS 12 V OFF, then AUTO CLEAR to try again.")
        self.add_log("AUTO stopped: " + why)

    def auto_run(self):
        now = time.monotonic()
        if self.auto in ("armed", "wait_off") and now - self.auto_hold_at > AUTO_REHOLD_S:
            self.send("L")                  # keep the pre-key-on hold alive (firmware limit 15 s)
            self.auto_hold_at = now
        if self.auto == "wait_off" and self.sas is not None and now - self.sas_at > SAS_GAP_S:
            self.auto = "armed"
            self.auto_lbl.setText("EPS is off. ARMED, holding LEFT.\nSwitch IG1 ON now.")
        if self.auto != "running":
            return
        el = now - self.auto_t0
        if self.auto_src == "SAS" and now - self.sas_at > SAS_GAP_S:
            self.auto_abort("the EPS was switched off (SAS stopped)")
            return
        while self.auto_next < len(AUTO_PLAN) and el >= AUTO_PLAN[self.auto_next][0]:
            t, cmd, _ = AUTO_PLAN[self.auto_next]
            self.send(cmd)
            self.auto_next += 1
        if self.auto_next >= len(AUTO_PLAN) and el >= AUTO_END_S:
            self.auto = "done"
            self.auto_lbl.setText("DONE. Did the lamp blink TWICE?\n"
                                  "Yes: press 'It blinked twice'.  No: EPS 12 V OFF, then AUTO CLEAR again "
                                  "(if the lamp never went off, pick a deeper Left level first: -1.00, -1.20, -1.40).")
            self.set_step(5)
            return
        now_txt = AUTO_PLAN[self.auto_next - 1][2] if self.auto_next else \
            "lamp ON (key-on). Holding left; it should go OFF at about 4-6 s"
        nxt = (f"next: {'HOLD LEFT' if AUTO_PLAN[self.auto_next][1] == 'L' else 'RELEASE'} in "
               f"{AUTO_PLAN[self.auto_next][0] - el:3.1f} s") if self.auto_next < len(AUTO_PLAN) else \
            "blinks expected at about 23 s"
        self.auto_lbl.setText(f"AUTO {el:4.1f} s   [{'LEFT' if self.left else 'released'}]   {nxt}\n{now_txt}")

    # ------------------------------------------------------------------ actions
    def toggle(self):
        if self.auto == "running":
            self.auto_abort("Space pressed")
            return
        if not self.clear_mode:
            self.add_log("(press START CLEAR MODE first)")
            return
        self.send("R" if self.left else "L")

    def blinked(self):
        self.auto = "idle"
        self.auto_lbl.setText("Erased. EPS 12 V OFF, remove the SCS short, press EXIT CLEAR MODE.")
        self.set_step(6)

    def exit_clear(self):
        self.auto_abort("clear mode exited")
        self.send("clr0")
        self.set_step(0)

    def retry(self):
        self.hide_warn()
        if self.left:
            self.send("R")
        if not self.clear_mode:
            self.send("clr")
        self.set_step(1)

    def set_step(self, i: int):
        self.step = i
        title, text = STEPS[i]
        self.step_title.setText(f"Step {i} of {len(STEPS) - 1}: {title}" if i else title)
        self.step_text.setText(text)

    def show_warn(self, text: str):
        self.warn.setText(text)
        self.warn.show()

    def hide_warn(self):
        self.warn.hide()

    def add_log(self, text: str):
        self.log.appendPlainText(time.strftime("%H:%M:%S ") + text)

    # ------------------------------------------------------------------ display
    def refresh(self):
        sas_ok = self.sas is not None and time.monotonic() - self.sas_at < 0.5
        sas = f"{self.sas:+.1f} deg" if sas_ok else "STALE (20 deg abort inactive)"
        self.live.setText(f"NEP {'off' if not self.rpm else str(self.rpm) + ' rpm'}    "
                          f"VT3 {self.vt[0]:.3f} V   VT6 {self.vt[1]:.3f} V    SAS {sas}")
        if self.clear_mode:
            self.mode.setText("CLEAR MODE: " + ("HOLDING LEFT" if self.left else "released"))
            self.mode.setStyleSheet("font-size: 16px; font-weight: bold; padding: 6px; border-radius: 6px;"
                                    f"background: {'#d35400' if self.left else '#1565c0'}; color: white;")
        else:
            self.mode.setText("NORMAL MODE")
            self.mode.setStyleSheet("font-size: 16px; font-weight: bold; padding: 6px; border-radius: 6px;"
                                    "background: #444; color: white;")
        self.big.setEnabled(self.clear_mode and self.ser is not None)
        self.big.setText("RELEASE  (Space)" if self.left else "HOLD LEFT  (Space)")
        self.big.setStyleSheet(
            "QPushButton { color: white; border-radius: 10px; background: %s; }"
            "QPushButton:disabled { background: #999; }" % ("#2e7d32" if self.left else "#d35400"))
        el = time.monotonic() - self.changed_at
        self.timer_lbl.setText(f"{el:4.1f} s since last press")
        self.bar.setValue(int(min(el, WINDOW_S) * 100))
        self.bar.setStyleSheet("QProgressBar::chunk { background: %s; }" % ("#c62828" if el > WINDOW_S else "#43a047"))
        self.start_btn.setEnabled(not self.clear_mode)
        self.exit_btn.setEnabled(self.clear_mode)
        self.blink_btn.setEnabled(self.step == 5 and self.auto in ("idle", "done"))
        connected = self.ser is not None
        self.auto_btn.setEnabled(connected and self.auto in ("idle", "done"))
        self.eps_on_btn.setEnabled(self.auto in ("armed", "wait_off"))
        self.abort_btn.setEnabled(self.auto in ("armed", "wait_off", "running"))
        if self.auto == "running":
            self.big.setEnabled(False)

    def closeEvent(self, e):
        if self.ser and self.clear_mode:
            self.send("clr0")       # never leave the DACs holding left torque
            time.sleep(0.1)
        if self.ser:
            self.ser.close()
        super().closeEvent(e)


def main():
    app = QApplication(sys.argv)
    w = ClearWindow(sys.argv[1] if len(sys.argv) > 1 else None)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
