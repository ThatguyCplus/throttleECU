#!/usr/bin/env python3
"""
can_commander.py — Throttle ECU CAN bus commander
==================================================
Runs on your PC via a USB CAN adapter (Rho2 Pro or any python-can compatible device).

Protocol:
  TX  ID 0x100  [4 bytes]  → ECU commands
      byte 0 — flags: bit0=RELAY, bit1=PID, bit2=ESTOP
      byte 1 — throttle target 0-100
      byte 2 — reserved (0)
      byte 3 — sequence counter

  RX  ID 0x101  [8 bytes]  ← ECU telemetry
      byte 0 — mode (0=MAN, 1=PID, 2=SAFE)
      byte 1 — throttle actual 0-100 (0xFF = no encoder)
      byte 2 — throttle target 0-100
      byte 3 — fault flags
      byte 4-5 — motor command int16 little-endian
      byte 6 — relay state (0/1)
      byte 7 — ECU TX sequence counter

Install:
  pip install python-can

Usage:
  python can_commander.py                          # auto-detect / prompt
  python can_commander.py --interface slcan --channel COM5
  python can_commander.py --interface pcan  --channel PCAN_USBBUS1
  python can_commander.py --interface kvaser
  python can_commander.py --interface socketcan --channel can0   # Linux

Interactive commands (type and press Enter):
  on          relay on
  off         relay off
  t50         set throttle to 50% (PID mode, relay on)
  t0          set throttle to 0%  (PID mode, relay on)
  stop        manual stop (duty=0, no relay change)
  estop       emergency stop (sends ESTOP flag)
  q / quit    exit
"""

import argparse
import struct
import sys
import threading
import time

try:
    import can
except ImportError:
    print("ERROR: python-can not installed.  Run:  pip install python-can")
    sys.exit(1)

# ── CAN IDs ──────────────────────────────────────────────────────────────────
RX_ID = 0x100   # PC → ECU  (commands)
TX_ID = 0x101   # ECU → PC  (telemetry)

# ── Command flag bits ─────────────────────────────────────────────────────────
FLAG_RELAY = 0x01
FLAG_PID   = 0x02
FLAG_ESTOP = 0x04

# ── Fault bit names ───────────────────────────────────────────────────────────
FAULT_NAMES = {
    0x01: "ENC_STALE",
    0x02: "ENC_INVALID",
    0x04: "OVERCURR_L",
    0x08: "OVERCURR_R",
    0x10: "POWER_LOW",
    0x20: "WDG_RESET",
    0x40: "CAN_TIMEOUT",
    0x80: "CAN_BUS_OFF",
}
MODE_NAMES = {0: "MAN", 1: "PID", 2: "SAFE"}

# ── Shared state ──────────────────────────────────────────────────────────────
_lock         = threading.Lock()
_relay_on     = False
_pid_mode     = False
_throttle_pct = 0
_tx_seq       = 0
_heartbeat_hz = 10          # send command frames at 10 Hz (every 100 ms)
_running      = True

# ── Build command frame ───────────────────────────────────────────────────────
def build_cmd(estop=False):
    global _tx_seq
    with _lock:
        flags  = 0
        if _relay_on: flags |= FLAG_RELAY
        if _pid_mode: flags |= FLAG_PID
        if estop:     flags |= FLAG_ESTOP
        thr    = max(0, min(100, _throttle_pct))
        seq    = _tx_seq & 0xFF
        _tx_seq += 1
    return bytes([flags, thr, 0, seq])


# ── Decode telemetry frame ────────────────────────────────────────────────────
def decode_telem(data):
    if len(data) < 8:
        return
    mode     = data[0]
    act_pct  = data[1]
    tgt_pct  = data[2]
    faults   = data[3]
    motor_cmd= struct.unpack_from('<h', bytes(data[4:6]))[0]
    relay    = data[6]
    ecu_seq  = data[7]

    fault_strs = [name for bit, name in FAULT_NAMES.items() if faults & bit]
    fault_str  = (" ".join(fault_strs)) if fault_strs else "OK"
    act_str    = f"{act_pct}%" if act_pct != 0xFF else "---"

    print(f"\r[ECU] mode={MODE_NAMES.get(mode,'?')}  "
          f"pos={act_str}  tgt={tgt_pct}%  "
          f"cmd={motor_cmd:+5d}  relay={'ON' if relay else 'OFF'}  "
          f"faults={fault_str}  seq={ecu_seq}          ",
          end="", flush=True)


# ── Heartbeat thread ──────────────────────────────────────────────────────────
def heartbeat_thread(bus):
    interval = 1.0 / _heartbeat_hz
    while _running:
        try:
            payload = build_cmd()
            msg = can.Message(arbitration_id=RX_ID,
                              data=payload,
                              is_extended_id=False)
            bus.send(msg)
        except can.CanError as e:
            print(f"\n[WARN] CAN TX error: {e}")
        time.sleep(interval)


# ── RX thread ─────────────────────────────────────────────────────────────────
def rx_thread(bus):
    while _running:
        try:
            msg = bus.recv(timeout=0.5)
            if msg is None:
                continue
            if msg.arbitration_id == TX_ID:
                decode_telem(msg.data)
        except can.CanError as e:
            print(f"\n[WARN] CAN RX error: {e}")
        except Exception:
            pass


# ── Interactive command loop ──────────────────────────────────────────────────
def cmd_loop(bus):
    global _relay_on, _pid_mode, _throttle_pct, _running

    print("\nThrottle ECU CAN Commander ready.")
    print("Commands: on | off | t<0-100> | stop | estop | q")
    print("-" * 55)

    while _running:
        try:
            raw = input("\ncmd> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if raw in ("q", "quit", "exit"):
            break

        elif raw == "on":
            with _lock:
                _relay_on = True
            print("[CMD] Relay ON")

        elif raw == "off":
            with _lock:
                _relay_on  = False
                _pid_mode  = False
                _throttle_pct = 0
            print("[CMD] Relay OFF, motor stop")

        elif raw == "stop":
            with _lock:
                _pid_mode  = False
                _throttle_pct = 0
            print("[CMD] Manual stop (relay unchanged)")

        elif raw == "estop":
            # Send ESTOP frame immediately (separate from heartbeat)
            try:
                payload = build_cmd(estop=True)
                msg = can.Message(arbitration_id=RX_ID,
                                  data=payload,
                                  is_extended_id=False)
                bus.send(msg)
                print("[CMD] !! ESTOP sent !!")
            except can.CanError as e:
                print(f"[ERR] {e}")
            with _lock:
                _relay_on     = False
                _pid_mode     = False
                _throttle_pct = 0

        elif raw.startswith("t") and len(raw) > 1:
            try:
                pct = int(raw[1:])
                if not 0 <= pct <= 100:
                    raise ValueError
                with _lock:
                    _throttle_pct = pct
                    _pid_mode     = True
                    _relay_on     = True
                print(f"[CMD] Throttle → {pct}%  (relay ON, PID mode)")
            except ValueError:
                print("[ERR] Usage: t0 to t100")

        elif raw == "":
            pass

        else:
            print(f"[ERR] Unknown command: {raw!r}")

    _running = False
    print("\n[INFO] Exiting...")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Throttle ECU CAN commander",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--interface", "-i", default=None,
                        help="python-can interface (slcan, pcan, kvaser, socketcan, …)")
    parser.add_argument("--channel",   "-c", default=None,
                        help="channel / port (e.g. COM5, PCAN_USBBUS1, can0)")
    parser.add_argument("--bitrate",   "-b", type=int, default=500000,
                        help="CAN bitrate in bps (default 500000)")
    args = parser.parse_args()

    # ── Interface auto-detection ──────────────────────────────────────────────
    iface   = args.interface
    channel = args.channel

    if iface is None:
        print("Available python-can interfaces on this system:")
        for name in ["slcan", "pcan", "kvaser", "socketcan", "usb2can", "ixxat"]:
            print(f"  {name}")
        print()
        iface = input("Interface (e.g. slcan): ").strip() or "slcan"

    if channel is None and iface in ("slcan", "serial", "usb2can"):
        # List COM ports to help the user
        try:
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            if ports:
                print("Detected serial ports:")
                for p in ports:
                    print(f"  {p.device}  —  {p.description}")
            else:
                print("No serial ports detected.")
        except ImportError:
            pass
        channel = input("Channel (e.g. COM5 or /dev/ttyUSB0): ").strip()

    # ── Open bus ──────────────────────────────────────────────────────────────
    print(f"\nOpening CAN bus: interface={iface}  channel={channel}  bitrate={args.bitrate}")
    try:
        bus_kwargs = dict(interface=iface, bitrate=args.bitrate)
        if channel:
            bus_kwargs["channel"] = channel
        bus = can.interface.Bus(**bus_kwargs)
    except Exception as e:
        print(f"ERROR: Could not open CAN bus: {e}")
        print("\nTroubleshooting:")
        print("  • For Rho2 Pro / SLCAN adapters: --interface slcan --channel COMx")
        print("  • Ensure adapter drivers are installed")
        print("  • Ensure S9 switch on LaunchPad is DOWN (CAN transceiver mode)")
        print("  • Ensure bitrate matches ECU (500 kbps)")
        sys.exit(1)

    print(f"Bus open. Sending heartbeat at {_heartbeat_hz} Hz to ID 0x{RX_ID:03X}")
    print(f"Listening for telemetry on ID 0x{TX_ID:03X}")

    # ── Start threads ─────────────────────────────────────────────────────────
    hb = threading.Thread(target=heartbeat_thread, args=(bus,), daemon=True)
    rx = threading.Thread(target=rx_thread,        args=(bus,), daemon=True)
    hb.start()
    rx.start()

    try:
        cmd_loop(bus)
    finally:
        _running = False
        time.sleep(0.2)
        bus.shutdown()
        print("CAN bus closed.")


if __name__ == "__main__":
    main()
