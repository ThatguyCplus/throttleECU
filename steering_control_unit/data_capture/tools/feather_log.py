"""Record the eps_bench_nep Feather's USB serial output to a file.

Every line is saved as  <host unix time>\t<line from the Feather>.
Type commands (r900, r0, z, ?) and press Enter to send them to the Feather.

    python feather_log.py COM7 bench_feather.log

Needs pyserial (pip install pyserial). Ctrl+C stops and closes the file.
"""
import argparse
import sys
import threading
import time

import serial


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="serial port, for example COM7 or /dev/ttyACM0")
    ap.add_argument("out", help="output log file")
    ap.add_argument("--quiet", action="store_true", help="don't echo A/S/C/V data lines to the console")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.2)

    def send_commands():
        for line in sys.stdin:
            ser.write((line.strip() + "\n").encode())

    threading.Thread(target=send_commands, daemon=True).start()
    n = 0
    with open(args.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(f"# feather_log {time.strftime('%Y-%m-%d %H:%M:%S')} port={args.port}\n")
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                t = time.time()
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue
                f.write(f"{t:.6f}\t{line}\n")
                n += 1
                if n % 200 == 0:
                    f.flush()
                if not args.quiet or line[:1] not in "ASCV":
                    print(line)
        except KeyboardInterrupt:
            pass
    print(f"saved {n} lines to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
