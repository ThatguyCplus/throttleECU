"""
decode_log.py — EPS data logger .BIN → CSV + summary

Reads the files written by firmware/eps_logger (format in log_format.h):
a 512-byte header, then one 32-byte record per 1 kHz sample tick.

    python decode_log.py EPS00001.BIN                    # writes EPS00001.csv, prints a summary
    python decode_log.py EPS00001.BIN --every 10         # 100 Hz CSV
    python decode_log.py EPS00001.BIN --vsp-ppkm 2548 --nep-ppr 1.5
    python decode_log.py EPS00001.BIN --summary-only

--vsp-ppkm / --nep-ppr add speed (km/h) and rpm columns once you've
calibrated pulses per km and pulses per engine rev (see BOM.md, calibration).
"""
import argparse
import csv
import datetime as dt
import struct
import sys
from pathlib import Path

HEADER_FMT = "<8sHHHHIIfffffBBBB16s448x"
RECORD_FMT = "<IIIHHHHHHHHHBB"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
RECORD_SIZE = struct.calcsize(RECORD_FMT)
assert HEADER_SIZE == 512 and RECORD_SIZE == 32

DIO_IG1, DIO_WLP_ON, DIO_RT_ON = 0x01, 0x02, 0x04
DIAG_NAMES = ["VBAT_MISSING", "VBAT_RANGE", "VCC2_MISSING", "VCC2_RANGE", "VT3_OPEN", "VT3_RANGE",
              "VT6_OPEN", "VT6_RANGE", "IMOT_SENSOR", "IMOT_ZERO", "IG1_OPEN", "WLP_ON_RUN",
              "RT_OPEN", "NEP_MISSING", "ADC_SEQ", "SD_FAULT"]   # bit order of lead_diag.h


def read_header(blob):
    (magic, version, header_size, record_size, sample_hz, start_unix, start_us,
     v_vt, v_vbat, v_imot, imot_v_per_a, imot_zero_v,
     adc_dual, have_rt, zero_measured, _reserved, firmware) = struct.unpack_from(HEADER_FMT, blob)
    if magic != b"EPSLOG1\0":
        sys.exit("not an EPS logger file (bad magic)")
    if version != 1 or header_size != HEADER_SIZE or record_size != RECORD_SIZE:
        sys.exit(f"unsupported format: version {version}, header {header_size}, record {record_size}")
    return dict(sample_hz=sample_hz, start_unix=start_unix, start_us=start_us,
                v_vt=v_vt, v_vbat=v_vbat, v_imot=v_imot, imot_v_per_a=imot_v_per_a,
                imot_zero_v=imot_zero_v, adc_dual=bool(adc_dual), have_rt=bool(have_rt),
                zero_measured=bool(zero_measured), firmware=firmware.split(b"\0")[0].decode(errors="replace"))


class EdgeRate:
    """Pulse frequency from the running edge count + latest edge time in each record."""

    def __init__(self):
        self.count = None
        self.last_edge = None   # absolute µs
        self.hz = 0.0

    def update(self, count, edge_us, t_us, t_abs):
        if self.count is None:
            self.count = count
            return 0.0
        new = (count - self.count) & 0xFFFF
        self.count = count
        if new:
            edge_abs = t_abs - ((t_us - edge_us) & 0xFFFFFFFF)
            if self.last_edge is not None and edge_abs > self.last_edge:
                self.hz = 1e6 * new / (edge_abs - self.last_edge)
            self.last_edge = edge_abs
        elif self.last_edge is not None:
            waited = t_abs - self.last_edge
            if waited > 2e6:
                self.hz = 0.0
            elif waited > 0:
                self.hz = min(self.hz, 1e6 / waited)   # no edge yet: the period is at least this long
        return self.hz


def decode(path, out_csv, every, vsp_ppkm, nep_ppr):
    blob = Path(path).read_bytes()
    if len(blob) < HEADER_SIZE:
        sys.exit("file shorter than its header")
    h = read_header(blob)
    n = (len(blob) - HEADER_SIZE) // RECORD_SIZE
    tail = (len(blob) - HEADER_SIZE) % RECORD_SIZE
    records = struct.iter_unpack(RECORD_FMT, memoryview(blob)[HEADER_SIZE:HEADER_SIZE + n * RECORD_SIZE])
    records = list(records)
    if not records:
        sys.exit("no records in file")

    # Current-sensor zero: measured by the logger with the relay open, else estimate it the same way
    zero_v, zero_src = h["imot_zero_v"], "measured at key-on"
    if not h["zero_measured"]:
        quiet = sorted(r[7] * h["v_imot"] for r in records
                       if r[12] & DIO_IG1 and not (h["have_rt"] and r[12] & DIO_RT_ON))
        if quiet:
            zero_v, zero_src = quiet[len(quiet) // 2], "estimated from relay-open samples"
        else:
            zero_src = "nominal (no relay-open samples)"

    first_offset_s = ((records[0][0] - h["start_us"]) & 0xFFFFFFFF) / 1e6
    vsp, nep = EdgeRate(), EdgeRate()
    t_abs, prev_t, prev_seq = 0, None, None
    gaps = missing = 0
    stats = {k: [float("inf"), float("-inf"), 0.0] for k in ("vt3", "vt6", "vcc2", "vbat", "imot")}
    diag_ms = [0] * 16
    lamp_ms = 0
    dt_ms = 1000.0 / h["sample_hz"]

    writer = None
    fh = None
    if out_csv:
        fh = open(out_csv, "w", newline="")
        writer = csv.writer(fh)
        cols = ["time_s"] + (["unix_time"] if h["start_unix"] else []) + [
            "vt3_V", "vt6_V", "vt_avg_V", "vt_diff_V", "vcc2_V", "vbat_V", "imot_A", "vsp_Hz", "nep_Hz"]
        cols += (["speed_kmh"] if vsp_ppkm else []) + (["rpm"] if nep_ppr else [])
        cols += ["ig1", "lamp_on", "relay_on", "diag_hex"]
        writer.writerow(cols)

    for i, r in enumerate(records):
        t_us, vsp_edge, nep_edge, c_vt3, c_vt6, c_vcc2, c_vbat, c_imot, vsp_n, nep_n, diag, dropped, dio, seq = r
        if prev_t is not None:
            t_abs += (t_us - prev_t) & 0xFFFFFFFF
            if seq != ((prev_seq + 1) & 0xFF):
                gaps += 1
                missing += ((seq - prev_seq - 1) & 0xFF) or 256
        prev_t, prev_seq = t_us, seq

        vt3, vt6 = c_vt3 * h["v_vt"], c_vt6 * h["v_vt"]
        vcc2, vbat = c_vcc2 * h["v_vt"], c_vbat * h["v_vbat"]
        imot = (c_imot * h["v_imot"] - zero_v) / h["imot_v_per_a"]
        vsp_hz = vsp.update(vsp_n, vsp_edge, t_us, t_abs)
        nep_hz = nep.update(nep_n, nep_edge, t_us, t_abs)

        for key, val in (("vt3", vt3), ("vt6", vt6), ("vcc2", vcc2), ("vbat", vbat), ("imot", imot)):
            s = stats[key]
            s[0] = min(s[0], val); s[1] = max(s[1], val); s[2] += val
        for b in range(16):
            if diag & (1 << b):
                diag_ms[b] += dt_ms
        if dio & DIO_WLP_ON:
            lamp_ms += dt_ms

        if writer and i % every == 0:
            time_s = t_abs / 1e6
            row = [f"{time_s:.4f}"]
            if h["start_unix"]:
                row.append(f"{h['start_unix'] + first_offset_s + time_s:.3f}")
            row += [f"{vt3:.4f}", f"{vt6:.4f}", f"{(vt3 + vt6) / 2:.4f}", f"{vt3 - vt6:.4f}",
                    f"{vcc2:.3f}", f"{vbat:.3f}", f"{imot:.2f}", f"{vsp_hz:.2f}", f"{nep_hz:.2f}"]
            if vsp_ppkm:
                row.append(f"{vsp_hz * 3600.0 / vsp_ppkm:.2f}")
            if nep_ppr:
                row.append(f"{nep_hz * 60.0 / nep_ppr:.0f}")
            row += [int(bool(dio & DIO_IG1)), int(bool(dio & DIO_WLP_ON)), int(bool(dio & DIO_RT_ON)), f"{diag:04x}"]
            writer.writerow(row)
    if fh:
        fh.close()

    # ---- summary
    dur = t_abs / 1e6
    print(f"file:       {path}  ({h['firmware']})")
    if h["start_unix"]:
        start = dt.datetime.fromtimestamp(h["start_unix"] + first_offset_s, dt.timezone.utc)
        print(f"start:      {start:%Y-%m-%d %H:%M:%S} UTC (logger clock)")
    print(f"duration:   {dur:.1f} s, {n} records at {h['sample_hz']} Hz"
          + (f", {tail} trailing bytes ignored" if tail else ""))
    print(f"gaps:       {gaps} (about {missing} records), buffer drops at close: {records[-1][11]}")
    print(f"ADC mode:   {'VT3/VT6 simultaneous' if h['adc_dual'] else 'sequential'}"
          f"; current zero {zero_v:.4f} V ({zero_src})")
    print(f"EPS lamp:   ON for {lamp_ms / 1000:.1f} s")
    print("channel     min        max        mean")
    for key, unit in (("vt3", "V"), ("vt6", "V"), ("vcc2", "V"), ("vbat", "V"), ("imot", "A")):
        lo, hi, total = stats[key]
        print(f"  {key:<8}{lo:>9.3f}{hi:>11.3f}{total / n:>11.3f} {unit}")
    seen = [(DIAG_NAMES[b], diag_ms[b] / 1000) for b in range(16) if diag_ms[b]]
    print("diag flags: " + (", ".join(f"{name} {secs:.1f} s" for name, secs in seen) if seen else "none"))
    if out_csv:
        print(f"csv:        {out_csv}" + (f" (every {every}th record)" if every > 1 else ""))


def main():
    ap = argparse.ArgumentParser(description="Decode EPS data logger .BIN files")
    ap.add_argument("file")
    ap.add_argument("-o", "--out", help="CSV path (default: next to the .BIN)")
    ap.add_argument("--every", type=int, default=1, help="write every Nth record (10 → 100 Hz)")
    ap.add_argument("--vsp-ppkm", type=float, help="VSP pulses per km → adds speed_kmh")
    ap.add_argument("--nep-ppr", type=float, help="NEP pulses per engine rev → adds rpm")
    ap.add_argument("--summary-only", action="store_true")
    a = ap.parse_args()
    out = None if a.summary_only else (a.out or str(Path(a.file).with_suffix(".csv")))
    decode(a.file, out, max(1, a.every), a.vsp_ppkm, a.nep_ppr)


if __name__ == "__main__":
    main()
