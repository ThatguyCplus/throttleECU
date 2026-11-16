"""Merge a Scope1008 roll recording (Hantek 1008B) with a feather_log.py file.

The Feather toggles D13 at pseudo-random 200-1000 ms intervals and logs the exact
micros() of every edge. One Hantek channel records D13. This script finds those
edges in the scope CSV, matches the interval pattern against the Feather's list,
fits  scope_time = a * feather_time + b  (offset plus clock drift), and writes the
scope CSV with the Feather's data added as extra columns on the scope's time axis:

    sas_deg, sas_dps, sas_zeroed_deg   steering angle sensor (blank if stale > 0.25 s)
    nep_rpm                            NEP command the Feather was sending

    python merge_hantek_feather.py scope.csv bench_feather.log merged.csv --sync CH4

Needs numpy.
"""
import argparse
import csv
import sys

import numpy as np

SAS_STALE_S = 0.25


def read_scope(path):
    with open(path, encoding="utf-8") as f:
        rows = [r for r in csv.reader(line for line in f if not line.startswith("#"))]
    header, body = rows[0], rows[1:]
    data = np.array([[float(x) if x not in ("", "nan") else np.nan for x in r] for r in body if r])
    return header, data


def scope_edges(t, v, lo, hi):
    """Edge times (interpolated) and directions, with hysteresis."""
    times, dirs = [], []
    state = v[0] > (lo + hi) / 2
    for i in range(1, len(v)):
        if not state and v[i] > hi:
            mid = (lo + hi) / 2
            j = i
            while j > 0 and v[j - 1] > mid:
                j -= 1
            j = max(j, 1)
            a, b = v[j - 1], v[j]
            frac = (mid - a) / (b - a) if b != a else 0.0
            times.append(t[j - 1] + frac * (t[j] - t[j - 1]))
            dirs.append(1)
            state = True
        elif state and v[i] < lo:
            mid = (lo + hi) / 2
            j = i
            while j > 0 and v[j - 1] < mid:
                j -= 1
            j = max(j, 1)
            a, b = v[j - 1], v[j]
            frac = (mid - a) / (b - a) if b != a else 0.0
            times.append(t[j - 1] + frac * (t[j] - t[j - 1]))
            dirs.append(0)
            state = False
    return np.array(times), np.array(dirs)


def read_feather(path):
    sync, sas, nep = [], [], []
    wraps, prev = 0, None
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#") or "\t" not in line:
                continue
            _, msg = line.rstrip("\n").split("\t", 1)
            parts = msg.split(",")
            if parts[0] not in ("S", "A", "N") or len(parts) < 3:
                continue
            try:
                us = int(parts[1])
            except ValueError:
                continue
            # micros() wraps every 2^32 us (71.6 min); a big backwards jump is a wrap
            if prev is not None and prev - us > 2**31:
                wraps += 1
            prev = us
            t = (us + wraps * 2**32) / 1e6
            if parts[0] == "S":
                sync.append((t, int(parts[2])))
            elif parts[0] == "A" and len(parts) >= 5:
                sas.append((t, float(parts[2]), float(parts[3]), float(parts[4])))
            elif parts[0] == "N":
                nep.append((t, float(parts[2])))
    return np.array(sync).reshape(-1, 2), np.array(sas).reshape(-1, 4), np.array(nep).reshape(-1, 2)


def match(ts, ds, tf, df, min_overlap=6):
    """Best index shift s (scope edge i <-> feather edge i + s) by interval pattern."""
    dts, dtf = np.diff(ts), np.diff(tf)
    best = None
    for s in range(-(len(ts) - min_overlap), len(tf) - min_overlap + 1):
        i0, i1 = max(0, -s), min(len(ts), len(tf) - s)
        if i1 - i0 < min_overlap:
            continue
        err = np.abs(dts[i0:i1 - 1] - dtf[i0 + s:i1 - 1 + s])
        score = np.median(err) + 0.1 * np.mean(ds[i0:i1] != df[i0 + s:i1 + s])
        if best is None or score < best[0]:
            best = (score, s, i0, i1)
    return best


def hold(t_query, t_src, values, max_age=None):
    """Zero-order hold: last value at or before each query time."""
    idx = np.searchsorted(t_src, t_query, side="right") - 1
    out = np.full((len(t_query),) + values.shape[1:], np.nan)
    ok = idx >= 0
    if max_age is not None:
        ok &= (t_query - t_src[np.clip(idx, 0, None)]) <= max_age
    out[ok] = values[idx[ok]]
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scope_csv")
    ap.add_argument("feather_log")
    ap.add_argument("out_csv")
    ap.add_argument("--sync", default="CH4", help="scope channel wired to Feather D13 (default CH4)")
    ap.add_argument("--lo", type=float, default=1.0, help="sync low threshold, V")
    ap.add_argument("--hi", type=float, default=2.3, help="sync high threshold, V")
    args = ap.parse_args()

    header, data = read_scope(args.scope_csv)
    col = next((i for i, h in enumerate(header) if h.upper().startswith(args.sync.upper())), None)
    if col is None:
        print(f"no column starting with {args.sync} in {header}")
        return 1
    t_scope = data[:, 0]
    ts, ds = scope_edges(t_scope, data[:, col], args.lo, args.hi)
    sync, sas, nep = read_feather(args.feather_log)
    print(f"scope: {len(t_scope)} samples, {len(ts)} sync edges | feather: {len(sync)} sync, "
          f"{len(sas)} SAS, {len(nep)} NEP lines")
    if len(ts) < 6 or len(sync) < 6:
        print("need at least 6 sync edges in both files; is the sync channel right and wired to D13?")
        return 1

    best = match(ts, ds, sync[:, 0], sync[:, 1].astype(int))
    if best is None:
        print("the two files don't overlap by 6 sync edges")
        return 1
    score, s, i0, i1 = best
    a, b = np.polyfit(sync[i0 + s:i1 + s, 0], ts[i0:i1], 1)
    resid = ts[i0:i1] - (a * sync[i0 + s:i1 + s, 0] + b)
    print(f"matched {i1 - i0} edges, interval error {score * 1e3:.2f} ms, fit residual "
          f"{np.sqrt(np.mean(resid ** 2)) * 1e3:.2f} ms rms, clock ratio {a:.6f}")
    if score > 0.02:
        print("WARNING: poor match (> 20 ms). Check the sync channel and that both logs cover the same time.")

    t_sas = a * sas[:, 0] + b if len(sas) else np.array([])
    t_nep = a * nep[:, 0] + b if len(nep) else np.array([])
    sas_cols = hold(t_scope, t_sas, sas[:, 1:], SAS_STALE_S) if len(sas) else np.full((len(t_scope), 3), np.nan)
    nep_col = hold(t_scope, t_nep, nep[:, 1:]) if len(nep) else np.full((len(t_scope), 1), np.nan)

    with open(args.out_csv, "w", newline="", encoding="utf-8") as f:
        f.write(f"# merged {args.scope_csv} + {args.feather_log}; feather->scope a={a:.9f} b={b:.6f}\n")
        w = csv.writer(f)
        w.writerow(header + ["sas_deg", "sas_dps", "sas_zeroed_deg", "nep_rpm"])
        for row, sc, nc in zip(data, sas_cols, nep_col):
            w.writerow([f"{x:.6g}" if not np.isnan(x) else "" for x in row]
                       + ["" if np.isnan(x) else f"{x:.1f}" for x in sc]
                       + ["" if np.isnan(nc[0]) else f"{nc[0]:.0f}"])
    print(f"wrote {args.out_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
