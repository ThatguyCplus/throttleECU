"""Design numbers for Icarus from Scope1008 + Feather bench captures.

Reads the merged.csv of every session folder under one bench_data/<date> folder
(columns: time_s, CH1_VT3_V, CH2_VT6_V, CH3_NEP_V, CH4_SYNC_V, sas_deg, sas_dps, ...)
and prints the numbers Icarus docs/14_EPS_DATA_GAME_PLAN.md is built on:

  - how well VT3 and VT6 track each other (gain, offset, low-passed difference)
  - rest level per hands-off segment, noise, slew rates, signal extremes
  - the NEP frequency really sent (scope axis corrected with the merge clock ratio)
  - an assist-current proxy: the NEP node idles at the 12 V supply through a pull-up,
    so its high level follows the supply. The supply sags when the EPS motor draws
    current. Sag against torque signal gives the shape of the assist curve.

    python bench_design_numbers.py ../bench_data/2026-11-15

The sag proxy is relative (percent of the level at rest), not amps. It only works
while NEP is pulsing and its pull-up shares the EPS supply. Needs numpy.
"""
import glob
import os
import re
import sys

import numpy as np

STILL_DPS = 1.0          # |steering rate| below this counts as "wheel still"
REST_BAND_V = 0.10       # still segments this close to the session's median are "rest"


def load(path):
    with open(path, encoding="utf-8") as f:
        head = f.readline()
    d = np.genfromtxt(path, delimiter=",", skip_header=2)
    m = re.search(r"a=([0-9.]+)", head)
    return dict(t=d[:, 0], v3=d[:, 1], v6=d[:, 2], nep=d[:, 3], sas=d[:, 5], dps=d[:, 6],
                clock=float(m.group(1)) if m else 1.0)


def smooth(x, fs, fc):
    """Zero-phase boxcar low-pass, about fc Hz (-3 dB), edges held."""
    n = max(1, int(round(0.443 * fs / fc)) | 1)
    pad = n // 2
    return np.convolve(np.pad(x, pad, mode="edge"), np.ones(n) / n, mode="valid")


def runs(mask, minlen):
    edges = np.flatnonzero(np.diff(np.concatenate(([0], mask.astype(int), [0]))))
    return [(a, b) for a, b in zip(edges[::2], edges[1::2]) if b - a >= minlen]


def rms_mv(x):
    return 1000 * float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def rank(x):
    order = np.argsort(x)
    r = np.empty(len(x))
    r[order] = np.arange(len(x))
    return r


def median5(x):
    return np.median(np.lib.stride_tricks.sliding_window_view(x, 5), axis=1)


def nep_high(t, nep):
    """One (time, level, start, end) per NEP high half-cycle. Level = median of the settled samples."""
    hi = nep > 5.0
    cut = np.flatnonzero(np.diff(hi.astype(int)) != 0) + 1
    bounds = np.concatenate(([0], cut, [len(hi)]))
    out = [(t[b - 4], np.median(nep[a + 3:b - 2]), a, b)
           for a, b in zip(bounds[:-1], bounds[1:]) if hi[a] and b - a >= 8]
    return hi, out


def session(folder):
    s = load(os.path.join(folder, "merged.csv"))
    t, v3, v6 = s["t"], s["v3"], s["v6"]
    fs = 1.0 / np.median(np.diff(t))
    mean = (v3 + v6) / 2
    name = os.path.basename(folder)
    print(f"\n== {name}: {t[-1]:.1f} s at {fs:.0f} Sa/s, scope/Feather clock ratio {s['clock']:.6f}")

    # --- NEP actually sent
    hi, halves = nep_high(t, s["nep"])
    rise = np.flatnonzero(hi[1:] & ~hi[:-1])
    if len(rise) > 3:
        f_scope = (len(rise) - 1) / (t[rise[-1]] - t[rise[0]])
        f_true = f_scope * s["clock"]
        print(f"   NEP: {f_true:.2f} Hz on the Feather clock = {f_true * 60 / 1.5:.0f} rpm "
              f"({f_scope:.2f} Hz on the scope axis)")

    # --- VT3 against VT6
    gain, offs = np.polyfit(v3, v6, 1)
    diff = smooth(v6 - v3, fs, 20.0)
    print(f"   VT6 = {gain:.4f} x VT3 {offs * 1000:+.0f} mV, r = {np.corrcoef(v3, v6)[0, 1]:.4f}")
    print(f"   VT6 - VT3 (20 Hz low-pass): mean {diff.mean() * 1000:.1f} mV, "
          f"stays within {(diff.min() - diff.mean()) * 1000:+.0f} / {(diff.max() - diff.mean()) * 1000:+.0f} mV of it")

    # --- rest segments
    still = np.abs(np.nan_to_num(s["dps"], nan=999.0)) < STILL_DPS
    quiet = still & (np.abs(np.gradient(smooth(mean, fs, 3.0), t)) < 0.08)
    rest_mask = np.zeros(len(t), bool)
    rests = []
    centre = np.median(mean[quiet]) if quiet.any() else np.median(mean)
    for a, b in runs(quiet, int(0.6 * fs)):
        a, b = a + int(0.15 * fs), b - int(0.05 * fs)
        if b - a >= int(0.3 * fs) and abs(np.mean(mean[a:b]) - centre) < REST_BAND_V:
            rests.append(float(np.mean(mean[a:b])))
            rest_mask[a:b] = True
    rest = float(np.median(rests)) if rests else float(centre)
    if rests:
        print(f"   rest (wheel still, {len(rests)} segments): {min(rests):.3f} to {max(rests):.3f} V, median {rest:.3f} V")
        print(f"   noise above 20 Hz at rest: VT3 {rms_mv((v3 - smooth(v3, fs, 20.0))[rest_mask]):.1f} mV rms, "
              f"VT6 {rms_mv((v6 - smooth(v6, fs, 20.0))[rest_mask]):.1f} mV rms")

    # --- slew and extremes
    slew = np.abs(np.gradient(smooth(mean, fs, 10.0), t))
    if (~still).sum() > 100:
        mv = slew[~still]
        print(f"   slew while steering (10 Hz low-pass): 95 % below {np.percentile(mv, 95):.1f} V/s, "
              f"99 % below {np.percentile(mv, 99):.1f} V/s, max {slew.max():.1f} V/s")
    f3, f6 = median5(v3), median5(v6)
    print(f"   extremes (5-sample median): VT3 {f3.min():.2f} to {f3.max():.2f} V, VT6 {f6.min():.2f} to {f6.max():.2f} V")

    # --- supply-sag proxy
    rows = []
    if len(halves) > 50:
        tc = np.array([h[0] for h in halves])
        lev = np.array([h[1] for h in halves])
        val = np.array([np.mean(mean[h[2]:h[3]]) for h in halves])
        at_rest = np.array([rest_mask[h[2]:h[3]].all() for h in halves])
        if at_rest.sum() > 10:
            lev0 = np.median(lev[at_rest])
            note = f"scatter {np.std(100 * (1 - lev[at_rest] / lev0)):.2f} %"
        else:                       # no hands-off stretch with NEP running: take the unloaded level
            lev0 = np.percentile(lev, 90)
            note = "no rest segment with NEP on, 90th percentile used"
        if lev0 > 5:
            sag = 100 * (1 - lev / lev0)
            print(f"   NEP high level unloaded {lev0:.2f} V ({note}), largest sag {sag.max():.1f} %")
            rows = list(zip(val, sag))
            # how quickly the sag follows the torque signal
            if sag.max() > 5:
                dev = smooth(np.abs(mean - rest), fs, 40.0)
                scan = []
                for lag_ms in range(-100, 201, 10):
                    x = np.interp(tc - lag_ms / 1000.0, t, dev)
                    sel = (x > 0.25) | (sag > 0.5)
                    scan.append((np.corrcoef(rank(x[sel]), rank(sag[sel]))[0, 1], lag_ms))
                best = max(scan)
                print(f"   sag follows the torque signal best at {best[1]:+d} ms lag "
                      f"(rank correlation {best[0]:.3f}; one sample per {1000 / 22:.0f} ms, so +/- 25 ms at best)")
    return rows, rests


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    folders = sorted(f for f in glob.glob(os.path.join(sys.argv[1], "*"))
                     if os.path.isfile(os.path.join(f, "merged.csv")))
    if not folders:
        sys.exit(f"no session folders with merged.csv under {sys.argv[1]}")
    pooled, all_rests = [], []
    for folder in folders:
        rows, rests = session(folder)
        pooled += rows
        all_rests += rests
    print(f"\n== all {len(folders)} sessions")
    if all_rests:
        print(f"   rest level over {len(all_rests)} wheel-still segments: {min(all_rests):.3f} to {max(all_rests):.3f} V "
              f"(median {np.median(all_rests):.3f} V)")
    if not pooled:
        return
    p = np.array(pooled)
    val, sag = p[:, 0], p[:, 1]
    full = np.percentile(sag, 99)
    if full < 5:
        print("   supply sag never passed 5 %: no assist curve from these sessions")
        return
    # centre of the assist curve: the voltage about which sag is most nearly a function of |V - c|
    cand = np.arange(2.40, 2.6501, 0.005)
    busy = sag > 0.5
    score = np.array([np.corrcoef(rank(np.abs(val[busy] - c)), rank(sag[busy]))[0, 1] for c in cand])
    c = float(cand[int(np.argmax(score))])
    ok = cand[score > score.max() - 0.01]
    print(f"   assist curve is most symmetric about {c:.3f} V (range {ok.min():.3f} to {ok.max():.3f} V)")
    print(f"   supply sag saturates at {full:.1f} % of the rest level; below, 100 % = that saturation")
    print("   |VT mean - centre|      n    sag %   share of saturation")
    edges = [0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.60, 0.70, 0.85, 1.00, 1.35]
    dev = np.abs(val - c)
    for lo, hi in zip(edges[:-1], edges[1:]):
        k = (dev >= lo) & (dev < hi)
        if k.sum() >= 5:
            print(f"   {lo:4.2f} to {hi:4.2f} V   {k.sum():5d}   {np.median(sag[k]):6.2f}   {100 * np.median(sag[k]) / full:5.1f} %")


if __name__ == "__main__":
    main()
