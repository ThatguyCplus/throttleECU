# EPS bench captures, 2026-11-15

> **Disclaimer: for research and bench testing only.** This is experimental work on a removed
> EPS unit on a test bench. It is not for use on a road vehicle, not safety-validated, and not
> a product. Anyone who uses it does so at their own risk.

Recorded with Scope1008 (Hantek 1008B, roll mode) and the eps_bench_nep Feather on COM11
(firmware a7af0e3: every CAN frame, rpm every second). NEP 900 rpm (22.5 Hz) during each run,
so the EPS assist was on.

Hantek channels: CH1 VT3, CH2 VT6, CH3 NEP node, CH4 Feather D13 sync. Feather: SAS angle
(CAN 0x25), NEP rpm, all CAN frames. Clocks aligned with the D13 sync pulses.

| Folder | Length | What was done | Alignment |
|---|---|---|---|
| 20261115_221920 | 30 s | first wheel turns (0-3 s and 11-17 s) | 57 edges, 0.48 ms |
| 20261115_222842 | 40 s | terminal-guided steps; moves lag the steps by ~1 step | 71 edges, 0.41 ms |
| 20261115_224026 | 60 s | free steering, slow and fast, holds, left end stop | 105 edges, 0.98 ms |
| 20261115_224602 | 60 s | lock to lock, pushing into both end stops | 103 edges, 0.44 ms |

## Files in each folder

| File | Contents |
|---|---|
| `merged.csv` | scope samples (800 Sa/s per channel) + `sas_deg`, `sas_dps`, `sas_zeroed_deg`, `nep_rpm` |
| `scope.csv` | raw Hantek roll recording |
| `can.csv` | every CAN frame, time on the scope axis |
| `feather.log` | every Feather line with PC time (feather_log.py format) |
| `analysis.md`, `*.png` | findings and plots (224026, 224602; 221920 has a plot) |

## Main findings

- VT3 and VT6 move together: VT6 = 1.003 x VT3 + 0.01 V (r = 0.9998), VT6 about 18-20 mV higher.
- Rest about 2.55-2.56 V. Noise about 6 mV rms.
- Turning (assist on): about +0.2 to +0.3 V right, -0.35 to -0.4 V left, nearly independent of
  angle and only slightly of speed. Signal leads steering rate by about 50 ms.
- Pushing into the end stops: -1.35 V (left, -596 deg) and +1.24 V (right, +616 deg); flattens at
  about 1.2 V and 3.8 V. Lock to lock is 1212 deg.
- On CAN only the SAS (0x25, 84/s) changes; 0x423 and 0x4C6 are constant at 1/s.
