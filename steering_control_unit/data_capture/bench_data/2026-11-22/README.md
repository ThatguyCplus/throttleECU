# EPS bench captures, 2026-11-22

> **Disclaimer: for research and bench testing only.** This is experimental work on a removed
> EPS unit on a test bench. It is not for use on a road vehicle, not safety-validated, and not
> a product. Anyone who uses it does so at their own risk.

Recorded with Scope1008 (Hantek 1008B, roll mode) and the eps_bench_steer Feather on COM11.
The Feather drives VT3/VT6 from its DACs; the real torque sensor is disconnected. NEP is at
900 rpm and sensor-power emulation is ON.

| Folder | Length | What was done | Alignment |
|---|---|---|---|
| 20261122_013517 | 177 s | angle-hold steps; small steps good, large steps run away | 284 edges, 1.52 ms |

Findings and recommended fixes are in `20261122_013517/analysis.md`. The file layout matches
`../2026-11-15/README.md`; `feather.log` also has `V` lines (DAC outputs) and `P` lines (angle loop).
