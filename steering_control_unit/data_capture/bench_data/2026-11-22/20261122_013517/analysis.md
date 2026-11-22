# Angle-hold run 2026-11-22 01:35: steps, overshoot and runaways

> **Disclaimer: for research and bench testing only.** This is experimental work on a removed
> EPS unit on a test bench. It is not for use on a road vehicle, not safety-validated, and not
> a product. Anyone who uses it does so at their own risk.

## Setup

- **Firmware:** eps_bench_steer (uncommitted), DAC-driven VT3/VT6, real torque sensor disconnected.
- **Angle loop settings:**
  - Kp 0.004 V/deg, Ki 0.002, Kd 0.0005 V per deg/s
  - Dead-band jump: R +0.20 V, L -0.26 V
  - D limit ±0.45 V, hold window ±2°
- **Aborts:** at more than 300 deg/s, or past 135°.
- **HV mode:** not used.
- **Engine and sensor:** NEP 900 rpm, sensor emulation ON for the whole run.
- **Length:** 177 s.
- **Alignment:** 284 sync edges, 1.52 ms rms.
- **Commands:** 39 angle commands, 14 automatic aborts.

The step metrics come from the Feather `P` lines (target, angle, D) and the `A` lines (angle, rate). The output D is from the `V` lines.

## What happened

### 1. Small steps (about 25° or less) work

Examples: 0 → 20 → 45 → 20 → 0 at 99–117 s, and 20 ↔ 45 at 58–66 s.

- **Speed:** 90 % of the step is reached in 0.25–0.45 s.
- **Overshoot:** 1–2° at most.
- **Settling:** the wheel stops inside the ±2° window and friction holds it with D = 0.
- **Peak D:** 0.27–0.31 V.
- **Peak rate:** 130–240 deg/s.

The plot is `angle_good_steps.png`.

### 2. Large steps (about 40° or more) run away and overshoot massively

There were 10 cases: 43 → 0, 88 → 20, -43 → +20, 2 → 45, -112 → +45 and others. In each one:

1. The PID asks for D = 0.42–0.45 V: dead band plus Kp × 40–60°.
2. Above about **0.35 V** the assist is far past breakaway. The wheel accelerates to **340–660 deg/s** within 0.2 s.
3. The 300 deg/s guard aborts angle mode, which is correct.
4. **The bug:** after the abort, D is *slewed* back to 0 at the manual slew rate of **0.5 V/s**. Going from 0.45 V to 0 takes about **0.9 s**, and assist stays on the whole time.
   - The wheel keeps spinning, passes the target and ends far on the other side:
     - -112° → +206°
     - -96° → +228°
     - +86° → -112°
     - +38° → -110°
   - 0.5 s after the abort, D was still ±0.07 to ±0.22 V.
   - `angle_abort_runaway.png` shows the linear ramp-down.
5. Re-engaging from beyond 135° is refused at once ("angle past 135 deg"), which is correct.

Without the slow ramp-down, the overshoot would have been much smaller. The guard fires 0.1–0.3 s into the move, at roughly 15–40° of travel.

### 3. The assist is a cliff, not a slope

Wheel speed versus the output D, all samples while D was applied (`angle_d_vs_rate.png`):

| \|D\| (V) | median rate toward D (deg/s) | 90th percentile (deg/s) |
|---|---|---|
| 0.15–0.20 | 0 | 183 |
| 0.20–0.25 | 1 | 29 |
| 0.25–0.30 | 1 | 91 |
| 0.30–0.35 | 3 | 125 |
| 0.35–0.40 | **225** | **625** |
| 0.40–0.45 | **400** | **579** |

What the table shows:

- **0.20–0.35 V:** the rack mostly sticks, then slips (stick-slip).
- **Above about 0.35 V:** it runs at hundreds of deg/s.

A proportional term that adds 0.1–0.2 V on large errors pushes D straight off the cliff. The unloaded bench rack (no tyres, no load) makes this worse.

### 4. Left is still harder

- At 98 s: target 0 with the wheel at +2.7°. D held at -0.27 V for 1.2 s and the wheel did not move.
- At 113–115 s: the wheel crept from 28° to 22° at -0.30 V before breaking away.
- The integrator (Ki 0.002) adds only about 0.005 V/s at a 2–3° error, which is too slow to help.

### 5. Ramp of small steps (15 → 90° in 6–9° steps, 22–54 s)

- Each step stopped 2–4° short. The wheel stops as soon as it enters the ±2° window, sometimes just outside it, then stalls.
- There were no runaways, because D stayed at 0.23–0.27 V.

## Recommended firmware changes

| # | Change | Why |
|---|---|---|
| 1 | **On any abort (rate, angle, stale SAS), set dOut = 0 at once.** Do not slew it. | This alone removes the runaways to ±200°. |
| 2 | **Lower the angle D limit to about 0.33 V** (`pmax 0.33`). | Stays below the 0.35 V cliff. HV mode remains the deliberate way to go higher. |
| 3 | **Brake on speed:** raise Kd to about 0.001–0.0015. Optionally, set u = 0 when moving toward the target faster than about 150 deg/s. | Cuts D before the cliff speed. 0.0005 × 300 deg/s is only 0.15 V. |
| 4 | **Left dead band 0.26 → 0.30 V** (`pdzl 0.30`). | Measured left creep and breakaway is around 0.30 V. |
| 5 | Optional: ramp the target in software (for example 60 deg/s) instead of stepping it. | A large step then becomes a series of small ones, which work. |

**Status:** changes 1-4 are now the eps_bench_steer defaults: abort drops D at once, `pmax 0.33`,
Kd 0.0012, a speed brake above 150 deg/s (`pbrk`), and `pdzl 0.30`. They are not yet tested on the rack.

## Files

| File | Contents |
|---|---|
| `feather.log` | every Feather line, including `P,us,target,angle,err,D` per SAS frame while holding |
| `merged.csv` | scope VT3/VT6/NEP/sync with SAS, rpm and commanded vt3/vt6/D |
| `scope.csv`, `can.csv`, `session.txt` | raw roll recording, CAN frames, session info |
| `angle_run_overview.png` | whole run: angle and target, D, rate; aborts in magenta |
| `angle_good_steps.png` | 97–117 s, the steps that worked |
| `angle_abort_runaway.png` | 73–92 s, aborts with the slow D ramp-down and the run to +206° |
| `angle_d_vs_rate.png` | wheel speed versus \|D\| |
