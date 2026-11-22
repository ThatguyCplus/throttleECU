# eps_bench_steer: steer the EPS from the Feather DACs (bench MVP)

> **Disclaimer: for research and bench testing only.** This is experimental work on a removed
> EPS unit on a test bench. It is not for use on a road vehicle, not safety-validated, and not
> a product. Anyone who uses it does so at their own risk.

The Feather M4 CAN replaces the torque sensor's VT3 and VT6 outputs with its two DACs, so the EPS
motor can be commanded to steer. Everything from [eps_bench_nep](../eps_bench_nep/README.md) still
runs: fake engine speed (NEP), steering angle sensor over CAN, full CAN dump, and the D13 sync
pulse for the Hantek.

**Why a separate sketch:** the wiring is different. This sketch drives A0/A1 into the EPS all the
time, so on a bench where the sensor is still connected it would fight the real sensor through
the 1 kΩ resistors. The other way round, eps_bench_nep on a cut-wire bench leaves VT3/VT6
floating, which latches DTCs. Two sketches make it obvious which firmware matches which wiring.

## How the outputs are driven

The 2026-11-15 captures showed VT3 and VT6 move **in phase with the same gain** (VT6 = 1.003 × VT3
+ 10–20 mV). So both DACs are driven as:

    VT3 = rest3 + D        VT6 = rest6 + D

| Limit | Default | Where |
|---|---|---|
| rest3 / rest6 | 2.540 V / 2.557 V (real sensor at the pins, engine running) | `rest3 2.540`, `rest6 2.557` (serial) |
| D | clamped to ±0.30 V, **only while NEP runs and the sensor is "powered"** | `d0.05` |
| pin correction | pin = 0.893 × DAC + 0.346 / 0.376 V (ECU loading, key ON) | `cal a b3 b6`, `cal off` |
| each output | clamped to 1.90–3.20 V (1.40–3.70 V in HV mode) | compile-time `OUT_MIN` / `OUT_MAX` |
| slew | 0.5 V/s (0.05–5) | `s0.5` |
| DAC reference | 3.210 V (calibrated, see below) | `vref 3.210` |

On the bench, turning the rack took about +0.2..+0.3 V (right) and −0.35..−0.4 V (left) of sensor
signal; the torsion bar stops at about ±1.3 V. The ±0.30 V clamp keeps commands inside the range
a hand produces.

### Sensor power (copies the ECU)

Logging with the real sensor (2026-11-21, Scope1008 session `20261121_230808`) showed the ECU only
powers the torque sensor once the engine runs:

| State | Real VT3 / VT6 |
|---|---|
| Key off, IG1, IG2 with the engine stopped | **0 V / 0 V** |
| 0.72 s after the first NEP pulse | VT3 steps to 2.540 V; VT6 to 2.557 V 0.14 s later |
| Engine stops, key still on | stays at rest |
| Key back to IG1 | 0 V (VT6 about 40 ms before VT3) |

The firmware does the same: 0 V at boot, rest 0.72 / 0.86 s after NEP starts, and **`k` (key
off) drops both to 0 V**. The Feather can't see the key, so send `k` (Bench window: **I turned the
key OFF**) every time the key goes back to IG1 / OFF, **after** turning it, never before. Earlier builds held rest the whole time, including
key ON with the engine stopped: a state the real sensor never shows, and the likely cause of the
DTC 16 that latched on 2026-11-16.

Measured on the Hantek at B10/B8 with the key ON (2026-11-21): 0 V before NEP; rest 2.544 /
2.557 V; VT3 up 0.70 s after NEP, VT6 0.12 s later; 0 V after `k`, VT6 first.

All voltages are **at the EPS pin**. With the key ON the ECU pulls the line up through the 1 kΩ
(rest 2.560 commanded read 2.648 V), so the DAC is driven at (target − b) / a. With the key OFF
the pin reads about 0.1 V lower than the target; nothing reads it then.

### Angle hold (`a<deg>`)

A PID on the SAS angle (every CAN 0x25 frame, ~84 Hz) sets D to steer to a target angle measured
from the SAS zero (`z`). + = right for both angle and D.

    D = dead band (in the error's direction) + Kp·err + Ki·∫err − Kd·rate,   0 inside ±hold°

| Setting | Default | Command |
|---|---|---|
| Kp / Ki / Kd | 0.004 V/° / 0.002 V/(°·s) / 0.0012 V/(°/s) | `pk` `pi` `pd` |
| dead band right / left | 0.20 / 0.30 V | `pdzr` `pdzl` (`pdz` sets both) |
| hold window | ±2° (D = 0, integral reset) | `ptol` |
| D limit in angle mode | ±0.33 V (at most 0.55 without HV mode) | `pmax` |
| speed brake | D = 0 while moving toward the target faster than 150°/s | `pbrk` |
| target | ±90° | `a20`, `a-45`, `ax` = off |

Drops out (D straight to 0, no slew) past ±135°, above 300°/s, on a stale SAS, NEP off, or sensor off. Any manual
`d` / `x`, button A, or the Bench window's 0 key also ends it.

The dead-band defaults come from the first angle-hold run (2026-11-22, session `20261122_000423`):
the rack never moved below 0.20 V of D, broke away at +0.21..+0.27 V going right and −0.27..−0.30 V
going left, and was stuck against the old ±0.30 V cap 39 % of the time going left.

The second run (session `20261122_013517`, see
[bench_data/2026-11-22](../../bench_data/2026-11-22/20261122_013517/analysis.md)) set the rest:
small steps settled in 0.25-0.45 s with 1-2° overshoot, but large steps asked for 0.42-0.45 V, and
above about 0.35 V the rack runs at 225-660°/s. The 300°/s abort then ramped D down at 0.5 V/s, so
the wheel overran to ±110-228°. Now an abort drops D at once, the limit is 0.33 V, Kd is higher, the
speed brake coasts fast approaches, and the left dead band is 0.30 V.

### High-voltage permission mode (`hv1` / `hv0`)

Lifts the D limit for manual D and angle hold to `hvmax` (default 0.60 V, at most 1.00 V; the real
sensor read about ±1.3 V at the end stops). Outputs may then go 1.40–3.70 V. Above about 0.6 V the
Icarus data showed full motor current, so the motor can turn the wheel hard. Needs NEP on and the
sensor ON; ends by itself after 2 min, on NEP off / key off, on `hv0`, and on the Bench window's
**0** key (which also ends angle hold and puts D back to rest). The Bench window asks for
confirmation before turning it on.

## Wiring (breadboard)

| From | Through | To |
|---|---|---|
| Torque sensor VT3 (BLU, 7-pin pin 3) and VT6 (RED, pin 1) | **cut** | sensor-side ends left open, insulated |
| Feather **A0** (DAC0) | 1 kΩ | EPS **B10** (VT3) and Hantek **CH1** |
| Feather **A1** (DAC1) | 1 kΩ | EPS **B8** (VT6) and Hantek **CH2** |
| Feather GND | — | EPS **B9** (T/S GND) |
| Sensor Vcc1 / Vcc2 / sensor GND | — | unchanged, still connected |
| Feather D4 | — | XY-MOS TRIG; NEP node to EPS B14 and Hantek **CH3** (as eps_bench_nep) |
| Feather D13 | — | Hantek **CH4** (sync) |
| Feather CAN-H / CAN-L | — | SAS (unchanged) |

Put the Hantek probes on the **EPS side** of the 1 kΩ resistors (B10, B8), so the scope shows what
the ECU actually sees. Comparing that with `vt3_cmd` / `vt6_cmd` in the merged log is the sag check.

## Power-up and power-down order

The lines must look like the real sensor: 0 V until the engine runs, rest after. The firmware
writes 0 V first thing in `setup()`. A0/A1 float while the Feather resets or is being flashed;
with the sensor cut they then read about 0 V, the same as an unpowered sensor, so a reset with the
key ON and **NEP off** is harmless. Never reset or flash with NEP running.

1. Plug in the Feather (USB). OLED badge `S-OFF`, VT3/VT6 0.000.
2. Key ON (IG2). VT3/VT6 stay 0 V.
3. NEP on (`r900`): 0.7 s later the badge shows `REST`, B10/B8 about 2.54 / 2.56 V.
4. Power down: NEP off, **key to IG1 / OFF first**, then **`k`** (Bench window: "I turned the key
   OFF"). Never `k` with the key still ON: the ECU has the sensor powered then, reads 0 V on both
   lines as an open sensor, and latches DTC 16. That happened on 2026-11-21 when a test script sent
   `k` with the key ON.

## Safety

- **The motor steers by itself.** Keep hands, sleeves and tools clear of the wheel, column and
  rack before sending any `d` command. Start with `d0.05`.
- Have a way to cut the EPS 12 V instantly within reach: a switch or breaker in the supply lead,
  not just the bench supply's front panel.
- To stop steering: `x` (or `d0`) ramps back to rest at the slew rate. **Button A** does the same.
  To stop the motor immediately, cut the EPS 12 V.
- **NEP interlock:** a steering offset is only applied while NEP is running. `d…` with NEP off is
  refused (`# NEP is off: D stays at rest. Send r900 first.`), and turning NEP off by any route
  (`r0`, button A, the 2,000 rpm time guard) ramps D back to rest. `d` is also refused until the
  sensor is "powered" (0.86 s after NEP starts). VT3/VT6 stay at rest after NEP stops, like the
  real sensor, until `k`.
- Keep the slew at 0.5 V/s or below for first tests. The outputs are clamped in firmware (D ±0.30 V,
  each line 2.0–3.1 V), so a typo can't command more.
- NEP rules still apply: never send vehicle speed without rpm (DTC 22), and don't hold ≥ 2,000 rpm at
  0 mph for 6 min. The firmware drops back to 900 rpm after 5 min at ≥ 2,000.
- **If a DTC latches** (EPS lamp stays on): EPS 12 V off, check the wiring and the DAC rest
  voltages. Codes 11, 15, 16, 17, 18 and 67 stay until erased, and live in EEPROM, so a power
  cycle won't clear them. Use the procedure in [EPS_PINOUT.md](../../EPS_PINOUT.md) ("Clearing
  codes"): short SCS (DLC pin 9) to ground before ignition ON, then the manual's 45°-left steering
  sequence; two blinks means erased. **That sequence reads the steering input through VT3/VT6**,
  which are now the DACs: use the DAC clear mode (`clr`, below). Don't keep testing with a latched
  code; the EPS may have disabled assist.

## Clearing a latched DTC (from the service manual)

Source: doppelhub/Insight-Service-Manual, `Green Service Manual/electrical_power_steering.pdf`,
"How to clear DTCs", Service Check Signal Circuit method (summary, not verbatim):

1. EPS 12 V off. Wheel straight ahead.
2. Short **SCS (DLC pin 9) to body ground**.
3. EPS 12 V on (ignition ON, **engine stopped**: NEP off). The lamp comes on for about 6 s.
4. **Within 4 s** of switching on, while the lamp is on: steer **45° left and hold** until the lamp
   goes off.
5. **Within 4 s** of the lamp going off: back to straight ahead and **let go**. The lamp comes on
   again 4 s later.
6. **Within 4 s** of it coming on: **45° left again and hold**. The lamp goes off after 4 s.
7. **Within 4 s**: back to straight and let go. **Two blinks** 4 s later = erased.
8. EPS 12 V off, remove the SCS short, read the codes again to confirm.

If it doesn't blink twice, a step was missed: switch off and start again from step 3.

**On the bench:** the ECU sees the "45° left" through the torque sensor (engine stopped, so no
assist). In a car the tyres make you push to hold 45°; the bench rack has almost no load, so just
holding the wheel gives almost no torque. **Push firmly left and keep pushing** during the hold
steps (turning into the left end stop works, about −1.3 V), and let go completely on the release
steps. With VT3/VT6 cut, use the DAC clear mode below instead.

### With the sensor cut: DAC clear mode (`clr`)

The ECU only sees the "45° left" as torque on VT3/VT6, so the DACs can supply it: in clear mode
both lines drop together by `clearD` (default −0.80 V, so VT3 ≈ 1.76 V and VT6 ≈ 1.78 V) for each
hold, and go back to rest for each release. That is about what a firm push left gives (hand turns
measured −0.35..−0.4 V, the left end stop −1.35 V).

| | |
|---|---|
| Enter | `clr`, only with NEP off (engine stopped) |
| Hold left / release | `L` / `R`, or **button A** (toggles). `x` also releases |
| Left level | `cd-0.80` (−0.30 to −1.40 V). Outputs may go down to 1.10 V in this mode only (the real sensor showed 1.2 V at the left end stop) |
| Ramp | 3 V/s (0.8 V in about 0.3 s), well inside the 4 s windows |
| Blocked | NEP on (`r900`, button A) and `d` values, until `clr0` |
| Auto release | after 15 s of holding, or if the SAS moves more than 20° during a hold |
| Leave | `clr0`: ramps back to rest, normal mode again |

The OLED shows `LEFT` or `REL` with a seconds counter that restarts at each change, so you can
count the 4 s windows. Serial prints `# CLR LEFT HOLD` / `# CLR RELEASE` alongside the V lines, so
a Scope1008 recording shows when each step happened.

**Why the 20° abort:** with NEP off the EPS should not assist, so DAC torque alone should not move
the wheel. If it moves, the motor is driving it: the firmware releases, and you cut the EPS 12 V.

#### Easiest: the clear window, AUTO

Double-click `data_capture/tools/eps_dtc_clear_gui.bat` (close Scope1008 first: one program per
COM port). IG1 off, SCS shorted, press **AUTO CLEAR**: the Feather starts holding left **before**
key-on. Then switch IG1 on and press **EPS IS ON NOW** at the same moment (on this bench the SAS
stays powered, so the window can't see key-on by itself). Timer from key-on: release at 7 s, hold
left at 13 s, release at 19 s; watch for **two blinks at about 23 s**. Space or **ABORT** releases
at once. It also sends `r0` first, because the sequence is engine stopped.

Why hold before key-on: owners who cleared it on a real car (insightcentral.net, "Troubleshooting
EPS") hold 45° left first, then switch on; the lamp comes on and goes off after about 4 s. One
needed the **wheels** at 45° (car on the ground), not just the steering wheel: a lot of torque,
hence the deeper levels. They also found the clear is refused, or the code comes straight back, if
the fault is still present.

2026-11-21 tries: −0.60 / −0.80 V held 1.5 s after key-on: lamp kept blinking code 16 (hold not
accepted).

The timings assume the manual's nominal lamp behaviour. If the lamp doesn't match what the window
says (for example it is still on at 8 s), abort, EPS off, and use the manual steps below.

#### Procedure (manual)

You need to see the EPS lamp and reach button A (or the laptop), with your other hand on the EPS
12 V switch. Hands off the wheel the whole time: the DACs supply the torque.

1. **EPS 12 V off.** Wheel straight ahead. Feather on USB running eps_bench_steer, NEP off.
2. Short **SCS to ground** (DLC pin 9 / the EPS SCS wire, as when you read code 16).
3. Send **`clr`**. OLED shows `DTC CLEAR` and `REL`.
4. **EPS 12 V on.** The lamp comes on (about 6 s).
5. **Within 4 s: press A (hold left).** OLED `LEFT`. Keep it there until the **lamp goes off**.
6. **Within 4 s of the lamp going off: press A (release).** The lamp comes back on about 4 s later.
7. **Within 4 s of it coming on: press A (hold left).** The lamp goes off after about 4 s.
8. **Within 4 s of it going off: press A (release).**
9. **Two blinks** about 4 s later = erased.
10. **EPS 12 V off.** Send `clr0`. Remove the SCS short.
11. Check: SCS short, EPS on, read the blinks: no code = clear. Then remove the short, EPS on,
    `r900`: the lamp should go off.

**If it doesn't work:**
- Lamp never goes off at step 5: the ECU didn't see enough torque. EPS off, go one level deeper
  (`cd-1.00`, then `-1.20`, then `-1.40`, about the left end stop), start again from step 3.
  2026-11-21: −0.80 V did not turn the lamp off.
- Missed a window or the lamp did something else: EPS 12 V off, `R`, start again from step 3.
  A failed attempt does no harm.
- Code 16 is back right after a good clear (EPS on, NEP 900, D at rest): the rest voltages or the
  DAC wiring are what the ECU dislikes. Stop and check B10/B8 on the scope first.

## Commands (115200 baud, Enter after each)

| Command | Effect |
|---|---|
| `d0.05` | in-phase offset target +0.05 V (slews there) |
| `x` or `d0` | back to rest |
| `s0.5` | slew rate, V/s |
| `rest3 2.540` / `rest6 2.557` | rest voltages at the pin |
| `k` | key off: NEP off, VT3/VT6 to 0 V (sensor unpowered) |
| `cal 0.893 0.346 0.376` / `cal off` | pin correction for the ECU's input loading |
| `vref 3.300` | DAC reference for the volts-to-code conversion |
| `r900` / `r0` | NEP rpm / off |
| `z` | zero the SAS |
| `?` | status (D, outputs, rest, vref, DAC codes, frames, sync) |
| `c1` / `c0` | every-CAN-frame dump on / off |
| `clr` / `clr0` | DTC clear mode on / off (NEP off only) |
| `L` / `R` | clear mode: hold left / release |
| `cd-0.80` | clear mode left level, −0.30..−1.40 V |

OLED buttons: **A** back to rest if D ≠ 0, otherwise NEP on/off (clear mode: hold left / release).
**B** next rpm preset. **C** zero SAS.

## Serial format

Everything from eps_bench_nep (`N`, `A`, `C`, `S`, `#`), plus:

| Line | Meaning |
|---|---|
| `V,<us>,<vt3_cmd>,<vt6_cmd>,<D>` | commanded DAC outputs in volts. Sent on every change (max 50/s) and once a second |

`merge_hantek_feather.py` and Scope1008 sessions add `vt3_cmd`, `vt6_cmd`, `d_cmd` columns.

## DAC notes

- **Never call `analogRead()` on A0/A1.** In this core (Adafruit SAMD 1.7.17) it switches that DAC
  channel off, which drops the line to 0 V.
- **Write both channels twice at boot.** The core's first `analogWrite` to A1 disables and
  re-enables the whole DAC, and A0 then sits at **0 V** until it is written again. Found on the
  bench (VT3 0 V after boot, VT6 correct); `dacsToRestNow()` now does a second pass. Verified on
  three fresh boots: VT3 2.557–2.558 V, VT6 2.588–2.589 V.
- Reference is VDDANA (nominally 3.3 V), 12 bits, about 0.8 mV per step.
- **Calibration (2026-11-16, EPS off):** with `vref 3.300` the Hantek read every output 2.7 % low
  (measured = 0.973 × commanded, offset +3 mV), a pure gain error. The rest values come from the
  real sensor recorded on the same Hantek, so the DAC is calibrated to that scope:
  `vref 3.210`. Result over D = −0.30..+0.30 V: VT3 −3..−4 mV, VT6 +7..+8 mV. Whether the scope
  or the 3.3 V rail is off doesn't matter as long as both use the same scope. Re-check with a
  different scope or a DMM.
- This was measured with the EPS **not powered**. Repeat the comparison with the EPS on (step 5
  of the bench test) to see whether its input loads the DACs.
- The DAC's output drive is modest. With the EPS connected, compare `vt3_cmd` with Hantek CH1 at
  B10. If they differ by more than about 20 mV, add an op-amp voltage follower (rail-to-rail,
  e.g. MCP6002 at 5 V) between each DAC and its 1 kΩ resistor.

## Build and flash

```bash
arduino-cli compile --fqbn adafruit:samd:adafruit_feather_m4_can steering_control_unit/data_capture/firmware/eps_bench_steer
arduino-cli upload -p COM11 --fqbn adafruit:samd:adafruit_feather_m4_can steering_control_unit/data_capture/firmware/eps_bench_steer
```
Libraries: Adafruit CAN, Adafruit SH110X, Adafruit GFX. **EPS 12 V off while flashing.**

## First bench test

1. EPS 12 V **off**: confirm B10/B8 sit at the rest values on the scope (2.56 / 2.58 V).
2. EPS on, `r900`, D = 0: no DTC, wheel still. Record 10 s.
3. Steps, each held a few seconds and logged: `d0.05`, `d0`, `d-0.05`, `d0.10`, `d-0.10`, `x`.
   Note the wheel/SAS response and whether the EPS lamp or a DTC appears.
4. Optional, with agreement: the same steps with NEP off (`r0`) to see what assist adds.
5. Sag check: commanded vs measured at B10/B8; > 20 mV means add the follower.
