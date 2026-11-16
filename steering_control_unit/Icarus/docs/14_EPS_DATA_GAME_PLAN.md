# 14: EPS bench data: what it changes, and the game plan

| | |
|---|---|
| Date | 2026-11-16 |
| Data | `../../data_capture/bench_data/2026-11-15/`: four sessions (30 s first turns, 40 s guided steps, 60 s free steering, 60 s lock to lock). Bench rack, real torque sensor, fake engine speed on, no road-speed signal |
| Numbers from | `../../data_capture/tools/bench_design_numbers.py` (re-run it on any new capture folder). Cross-checked against the laptop's own `analysis.md` files, which were written separately |
| Status | **Plan.** No requirement, schematic or FMEDA model is changed yet. The only edit is the note against assumption A1 in `01_ITEM_DEFINITION.md` |
| One-page version | `15_GAME_PLAN_SUMMARY.md` |

## 1. Bottom line

1. **VT3 and VT6 move together.** Icarus is drawn to push them apart (+Δ on one, −Δ on the other). That offset would add no torque, and the EPS would read it as a sensor fault. The overlay has to add the same Δ to both lines.
2. **That flips which fault is dangerous.** Both lines shifting the same way is a steering command the EPS can't tell from the driver. One line shifting is a mismatch the EPS checks for by itself. Several shared parts in Icarus's analog path can shift both lines at once, so the dependent-failure analysis has to be redone.
3. **The EPS was assisting, and its assist curve has a cliff.** There's no measurable motor current within ±0.2 V of centre, a slow rise up to 0.6 V, then full current by 0.7 V.
4. **Icarus's ±0.18 V limit sits in the flat part of that curve.** As drawn, it may not steer at all with hands off. The limit has to be sized from two measurements that don't exist yet: volts per N·m of driver torque, and the same curve at road speed.
5. **Most of rev 2.0 isn't blocked.** Power, MCUs, CAN, relay drivers and the connector can go ahead. Only the overlay block and its checks wait for the gates in §6.

## 2. What the four sessions show

### 2.1 Measured

| Quantity | Value | Note |
|---|---|---|
| VT6 against VT3 | VT6 = 1.003 × VT3 + 10 mV (the two wide-range sessions), r = 0.996 to 0.9998 | In phase, same gain |
| VT6 − VT3, 20 Hz low-pass | Stays within ±13 mV of its session mean everywhere, both end stops included | Session means are 9 to 20 mV; the scope's zero is only good to about ±10 mV |
| Rest level, wheel still | 2.533 to 2.584 V over 31 segments, median 2.546 V | A 50 mV band, not one value |
| Centre of the assist curve | **2.505 V** (2.495 to 2.515) | Also the midpoint of the two end-stop plateaus |
| End-stop plateaus | About 1.2 V and 3.8 V on both lines | ±1.3 V about centre. The manual's "1.5–3.5 V" is narrower than the real range |
| Noise per line at rest | 5 to 6 mV rms above 20 Hz | At the scope's 10 mV step, so an upper bound |
| NEP crosstalk into VT | Under 1 mV | Edge-averaged over 300 to 540 edges per session |
| Hand slew rate, 10 Hz low-pass | 95 % under 3 to 5 V/s, 99 % under 5 to 9 V/s, peak 24 V/s | Peaks are end-stop hits |
| Signal bandwidth | 98 % of the energy below 5 Hz | Laptop analysis |
| 12 V supply sag at full assist | 14 % | From the NEP high level: 10.45 V unloaded, about 9.0 V at the stops |
| Motor current against torque signal | No delay resolved: best fit +10 ms, resolution ±25 ms | One supply sample per 45 ms |
| Fake engine speed really sent | 22.0 Hz = **880 rpm**, not 900 | Arduino `tone()` takes whole hertz. Harmless here |

### 2.2 The assist curve at 0 mph

The NEP pull-up shares the EPS supply, so the NEP high level tracks the 12 V rail. The rail sags when the motor draws current. A control channel (the Feather's 3.3 V sync pin) doesn't move, so the sag is real. This gives motor current in relative units for free:

| Torque signal, distance from 2.505 V | Supply sag | Share of full assist current |
|---|---|---|
| 0 to 0.20 V | ≤ 0.2 % | **Nothing measurable** (the proxy resolves about 1 %) |
| 0.20 to 0.25 V | 0.2 % | 1 % |
| 0.25 to 0.30 V | 0.4 % | 3 % |
| 0.30 to 0.35 V | 0.7 % | 5 % |
| 0.35 to 0.40 V | 1.2 % | 9 % |
| 0.40 to 0.50 V | 1.7 to 2.4 % | 12 to 17 % |
| 0.50 to 0.60 V | 3.8 % | 27 % |
| 0.60 to 0.70 V | 8.5 % | **60 %** |
| 0.70 V and above | 13.1 to 13.6 % | 94 to 97 % (saturated) |

**Read it as:** a dead band of about ±0.2 V, a gentle slope to 0.6 V, a cliff between 0.6 and 0.7 V, and full assist beyond. The wheel rests about 0.04 V to the right of centre, which is why turning left looked like more effort than turning right in the first analysis.

**Where Icarus's present limits land** (rest is +0.04 V from centre):

| Offset | Right | Left |
|---|---|---|
| ±0.18 V, the normal limit | 0.22 V from centre: about 1 % | 0.14 V: nothing |
| ±0.37 V, the single op-amp fault bound | 0.41 V: about 12 % | 0.33 V: about 5 % |

### 2.3 What the data doesn't show

- **Volts per N·m.** Without it, no offset can be turned into a hand torque.
- **Anything at road speed.** VSP was never driven, so every number above is the parking-speed curve.
- **What the EPS does when the speed signal is lost.**
- **How big and how long a VT3/VT6 mismatch must be before the EPS sets a latching code.**
- **What the EPS inputs look like electrically** (input resistance, where an open line floats to).
- **Real motor current.** The sag proxy is relative and slow.

## 3. What changes in Icarus

### 3.1 Requirements and documents to change (one change request, "CR-01")

| Where | Says now | Becomes |
|---|---|---|
| `01` A1, F2, I1 | VT3/VT6 are differential; antisymmetric ±Δ | In phase. A1 is disproved. The offset is the same on both lines |
| `03` FSR-03 | Offset is antisymmetric within ±ε | Offsets on VT3 and VT6 are **equal** within ±ε_diff |
| `03` FSR-06, `04` SW-03 | Driver torque = VT3 − VT6 | Driver torque = mean of VT3 and VT6 minus a **zero reference**. The difference of the two lines carries no torque |
| `04` SM-06 | One DAC, mirrored summers | One DAC, **identical** summers |
| `04` SM-08, `12` T15 / T40 | VT3 + VT6 sum unchanged | VT6 − VT3 **difference** unchanged |
| `hardware/gen/design.py` overlay sheet | VT6 summer has DCMD and VMID swapped | Wire the VT6 summer exactly like the VT3 one: two nets swap on two resistors |
| `04` §6, `11` §4.4 | Dependent failures | Redo for the analog path (rule R1 below) |
| `safety/element_model.py`, `rev2_trade.py` | E08 "Delta, inverter, summers" | Split into same-direction and one-line failure modes, then re-run |
| `11` §4.1 | Speed stage cut because the EPS lowers assist with speed | Unproven. Decide at gate G2 |
| Outside Icarus | `EPS_PINOUT.md` "probably opposite directions"; bench stim wing notes | Same correction (follow-up, §7) |

### 3.2 New design rules

| # | Rule | Why |
|---|---|---|
| **R1** | **No single fault may move VT3 and VT6 the same way by more than Δmax.** | This replaces "keep the offset antisymmetric" as the SG1 rule. Today both raw-signal followers share one quad op-amp and both summers share another, on one 5 V rail. A lost supply pin or a latch-up in either package rails **both** lines the same way, and nothing bounds it until the relays drop. Under the old assumption that looked like a harmless sum error. It's a full-scale steering command |
| **R2** | **A railed op-amp must not exceed the normal limit.** | Today the DAC can command 0.18 V, but a railed buffer can reach 0.37 V, because the buffers run from 5 V and the DAC from 3.3 V. Running the command buffer from the DAC's own 3.3 V closes that gap for that buffer. The midpoint buffer needs its own answer, for example a resistor-only midpoint |
| **R3** | **Driver torque needs a zero reference.** | Two opposite signals would have cancelled supply, ground and offset drift. In-phase signals don't. Measure both lines against the T/S GND tap, scale by Vcc2, and use a stored zero that is checked hands-off at key-on. Budget for a rest band of ±25 mV |
| **R4** | **The override threshold must sit between the rest band and Δmax.** | A driver holding against the overlay shows up on the sensor side as at most Δmax in the opposite direction. The threshold has to be below that and above about ±0.05 V of zero uncertainty. With Δmax = 0.18 V that leaves a narrow gap |
| **R5** | **Keep the EPS side matched as well as the sensor is.** | The real pair agrees within ±13 mV. Provisional ε_diff: 15 mV, to be set below the EPS's own mismatch threshold once that's measured |
| **R6** | **The EPS side must stay inside 1.2 to 3.8 V.** | That's all the real sensor can produce. Engaging only below the override threshold keeps it there without an extra clamp; the proof test checks it |
| **R7** | **Don't count on the EPS to filter a short fault.** | Motor current followed the torque signal with no delay the data could resolve (±25 ms). Icarus's 20 ms reaction target is the same order. The 100 ms FTTI placeholder in `02` can't be relied on until phase 4 measures it |

**Two ways to meet R1,** to be chosen at gate G3:
- **Split the paths.** VT3's buffer and summer in one package on CH-A's 5 V, VT6's in another on CH-B's 5 V. A single package or rail fault then moves one line only, and the EPS sees a mismatch. This depends on how the EPS reacts to a mismatch (phase 5).
- **Bound it passively.** Leave the sensor wired to the EPS through a small series resistor and inject through a larger one, so no active part can move the line by more than the resistor ratio allows. This loads the sensor slightly, so it depends on the sensor's output and the EPS's input resistance (phase 2).

### 3.3 The authority window

Icarus only works if one resistor value satisfies both of these at every speed it's allowed to engage:

- **Enough to steer.** The offset has to get out of the dead band, or the motor does nothing. At 0 mph that needs more than about 0.2 V from centre.
- **Small enough to override.** In steady state a driver cancels the overlay by applying the opposite torque, so the hold torque is never more than Δmax expressed in N·m, whatever the assist gain. That makes volts per N·m the number that matters, and it can be measured with a spring scale and no motor power.

For scale only: if this torsion bar reaches its stops near 8 to 10 N·m, as EPS torque sensors commonly do, then 1.3 V is about 8 to 10 N·m and 0.18 V is about 1.2 N·m. **That's an estimate, not a measurement.** UN R79 caps the effort to override an automatically commanded or corrective steering function at 50 N at the rim, which is about 9 N·m on a 370 mm wheel. I'd aim far lower: a hold torque of 3 N·m or less.

The transient is the other half: how far the car moves before the driver reacts depends on the assist gain at that speed, and on what happens if the speed signal is lost. That needs the curve at speed (phase 3) and the response time (phase 4).

If the window is empty at some speed, Icarus doesn't engage at that speed, or the concept stops there.

### 3.4 What doesn't change

- The safety goals and the HARA: they rate the function, not the design.
- Pass-through by NC relay contacts, the two cascaded relays, cross-driven coils, both MCUs, the safety PMICs, and the self-test windows.
- The resistor-set authority limit as a principle. Only its value and its fault bound change.
- The W1 key-on test offset gets safer: an in-phase test offset looks like ordinary torque to the EPS, where the old antisymmetric one would have looked like a 0.7 V sensor mismatch.

## 4. Bench campaign

Each phase has one job. Don't start a phase until the one before it has met its exit line.

| Phase | Job | Injection | Exit: what must be known |
|---|---|---|---|
| **0** | Instrument the bench | None | Extra channels recording, kill switch and EPS lamp fitted, cut made reversible |
| **1** | Characterise with the real sensor | None | Volts per N·m; assist on against off; rest band |
| **2** | Feather DACs replace the sensor, 0 mph | ±0.30 V, hardware-bounded | In-phase offset steers; sign; EPS zero; first-motion offset; EPS input resistance |
| **3** | Add road speed | ±0.30 V | Curve at 7 speeds; behaviour on speed-signal loss |
| **4** | Dynamics | ±0.30 V free; pulses into an end stop | Current rise time; shortest pulse that moves the rack |
| **5** | EPS fault limits (codes will latch) | One line offset; short opens | Mismatch size and time to a code; open-line tolerance |
| **6** | Icarus board on the bench | Icarus itself | Transparent in pass-through; faults bounded and cleared inside the FTTI |

**Phase 0, instrumentation.** The Hantek has 8 channels and 4 are in use.
- Add +B at A2, RT+ at B11 (motor relay drive: shows when assist is armed), WLP at B13 (lamp) and Vcc2 at B2. A clamp-on current probe on the motor wire is better than the sag proxy if one is available.
- Fit a bench EPS lamp: an LED and 1 kΩ from 12 V to B13. It shows faults at a glance and blinks the codes.
- Put VT3 and VT6 on the smallest scope range that still covers 0 to 5 V. 10 mV steps are too coarse for mismatch work.
- Make the VT3/VT6 cut reversible with a plug or a two-pole changeover switch, so the real sensor can go back in seconds. The code-erase procedure needs it.

**Phase 1, real sensor, no injection.**
- 1a: the same slow lock-to-lock run with NEP on and with NEP off. The difference is the assist.
- 1b: NEP off, spring scale on the rim at a measured radius, five forces each way, three seconds each. This gives volts per N·m and the true zero.
- 1c: five minutes hands-off from cold with Vcc2 recorded, to see drift and whether the sensor follows its supply.

**Phase 2, DAC drive at 0 mph.** Use the bounded network in §5.3.
- 2a: EPS unplugged, temporary 5 V on the pull-ups. Trim the two lines to each other and note the DAC codes for centre (§5.3).
- 2b: EPS on, both lines at centre for 60 s. Check Vcc2 and both levels with the multimeter first. No lamp, no motion.
- 2c: sweep the common level slowly ±0.1 V to find the EPS's own zero.
- 2d: in-phase steps of 0.05 V out to ±0.30 V. Record the first offset that moves the rack each way.
- 2e: repeat one sweep with a different series resistor. The change in level gives the EPS input resistance.

**Phase 3, speed.** Needs a second open-collector output on B4 with a pull-up to 5 V.
- Repeat 2d at about 0, 10, 20, 30, 45, 60 and 75 mph (roughly 1.24 Hz per mph, unverified).
- Then hold 60 mph with a fixed 0.2 V offset and stop the pulses. Record current, lamp and time.
- Rules from the manual: ramp speed smoothly, never send speed without rpm, and keep rpm plausible for the speed.

**Phase 4, dynamics.**
- 4a: steps from 0.1 to 0.3 V with motor current sampled at 1 kHz or faster.
- 4b: park the rack against a stop, then pulse **into** the stop: 5, 10, 20, 50 and 100 ms. The motor can't travel, so this measures response time safely. It needs a different resistor network, fitted deliberately for this test only.

**Phase 5, fault limits.** Do it last, and rehearse reading and erasing codes first.
- Offset one line by 10, 20, 40, 80 and 160 mV for one second. At the first level that sets a code, shorten the time: 100, 50, 20, 10, 5 ms.
- Open one line for 1, 2, 3, 5 and 10 ms with the planned hold capacitor fitted (assumption A2). This needs a small relay or analog switch in that line.
- Record motor current during each mismatch, to see whether the EPS acts on the bad line before it rejects it.

**Phase 6, Icarus on the bench.** After the board exists: pass-through adds no measurable offset or lag; engage and disengage steps stay under ε; every self-test runs; then inject the top FMEDA faults through test pads (railed DAC, shorted op-amp output, 5 V rail pulled down) and measure the excursion and the reaction time.

## 5. Safety plan

### 5.1 Bench hazards and controls

| # | Hazard | Control |
|---|---|---|
| H1 | **The cliff.** An even ramp in volts is not an even ramp in force. Past 0.6 V from centre the motor goes to full current, and the rack runs to its stop | Hardware bound (§5.3) keeps the EPS inputs inside ±0.30 V in phases 2 and 3. Software clamp, slew limit and steps of 0.05 V on top |
| H2 | **Firmware hang.** A stuck Feather holds its last DAC value, so the rack keeps going | Watchdog in the sketch; dead-man (offset returns to zero unless a keep-alive arrives every 0.5 s); angle and rate cage from the SAS (zero the offset outside ±90° or above 200°/s) |
| H3 | Software can't be the last line | **Kill switch on the EPS supply positive,** rated for the motor current, within reach, tested before every session |
| H4 | Spinning wheel and moving rack ends | Rack bolted down. Nothing and nobody inside the swept area with an offset applied. Hands off the wheel; no loose sleeves or leads |
| H5 | High current | Fuse at the battery, 40 A or less. The supply already sags 14 % at the stops, so use a charged battery and record +B |
| H6 | Latching codes (11, 15, 16, 17, 18, 67) | Feather powered before the EPS and off after it. Reversible cut. Erase procedure rehearsed before phase 5 |
| H7 | 12 V into a DAC pin | Series resistors stay. Keep 12 V wiring physically apart from A0/A1 |
| H8 | Thermal derating skews results | End-stop pushes under 3 s, 30 s apart. Repeat one reference point through the session; if its current falls, stop for 15 minutes |
| H9 | Energised bench left alone | Kill switch off whenever you step away |

### 5.2 Limits by phase

| Phase | EPS input range | Slew | Where the bound lives |
|---|---|---|---|
| 1 | Real sensor only | — | — |
| 2, 3 | 2.20 to 2.80 V (centre ±0.30 V) | 0.5 V/s | Resistor network, then firmware |
| 4a | Same | Steps allowed | Resistor network |
| 4b | Up to centre ±0.8 V, only into a stop, pulses of 100 ms or less | — | A second network, fitted for this test and removed after |
| 5 | Common level inside ±0.30 V; one line offset up to 0.16 V | — | Resistor network |

### 5.3 A hardware bound for the DAC rig

This is Icarus's own principle (SM-05) applied to the breadboard: resistors set the largest offset, not code. One network per line:

| Part | Value | Between |
|---|---|---|
| Series | 10.0 kΩ | DAC pin (A0 or A1) and the EPS input (B10 or B8) |
| Pull-up | 4.02 kΩ | EPS input and Vcc2 (B2, 5 V) |
| Pull-down | 4.75 kΩ | EPS input and T/S GND (B9) |

- The DAC's whole 0 to 3.3 V range maps to **2.22 to 2.81 V** at the EPS (gain 0.179). No code, crash or reset can leave the flat part of the curve.
- 2.505 V needs about 1.57 V from the DAC.
- **Trim the two lines to each other before the EPS ever sees them.** With 1 % resistors each line can sit up to 22 mV off, so the pair can disagree by about 40 mV, which is more than the real sensor's ±13 mV. Vcc2 only exists with the EPS on, so do the trim with the EPS unplugged and a temporary 5 V on the pull-ups. Use a multimeter, not the scope, and bring the two lines within 2 mV. The 5 V value only shifts both lines together.
- With the Feather unpowered or its DAC off, the lines sit between 2.2 and 2.7 V: inside or at the edge of the dead band (about 1 % current at most), and no open-input code.
- With common values (10 kΩ, 3.9 kΩ, 4.7 kΩ) the range is 2.25 to 2.83 V.
- The networks sit on the Feather side of the changeover, so the real sensor never drives them.
- **Two checks before trusting it.** Vcc2 must not move more than 10 mV when the networks are attached (they draw about 1.2 mA). And the measured level must match the calculated one; if it doesn't, the EPS input is loading the network, which is the phase 2e result arriving early.
- This replaces the plain 1 kΩ series resistors and the 2.0 to 3.1 V software clamp in the earlier message to the laptop session. That clamp reached 0.6 V from centre, which is on the cliff.

### 5.4 Before every powered run, and when to stop

**Go only if all are true:**
1. The expected result is written down.
2. Kill switch tested this session; fuse fitted.
3. Feather running and both lines at centre before the EPS gets 12 V.
4. Correct network fitted for this phase.
5. Rack clamped, swept area clear, hands off.
6. Scope and Feather logging, sync channel live.

**Stop the session if:**
- anything moves with zero offset commanded;
- the response is more than 30 % above what the last run predicted;
- the lamp comes on outside phase 5;
- +B falls below 10 V;
- a result can't be explained. Explain it before the next run.

### 5.5 Managing the change

- **CR-01** (§3.1) is a safety-relevant change. Apply it to `01`, `03`, `04`, `12`, `design.py` and the FMEDA model together, not piecemeal.
- **Independent review** of the updated concept before the overlay schematic is frozen. The reviewer must be someone who didn't write it. I can't be that reviewer for my own documents.
- **Assumption register:** A1 is closed as disproved. A2 and A3 stay open until phases 5 and 1.
- **Every new number traces to a test** in §4. No requirement is frozen on an estimate.
- **Tool confidence:** `bench_design_numbers.py` now feeds safety numbers. Its overlap with the laptop's separate analysis agrees (gain 1.003, rest level, end-stop levels). It still needs a human read-through.

### 5.6 Vehicle stages (after phase 6 and the review)

| Stage | What | Limit | Pass to move on |
|---|---|---|---|
| V0 | Logger only, front wheels off the ground, engine running | No Icarus | Phase 1 repeated in the car |
| V1 | Icarus fitted, pass-through only, on the ground, stationary | Relays never energised | 20 key cycles, no code, signals unchanged |
| V2 | Engaged, stationary, hands on | Δmax as built | Hold torque at Δmax measured at or under target; kill switch and bypass plug proven |
| V3 | Private ground, walking pace | 10 mph | Deliberate stuck-Δmax demo held without effort |
| V4, V5 | Private ground | 25 mph, then 45 mph | Same demo, lateral deviation inside the HARA's 0.3 m |

Two people from V2 on, no passengers, hardware kill switch (M1) in reach, bypass plug in the car. Public roads are outside this work (`01` §5).

## 6. Gates and order of work

| Gate | After | Decides | Unblocks |
|---|---|---|---|
| **G1** | Phases 1 and 2 | In-phase overlay proven; volts per N·m known | CR-01 text; overlay schematic with in-phase summers (resistor values left open) |
| **G2** | Phase 3 | Does an authority window exist, and at which speeds? Does the speed stage come back? | Δmax and the summer resistor values; the engage speed window |
| **G3** | Phases 4 and 5 | FTTI; monitor thresholds; split paths or passive bound; hold capacitor | Overlay layout; FMEDA re-run |
| **G4** | Independent review | Concept accepted | Board order |
| **G5** | Phase 6 | Board does what the FMEDA assumed | Vehicle stage V0 |

**In parallel, not blocked:** the PMIC, MCU, CAN, coil-driver and connector sheets of the rev 2.0 schematic; the open parts from doc 13 (load-dump strategy D1, core FET D2, ERRORSTS pin D7); the hardware-software interface list.

## 7. Follow-ups outside this plan

- `eps_bench_nep`: `tone()` rounds 22.5 Hz down to 22 Hz, so "900 rpm" is 880 and "1500" is 1480. Use a timer if exact rpm ever matters.
- `EPS_PINOUT.md` and the bench stim wing notes still say VT3/VT6 probably move in opposite directions.
- Doc 11 §4.1 states that the EPS lowers assist with speed as if it were known. It's an assumption until phase 3.

## Sources

- Bench captures and laptop analyses: `../../data_capture/bench_data/2026-11-15/`
- UN Regulation No. 79, override effort limit of 50 N: [consolidated text, Rev. 3 Amend. 2](https://www.marklines.com/statics/unece/pdf_201901/R079r3am2e.pdf), [Rev. 4](https://villanyautosok.hu/wp-content/uploads/2019/05/r079r4e.pdf), [amendment history](https://globalautoregs.com/rules/90/documents?year=2017)
- Honda service manual figures as summarised in `../../data_capture/EPS_PINOUT.md`
