# 03 — Functional safety concept (ISO 26262-3, clause 7)

| | |
|---|---|
| Revision | 0.1 draft, 2026-11-01 |
| Based on | `02_HARA.md`, SG1–SG5 |

## 1. Architecture strategy

The **overlay path** sits between the torque sensor and the EPS. It is enabled only by a **2-out-of-2 decision**:

- the **control channel (CH-A)** computes and outputs the overlay;
- the **monitor channel (CH-B)** checks it independently.

Each channel, and each channel's external watchdog, can **remove the enable on its own** (1-out-of-2 shutdown). With no enable, NC relay contacts restore the OEM path.

A **hardware authority limiter** in the analog path bounds the injected offset to Δmax. It holds regardless of what either MCU does.

**Overbuilt on purpose, beyond what the B(D)+B(D) split needs:**
- **CH-C analog guard.** A third, software-free channel that compares the EPS-side signals against the raw sensor and latches the overlay off.
- **Two cascaded relays.** The overlay needs both relays energized.
- **One regulator chain per channel**, so the channels share no supply after the input protection.

Even if both MCUs fail in the same way (common-cause software or silicon), the overlay is removed.

### ASIL decomposition (ISO 26262-9, clause 5)

| Requirement | Decomposed into | Independence argument |
|---|---|---|
| SG1/SG2: no unintended or un-overridable overlay (D) | **CH-A control + plausibility: ASIL B(D)** | Separate MCU, clock source, LDO, watchdog, firmware build. See `04_TECHNICAL_SAFETY_CONCEPT.md` §6 |
| | **CH-B monitor + enable path + hardware limiter: ASIL B(D)** | Same independence measures. The hardware limiter needs no software |

The F280049C (F280049CPZQR) is TÜV-certified for hardware integrity up to ASIL B, with systematic capability up to ASIL D. That matches a B(D)+B(D) split.

## 2. Functional safety requirements

The "→" column shows which channel or element carries each requirement:
- **A**: CH-A
- **B**: CH-B
- **HW**: hardware only

| FSR | Requirement | SG | ASIL | → |
|---|---|---|---|---|
| FSR-01 | With no power, a reset, or either enable missing, the EPS shall receive the unaltered torque-sensor signals. | 1, 2 | D | HW |
| FSR-02 | The injected offset shall never exceed Δmax (hardware-set, TBD-DATA), even with both MCUs failed at full DAC output. | 1 | D | HW |
| FSR-03 | The offset applied to VT3 and VT6 shall be antisymmetric (VT3 +Δ, VT6 −Δ) within ±ε. | 1, 3 | D | A, B |
| FSR-04 | The measured EPS-side offset shall match the commanded offset within ±ε; otherwise pass-through within 20 ms. | 1 | D | A, B |
| FSR-05 | The overlay slew rate shall be ≤ dΔ/dt max (hardware RC and software limit). | 1, 5 | D/B | HW, A, B |
| FSR-06 | Driver torque (sensor-side VT3−VT6) above T_override, or the disengage switch, shall start ramp-out within 20 ms and reach pass-through within 200 ms. | 2 | D | A, B |
| FSR-07 | Only commands that pass E2E protection shall be acted on: CRC, rolling counter, timeout ≤ 50 ms, and range check. | 1 | D | A, B |
| FSR-08 | Engagement shall be refused outside the operating envelope (speed window from VSP *and* CAN, EPS lamp off, RT+ on, Vcc2 in range, ignition on). | 1 | D | A, B |
| FSR-09 | CH-A and CH-B shall check each other's liveness and state over a protected link. Disagreement for > 10 ms → pass-through. | 1 | D | A, B |
| FSR-10 | Each MCU shall be supervised by an external window watchdog that removes that channel's enable on a timeout or early service. | 1 | D | HW |
| FSR-11 | Supply under- or overvoltage on any rail shall remove the enables. | 1 | D | HW |
| FSR-12 | The relay shall be proof-tested at every key-on: both states checked by readback. A failure blocks engagement for the cycle. | 1, 3 | D | A, B |
| FSR-13 | The pass-through transition shall not float the EPS inputs long enough to set an EPS DTC. Uses a hold capacitor plus zero offset before the relay drops. | 3 | A | HW, A |
| FSR-14 | Icarus shall not load the torque-sensor lines (sensor-side input impedance ≥ 1 MΩ, protected) or the shared VSP/NEP lines. | 3 | A | HW |
| FSR-15 | Loss of overlay while engaged shall be reported on CAN and to the driver indicator within 200 ms. | 4 | B | A, B |
| FSR-16 | After a critical fault, pass-through is latched until the next ignition cycle. | 1 | D | A, B |

## 3. Safe states and transitions

| From | Trigger | Reaction | Time budget |
|---|---|---|---|
| ENGAGED | Critical fault: FSR-02/04/07/09/10/11 violated | Remove the enable at once (relay drops), latch SAFE | ≤ 20 ms detection + ≤ 10 ms relay release |
| ENGAGED | Driver override, disengage, request withdrawn, envelope exit | RAMP-OUT: Δ → 0 at the limited rate, then relay drops | ≤ 200 ms |
| Any | Power loss | Relay drops by itself (NC contacts) | Relay release time |
| INIT | POST or relay proof-test fails | Stay in pass-through, report the fault | — |

**Why a critical fault cuts immediately instead of ramping:**
- The NC path restores the *true* sensor signal, so the step the driver feels equals the overlay that was there. That's at most Δmax, which by FSR-02 is controllable.
- Waiting would keep a possibly wrong overlay in place.

## 4. Warning and degradation

- The driver indicator shows three states, mirrored in a CAN status frame: **available**, **engaged**, and **fault** (latched).
- **There is no degraded mode.** Icarus is either fully checked, or it is out of the loop.
