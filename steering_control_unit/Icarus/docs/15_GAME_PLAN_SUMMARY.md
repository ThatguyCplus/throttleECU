# 15: Icarus game plan after the first EPS bench data (one page)

2026-11-16. Full version: `14_EPS_DATA_GAME_PLAN.md`. Data: `../../data_capture/bench_data/2026-11-15/`.

## What the data says

- **VT3 and VT6 move together**, same gain, within ±13 mV of each other from stop to stop. Icarus was drawn to push them apart. That would add no torque and look like a sensor fault.
- **The EPS was assisting.** The 12 V rail sagged 14 % whenever the torque signal was large.
- **The assist curve has a cliff.** Measured from its centre at 2.505 V: nothing within ±0.2 V, a slow rise to 0.6 V, full motor current by 0.7 V.
- **Icarus's ±0.18 V limit sits in the flat part.** As drawn it may not steer at all with hands off.
- **Still unknown:** volts per N·m, the curve at road speed, what the EPS does when it loses the speed signal, and how big a VT3/VT6 mismatch sets a latching code.

## What changes in Icarus

| Change | Why |
|---|---|
| Add the **same** offset to both lines (two nets swap on the VT6 summer) | The signals are in phase |
| Check that the VT6 − VT3 **difference** is unchanged, not the sum | Same reason |
| Driver torque = mean of both lines minus a stored zero | The difference no longer carries torque |
| **New top rule: no single fault may move both lines the same way by more than the limit** | Today two shared op-amp packages can rail both lines at once. That is now a full steering command, not a harmless error |
| Size the limit from data, not 0.18 V by default | It must clear the dead band and stay easy to override |

Unchanged: the safety goals, pass-through relays, both MCUs, the safety power chips, the self-test windows. Power, MCU, CAN and relay-driver schematic work can go ahead now.

## Order of work

| Step | Do | Decides |
|---|---|---|
| 0 | Add scope channels (+B, motor relay drive, lamp, Vcc2), a kill switch, a bench EPS lamp, a reversible cut | — |
| 1 | Real sensor: assist on against off, spring-scale torque calibration | Volts per N·m |
| 2 | Feather DACs replace the sensor, 0 mph, bounded to ±0.30 V | **Gate 1:** in-phase offset steers |
| 3 | Add road speed; then remove it | **Gate 2:** is there a usable limit, and at which speeds |
| 4, 5 | Response time; EPS fault limits (codes will latch) | **Gate 3:** reaction-time budget, thresholds, how to meet the new top rule |
| — | Independent review of the updated safety concept | **Gate 4:** order the board |
| 6 | Icarus on the bench, with injected faults | **Gate 5:** first vehicle stage |

## Safety plan in six lines

1. **Bound the DAC rig with resistors, not code:** 10 kΩ series, 4.02 kΩ to Vcc2, 4.75 kΩ to ground on each line. The EPS then never sees outside 2.22 to 2.81 V, whatever the Feather does. Trim the two lines to each other before the EPS sees them.
2. **Kill switch on the EPS supply**, rated for motor current, tested every session.
3. **Firmware cage:** watchdog, dead-man timeout, steps of 0.05 V, and zero offset outside ±90° or above 200°/s.
4. **Hands off and area clear** whenever an offset is applied. Rack bolted down.
5. **Stop on any surprise:** motion at zero offset, a response 30 % over prediction, or a lamp.
6. **In the car:** pass-through first, then stationary, then private ground at 10, 25 and 45 mph. Two people. No public road.

**Correction to my earlier note for the laptop session:** its 2.0 to 3.1 V software clamp reaches 0.6 V from centre, which is on the cliff. Use the resistor network above and centre on 2.505 V.
