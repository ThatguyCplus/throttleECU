# 12: Self-test plan and added safety functions (rev 2.0)

| | |
|---|---|
| Revision | 2.0 concept, 2026-11-15 |
| Builds on | `11_REV2_DESIGN_REVIEW.md` (rev 2.0 architecture) |
| Scored in | `safety/rev2_trade.py`, variant "Rev 2.0 + self-test plan" |
| Status | Plan only. Hardware additions (§5) go into the rev 2.0 schematic; tests go into both channels' firmware |

## 1. The idea in one paragraph

A test is only safe if a test signal **can't reach the steering**. Icarus has three moments when that's guaranteed by the car itself:

- **Disengaged.** The two relays are off, so the EPS is wired straight to the sensor through the NC contacts, and the overlay circuit can be exercised freely behind them.
- **EPS motor relay open.** At key-on with the engine off, and at key-off, the EPS has no motor power. `EPS_PINOUT.md`: "the motor relay only closes after the engine starts". Even a full-amplitude test offset can't turn the wheel then.
- **Engaged.** Nothing intrusive is allowed; only continuous monitoring runs.

Every test is **run by one channel and watched by the other**. Engagement needs a valid PASS from both (2oo2). Any FAIL from either blocks engagement for the trip (1oo2).

## 2. Test windows

| Window | When | Interlock (all must hold, checked by **both** MCUs) | What can be tested |
|---|---|---|---|
| **W0 Power-up** | Every reset | — | Built-in self-tests of the PMICs and MCUs |
| **W1 Key-on, engine off** | IG1 on, before the engine starts | RT+ off (EPS motor relay open) · NEP absent · VSP = 0 · overlay not engaged | Everything, including a real test offset through both relays |
| **W2 Parked, post-run** | After IG1 goes off, ~1 s, then sleep | IG1 off · Vcc2 < 0.5 V (EPS unpowered) · RT+ off · VSP = 0 | Same as W1. Leaves a fresh PASS record for the next trip |
| **W3 Driving, disengaged** | Any time the overlay is off | Both coils commanded off **and** both coil currents read zero | Overlay circuit up to the relay NO contacts, ADCs, rails, inputs |
| **W4 Driving, engaged** | Overlay active | — | Continuous, non-intrusive monitors only |

**Why the W1/W2 interlock is enforced in hardware, not just software:** the test offset reaches the EPS only with both relays energized. Each relay needs a switch from CH-B (doc 11 §3). CH-B closes its switches for the test only after it has checked the interlock on its *own* inputs. A CH-A firmware bug therefore can't inject a test offset while the motor relay is live.

If the engine starts during W1 (NEP or RT+ appears), the test offset is removed within one 1 ms cycle and the relays drop. The test is marked "incomplete", not "failed".

## 3. The test list

"Runner" drives the stimulus; "observer" independently measures the result. Times are targets.

### W0: power-up (every reset, ~100 ms)

| # | Test | Runner / observer | Catches |
|---|---|---|---|
| T01 | PMIC analog + logic BIST (ABIST/LBIST), register CRC | each PMIC / own MCU reads the status | PMIC monitor faults |
| T02 | TMS570 PBIST (RAM), LBIST (CPU), CCM-R4 lockstep self-test, flash ECC, ESM | CH-B / PMIC-B via the ESM pin | Monitor CPU, memory |
| T03 | F28004x self-test library: CPU, RAM, flash CRC, DCC clock check, ERRORSTS path | CH-A / PMIC-A via ERRORSTS | Control CPU, memory, clock |
| T04 | ADC self-tests (VREFLO / VREFHI conversions on both MCUs) | own MCU | ADC gain / offset |
| T05 | Link up: UART CRC + counter handshake, firmware version / CRC exchange | A ↔ B | Mismatched firmware, dead link |

### W1 / W2: key-on engine off, and parked post-run (~500 ms total)

| # | Test | How | Pass criterion | Catches |
|---|---|---|---|---|
| T10 | Cross-channel input plausibility | Both MCUs read VBAT, IG1, Vcc2, VSP, NEP, RT+, WLP, seat sense through their own dividers | A = B within tolerance | Divider, clamp, ADC faults |
| T11 | Watchdog and ENDRV, per PMIC | In the PMIC's DIAGNOSTIC state, the MCU gives a wrong Q&A answer, then forces its error pin | ENDRV goes low each time (ENDRV_ERR bit + the switch node, read by **both** MCUs) | Latent "PMIC can't stop" |
| T12 | Enable paths, per switch (4 switches) | Each switch commanded with GPIO only, then ENDRV only | No coil current in either case | Stuck GPIO, stuck gate, AND-gate fault |
| T13 | Coil switches, one at a time | Each high side and low side alone, then each pair | Coil current only when both switches of that coil are on; coil node voltages correct | Shorted switch (latent), open coil, open shunt |
| T14 | **Relay blocking**, each relay alone | K1 energized alone with the test offset applied; then K2 alone | EPS-side readback **unchanged** (the de-energized relay must block) | Contact stuck on NO: the latent that matters most |
| T15 | **Relay transfer + authority limit** | Both relays energized, DAC to +full scale, then −full scale, then a mid step | Offset at the EPS = commanded × k ± tolerance; **|Δ| at full scale ≤ Δmax + tolerance and ≥ Δmax − tolerance**; VT3 + VT6 sum unchanged | Drifted summer resistor (the hardware limit SM-05 itself), open contact, readback gain |
| T16 | **Release time** | With the offset applied, drop K1; time until EPS-side = sensor. Repeat for K2 | < 5 ms each | Shorted coil clamp (slow release), sticky contact |
| T17 | Readback agreement | During T15, CH-A readback, CH-B readback and the new OVL pre-relay taps all measure the same offset | All three agree within tolerance | A readback that would mask a real fault |
| T18 | Disengage loop at rest | NC coil-return loop must read closed, NO contact open | Complementary | Broken or shorted switch wiring |
| T19 | Thermal | Relay-area NTC + both MCU die sensors | < 80 °C (relays are 85 °C parts) | Over-temperature |
| T20 | Driver lamp | Lamp on for the test; both channels read the lamp node | Lamp current path present | Open lamp or wire |

**Test offset amplitude:** full scale is allowed only in W1/W2, because the EPS motor can't move. In W2 the EPS is unpowered: the offset goes into an unpowered EPS input through the summer's 100 Ω, at most 0.37 V. **TBD-DATA:** confirm on the bench that this sets no EPS DTC in W1 and back-powers nothing in W2. If it does, W2 uses half amplitude.

### Arm gesture (once per trip, before the first engagement)

| # | Test | How | Catches |
|---|---|---|---|
| T21 | Driver presses and releases the disengage button | NC loop must open (pull-up rises, seen by both MCUs) **and** NO must close, then both return | A welded or bypassed hardware kill switch. It's the only latent no automatic test can reach, so the driver checks it, in one press |

This doubles as a sensible user step: the driver confirms they're ready, and proves their off-switch works.

### W3: driving, disengaged (every 10 s, 50 ms each)

| # | Test | How | Pass | Abort |
|---|---|---|---|---|
| T30 | Overlay circuit | DAC small steps (±25 % full scale); OVL taps measure the summer outputs before the relays | Matches k × command | **If the EPS-side reading moves at all during the test → stop, latch a relay fault** |
| T31 | Coil currents | Both coils off | Both read zero | — |
| T32 | Rails, temperature, inputs | As T10, T19 | — | — |

### W4: driving, engaged (every 1 ms cycle unless noted)

| # | Monitor | Reaction |
|---|---|---|
| T40 | Both readbacks vs each channel's own expected offset, VT3 + VT6 sum | > ε for 5 ms → disengage |
| T41 | OVL taps vs EPS-side: equal through closed contacts | Mismatch → disengage (contact resistance or bypass fault) |
| T42 | Coil currents both present and in range | Drop or excess → disengage |
| T43 | Q&A watchdogs, ESM / ERRORSTS, rail monitors | PMIC drops ENDRV by itself |
| T44 | E2E on CAN (CRC, counter, 50 ms timeout), SAS plausibility (rate, range) | Ramp-out |
| T45 | Cross-channel heartbeat with timestamps: each MCU checks the other's cycle time | > 10 % drift → disengage (catches a slow clock that each watchdog alone may accept) |
| T46 | VBAT 9–16 V, temperature < 80 °C, seat sense | Out of window → ramp-out, before the relays could drop on their own |
| T47 | Background: RAM/flash CRC slices, CPU tests from the safety libraries, ADC self-test channels | Fail → disengage, latch |

## 4. Results: redundant, simple to read

- **2oo2 PASS to arm.** Each MCU keeps its own result bitmap. Both are swapped over the link and must match. Engagement is allowed only if both say PASS. Either one can block.
- **Fresh enough.** A PASS from W1 this trip, or from W2 at the last key-off with the same power-cycle counter (so a battery disconnect forces a new W1). Plus T21 this trip.
- **Fail-safe defaults.** A test that can't finish (engine started, interlock lost) gives "not ready", never PASS.
- **Remembered.** Faults are stored in each MCU's non-volatile memory (F28004x flash emulation, TMS570 EEPROM-emulation bank) with a CRC. Clearing needs a service command accepted by **both** MCUs.
- **No re-engagement after a fault in the same trip.** A clean, planner-requested disengage may re-engage. A fault-driven one can't until the next key-on.
- **Easy to see:**

  | Where | Shows |
  |---|---|
  | Per-channel status LED | Off = asleep · slow blink = testing · solid green = ready · red blink code = failed test number |
  | Driver lamp (IND_OUT) | On during the self-test, off when ready, on steady for any fault or when disengaged by a fault |
  | CAN frame `ICARUS_DIAG` | Window, test bitmap per channel, last failed test, firmware CRCs, temperatures, coil currents |
  | Bench command (CAN or UART) | Runs W1 on demand and prints a PASS/FAIL table |

## 5. Hardware added for testability and the missing safety functions

| # | Addition | Why it was missing | Parts | ≈ $ |
|---|---|---|---|---|
| **M1** | **Hardware disengage:** the driver switch's NC contact sits in **both relays' coil return** (J1 pin 4). Pressing it drops the relays with no MCU involved. A 10 kΩ pull-up lets both MCUs see the loop open | The switch only went to the MCUs. A driver's off-switch must work even if both MCUs are wrong. The loop also wets the switch contacts with the coil current (~24 mA). A broken wire drops the relays, which is safe | Pull-up, TVS on the pin, 2 dividers | 0.40 |
| **M2** | **Coil-current shunts**, one per coil (10 Ω), read by both MCUs | The TPS1H100 current sense went away in rev 2.0. Without it, "coil energized" and "contact moved" can't be told apart | 2 shunts, 4 RC filters | 0.30 |
| **M3** | **OVL pre-relay taps** (summer outputs at the K1 NO contacts), scaled to both MCUs | Needed for T30 (testing the overlay while driving disengaged) and T41 (contact integrity while engaged) | 4 dividers + RC | 0.40 |
| **M4** | **Redundant driver warning:** a second low-side FET from CH-A in parallel with CH-B's on IND_OUT; both MCUs read the lamp node | One channel alone could warn the driver. A fault-driven disengage must never be silent (SG4) | FET, divider | 0.30 |
| **M5** | **Connector-seated sense:** J1 pin 14 (a spare GND) becomes SEAT_N, jumpered to pin 13 GND inside the mating plug | A half-seated plug can intermit the torque lines. Now it's seen, and engagement is blocked | Pull-up, 2 dividers, ESD | 0.25 |
| **M6** | **Relay-area NTC** (AEC-Q200), read by both MCUs | The relays are 85 °C parts in a cabin that can exceed that when parked in the sun | NTC, 2 dividers | 0.40 |
| **M7** | **Sleep and post-run hold:** IG1 wakes each PMIC (IGN pin). The PMIC's own POST_RUN latch (IGN_PWRL) keeps it on after key-off until the MCU clears the bit (doc 13). **No extra transistors** | Rev 1.0 F34: the board never turned off. A hung MCU: with WD_RST_EN set, a failed Q&A watchdog resets it; at every boot the firmware clears IGN_PWRL if IGN is low and no post-run work is pending. **Open (S1):** SLVSDJ1A doesn't say whether IGN_PWRL survives a RESET, so an MCU that never boots could keep the board on. Check with TI or on the bench | IGN filter (10 kΩ, 100 nF, TVS) per PMIC | 0.20 |
| **M8** | **VBAT engagement window** (9–16 V) | The relays pick up at 9 V. Engagement has to end before a dip drops them uncontrolled | Firmware (existing VBAT monitor) | 0 |
| **M9** | **Cross-channel clock check** (T45) | Each watchdog tolerates its own clock drift; the other MCU doesn't | Firmware | 0 |
| **M10** | **Authority-limit proof test** (T15) | The resistor limit SM-05 was never tested. A drifted resistor would raise the bound silently | Firmware | 0 |

About 40 extra placements, around $3. **ADC budget:** CH-B goes to about 13 analog inputs (VT raw ×2, VT EPS ×2, OVL ×2, coil current ×2, 5 V, 3.3 V, VBAT, Vcc2, NTC). Seat, disengage loop and lamp go to digital pins. Check against the TMS570LS0432 PZ ADC channel count at schematic time (C3).

**J1 change:** pin 14 GND → SEAT_N. Five GND pins remain (2, 11, 13, 18, 21), and the ones shielding the VT lines (18, 21) stay. The R1 rule still holds: no pulsed signal next to a VT pin.

## 6. What the tests buy (placeholder rates, `rev2_trade.py`)

| Variant | Case | SPFM | LFM | PMHF (FIT) |
|---|---|---|---|---|
| Rev 2.0 (doc 11) | Baseline | 99.99 % | 99.08 % | 0.062 |
| **Rev 2.0 + this plan** | Baseline | 99.99 % | **99.50 %** | 0.062 |
| **Rev 2.0 + this plan** | FIT ×3, DC capped at 90 % | 99.89 % | **99.01 %** | 1.739 |
| Rev 2.0 + this plan, no credit for CH-A self-diagnostics | FIT ×3, DC capped at 90 % | 99.28 % | 99.00 % | **11.17 (fails)** |

**Reading:**
- The tests do their job: latent-fault coverage goes from 99.08 to 99.50 %. Every latent failure mode in the model now has a named test.
- **They don't change PMHF,** which is set by faults that happen *while driving*. Self-tests can't catch those; only the continuous monitors can.
- **Condition C1 from doc 11 is unchanged:** CH-A's own error detection (ERRORSTS + Q&A watchdog) has to be backed by TI's FMEDA, or CH-C comes back.
- **What the model doesn't score:** M1 (hardware disengage) and M4 (redundant warning) are about the driver's controllability and SG4. They matter for the HARA argument, not the SG1 hardware metric.

## 7. Open items

| # | Item |
|---|---|
| S1 | ~~IGN wake, DIAGNOSTIC state, ENDRV~~ **Confirmed** in SLVSDJ1A §5.3.7, §5.4.16 (doc 13). Open detail: whether a stuck MCU holding IGN_PWRL is released by a watchdog-driven RESET; check with TI or on the bench |
| S2 | Bench-check that the W1 test offset sets no EPS DTC and that W2 back-feeds nothing (logger) |
| S3 | Set the tolerances (ε, Δmax ± tol, release-time limit) from bench data |
| S4 | ~~ADC count~~ **Confirmed:** MibADC has 16 inputs; CH-B needs 14, plus the PMIC DIAG_OUT = 15 |
| S5 | T21 makes the driver press the button once per trip. If that's unacceptable, T21 moves to a service interval, and the hardware-kill latent gets a longer exposure time in the model |
