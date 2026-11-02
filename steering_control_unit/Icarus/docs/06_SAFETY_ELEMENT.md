# 06: Icarus safety element and analysis findings

| | |
|---|---|
| Revision | 0.7, 2026-11-01 (diverse CH-B MCU; rev 0.5: indicator LEDs; rev 0.4: speed authority, LiPo backup, PCB) |
| Based on | Schematic rev 0.5 (`hardware/gen/design.py`), `04_TECHNICAL_SAFETY_CONCEPT.md` |
| Model | `safety/element_model.py` (elements, failure modes, safety mechanisms) |
| Analysis | `safety/analyze.py` → `07_FMEDA_FTA.md` + `safety/fmeda.csv` |

## 1. The safety element

Icarus is modelled as one hardware **safety element** that carries **SG1** (ASIL D): *no unintended or out-of-envelope overlay torque at the EPS*. The element is split into 20 sub-elements. Each one is tied to the schematic by sheet and group, so the part counts and failure rates follow the design.

```
Icarus (SG1, ASIL D)
├── Shared, not decomposed → must meet ASIL D on its own
│   ├── E01 Input protection             ├── E13 Signal relays K1, K2
│   ├── E14 EPS-side node (hold caps)    └── E18 Vehicle connector J1
├── CH-A control, ASIL B(D)
│   ├── E02 Supply chain A    ├── E04 MCU A + clock    ├── E05 Supervision A
│   ├── E08 Overlay analog    └── E09 Readback A
├── CH-B monitor, ASIL B(D)
│   ├── E03 Supply chain B    ├── E06 MCU B    ├── E07 Supervision B + charge pump
│   ├── E11 Readback B        └── E19 Expected-Δ reference (R2)
├── CH-C guard, ASIL B(D) (adds margin beyond the B(D)+B(D) split)
│   └── E10 Guard comparators + latch
├── Cross-channel
│   ├── E12 Coil drivers (high side A, low side B, guard FET C)
│   ├── E15 CAN interfaces    ├── E16 Car-side inputs    └── E17 Analog monitors
└── E99 Not SG1-relevant (debug, LEDs, FSI series R, ...)
```

**Allocation rule.** Elements inside a channel inherit that channel's decomposed ASIL. Elements every channel depends on are the relays, the connector, the EPS-side node and the input protection. Those can't take a decomposed ASIL; they have to be argued at ASIL D themselves (ISO 26262-9 §5). They're the first place to look for weaknesses, and the analysis found one there.

**Effect classes** (per failure mode):
- **direct:** violates SG1 unless a mechanism detects it and reacts within the FTTI.
- **latent:** removes a barrier, so it needs a second fault to violate SG1.
- **safe:** no SG1 effect. SG3 and SG4 effects are noted in the model but not scored here.

## 2. Results (placeholder failure rates)

| Metric | Rev 0.1 | Rev 0.2 (R1 + R2) | Rev 0.3 (R3–R5, R7, R8) | **Rev 0.4 (SM-23, backup + SM-24)** | ASIL D target |
|---|---|---|---|---|---|
| SPFM | 99.92 % | 99.99 % | 99.99 % | **99.99 %** | ≥ 99 % |
| LFM | 98.34 % | 98.35 % | 98.92 % | **98.69 %** | ≥ 90 % |
| PMHF | 0.47 FIT | 0.064 FIT | 0.065 FIT | **0.095 FIT** | < 10 FIT |
| Single-point faults | 1 (J1 pins 6–7) | 0 | 0 | **0** | — |

Rev 0.4's backup supply adds a common-cause path (boost overvoltage into both 5 V rails). Without SM-24 it took PMHF to 0.33 FIT; with SM-24 it's 0.095 FIT. The SM-24 cutoff itself is an untested latent fault, which is why LFM dipped slightly (finding F13).

Rev 0.3 adds latent-fault tests (LFM) and SG3 fixes, plus two logic corrections (F8, F9) that a block-level FMEDA can't score but that matter for the argument. PMHF is unchanged, as expected.

**Sensitivity** (rev 0.3), which matters more than the baseline because the input data are placeholders:

| Case | PMHF | ASIL D? |
|---|---|---|
| All failure rates ×3 | 0.19 FIT | yes |
| Every DC claim capped at 90 % | 0.48 FIT | yes |
| Rates ×3 **and** DC capped at 90 % | 1.43 FIT | yes |
| Same pessimistic case **without R2** | **10.8 FIT** | **no** |

**Reading:** with R1 and R2, the architecture meets the ASIL D hardware metrics with a margin of about 7× even under pessimistic data. Without R2, the pessimistic case fails.

## 3. Findings

| # | Finding | Severity | Evidence |
|---|---|---|---|
| **F1** | **Single-point fault in the connector.** J1 pin 6 (CAN_L) sits next to pin 7 (VT3_SNS). A short couples CAN pulses into the torque line *upstream of every check*, so no channel can see it as an Icarus fault. It is the only SPF in the design. | High (ASIL D shared element) | **Fixed by R1.** Rev 0.1: cut set CS2, 0.10 FIT, 21 % of PMHF |
| **F2** | **A wrong CH-A command inside the envelope has only one check.** CH-B's software cross-check (SW-04) is the single mechanism. The guard can't see it, because it only bounds magnitude. This is the largest residual: **73 % of PMHF**, and it alone breaks the target under pessimistic data. | High | **Fixed by R2.** Rev 0.1: 0.34 FIT residual. Rev 0.2: 0.034 FIT |
| F3 | **"Guard cannot trip" was the largest latent fault** (2.0 FIT latent). The key-on self-test only pulled TH_HD, so the low-side windows weren't exercised. | Medium | **Fixed by R3** (rev 0.3): 0.2 FIT latent, off the top-5 list |
| F4 | **An open relay contact leaves an EPS input floating** (SG3, not SG1). A floating torque input can set **latching** DTCs 11/15/16, which means no assist until the code is erased. | Medium (SG3, ASIL A) | **Fixed by R4** (rev 0.3) |
| F5 | **Pin 10 (VT6_EPS) is next to pin 11 (VCC2_TAP, 5 V).** A short forces VT6 to 5 V: an out-of-range latching DTC (SG3). | Low–medium | 07 §4 |
| F6 | **The redundant relays and coil switches barely change PMHF**: the double-weld cut set is 9e-12 FIT. Their value is fault *tolerance* (one stuck part still reaches the safe state), not the metric. Each one adds a latent fault that needs a key-on test (SM-09, SM-19). | Info | CS6, CS7 |
| F7 | **The supply supervisors' own failures were latent** (0.73 FIT each). Nothing tested that the TPS3702 could trip. | Low | **Fixed by R5** (rev 0.3): UV path tested, 0.07 FIT latent |
| **F8** | **The hardware authority limit (SM-05) didn't hold against an op-amp fault.** The attenuator sat ahead of a follower; a railed follower could apply ~3.3 V to the EPS line until the relays reacted (≤ 20 ms). FSR-02 says "even with both MCUs failed", but the circuit said nothing about the op-amps. | High (logic) | **Fixed by R8** (rev 0.3): bound is now 0.37 V for any upstream op-amp fault |
| **F9** | **CH-B alone could clear the guard latch.** A monitor-channel firmware fault could un-latch a real trip at runtime, turning the latching guard into a non-latching one. | Medium (logic) | **Fixed by R7** (rev 0.3): 2oo2 clear |
| **F10** | **Speed-authority switch wired backwards (caught before layout).** The first rev 0.4 draft swapped the TS5A3159's COM and NC pins, so high speed, or a floating control line, selected **full** authority. | High (logic) | **Fixed** in rev 0.4: DBV pinout checked against TI's pin table; every failure direction now gives HALF or zero |
| F11 | **Wrong boost part number.** The first draft used the TPS61091, which is the **3.3 V** fixed version. | Medium | **Fixed:** TPS61092 (5.0 V) |
| F12 | **Boost low-battery output not connected** (TPS61092 pin 12 missing). | Low | **Fixed:** LBO → CH-B GPIO with pull-up |
| **F13** | **The LiPo backup is a common-cause path into both channels.** A boost overvoltage reaches +5V0_A and +5V0_B together through the ORing diodes. | High (DFA) | **Mitigated by SM-24** (OV cutoff on the ideal-diode enables). Its own failure is latent and untested; a key-on test hook is the open item |
| F14 | **Indicator LEDs touch safety nets** (rev 0.5: 22 LEDs). Each was placed so that no LED fault can work against a safety mechanism: (a) rails and spare GPIOs (no safety function); (b) the guard-trip LED is on the latch's Q output, while the coil gate uses Q-bar; (c) the relay-energized LEDs sit across each coil behind a 1N4148WS, so they can't energize a coil and can't shunt the release Zener (SM-12); (d) the SUP_OK and LBO LEDs hang on open-drain lines, where a shorted LED only adds 2.2 kΩ of pull-up, which the open-drain driver still sinks (< 2 mA). There's no LED on VLI or VBK5, so the cell isn't drained. Side effects: the coil LED adds ~0.9 mA to the TPS1H100 current-sense reading. | Low | Kept out of the SG1 elements (E99, "Indicators" groups); metrics unchanged: SPFM 99.986 %, LFM 98.73 %, PMHF 0.095 FIT |
| F15 | **Test pads sit on safety nets** (rev 0.5: 37 bare 1.5 mm pads in three clusters: incoming, outgoing, measured). A pad is copper on its net, never in series, so it can't open a path. The risks are a probe bridging two pads, and stub length on the torque lines. Mitigations: a GND pad at the start of each row; the torque-line pads sit in the incoming / outgoing clusters next to the connector and relays, not across the board. The silk and the schematic say **bench only, don't probe VT3/VT6 while driving**. | Low | E99; metrics unchanged |
| **F16** | **Both channels used the same MCU** (F280049C), so one silicon erratum, compiler bug or toolchain fault could hit control and monitor together: the weak spot of the B(D)+B(D) independence argument. | High (DFA) | **Fixed in rev 0.7:** CH-B is a TMS570LS0432-Q1 (Arm Cortex-R4F, lockstep cores, ESM nERROR into the enable gate, SM-25). The link moves from FSI to a CRC-protected UART; the expected-Δ reference from a DAC to N2HET PWM + RC (still read back on REF3M_B); the clock from INTOSC to a quartz crystal. PMHF 0.096 → 0.086 FIT (placeholder rates) |
| F17 | **The CH-B 1.2 V core regulator dissipates up to 0.55 W.** It's fed from +5V0_B because the 300 mA 3.3 V LDO can't also carry the 145 mA core. | Low | WSON thermal pad to the planes; check the copper area and T_A max at layout review. Its output is supervised by a TPS3702-Q1 window into nPORRST |

## 4. Recommendations

| # | Change | Fixes | Effect (modelled) |
|---|---|---|---|
| **R1** ✅ applied, rev 0.2 | **Re-pin J1.** New order: …CAN_H · CAN_L · **GND** · VT3_EPS · VT3_SNS · **GND** · VT6_SNS · VT6_EPS · TSGND_TAP · VCC2_TAP… A torque pin can now only short to GND, a static tap, or its own SNS/EPS partner. | F1, F5 | SPF → 0 |
| **R2** ✅ applied, rev 0.2 | **Give CH-C the expected Δ.** CH-B's DACA → slew RC → the same k = 0.109 attenuator → REF3 = VMID − Δexp, plus an inverter for REF6 = VMID + Δexp. These replace VMID_B as the D3/D6 references, so D3/D6 sit at VMID only when the applied Δ equals CH-B's expected Δ. VMID_B and the window ladder now run from +3V3_B, which keeps them ratiometric with the CH-B DAC. CH-B reads REF3 back (REF3M_B). | F2 | PMHF 0.47 → 0.064 FIT; pessimistic case passes |
| R3 ✅ applied, rev 0.3 | **Test every guard window at key-on.** Two test modes: GUARD_TEST collapses the ladder (high windows), GTEST_LO opens the ladder-bottom FET so all thresholds rise (low windows). All six comparators exercised; SM-17 DC claim raised to 99 %. | F3 | LFM 98.35 → 98.92 % |
| R4 ✅ applied, rev 0.3 | **100 kΩ bypass from each VT_SNS to its VT_EPS**, across the relay pair. If a contact opens, the EPS input follows the sensor instead of floating; during the break-before-make gap it also keeps the hold cap tracking the sensor. Loads the 100 Ω overlay output by 0.1 %. TBD: confirm the EPS input impedance is ≫ 100 kΩ. | F4 | SG3: avoids latching DTCs |
| R5 ✅ applied, rev 0.3 | **Key-on supervisor test (SM-21).** 10 kΩ between +5V0 and the TPS3702 SENSE; a test FET pulls SENSE to 2.5 V, UV must assert, SUP_OK must drop. The TPS3850 is deliberately not tested this way: pulling its SENSE resets the MCU. Its watchdog path is covered by SM-03L and its UV path by the MCU's own brown-out reset. | F7 | LFM up |
| **R7** ✅ applied, rev 0.3 | **2oo2 clear of the guard latch (SM-22).** SN74LVC1G08-Q1 ANDs a CH-A and a CH-B GPIO ahead of the clear FET. | F9 | Logic fix; not scored |
| **R8** ✅ applied, rev 0.3 | **Resistor-weighted summers (SM-05).** k = 1.1 k / 10 k inside each summer; the DAC-buffer and VMID-buffer stages can no longer exceed 0.37 V at the EPS even when railed. Removes one op-amp quad and the inverter stage. | F8 | Logic fix; E08 residual slightly lower |
| R6 | **Replace the placeholder failure rates** with SN 29500 / IEC 61709 values and vendor FIT data. Use the F28004x safety manual's FMEDA for E04/E06. **Justify every DC claim** by fault injection on the HIL bench. | All | Turns the metrics into evidence |

R1 and R2 were the two that changed the safety argument. Both are now in the schematic (rev 0.2: 400 parts, ERC 0, netlist identical to `design.py`).

**Side effects of R2 to keep in mind:**
- **Bound on the applied overlay.** The guard no longer bounds |Δ| by itself; it bounds |Δ − Δexp| ≤ 0.25 V. Δexp is limited in hardware by the same attenuator, so |Δ| ≤ 0.18 + 0.25 V even in the worst case. The CH-A authority limit (SM-05) still caps the applied Δ at 0.18 V.
- **Firmware.** CH-B must drive its DAC before engagement, at Δexp = 0 (mid-scale). An unprogrammed DAC gives a false trip, which is safe.
- **New latent fault.** If CH-B computes the same wrong value as CH-A, the reference masks it. That needs a common-cause fault in both MCUs, and the DFA covers it. It's modelled as element E19.

## 5. Limits of this analysis

- The FMEDA is at **block level**. Failure modes are shared out per element, not computed per pin of every part. That's normal for the concept phase. Before release it needs a part-level FMEDA on the frozen schematic.
- **Common-cause and dependent failures** are covered qualitatively in `04_TECHNICAL_SAFETY_CONCEPT.md` §6. The same-type MCU in CH-A and CH-B is still the main open item for the DFA. R2 also helps here, because the comparison moves into the software-free CH-C.
- The analysis assumes the overlay is **always engaged** (worst case). It doesn't credit pass-through time.
- **SG3 (loss of assist) and SG4 (silent dropout)** aren't scored. Their failure modes are tagged in the model, so they can be added as further targets.
