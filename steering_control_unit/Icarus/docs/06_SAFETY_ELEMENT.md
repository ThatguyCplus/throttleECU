# 06: Icarus safety element and analysis findings

| | |
|---|---|
| Revision | 0.2, 2026-11-01 (R1 + R2 applied) |
| Based on | Schematic rev 0.2 (`hardware/gen/design.py`), `04_TECHNICAL_SAFETY_CONCEPT.md` |
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

| Metric | Rev 0.1 (first analysis) | **Rev 0.2 (R1 + R2 applied)** | ASIL D target |
|---|---|---|---|
| SPFM | 99.92 % | **99.99 %** | ≥ 99 % |
| LFM | 98.34 % | **98.35 %** | ≥ 90 % |
| PMHF | 0.47 FIT | **0.064 FIT** | < 10 FIT |
| Single-point faults | 1 (J1 pins 6–7) | **0** | — |

**Sensitivity** (rev 0.2), which matters more than the baseline because the input data are placeholders:

| Case | PMHF | ASIL D? |
|---|---|---|
| All failure rates ×3 | 0.19 FIT | yes |
| Every DC claim capped at 90 % | 0.47 FIT | yes |
| Rates ×3 **and** DC capped at 90 % | 1.42 FIT | yes |
| Same pessimistic case **without R2** | **10.7 FIT** | **no** |

**Reading:** with R1 and R2, the architecture meets the ASIL D hardware metrics with a margin of about 7× even under pessimistic data. Without R2, the pessimistic case fails.

## 3. Findings

| # | Finding | Severity | Evidence |
|---|---|---|---|
| **F1** | **Single-point fault in the connector.** J1 pin 6 (CAN_L) sits next to pin 7 (VT3_SNS). A short couples CAN pulses into the torque line *upstream of every check*, so no channel can see it as an Icarus fault. It is the only SPF in the design. | High (ASIL D shared element) | **Fixed by R1.** Rev 0.1: cut set CS2, 0.10 FIT, 21 % of PMHF |
| **F2** | **A wrong CH-A command inside the envelope has only one check.** CH-B's software cross-check (SW-04) is the single mechanism. The guard can't see it, because it only bounds magnitude. This is the largest residual: **73 % of PMHF**, and it alone breaks the target under pessimistic data. | High | **Fixed by R2.** Rev 0.1: 0.34 FIT residual. Rev 0.2: 0.034 FIT |
| F3 | **"Guard cannot trip" is the largest latent fault** (2.0 FIT latent). The key-on self-test only pulls TH_HD, so the low-side windows and the sum window aren't exercised. | Medium | 07 §7 |
| F4 | **An open relay contact leaves an EPS input floating** (SG3, not SG1). A floating torque input can set **latching** DTCs 11/15/16, which means no assist until the code is erased. | Medium (SG3, ASIL A) | E13 mode 2 |
| F5 | **Pin 10 (VT6_EPS) is next to pin 11 (VCC2_TAP, 5 V).** A short forces VT6 to 5 V: an out-of-range latching DTC (SG3). | Low–medium | 07 §4 |
| F6 | **The redundant relays and coil switches barely change PMHF**: the double-weld cut set is 9e-12 FIT. Their value is fault *tolerance* (one stuck part still reaches the safe state), not the metric. Each one adds a latent fault that needs a key-on test (SM-09, SM-19). | Info | CS6, CS7 |
| F7 | **The supply supervisors' own failures are latent** (0.73 FIT each). Nothing tests that TPS3702/TPS3850 can actually trip. | Low | 07 §7 |

## 4. Recommendations

| # | Change | Fixes | Effect (modelled) |
|---|---|---|---|
| **R1** ✅ applied, rev 0.2 | **Re-pin J1.** New order: …CAN_H · CAN_L · **GND** · VT3_EPS · VT3_SNS · **GND** · VT6_SNS · VT6_EPS · TSGND_TAP · VCC2_TAP… A torque pin can now only short to GND, a static tap, or its own SNS/EPS partner. | F1, F5 | SPF → 0 |
| **R2** ✅ applied, rev 0.2 | **Give CH-C the expected Δ.** CH-B's DACA → slew RC → the same k = 0.109 attenuator → REF3 = VMID − Δexp, plus an inverter for REF6 = VMID + Δexp. These replace VMID_B as the D3/D6 references, so D3/D6 sit at VMID only when the applied Δ equals CH-B's expected Δ. VMID_B and the window ladder now run from +3V3_B, which keeps them ratiometric with the CH-B DAC. CH-B reads REF3 back (REF3M_B). | F2 | PMHF 0.47 → 0.064 FIT; pessimistic case passes |
| R3 | **Test each guard window on its own at key-on**: inject on D3, D6 and the sum, high side and low side, via the CH-B DAC-B. That allows a DC claim of ≥ 99 % for SM-17. | F3 | LFM up; not yet modelled |
| R4 | **Fit a 100 kΩ bypass from each VT_SNS to its VT_EPS**, across the relay pair. If a contact opens, the EPS input follows the sensor instead of floating. The load on the overlay output (100 Ω source) is negligible. | F4 | SG3: avoids latching DTCs |
| R5 | **Add a key-on supervisor test.** Briefly pull each SENSE/VDD input through a test FET, or use the TPS3850's manual-reset path, and confirm SUP_OK drops. | F7 | LFM up |
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
