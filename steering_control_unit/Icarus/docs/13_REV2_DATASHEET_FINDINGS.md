# 13: Rev 2.0 datasheet findings

| | |
|---|---|
| Date | 2026-11-15 |
| Data file | `hardware/gen/rev2_parts.py`: pin maps, footprints, external values, with a self-check (`python rev2_parts.py`) |
| Datasheets | TI SLVSDJ1A (TPS65381A-Q1), TI TCAN1044A-Q1 family, TE Axicom P2 V23079, Panasonic TX catalog (Rev 2010). Downloaded with the user's OK to the session scratchpad, not the repo |

## 1. Confirmed

| Question (from docs 11 / 12) | Answer | Source |
|---|---|---|
| Can each PMIC watch its MCU's error pin? | Yes. The MCU ESM on ERROR/WDI has two modes: TMS570 mode (low pulse longer than a programmable threshold) and PWM mode. It's off by default; enable it with the watchdog in Q&A mode | SLVSDJ1A §5.4.16 |
| C4: does the F280049C need an inverter? | **No.** F28004x ERRORSTS is active low, fixed polarity, with a pull-down. That matches TMS570 nERROR, so both PMICs use TMS570 mode | TI F28004x→F28003x migration guide, error-status table |
| Can the MCU test the error monitor itself (T11)? | Yes. TI gives a DIAGNOSTIC-state procedure: inject a pin failure, check ERROR_PIN_FAIL = 1 and ENDRV low, then clear | §5.4.16 |
| S1: power-up and post-run | **The PMIC has its own POST_RUN latch.** At boot the MCU sets IGN_PWRL (only writable in DIAGNOSTIC). After key-off, IGN low is ignored until the MCU clears the bit, then the PMIC goes to STANDBY. **The self-hold transistors from doc 12 M7 aren't needed** | §5.3.7, Fig. 5-1 |
| Does the PMIC test itself? | Analog BIST at every power-up, logic BIST when leaving DIAGNOSTIC; either failing → SAFE (ENDRV low). Matches self-test T01 | §5.3, Fig. 5-1 notes |
| Low-battery behaviour | Initial power-up needs 5.8 V. Full operation 7–36 V; 5.8–7 V stays functional with the buck in dropout | Recommended operating R1.1, R1.2 |
| S4: CH-B ADC inputs | The TMS570 MibADC has 16 inputs. Rev 2.0 CH-B needs 14 (VT raw ×2, VT EPS ×2, OVL ×2, coil current ×2, 5 V, 3.3 V, VBAT, Vcc2, NTC, ground tap), plus the PMIC's DIAG_OUT = **15 of 16** | `design.py` TMS570 map (from SPNS186C) |
| CAN transceiver pins | TCAN1044AV-Q1 (SOIC-8): 1 TXD, 2 GND, 3 VCC (5 V), 4 RXD, 5 VIO (3.3 V), 6 CANL, 7 CANH, 8 STB. STB and TXD have internal pull-ups, so a dead MCU leaves it in standby (bus quiet) | TCAN1044A-Q1 §5 |
| K2 relay | **Panasonic TX2SA-12V-Z.** 2 Form C, gold-clad twin crossbar contacts, 1,028 Ω / 11.7 mA coil (same as K1), pick-up 9 V, drop-out 1.2 V, release ≤ 4 ms, -40 to +85 °C. **Same pin numbers and positions as the TE P2:** coil 1(+)/12, COM 4/9, NC 3/10, NO 5/8. The pad row spacing differs (P2 8.05 mm, TX-SA 7.24 mm), so each has its own footprint. $2.31, 244 in stock | Panasonic TX p.3, p.6 |
| K1 relay | TE V23079D1003B301: 1,029 Ω coil, AgNi gold-covered **bifurcated** contacts rated down to 100 µV (true dry-circuit), release ≤ 4 ms without diode, ≤ 6 ms with. TE lists "automotive equipment as CAN bus" among its uses; no AEC claim | TE P2 p.2, p.3, p.5 |

## 2. New findings

| # | Finding | Severity | Action |
|---|---|---|---|
| **D1** | **Load dump vs the PMIC's 40 V absolute maximum.** VBATP and VBAT_SAFING are rated -0.3 to 40 V. A 24 V-standoff TVS (needed to survive a 26 V jump start, F20) clamps near 39 V at its full rated current, which leaves under 1 V of margin. | High | Before freezing: either (a) confirm the Insight has centralized load-dump suppression and size the TVS for the residual pulse, or (b) add a series element that limits TVS current so the clamp stays near 30 V. The 1.1 A PTC already in series helps but isn't a clamp. Check against ISO 16750-2 test B |
| **D2** | **The TMS570 core regulator FET has no current limit or thermal protection of its own.** TI recommends feeding it from VDD6, so the buck's 1.3 A limit protects it. Worst case dissipation is (6.6 − 1.14) V × 145 mA ≈ 0.8 W during LBIST. | Medium | AEC-Q101 logic-level NMOS in DPAK/LFPAK, rated well above 1.3 A, drain on VDD6, with copper area for 0.8 W. Part to pick at schematic time |
| **D3** | **The 6 V buck needs a deliberate ESR.** Its hysteretic control wants 100–300 mΩ output ESR, which ceramic caps don't have. | Medium | 2× 22 µF X7R plus a 0.1 Ω series resistor (TI's rule: R ≈ L / (15 × C) with 33 µH and 22 µF) |
| **D4** | **DIAG_OUT needs ≥ 100 kΩ series resistance** into the ADC filter. | Low | 100 kΩ + 1 nF. This also lets each MCU read the PMIC's internal analog monitors as an extra self-test |
| **D5** | **Unused sensor supply (VSOUT1):** it's off by default (VSOUT1_EN = 0, reset only by power-on). If it were ever enabled and then faulted, the next BIST would put the PMIC in SAFE. SLVSDJ1A gives no explicit unused-pin rule. | Low | Firmware never sets VSOUT1_EN. Ask TI for the unused-pin recommendation |
| **D6** | **Can a hung MCU keep the board on after key-off?** It's unclear whether IGN_PWRL survives a watchdog-driven RESET. | Medium (battery drain, not SG1) | Firmware clears IGN_PWRL at every boot when IGN is low. Confirm with TI or on the bench (doc 12 S1) |
| **D7** | **The ERRORSTS position:** rev 1.0 used GPIO24, which is also a boot-mode pin (F23). ERRORSTS can also be routed to GPIO28 or GPIO29. | Medium (bring-up) | Use GPIO28/29 for ERRORSTS in the rev 2.0 pin map, if the F28004x mux table confirms it, and keep GPIO24 free for flash boot |
| **D8** | **TE's site blocks scripted downloads.** The P2 datasheet came through the web reader instead. The reader's text summary was empty; the terminal drawing was read from the rendered page. | Info | Pins confirmed visually on both relays |

## 3. What's still needed before drawing the schematic

1. The F28004x pin-mux table, to confirm ERRORSTS on GPIO28/29 (D7). TI's online TRM pages are enough; no download needed.
2. Part numbers still open: the VDD1 NMOS (D2), the 6 V buck Schottky, the 33 µH inductor, the reverse-battery Schottky (NRVBA340 is obsolete), and the TVS strategy (D1).
3. Then rewrite `design.py` for rev 2.0 using `rev2_parts.py`.
