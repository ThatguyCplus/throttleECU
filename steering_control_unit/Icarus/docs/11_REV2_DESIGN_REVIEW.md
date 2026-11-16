# 11: Rev 2.0 design review: ISO 26262 alignment, automotive parts, cost

| | |
|---|---|
| Revision | 2.0 concept, 2026-11-15 |
| Supersedes | The rev 1.0 architecture in `04`, `05`, `06` where they disagree with this file |
| Based on | Rev 1.0 schematic (`hardware/gen/design.py`, 541 parts), `safety/analyze.py` (rev 1.0 FMEDA), `safety/rev2_trade.py` (this review's variant study) |
| Status | **Architecture decision only.** No schematic or PCB change yet. The schematic needs datasheet pin tables, which means downloading PDFs: that waits for the user's OK |

## 1. Verdict

Rev 1.0 is safe but **heavier than its own failure analysis justifies**. It has three channels, a LiPo backup, a speed stage, 22 LEDs and 541 parts. A quarter of the BOM isn't automotive grade, and six -Q1 parts are on 26-week backorder.

Rev 2.0 keeps everything the FMEDA says is pulling its weight, and cuts or replaces what isn't:

- **Kept:** the same two diverse MCUs (F280049C-Q1 control, TMS570LS0432-Q1 lockstep monitor), two cascaded relays, cross-driven coils, the resistor-bounded authority limit and both readbacks.
- **Replaced:** each channel's regulator, supervisor and watchdog chain becomes **one TI safety PMIC per channel (TPS65381A-Q1)**. This is the companion part TI documents for exactly these two MCU families.
- **Dropped:** the LiPo backup, the CH-C analog guard and the speed stage.

**Result on placeholder rates** (same method and rates as rev 1.0):

| | Rev 1.0 | **Rev 2.0** |
|---|---|---|
| Parts | 541 | **~300** (est.) |
| SPFM / LFM / PMHF | 99.99 % / 98.92 % / 0.088 FIT | **99.99 % / 99.08 % / 0.062 FIT** |
| Pessimistic case (FIT ×3, every DC capped at 90 %) | 1.23 FIT | **1.74 FIT** (target < 10) |
| Non-automotive lines in the BOM | 12+ (relays, charger, boost, one-shot, latch, LEDs, connector...) | **0 planned** (relays: see §5.3) |

Rev 2.0 loses a little margin in the pessimistic case (1.23 → 1.74 FIT, still 5.7× under the target). In exchange it drops about 240 parts, two common-cause paths (F13 LiPo backup, F24 hot-car cell) and every non-automotive IC.

## 2. Where Icarus stands against ISO 26262

The standard is a process, not a schematic checklist. This table shows what exists, against the clauses an ASIL D item needs:

| Part / clause | Work product | Icarus status |
|---|---|---|
| 2 §6 | Safety plan, safety culture, confirmation measures | **Missing.** Needed first: who reviews, who assesses independently (for ASIL D, I3 independence for the functional safety assessment) |
| 3 §5–7 | Item definition, HARA, functional safety concept | Done (`01`–`03`) |
| 4 §6 | Technical safety concept | Done (`04`), **updated by this file** |
| 4 §6.4.8 | Hardware-software interface (HSI) spec | **Missing.** Needed before firmware: every pin, register and timing that a safety mechanism relies on |
| 5 §8 | Hardware metrics (SPFM, LFM, PMHF) | Block level, placeholder rates (`07`). **Needs** a part-level FMEDA on the frozen rev 2.0 schematic, with SN 29500 / IEC 61709 rates and the vendors' FMEDA data (TI provides these for the F28004x-Q1, TMS570LS04x and TPS65381A-Q1 on request) |
| 5 §9 | Evaluation of random hardware failures | As above |
| 6 | Software (both channels) | Not started. ASIL B(D) each: MISRA C, unit tests with MC/DC for ASIL B is recommended, not required |
| 8 §11 | Tool confidence (compilers, KiCad, Freerouting, these Python scripts) | **Missing.** `analyze.py` and `rev2_trade.py` compute safety metrics, so they count as tools: they need at least a review and a test (TCL 2 at most) |
| 8 §13 | Qualification of hardware parts | Covered by buying AEC-Q100/101/102/200 parts (§5). Relays need an argument (§5.3) |
| 9 §5 | ASIL decomposition | B(D) + B(D), CH-A + CH-B, kept |
| 9 §7 | Dependent-failure analysis | Qualitative (`04` §6), updated in §4.4 below |

**Honest limit, unchanged from `00`:** documents and a board can't make Icarus compliant by themselves. The safety plan, an independent assessment and the safety case are people and process. These files are the engineering inputs to that.

## 3. Rev 2.0 architecture

```
 POWER: one safety PMIC per channel; nothing shared except the protected battery input
 VBAT ─ PTC (AEC-Q200) ─ Schottky (rev. battery) ─ TVS ─┬─ TPS65381A-Q1 (A): VDD6 buck → VDD5 5 V (overlay op-amps, CAN-A)
   (fuse in the harness, not on the board)             │                    VDD3 3.3 V (F280049C I/O; core from its own internal VREG)
                                                        │                    Q&A watchdog, ERROR pin ← ERRORSTS (via inverter), ENDRV_A
                                                        └─ TPS65381A-Q1 (B): VDD5 5 V (CH-B readback buffers, CAN-B)
                                                                             VDD3 3.3 V + VDD1 1.2 V core (external FET) for the TMS570
                                                                             Q&A watchdog, ERROR pin ← ESM nERROR, ENDRV_B

 ENABLES: each coil needs one switch from CH-A AND one from CH-B; each switch needs its MCU's GPIO AND its PMIC's ENDRV
   K1: high side = GPIO_A · ENDRV_A      low side = GPIO_B · ENDRV_B
   K2: high side = GPIO_B · ENDRV_B      low side = GPIO_A · ENDRV_A
   (SN74LVC1G08-Q1 per switch; discrete AEC-Q101 P-FET high side, N-FET low side; Zener-clamped coil)

 SIGNAL PATH: unchanged from rev 1.0 (two DPDT relays in cascade, NC = OEM pass-through, 100 kΩ bypass, hold caps)
   CH-A DAC → slew RC → resistor-weighted summers (k = 0.110, bounds any upstream op-amp fault to 0.37 V) → K1/K2 NO
   Readback: CH-A on its own scaling; CH-B on its own TLV9064-Q1 buffers, compared against CH-B's OWN expected Δ

 COMMS: CAN to each MCU through its own TCAN1044AV-Q1 (same bus); CH-A ⇄ CH-B UART with CRC + counter
 CONNECTOR: TE AMPSEAL 23 (sealed), R1 pin order kept
```

### Why a safety PMIC is "best practice for this class"

An ASIL-rated controller is normally built as **safety MCU + safety PMIC/SBC**. The PMIC watches the MCU from outside on an independent oscillator and bandgap: a Q&A watchdog, MCU error-pin monitoring, OV/UV on every rail, and an ENDRV output that drops on any fault. The Korean EPS ECU study below uses the same pattern: an asymmetric dual-processor design plus an external watchdog with sequence and time monitoring. TI documents the TPS65381A-Q1 with both of Icarus's MCU families:

- **TMS570:** the ERROR/WDI pin monitors the ESM error output.
- **C2000:** the ERROR input monitors ERRORSTS, with the watchdog in Q&A mode.

One $7.20 part per channel replaces a buck, an LDO, two window supervisors, a window watchdog, an AND gate, the 1.2 V LDO with its 16 Ω dropper (F17, 0.55 W), and the dynamic charge-pump enable. The rev 1.0 supervisors only checked rails and timing. The PMIC also checks the MCU's own error pin and answers a question-and-answer watchdog, which a stuck or looping program can't fake.

## 4. Every block, questioned

### 4.1 Cut

| Block (rev 1.0 parts) | Question asked | Answer | Decision |
|---|---|---|---|
| **1S LiPo backup** (35 parts: charger, boost, 3 ideal diodes, SM-24, JST) | What hazard does it reduce? | None for SG1. The safe state is *pass-through with no power*: the NC contacts need no energy. The backup only keeps the overlay alive through a 12 V loss, which is an availability feature. It *added* a common-cause path into both channels (F13), a hot-car cell hazard (F24, F28–F30) and three non-automotive ICs | **Cut.** Losing 12 V gives pass-through, which is the designed safe state |
| **CH-C analog guard** (106 parts incl. speed stage) | Does the FMEDA need it? | Not if CH-A's own diagnostics are credited (§5.4 table). ISO 26262-9 decomposition is B(D) + B(D). CH-C was a third channel "beyond the split" (`06` §1). With CH-B now comparing readback against its *own* expected Δ, rev 1.0's F2 gap (a wrong CH-A command inside the envelope) is closed in CH-B, which is what R2 did in hardware. The software-free bound stays: the summers' resistor ratios cap Δ whatever either MCU does | **Cut, conditionally.** If TI's F28004x FMEDA can't support ≥ 90 % DC for SM-26A, the pessimistic case fails (11.2 FIT) and CH-C comes back (1.43 FIT with it). See §8 |
| **Speed stage SM-23** (29 parts, one non-automotive one-shot) | Is a hardware speed limit needed if Δmax is fixed? | Size Δmax for the worst speed and it isn't. The Insight EPS already reduces assist with road speed (VSP at 0 mph gives full assist, `EPS_PINOUT.md`), so a fixed ΔV moves less torque at speed. Both MCUs still apply a speed table in software, 2oo2 | **Cut.** Δmax becomes "controllable at the highest speed": TBD-DATA from the logger |
| **Dynamic charge-pump enable** (6 parts) | Does it cover anything the PMIC doesn't? | It defended against a stuck GPIO. Rev 2.0 ANDs every switch with the PMIC's ENDRV, which the MCU can drop over SPI, so a stuck GPIO still has a second, independent way off | **Cut** |
| **TPS1H100-Q1 smart high sides** (2) | Why short-circuit protection for an on-board coil? | The load never leaves the board. Diagnostics come from the coil-node readback in both MCUs. Two -Q1 parts were on 26-week backorder | **Replace** with AEC-Q101 P-FETs (about $0.15 each) |
| **Ideal-diode reverse protection** (LM74700-Q1 + SQJ486EP) | Why an ideal diode for a ~200 mA board? | It was added to stop the 470 µF hold-up draining backwards (F19). Without the hold-up cap, a plain Schottky costs 0.08 W and blocks reverse current by itself | **Replace** with an AEC-Q101 Schottky. The LM74700-Q1 was also backordered |
| **470 µF hold-up electrolytic** | Ride-through for what? | The PMIC keeps 3.3 V and core alive down to 4.5 V, and a relay holds far below its 9 V pick-up. A crank dip that drops the relay is safe anyway | **Cut**, replaced by 2× 10 µF/50 V AEC-Q200 MLCC |
| **On-board blade fuse + holder** | Why is a non-automotive holder on the board? | Automotive practice puts the fuse in the vehicle's fuse box or harness. Keystone 3568 has no automotive rating | **Move** the 3 A fuse to the harness. Keep the AEC-Q200 PTC on board |
| **Spare-pin headers** (`expansion` sheet) | Needed in a product? | They create a DFA risk (F22) and aren't automotive parts | **Cut.** Test pads stay |
| **2×5 / 2×10 debug headers** | Needed populated? | No | **Tag-Connect TC2050 footprints** instead: no part cost, no plug to vibrate loose, standard on production ECUs |

### 4.2 Kept, and defended

| Block | Why it stays | Cost of keeping |
|---|---|---|
| **Two diverse MCUs** (user's choice, unchanged) | Required by the B(D) + B(D) decomposition. The TMS570 is a lockstep, ASIL D-capable monitor (SM-25), and the diversity (C28x vs Cortex-R4F, two compilers) is the main DFA argument against common software faults | Unchanged |
| **Two cascaded relays K1, K2** | The FMEDA says a single relay gives the same PMHF (0.580 vs 0.580 FIT, §5.4). They stay for a reason the metric doesn't capture. The relay is the **only** element every reaction ends at, and it is shared, so it can't be decomposed (ISO 26262-9 §5). Its stuck-contact rate is the least certain number in the model. A second relay from a **different vendor** makes the final barrier fault-tolerant against that uncertainty | About $4.50 and 6 parts. **This is the one place redundancy is kept on judgement, not numbers;** dropping K2 is a valid choice if cost matters more |
| **Cross-driven coils** (each coil needs both channels) | 1oo2 stop with 2oo2 go, per relay. Any channel can kill both relays | 4 FETs + 4 gates |
| **Resistor-bounded authority (SM-05)** | The software-free layer: no firmware fault in either MCU can exceed 0.18 V (0.37 V with a railed op-amp) | 0.1 % thin-film resistors |
| **Both readbacks + VT3+VT6 sum check (SM-08A/B)** | They give most of the detection coverage | — |
| **Series ESD pairs, 100 kΩ bypass, hold caps, 1 MΩ taps** | Each closes a named finding (F18, F4, SM-11, SM-10) | Few cents each |
| **Two CAN transceivers** | CH-B has to receive the planner command itself, not through CH-A, or the monitor isn't independent | $1.05 each |
| **Indicator LEDs** (user preference) | Kept, but cut from 22 to about 12 (rails, PMIC fault, ENDRV, relay, CAN, heartbeat per channel). Rules from F14 still apply: never on a safety net in a way that can energise or hold a coil | AEC-Q102 ROHM CSL0902 at $0.65 |
| **Test pads** | Free, bench-useful | — |

### 4.3 New safety mechanisms in rev 2.0

| SM | Mechanism |
|---|---|
| SM-08B (changed) | CH-B compares the EPS-side reading against **its own** expected Δ, computed from its own copy of the CAN command, the SAS and speed, and no longer against CH-A's reported Δ |
| SM-26A | F28004x self-diagnostics (ECC, CPU/PIE tests, DCC, missing clock) drive ERRORSTS. ERRORSTS goes to PMIC-A's ERROR input, through an inverter if its polarity needs one, per TI SLVA528D §6.2. PMIC-A also runs the Q&A watchdog. Either one drops ENDRV_A |
| SM-PA / SM-PB | PMIC rail monitors on their own bandgap → reset and ENDRV low |
| SM-PBIST | PMIC analog and logic BIST at power-up, ENDRV read-back (ENDRV_ERR bit), and a key-on test where the MCU deliberately fails the watchdog and checks that ENDRV drops |
| SM-03L (changed) | Key-on test of each enable path: drop GPIO alone, then ENDRV alone, for each of the 4 switches. The coil node must follow every time |

### 4.4 Dependent-failure changes

| Coupling factor | Rev 1.0 | Rev 2.0 |
|---|---|---|
| Backup supply into both rails | F13, mitigated by SM-24 (untested latent) | **Gone** |
| Same PMIC type in both channels | — | **New.** A systematic PMIC defect could hit both. Mitigations: the PMIC is developed for ISO 26262 (TI functional safety documentation on request); each MCU's GPIO is a second, independent way off for its switches; the MCUs are diverse. Option: a diverse PMIC for CH-B if one is found in stock |
| Same relay type | "Consider two vendors" | **Decided:** TE P2 for K1, a different vendor for K2 |
| Shared board and connector | Coating, creepage | Unchanged, plus an enclosure (§6) |

## 5. Automotive-qualified parts

### 5.1 Rule

Every part on the BOM is either **AEC-Q100** (ICs), **AEC-Q101** (discretes), **AEC-Q102** (LEDs), **AEC-Q200** (passives, crystals, inductors, PTCs) or **USCAR-2** (connector), from a franchised distributor. No "commercial version because the -Q1 is out of stock". Rev 1.0's list had 20+ such substitutions (Samsung CL commercial caps, Yageo RC resistors, LTST LEDs, TLV9064IPWR, SN74LVC1G11DBVR, G6K relays). Where the -Q1 part is on backorder, the fix is to **buy from TI direct or wait**, not to substitute.

### 5.2 Key parts (DigiKey, 2026-11-15)

| Function | Part | Grade | Unit price | Stock |
|---|---|---|---|---|
| Safety PMIC ×2 | TI **TPS65381AQDAPRQ1** (HTSSOP-32) | AEC-Q100, functional safety | $7.20 | 1,430 |
| MCU A | TI F280049CPZQR | AEC-Q100, functional safety | unchanged | — |
| MCU B | TI TMS5700432BPZQQ1 | AEC-Q100, functional safety | unchanged | — |
| CAN ×2 | TI **TCAN1044AVDRQ1** (replaces TCAN3404, backordered) | AEC-Q100 | $1.05 | 0, 26 wk: buy from TI |
| Op-amps | TI TLV9064QPWRQ1 | AEC-Q100 | $2.53 | 0: buy from TI |
| Logic AND ×4 | TI SN74LVC1G08QDBVRQ1 (already in rev 1.0) | AEC-Q100 | ~$0.40 | in stock |
| Relay K1 | TE **V23079D1003B301** (P2, SMD, 12 V, gold bifurcated) | see §5.3 | $4.53 | 3,017 |
| Relay K2 | Panasonic **TX2SA-12V-Z** (TX, SA surface-mount, gold-clad twin crossbar, same pin numbering as K1) | see §5.3 | $2.31 | 244 |
| LEDs | ROHM CSL0902VT1C (red), matching yellow/green | AEC-Q102 | $0.65 | 10,372 |
| Crystal (CH-B) | Abracon ABM8AIG-16.000MHZ (already) | AEC-Q200 | — | — |
| MEMS clock (CH-A) | SiTime SiT2024B (already) | AEC-Q100 | — | — |
| Resistors | Yageo **AC**0805 series (not RC); Panasonic ERA-6A 0.1 % thin film for the summers | AEC-Q200 | ~$0.02–0.30 | stocked |
| MLCC | Murata GCM / TDK CGA / Samsung CL-*-B automotive codes | AEC-Q200 | ~$0.03–0.30 | stocked |
| PTC | Yageo SMDH1812B110TF/33 (already) | AEC-Q200 | — | — |
| TVS | Littelfuse 5.0SMDJ24CA + SMBJ24A-Q (already) | AEC-Q101 | — | — |
| Reverse Schottky | 3 A, 60 V SMA, AEC-Q101 (NRVBA340T3G is **obsolete**: pick a live one at schematic time) | AEC-Q101 | ~$0.40 | — |

### 5.3 Relays: the one honest gap

There's essentially **no AEC-Q200-qualified DPDT signal relay on the market**. Automotive relay makers qualify to their own and the OEMs' specs plus PPAP, not to AEC-Q200; the search found none. What rev 2.0 does:

- **TE P2 (V23079).** TE markets it for automotive, it has gold-plated bifurcated contacts for dry circuits, a 4 ms release and 11.7 mA coil current. It is rated -40 to +85 °C, which is enough for a **cabin** location (the interception point already chosen, `04` §7.4), and **not** for under the hood.
- **Rev 1.0's Omron G6K-2F-Y** isn't marketed for automotive. It's dropped.
- **The safety case** argues the relay's suitability per ISO 26262-8 §13 (qualification of hardware components) with the key-on proof test (SM-09) as the in-use evidence.

### 5.4 FMEDA trade study (`safety/rev2_trade.py`)

| Variant | Case | SPFM | LFM | PMHF (FIT) | ASIL D? |
|---|---|---|---|---|---|
| Rev 1.0 as drawn | Baseline | 99.99 % | 98.92 % | 0.088 | yes |
| Rev 1.0 as drawn | FIT ×3, DC capped at 90 % | 99.94 % | 98.53 % | 1.233 | yes |
| **Rev 2.0 proposed** | Baseline | 99.99 % | 99.08 % | **0.062** | yes |
| **Rev 2.0 proposed** | FIT ×3, DC capped at 90 % | 99.88 % | 98.94 % | **1.740** | yes |
| Rev 2.0 + CH-C kept | FIT ×3, DC capped at 90 % | 99.97 % | 98.65 % | 0.483 | yes |
| Rev 2.0, one relay | FIT ×3, DC capped at 90 % | 99.88 % | 98.98 % | 1.741 | yes |
| Rev 2.0, **no credit** for CH-A self-diagnostics | FIT ×3, DC capped at 90 % | 99.26 % | 98.93 % | **11.17** | **no** |
| Rev 2.0 + CH-C, no credit for CH-A self-diagnostics | FIT ×3, DC capped at 90 % | 99.92 % | 98.65 % | 1.427 | yes |

**How to read it:**
1. Rev 2.0 meets the ASIL D metrics in every case **as long as SM-26A is real**: CH-A's ERRORSTS and the PMIC's Q&A watchdog really catch most of CH-A's own random faults. That's the one claim this design rests on, and it needs TI's F28004x FMEDA to back it.
2. **CH-C is the fallback.** If that evidence doesn't come, CH-C (or a smaller version of it) goes back in.
3. **The second relay doesn't move the metric.** It is kept on the reasoning in §4.2.

## 6. Professional board

| Item | Rev 2.0 choice | Why |
|---|---|---|
| Connector | **TE AMPSEAL 23, kept** (header 776087-1 tin $10.54 in stock, or 1-776087-1 gold-palladium $20.47) | The user asked for cheap and readily available. AMPSEAL 23 is the de-facto aftermarket ECU connector (rusEFI, many standalone ECUs), so mating kits, contacts and generic crimp tools are cheap and everywhere. It's sealed and panel-mount, with locking. Molex MX150 20-way was checked and costs more ($11.53) with fewer kits around. **Plating:** VT3/VT6 are µA dry-circuit signals, so **gold** contacts on both halves are recommended (tin fretting gives intermittents → SG3). Never mix tin and gold |
| Harness | Y-harness at EPS connector B. **Plus a bypass plug** (an AMPSEAL plug with VT3_SNS–VT3_EPS and VT6_SNS–VT6_EPS jumpered) | Closes F21: the car steers stock with Icarus unplugged |
| Board | 4 layers, 1 oz inner (F33), ENIG, matte black mask, white silk both sides, mounting holes at the corners, fiducials, panel rails for JLC | Looks and solders like a production ECU |
| Passives | 0805 AEC-Q200 (hand-solder friendly, as rev 1.0 decided), assembly-ready | Same footprint either way |
| Debug | Tag-Connect TC2050 footprints, one per MCU | No populated headers |
| Enclosure | Die-cast aluminium box with the AMPSEAL header through the wall and its gasket; conformal coat | ECUs are judged by the box as much as the board |
| Layout | Unchanged rules (`pcb-layout` preferences, `04` §6): noisy zone (input, PMICs, coils) at the connector end; two MCU islands; analog overlay between them; creepage ≥ 1 mm from VBAT/coil copper to VT nets | — |

## 7. Cost

Approximate change in the per-board BOM, from DigiKey unit prices at quantity 1:

| Removed | ≈ $ | Added | ≈ $ |
|---|---|---|---|
| 2× LMR38020-Q1, 2× TPS7B82-Q1, 3× TPS3702-Q1, 2× TPS3850-Q1, TLV733-Q1 | ~25 | 2× TPS65381A-Q1 + core FET + 2 inductors + diodes | ~19 |
| 2× TPS1H100-Q1, charge pumps | ~5 | 4 FETs + 4 AND gates | ~2.5 |
| LiPo chain (charger, boost, 3 ideal diodes, JST, PTC, SM-24) | ~10 | — | — |
| CH-C guard + speed stage (2× LM2901, 3× TLV9064, latch, one-shot, switch, ~80 passives) | ~15 | CH-B readback quad op-amp + passives | ~3 |
| LM74700 + SQJ486EP, blade holder + fuse, 470 µF | ~6 | Schottky, 2× 50 V MLCC | ~1 |
| 2× G6K relays | ~9 | 2× gold-contact automotive relays | ~9 |
| ~10 LEDs | ~1 | AEC-Q102 premium on ~12 LEDs | +4 |
| **Total removed** | **~71** | **Total added** | **~39** |

**Net about $30 less per board**, with roughly 240 fewer placements, a smaller board, and every line automotive grade. The AEC-Q200 passives cost a few dollars more than the commercial ones rev 1.0 slipped in; that's included in the estimate's noise, not hidden.

## 8. Conditions and open items

| # | Item | Blocks | Owner |
|---|---|---|---|
| C1 | **Evidence for SM-26A ≥ 90 % DC:** request TI's F28004x-Q1 and TPS65381A-Q1 safety manuals and FMEDA (functional safety docs on request). If it can't be supported, reinstate CH-C | ASIL D argument | User (TI request) |
| C2 | **Δmax sized for the highest speed** (logger data: VT torque gradient vs speed) | Cutting SM-23 | Logger |
| C3 | ~~Datasheet pin tables~~ **Done 2026-11-15:** `hardware/gen/rev2_parts.py`, findings in `13_REV2_DATASHEET_FINDINGS.md` | Schematic | — |
| C4 | ~~ERRORSTS polarity~~ **Resolved:** F28004x ERRORSTS is active low (fixed, needs a pull-down), same as TMS570 nERROR, so both PMICs run the ESM in TMS570 mode with no inverter (doc 13) | — | — |
| C5 | F23 boot pins: keep GPIO24/GPIO32 boot-mode safe in the new pin map, so OTP isn't needed | Bring-up | Schematic |
| C6 | Safety plan, HSI spec, tool-confidence notes for the analysis scripts | Compliance | User + review |
| C7 | Rev 1.0 model bug: two elements both use id **E21** (ESD pairs and the speed stage) in `element_model.py`. It doesn't change the metrics, but the element table is ambiguous. Moot once the rev 2.0 model replaces it | Docs | Next model edit |
| C8 | Self-test plan and added safety functions M1–M10: see `12_SELF_TEST.md` | Schematic, firmware | — |

## Sources

- TI, [TPS65381A-Q1 product page](https://www.ti.com/product/TPS65381A-Q1): rails, Q&A watchdog, MCU error monitor, ENDRV, supported MCUs incl. TMS570 and C2000
- TI, [SLVA528D safety manual, C2000 section](https://www.ti.com/document-viewer/lit/html/SLVA528D/GUID-F5C84897-68A4-41B9-A8A7-47E2C0DA55EF) and [main document](https://ti.com/document-viewer/lit/html/SLVA528D/GUID-4DB6C92A-6916-4A1E-8309-841CE4C6548D)
- TI E2E, [ENDRV from ERROR, WDI and SPI](https://e2e.ti.com/support/power-management-group/power-management/f/power-management-forum/883707/tps65381a-q1-controlling-endrv-signal-from-error-wdi-and-spi), [when ENDRV clears](https://e2e.ti.com/support/power-management-group/power-management/f/power-management-forum/577970/tps65381a-q1-when-does-endrv-pin-clear)
- TI, [F28004x functional safety manual SPRUID8](https://www.ti.com/document-viewer/lit/html/SPRUID8D/GUID-B7934014-41D8-4E15-9CF8-2FF0686DC53F)
- Lee et al., [functional-safety EPS ECU with asymmetric dual processors and an external watchdog](https://koreascience.kr/article/JAKO201525961737133.pdf)
- eeNews Europe, [electronics architecture for EPS systems](https://www.eenewseurope.com/en/electronics-architecture-for-electric-power-steering-systems); TI, [EPS functional safety (SLYY111)](https://ti.com/lit/pdf/SLYY111)
- SAE, [EPAS safety per ISO 26262 (2015-26-0086)](https://saemobilus.sae.org/content/2015-26-0086), [EPS safety architecture (2020-01-0649)](https://saemobilus.sae.org/content/2020-01-0649)
- DigiKey listings: [TPS65381AQDAPRQ1](https://www.digikey.com/en/products/detail/texas-instruments/TPS65381AQDAPRQ1/7244182), [TCAN1044AVDRQ1](https://www.digikey.com/en/products/detail/texas-instruments/TCAN1044AVDRQ1/15857027), [TLV9064QPWRQ1](https://www.digikey.com/en/products/detail/texas-instruments/TLV9064QPWRQ1/13283308), [V23079D1003B301](https://www.digikey.com/en/products/detail/te-connectivity-potter-brumfield-relays/V23079D1003B301/1634794), [AMPSEAL 776087-1](https://www.digikey.com/en/products/detail/te-connectivity-amp-connectors/776087-1/350072), [1-776087-1 (gold)](https://www.digikey.com/en/products/detail/te-connectivity-deutsch-ict-connectors/1-776087-1/350071), [CSL0902VT1C](https://www.digikey.com/en/products/detail/rohm-semiconductor/CSL0902VT1C/16285849)
