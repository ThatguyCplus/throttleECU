# Tasaru V0.0.1 — Board Bring-Up Plan

> Derived from `TasaruV0.0.1.kicad_pcb` net extraction + `production/bom.csv` on 2026-09-04.
> Symptom at start: 1V2 core rail was low (0.465 V), now fixed; **JTAG will not connect**.

---

## 1. Chip inventory (what's actually on this board)

| Ref | Part | Pkg | Function | Supply | Talks to MCU via |
|-----|------|-----|----------|--------|------------------|
| **U4** | F280049CPZS | TQFP-100 | C2000 MCU | VDDIO/VDDA 3V3, VDD 1V2 (internal VREG) | — |
| **U5** | LMR38020 (`PLMR38020FSQDDARQ1`) | HSOP-8 + L1 10 µH | Buck: Vin → **3V3** | /Vin | PGOOD → GPIO33 (pin 53) |
| **U9** | TPS3702 (variant **unspecified in BOM**) | SOT-23-6 | 3V3 window supervisor (OV+UV) | +3V3 | **drives /XRSn via JP13** |
| **U1** | SQJ147ELP | PowerPAK SO-8 | Reverse-battery P-FET | /15aFusedBATT → /+BATTPROTECTED | none (passive) |
| **U6** | SiT2024BA `-XXE-` 10.000 MHz | SOT-23-5 | MEMS oscillator | +3V3 | OUT → **X1 (pin 69)** |
| **U3** | DRV8873H | HTSSOP-24 | H-bridge motor driver | /+BATTPROTECTED | EN/PH, nSLEEP, nFAULT, DISABLE, 2x IPROPI |
| **U2** | TPS1H100B-Q1 | HTSSOP-14 | Protected high-side switch (solenoid) | /+BATTPROTECTED | IN (via R8 + Q2 kill), DIAG_EN, CS |
| **U8** | AS5147U | TSSOP-14 | 14-bit magnetic encoder | +3V3 | **SPI-B** (GPIO56/57/58/59) |
| **U7** | TCAN3404 (`PTCAN3404DRQ1`) | SOIC-8 | CAN FD transceiver | +3V3 | CANA_TX/RX (GPIO31/GPIO30) |
| Q2 | MMBT2222A | SOT-23 | Hardware kill: pulls solenoid IN low | — | base <- /12V_BRAKEIN via R3 |
| Q4 | MMBT2222A | SOT-23 | Brake-switch level shifter | — | collector -> /BRK_SW (pin 57) |

Passive power path: `F2 (15 A blade) -> U1 (rev-batt FET) -> /+BATTPROTECTED -> D2 SMBJ20A TVS`, and
`D3 (SS24) -> F1 (2 A PTC) -> /Vin -> U5`. Bulk: C22/C24/C34 = 3x 2200 µF electrolytic.

### Power tree

```
VBAT --F2(15A)--> U1(SQJ147, rev prot) --> /+BATTPROTECTED --> U3 VM, U2 VS
                                                 |
                                            D3 --> F1(2A PTC) --> /Vin --> U5 EN+VIN
                                                                            |
                                                              U5 SW --L1 10uH--> +3V3
                                                                            |
                                          +---------------+-----------------+------------+
                                          |               |                 |            |
                                    FB2 220R -> VDDIO  FB1 60R -> VDDA   U6,U7,U8,U9   J1.1
                                          |
                                    U4 internal VREG (VREGENZ low via R29) --> 1V2_Core
```

---

## 2. Prime suspect for "can't talk": U9 + JP13 holding XRSn low

`/XRSn` = `U4.2, J1.10, R20 2k2 up to 3V3, R31 2k2 up to 3V3, R30 120 -> SW1, C26 10n, C29 100n, **JP13.2**`

`JP13` is a 3-way solder jumper with footprint **`Bridged123`** — it ships with **both** TPS3702 outputs
(OV on pin 1, UV on pin 3) shorted onto `/XRSn`. Both outputs are open-drain active-low, so **either**
an OV or a UV trip parks the MCU in reset permanently and JTAG cannot connect.

The BOM lists U9 only as `TPS3702` with **no orderable suffix and no LCSC part number**. The TPS3702
is a *fixed-threshold* part — a `TPS3702CX25` (2.5 V) on a 3.3 V rail will scream OV forever. That
single-part mixup produces exactly the symptom you have.

**Test this first, it costs 30 seconds.**

---

## 3. Stage 0 — Unpowered checks (DMM only)

| # | Check | Expect | If bad |
|---|-------|--------|--------|
| 0.1 | `/Vin` to GND | > 1 kΩ (rising, big caps) | short in U5 input / C32/C33/C35 |
| 0.2 | `+3V3` (T14) to GND | > 500 Ω rising | pull U5, then bisect by lifting caps |
| 0.3 | `1V2_Core` to GND | > 100 Ω rising | see previous debug — MCU core short |
| 0.4 | `VDDIO` to `+3V3` across **FB2** | < 1 Ω DC | FB2 unsoldered = whole MCU dead |
| 0.5 | `VDDA` to `+3V3` across **FB1** | < 1 Ω DC | FB1 unsoldered = ADC dead, may block boot |
| 0.6 | `/VREGENZ` (U4.73) to GND through **R29** | < 1 Ω | VREG off -> no 1V2 |
| 0.7 | **JP8 open?** | open | if closed *and* R29 fitted -> 3V3 shorted to GND |
| 0.8 | `/XRSn` to `+3V3` | ~1.1 kΩ (R20 ∥ R31) | pull-ups missing |
| 0.9 | `/TMS` to `VDDIO` through R26 | 2.2 kΩ | JTAG pull-up |
| 0.10 | Continuity J1.3-U4.62, J1.5-U4.60, J1.7-U4.61, J1.9-U4.63, J1.10-U4.2, J1.1-3V3, J1.2-GND | all pass | bad reflow on the JTAG corner |

---

## 4. Stage 1 — Power up on a bench supply (no battery, motor unplugged)

Feed **/Vin via T3 or the fuse holder at 12 V, current limit 500 mA** for the first hit.
The 3x 2200 µF bulk caps will slam the limit briefly — ramp the supply from 0 V rather than hot-plugging.

| # | Measure | Expect |
|---|---------|--------|
| 1.1 | Idle current at 12 V | ~30–80 mA (MCU + encoder + CAN idle) |
| 1.2 | `+3V3` at T14 | **3.30–3.32 V** (R14 100k / R25 43.2k, Vref 1.0 V -> 3.315 V) |
| 1.3 | `/PGOOD` (T7) | high (~3.3 V, R34 100k pull-up) |
| 1.4 | `VDDIO` at U4.3 | = 3V3 within a few mV |
| 1.5 | `VDDA` at U4.11 | = 3V3 within a few mV |
| 1.6 | `1V2_Core` at U4.4 | **1.14–1.26 V** |
| 1.7 | U5 SW node | switching square wave (R12 64.9 k sets f_sw — check the LMR38020 RT table) |

Anything failing here stops the plan; fix before touching JTAG.

---

## 5. Stage 2 — Release reset (the actual blocker)

1. **Scope `/XRSn` at U4.2 through power-up.**
   - Rises to 3.3 V and stays -> reset is fine, skip to Stage 3.
   - Stuck low -> continue.
   - Pulsing/oscillating -> supervisor is chattering on a marginal rail; recheck 1.2.
2. **Open JP13** (cut both bridges with a knife). This disconnects U9 entirely; R20/R31 then pull XRSn high.
3. Re-measure `/XRSn`. If it now sits at 3.3 V, **U9 was the problem** — either the wrong TPS3702 variant
   or a genuine OV/UV trip. Leave JP13 open for the rest of bring-up.
4. Confirm SW1 still works: pressing it should pull XRSn to ~0 V through R30 (120 Ω).
5. Note for later: decide whether you want OV *and* UV both wired to reset. Re-bridge only the leg you
   want once you have confirmed the correct U9 part number.

---

## 6. Stage 3 — JTAG connect

Wire XDS110 to J1: **VTREF <- J1.1, GND <- J1.2, TMS -> J1.3, TCK -> J1.5, TDO <- J1.7, TDI -> J1.9,
nRESET -> J1.10.** Use 4-wire JTAG (not cJTAG) for bring-up.

- In CCS: new Target Configuration -> XDS110 -> TMS320F280049C -> **Test Connection**.
- The device boots on **INTOSC2 (internal 10 MHz)**, so U6 is *not* required to connect. Do not chase
  the clock until JTAG works.
- Boot mode straps: R21 56 k pulls GPIO24 high, R23 56 k pulls GPIO32 high, JP3/JP4 (both **open** by
  default) would pull them low through 2k2. Both-high selects the "get mode from OTP" boot on F28004x —
  confirm against the boot-mode table in the TRM (SPRUI33) before relying on it.
- If Test Connection passes but "connect target" fails, that is a clock/PLL problem, not a wiring problem.

---

## 7. Stage 4 — Clock

`U6.5 (OUT) -> U4.69 (X1)`, and `U4.68 (GPIO18/X2)` is **unconnected** — that is the correct topology
for a single-ended external clock.

Two things to verify before enabling XTAL mode in firmware:

- **U6 part number is incomplete in the BOM**: `SIT2024BA-S2-XXE-10.000000E` — `XX` is the supply/stability
  code. Confirm the part actually fitted is the **3.3 V** variant, and scope U6.5 for a clean 10 MHz swing.
- **X1 input level**: check the F28004x datasheet (SPRS945) crystal / external-clock section for the maximum
  X1 input swing. On some C2000 families X1 is *not* a 3.3 V-tolerant input when driven externally. If that
  applies here, a 3.3 V oscillator into X1 is a hardware error and you will need a divider / AC-couple, or to
  move the clock to an XCLKIN-capable GPIO.

Until that is settled, **run everything on INTOSC2** — fine for all of Stage 5.

---

## 8. Stage 5 — Per-chip functional tests

Run these as small standalone CCS test programs, one peripheral at a time, motor and solenoid loads
disconnected until their own step. Order matters: each step assumes the previous passed.

### 5.1 GPIO / MCU alive

Blink any spare test point (IO12…IO40 pads). Confirms clock, flash/RAM, and the toolchain.
**Pass:** square wave at the expected rate on a scope.

### 5.2 U9 supervisor (bench-verify before re-bridging JP13)

With JP13 open, power +3V3 from a variable supply through T14 (U5 removed or Vin off). Sweep 2.8 -> 3.6 V
and watch U9 pin 6 (OV) and pin 1 (UV). Record the actual trip points.
**Pass:** UV releases below ~3.1 V, OV trips above ~3.5 V (exact numbers depend on variant + SET pin,
which is tied to +3V3 here). **Fail:** trips anywhere inside 3.2–3.4 V -> wrong part, replace it.

### 5.3 U5 buck under load

Load +3V3 with ~500 mA (6.8 Ω). Check regulation, ripple (< 50 mVpp), and that PGOOD stays high.
Then sag Vin toward 4 V to emulate cranking and confirm 3V3 holds until dropout.

### 5.4 U6 oscillator

Scope U6.5. **Pass:** 10 MHz, rail-to-rail, fast edges. Then (only after §7 is resolved) switch the
firmware clock source to XTAL and verify with XCLKOUT on GPIO16 or GPIO18.

### 5.5 U8 encoder (SPI-B)

Configure **SPI-B**, mux: GPIO56=SPIB_SIMO, GPIO57=SPIB_SOMI, GPIO58=SPIB_CLK, GPIO59=SPIB_STE.
Read the AS5147U `DIAAGC` (0x3FFC) and `ANGLEUNC` (0x3FFE) registers.
**Pass:** DIAAGC returns a sane AGC value with MAGL/MAGH clear when a magnet is at the right air gap;
angle changes monotonically as you rotate the magnet.
**Fail modes:** all 0x0000 or 0xFFFF -> clock polarity/phase wrong (AS5147U wants CPOL=0, CPHA=1) or CS
not toggling. Probe T10–T13 — note those pads are **mislabelled `A_SPI_*`**; the silkscreen is wrong,
the nets are SPI-B.

### 5.6 U7 CAN transceiver

JP5/JP6 are bridged by default so CANA_TX/RX reach the transceiver. SHDN and STB are both hard-tied to
GND = always-on normal mode.

- **Loopback first:** put CANA in internal loopback, confirm the MCU can send/receive without a bus.
- **Then external:** connect a USB-CAN adapter to J2.18 (CAN_H) / J2.8 (CAN_L). JP7 is bridged, so R27
  120 Ω termination is **already on the board** — do not add a second terminator unless you are at the
  far end of a 2-node bus.
- **Pass:** frames appear on the analyzer at the configured bitrate with no error frames.
- Scope CANH/CANL: recessive both ~2.5 V, dominant ~3.5 / 1.5 V.

### 5.7 U3 motor driver (no motor yet)

DVDD (U3.1) should read ~3.3 V (internal LDO) as soon as VM is present and nSLEEP is high.

- Drive nSLEEP (GPIO03, pin 76) high, read nFAULT (GPIO01, pin 78) — should be high (R15 10 k pull-up).
- MODE is tied GND and DISABLE has R5 10 k to GND with R11 4k7 to GPIO (pin 75). Confirm the DRV8873
  datasheet's MODE=low = PH/EN interface before writing PWM code.
- **Then with motor connected and a current-limited supply:** ramp EN duty from 0, watch MOT_OUT+/-
  (J2.5 / J2.4) and the IPROPI voltages at T6/T5 (R16/R17 = 360 Ω to GND).
- **Pass:** IPROPI voltage tracks motor current linearly and both directions work via PH.
- `JP3_10amp/4amp1` selects the niTRIP current-limit tap — pick the leg you want before loading it.

### 5.8 U2 high-side switch (solenoid)

Note the hardware interlock: `/SOL_GATE` (pin 84) drives U2.IN through R8 4k7, and **Q2 can pull IN low
regardless of the MCU** — its base comes from `/12V_BRAKEIN` via R3 22 k. So with brake voltage applied,
the solenoid is force-disabled. Test both paths:

1. Brake input at 0 V: assert /SOL_GATE -> `/+SOL` (J2.6) should go to battery voltage.
2. Apply 12 V to `/12V_BRAKEIN` (J2.7) -> `/+SOL` must drop out even with /SOL_GATE still high.

- DIAG_EN (pin via R9 4k7 to 3V3) enables the CS output; read CS through R10/R13 into the ADC (pin 29, via JP10).
- **Pass:** CS voltage scales with solenoid current; U2 latches off into a short and reports it.

### 5.9 Brake sense analog chain

`/12V_BRAKEIN -> R1 10 k -> /BRK_SENSE -> R7 2.2 k -> GND`, clamped by D1 BAV99, then R2 1 k -> JP9 -> U4.19.

- 12 V in -> expect **2.16 V** at the ADC pin. 14 V -> 2.52 V. Both safely under 3.3 V.
- Also confirm `/BRK_SW` (pin 57) goes low when Q4 turns on (R33 4k7 pull-up).

### 5.10 ADC reference

VREFHIA (U4.24/25) is decoupled by C36/C38 and has **no external reference driver** — you are using the
internal reference. Set VREFHI to the internal/VDDA mode in firmware, then verify a known divider reads correctly.

---

## 9. Design issues found during extraction — verify these

| # | Item | Why it matters |
|---|------|----------------|
| 1 | **U9 TPS3702 variant unspecified** in BOM (no suffix, no LCSC #) and **JP13 ships bridged 1-2-3** | Wrong threshold part = MCU permanently in reset. Top suspect for the current symptom. |
| 2 | **U5 identity conflict**: `production/bom.csv` says `PLMR38020FSQDDARQ1` (HSOP-8 + external L1), `HANDOFF.md` §2 says `TPSM33620S3QRDNRQ1` (module, no external L) | The board has an L1 10 µH footprint, so the BOM is probably right and the handoff doc is stale. Fix the doc so the next order is not wrong. |
| 3 | **U6 part number has an `XX` placeholder** (`SIT2024BA-S2-XXE-10.000000E`) | Supply-voltage code unresolved — could be a 1.8 V part on a 3.3 V rail. |
| 4 | **X1 driven by a 3.3 V oscillator** (U6.5 -> U4.69) | Confirm X1's external-clock input rating in SPRS945; may need level adjustment. |
| 5 | **FB1/FB2 assigned backwards from convention**: FB2 = 220 Ω on **VDDIO**, FB1 = 60 Ω on **VDDA** | Usually the analog rail gets the higher-impedance bead. Check DCR is low enough for VDDIO's current. |
| 6 | **GPIO22/VFBSW (pin 83) and GPIO23/VSW (pin 81) go only to test points** | These are the F28004x internal DC-DC switcher pins. Check the datasheet for required termination when the DC-DC is unused — floating may not be legal. Relevant given the 1V2 trouble. |
| 7 | **Test points T10–T13 silkscreened `A_SPI_*`** but wired to SPI-**B** pins (GPIO56–59) | Will mislead firmware and probing. Rename in the next rev. |
| 8 | **`MCU_PIN_FUNCTION_MAP.md` claims pin 69 has no net** | Stale — the PCB now has `/10Mhz_Ocillator_Out` on pin 69. Update the doc. |
| 9 | **No TRSTn anywhere** on the symbol/footprint/netlist | Confirm the F28004x has no TRSTn pin; if it does, it needs a pulldown. |
| 10 | DRC snapshot: **132 violations** (2026-04-09, `DRC.rpt`) | Re-run before the next fab spin. |

---

## 10. Jumper default-state cheat sheet

| Jumper | Default | Function |
|--------|---------|----------|
| JP1, JP2 | **bridged** | MOT_PWM / MOT_PH -> DRV8873 EN/PH |
| JP5, JP6 | **bridged** | CANA TX/RX -> transceiver |
| JP7 | **bridged** | 120 Ω CAN termination (R27) in circuit |
| JP9 | **bridged** | brake divider -> ADC |
| JP10, JP11, JP12 | **bridged** | current-sense signals -> ADC |
| JP13 | **bridged 1-2-3** | **both** U9 OV *and* UV -> /XRSn |
| JP3 (GPIO24BOOT) | **open** | close = pull GPIO24 low (2k2) |
| JP4 (GPIO32BOOT) | **open** | close = pull GPIO32 low (2k2) -> serial boot |
| JP8 (VREGENZPU+) | **open** | close = VREGENZ high = internal VREG **disabled** |
| JP3_10amp/4amp1 | bridged 1-2 | DRV8873 niTRIP current-limit tap |

---

## 11. Suggested order of attack

1. Stage 0 continuity + resistance (10 min, unpowered).
2. Stage 1 rails at 12 V / 500 mA limit.
3. **Scope XRSn. If low -> cut JP13.**  <- most likely single fix
4. JTAG Test Connection on INTOSC.
5. Blink an LED / toggle a test point.
6. Encoder (SPI-B) -> CAN loopback -> CAN external.
7. Motor driver with no motor, then motor on a current-limited supply.
8. Solenoid path including the Q2 brake interlock.
9. Only then resolve the X1 clock question and switch off INTOSC.
