# Tasaru V0.0.1 — Agent Handoff Document

> **Created:** 2026-04-02
> **Purpose:** Complete context for any AI agent continuing work on this project
> **Project:** Self-driving throttle actuator ECU for automotive start-stop vehicle

---

## 1. Project Overview

This is a custom PCB (KiCAD project: `TasaruV0.0.1`) for a cruise control / self-driving throttle actuator. It controls a DC motor that actuates the throttle body, reads position via a magnetic encoder, communicates over CAN bus with the vehicle's ECM, and must survive all automotive electrical transients including start-stop engine cranking.

### Key Files

| File | Location | Purpose |
|------|----------|---------|
| KiCAD schematic | `TasaruV0.0.1.kicad_sch` | Single flat sheet (no hierarchical sheets) |
| KiCAD project | `TasaruV0.0.1.kicad_pro` | Project config |
| Pin map & HW reference | `FIRMWARE_PIN_MAP.md` | Authoritative hardware reference — kept up to date |
| Research data | `researchdata/` | KiCAD symbols, footprints, and datasheets for components |
| WEBENCH BOM export | User's Downloads folder `WBBOMDesign2.csv` | TI WEBENCH output for TPSM33620 design |

### Repository Structure

- Git repo root: `C:/Users/Muhammed Shah/Documents/Arduino/throttleECU`
- Branch: `feature/c2000-f280049c-port`
- PCB project path: `f411testingreliablitymotor/c2000_f280049_throttle/throttleecupcb/TasaruV0.0.1/`

---

## 2. Major ICs

| Ref | Part | Package | Function |
|-----|------|---------|----------|
| U1 | TMS320F280049C (F280049CPZS) | TQFP-100 | MCU — 5 units in KiCAD symbol (GPIO A, GPIO B, ADC/analog, system, power) |
| U2 | PTCAN3404DRQ1 | SOIC-8 | CAN transceiver — 500kbps, CAN-A on GPIO30/31 |
| U3 | DRV8873HPWPR | HTSSOP-24 | Motor driver — PH/EN mode, dual IPROPI current sense |
| U4 | AS5147U-HTSM | TSSOP-14 | Magnetic encoder — SPI, 14-bit, AEC-Q100/ASIL-D |
| U5 | TPSM33620S3QRDNRQ1 | QFN 4.5x3.5mm HotRod | Power module — fixed 3.3V/2A buck with integrated inductor |
| Y1 | SIT2024BA-S2-XXE-10.000000 | SOT-23-5 | 10MHz MEMS oscillator — AEC-Q100, feeds MCU X1 pin |

---

## 3. Power Supply Design — Complete Design History

### Evolution of Design Decisions

The power supply went through several iterations during this design session. Understanding WHY each decision was made is critical for future changes.

1. **Started with LMR33630CDDA** (HSOP-8, 3.8V-36V, 3A) — original part in schematic
2. **User raised start-stop concern** — car has auto start-stop, engine shuts off at red lights, cranking sags battery to 3-6V
3. **Considered LM43602-Q1** (HTSSOP-16, 3.5V-36V, 2A) — handles cranking better, but still a discrete buck needing external inductor + ~12 components
4. **User wanted simpler solution** — "is there a simple VIN 12V → 3.3V out chip?"
5. **Found TPSM33620-Q1** — integrated power module with inductor + FETs + boot cap inside, only needs ~5 external caps + 1 resistor
6. **User confirmed hot plate reflow** — QFN package is acceptable, all SMD assembly
7. **WEBENCH design run** — user ran TI WEBENCH, got BOM with EMI filter components
8. **Final part number: TPSM33620S3QRDNRQ1** — the "S" variant has Dual Random Spread Spectrum (DRSS) for better EMI. This is BETTER than the originally recommended TPSM336203QRDNRQ1.

### Final Power Chain (Signal Flow Order)

```
BAT+ (9-14V nominal, survives 2.8V-60V transients)
│
├── F1: PTC Resettable Fuse, 3A hold / 6A trip, 1812 SMD
│       Example: Littelfuse 1812L300/16MR
│
├── D2: SS34 Schottky Diode, 3A / 40V, SMA package
│       Anode to F1 output, cathode to protected rail
│       Purpose: Reverse battery polarity protection
│
├── D1: SMBJ20A TVS Diode, 20V standoff / ~32V clamp, SMB package
│       Cathode to protected rail, anode to GND
│       Purpose: Load dump overvoltage clamp (ISO 7637-2 Pulse 5)
│       NOTE: SMBJ33A was originally in FIRMWARE_PIN_MAP — it was WRONG
│       (33V standoff → 53V clamp exceeds LMR33630 max). SMBJ20A corrects this.
│
├── C24: 1000uF / 25V Electrolytic, 10x12mm radial
│       Purpose: Cold crank hold-up buffer
│       Math: At 50mA draw, holds input at 6.0V through 50ms sag to 2.8V (worst-case ISO)
│       NOTE: Originally 470uF — increased to 1000uF after user read TI article showing
│       worst-case cold crank is 50ms (not 15ms as initially assumed)
│
├── EMI INPUT FILTER (CISPR 25 Class 5 compliance):
│   ├── Cb: 22uF / 50V Electrolytic (Panasonic EEHZA1H220P), battery side
│   │       Polarized is safe here — D2 Schottky blocks reverse voltage
│   ├── Rd: 100mΩ / 0603 (Panasonic ERJ-3RSFR10V), damping resistor
│   │       CRITICAL: Must be 100mΩ, NOT 2.2k. User's schematic had wrong value.
│   ├── Lf (L7): 100nH / 3.2x2.5mm (TDK NLCV32T-R10M-PFR), filter inductor
│   └── Cf: 10uF / 50V / 1210 (Taiyo Yuden MSASU32MSB5106KPNA01), regulator side
│       NOTE: Derated to 2uF at operating voltage — this is expected and sufficient
│
├── INPUT CAPS (right at U5 VIN pins):
│   ├── Cin: 2x 10uF / 63V / 1210 (MuRata GRM32ER71J106KA12L)
│   │       WEBENCH specifies 2x — user's schematic only had 1x. Must add second.
│   └── Cinx: 0.1uF / 100V / 0603 (TDK CGA3E3X7S2A104K080AB), HF bypass
│
├── U5: TPSM33620S3QRDNRQ1 (fixed 3.3V / 2A power module)
│   ├── Pin 3 (VIN) ← from input caps
│   ├── Pin 8 (VCC) ← 2.2uF to GND (substituted from WEBENCH's 1uF — safe, exceeds min)
│   ├── Pin 2 (EN) ← 100k resistor to VIN (always-on, no UVLO divider needed)
│   ├── Pin 1 (PGOOD) ← 100k pullup to +3V3 → MCU GPIO (substituted from WEBENCH's 10k)
│   ├── Pin 11 (MODE/SYNC) ← tied to GND (auto PFM mode)
│   ├── Pin 7 (BOOT) — internal bootstrap cap, connects to SW internally
│   ├── Pins 6,5 (SW) — internal switch node, connects to integrated inductor
│   ├── Pin 4 (VOUT) → +3V3 output
│   ├── Pin 9 (FB) → connected to VOUT (fixed output version)
│   ├── Pin 10 (GND) → ground
│   └── EP (thermal pad) → GND with thermal vias
│
└── OUTPUT CAPS:
    ├── Cout: 2x 22uF / 1206 / 10V (substituted from WEBENCH's 1x 47uF — lower ESR, same BOM)
    └── Coutx: 0.1uF / 0402 / 16V, HF bypass (from WEBENCH)
        Plus: 2.2uF / 0603 visible in user's schematic (additional output bypass — fine to keep)
```

### MCU Power Distribution (from +3V3 rail)

```
+3V3 ──┬── VDD (MCU pins 4, 46, 71, 87) — direct, 2x 10uF + 4x 0.1uF decoupling
       ├── VDDIO (MCU pins 3, 47, 70, 88) — direct, 2x 10uF + 5x 0.1uF decoupling
       ├── L2 (60Ω ferrite bead, 0603) → VDDA (MCU pins 11, 34) — 4x 2.2uF decoupling
       └── L3 (220Ω ferrite bead, 0603) → VDDIO_SW (MCU pin 80)
```

### WEBENCH BOM Substitutions Made

| WEBENCH Part | WEBENCH Value | Substituted To | Reason |
|-------------|--------------|----------------|--------|
| Cout | 1x 47uF 1210 | 2x 22uF 1206 | Keeps 22uF in BOM, lower ESR in parallel |
| Cvcc | 1uF 0603 | 2.2uF 0603 | Exceeds minimum, already in BOM |
| Rpg | 10k 0402 | 100k 0402 | PGOOD is slow signal, already in BOM |

### EMI / EMC Compliance

| Standard | Class | Status |
|----------|-------|--------|
| CISPR 25 | Class 5 (most strict) | Covered — TPSM33620S3Q has DRSS spread spectrum + shielded inductor + input LC filter |
| ISO 7637 | All pulses 1-5 | Covered — fuse + Schottky + TVS + hold-up cap |
| ISO 16750-2 | Cranking profiles | Covered — 1000uF rides through 50ms worst-case cold crank |
| ISO 11452 | Immunity | Addressed at PCB layout stage (solid ground plane, placement) |
| AEC-Q100 | Grade 1 (-40°C to +125°C) | TPSM33620-Q1 is AEC-Q100 qualified |

### Cold Crank Survival Math

Assumptions: Cap starts at 8.5V (9V battery minus 0.5V Schottky), system draws 50mA idle.

| Scenario | Battery Sag | Duration | Cap Voltage After | TPSM33620 Min (4.5V) | Result |
|----------|------------|----------|-------------------|----------------------|--------|
| Warm crank (start-stop) | 5-6V | 5ms | 8.45V | > 4.5V | SAFE |
| Cold crank (worst case) | 2.8V | 50ms | 6.0V | > 4.5V | SAFE |
| Load dump | 60V spike | 400ms | Clamped to 32V | < 36V max | SAFE |

---

## 4. Passive BOM Strategy

The user explicitly asked to minimize BOM line items to simplify assembly. All substitutions were made with this goal.

### Capacitors — 4 Unique MLCC Values

| Value | Packages Used | Voltage Ratings | Usage |
|-------|--------------|-----------------|-------|
| 0.1uF | 0402 (16V), 0603 (100V) | 16V for logic side, 50-100V for input side | HF bypass everywhere |
| 2.2uF | 0603 X5R 16V | 16V | VDDA decoupling, VREF caps, CVCC, VDDIO_SW |
| 10uF | 0805 (16V), 1210 (50-63V) | 16V for bulk decoupling, 50-63V for input | VDD/VDDIO bulk, CIN, EMI filter |
| 22uF | 1206 X5R 10V | 10V | Buck output caps only |

Plus 2 electrolytics (not MLCCs):
- 1000uF/25V radial — hold-up cap (C24)
- 22uF/50V radial — EMI filter battery side (Cb_inpflt)

### Resistors — 4 Unique Values (Power Section)

| Value | Package | Qty | Usage |
|-------|---------|-----|-------|
| 0 ohm | 0402 | ~5 | Config jumpers (VREGENZ, etc.) |
| 2.2k | 0402 | 1-2 | XRS_N reset pullup |
| 100k | 0402 | 3 | EN pullup, PGOOD pullup, RFBT (if using adjustable regulator variant) |
| 100mΩ | 0603 | 1 | EMI filter damping (Rd_inpflt) |

### Inductors / Ferrites — 3 Values

| Value | Package | Qty | Usage |
|-------|---------|-----|-------|
| 100nH | 3.2x2.5mm | 1 | EMI filter (Lf_inpflt) |
| 60Ω @ 100MHz ferrite | 0603 | 1 | VDDA isolation (L2) |
| 220Ω @ 100MHz ferrite | 0603 | 1 | VDDIO_SW isolation (L3) |

No external power inductor needed — integrated in TPSM33620 module.

### Protection Components

| Part | Package | Qty | Usage |
|------|---------|-----|-------|
| PTC Fuse 3A/6A | 1812 | 1 | Overcurrent (F1) |
| SS34 Schottky | SMA | 1 | Reverse polarity (D2) |
| SMBJ20A TVS | SMB | 1 | Overvoltage clamp (D1) |

---

## 5. Board External Connections

8 wires leave the PCB in the final product:

| # | Signal | Direction | Notes |
|---|--------|-----------|-------|
| 1 | BAT+ | In | 12V from battery |
| 2 | BAT- / GND | In | Battery ground = chassis ground |
| 3 | CAN_H | Bidirectional | CAN bus high (differential) |
| 4 | CAN_L | Bidirectional | CAN bus low (differential) |
| 5 | MOT_OUT1 | Out | Motor output 1 (from DRV8873H) |
| 6 | MOT_OUT2 | Out | Motor output 2 (from DRV8873H) |
| 7 | BRK_SENSE | In | Brake pedal signal (9-14V, voltage divided on board) |
| 8 | SOL_OUT | Out | Solenoid drive (VBAT through Q3 MOSFET) |

Plus development-only headers: JTAG (2x5 pin header), UART (2 pins), spare ADC breakouts.

---

## 6. PCB Layout Guidelines (Discussed but Not Yet Implemented)

### Stackup — 4-Layer Required

```
Layer 1 (Top):     Components + signal traces
Layer 2 (Inner 1): SOLID unbroken ground plane — DO NOT CUT SLOTS
Layer 3 (Inner 2): Power plane (+3V3 and +VBAT zones)
Layer 4 (Bottom):  Signal traces + some components
```

### Placement Zones

```
┌─────────────────────────┬──────────────────────────┐
│ NOISY SIDE              │ QUIET SIDE               │
│ (near connector)        │ (far from connector)     │
│                         │                          │
│ • J1 connector          │ • U1 MCU (all units)     │
│ • F1, D1, D2, C24       │ • U4 AS5147U encoder     │
│ • EMI filter            │ • JTAG header             │
│ • U5 TPSM33620          │ • ADC input resistors     │
│ • U3 DRV8873H           │ • Decoupling cap farm     │
│ • U2 TCAN3404           │ • SIT2024B oscillator     │
│                         │                          │
│ Ferrite beads L2, L3 sit on the boundary           │
└─────────────────────────┴──────────────────────────┘
```

### Ground Strategy

- **Single GND net** — never split in schematic
- **Solid ground plane** on layer 2 — return currents self-organize
- **No separate chassis ground** — battery negative IS chassis in automotive
- Noisy return currents (buck switching, motor PWM) stay near connector
- Quiet return currents (ADC, SPI) stay near MCU
- Ferrite beads L2/L3 isolate analog supply from digital switching noise

---

## 7. Assembly Method

**Hot plate reflow.** User confirmed. All components are SMD except:
- C24 (1000uF electrolytic) — radial through-hole, solder by hand after reflow
- Cb_inpflt (22uF electrolytic) — radial through-hole, solder by hand after reflow
- Connectors / headers — solder by hand after reflow

Equipment needed: hot plate (~$30), solder paste syringe (~$8), tweezers, flux.

---

## 8. Open Items / TBD

These items were NOT resolved in the design session and need future work:

### GPIO Pin Assignments Still TBD

| Function | Net Label | Peripheral | Status |
|----------|-----------|-----------|--------|
| SPI CS | `SPI_CS` | GPIO manual | Need to pick GPIO and check mux table |
| SPI CLK | `SPI_CLK` | SPIA_CLK | Need to check F280049C mux table |
| SPI MISO | `SPI_MISO` | SPIA_SOMI | Need to check F280049C mux table |
| SPI MOSI | `SPI_MOSI` | SPIA_SIMO | Need to check F280049C mux table |
| Motor PWM | `MOT_PWM` | ePWM | Candidates: GPIO0, GPIO2, GPIO4 |
| Motor Phase | `MOT_PH` | GPIO out | Any free GPIO |
| Motor nFAULT | `MOT_nFAULT` | GPIO in | Any free GPIO |
| Motor nSLEEP | `MOT_nSLEEP` | GPIO out | Any free GPIO |
| Safe Disable | `SAFE_DISABLE` | GPIO out | Any free GPIO |
| Solenoid Gate | `SOL_GATE` | GPIO out | Any free GPIO |
| Power Good | `PWR_PGOOD` | GPIO in | Candidates: GPIO5, GPIO8, GPIO11 |

### Schematic Issues Found in User's Current KiCAD File

1. **EMI filter damping resistor** is 2.2k in schematic — must be **100mΩ**
2. **Missing second 10uF input cap** — WEBENCH requires 2x 10uF at VIN
3. **Protection components (F1, D1, D2, C24) not yet placed** in schematic
4. **All cap footprints are THT disc** (`Capacitor_THT:C_Disc_D3.0mm_W2.0mm_P2.50mm`) — must be changed to SMD MLCCs
5. **Ferrite beads L2/L3 are entered as resistors (R1/R2)** — should be inductor/ferrite bead symbols
6. **PGOOD not yet connected** to a MCU GPIO pin
7. **U5 reference** still shows LMR33630CDDA in some places — needs updating to TPSM33620S3QRDNRQ1
8. **Duplicate C1 reference designator** — two caps both labeled C1

### Other Design Work Not Yet Done

- DRV8873H application circuit (motor driver passives, IPROPI sense resistors)
- AS5147U application circuit (SPI decoupling, magnet placement considerations)
- TCAN3404 application circuit (CAN bus termination, ESD protection)
- Brake sense voltage divider component values
- Solenoid drive MOSFET circuit (Q3)
- Safety disable MOSFET circuit (Q1)
- Connector selection (Molex MX150, Deutsch DT, or other sealed automotive connector)
- PCB layout
- Firmware porting (documented in FIRMWARE_PIN_MAP.md migration notes section)

---

## 9. Key Design Rationale (Why These Choices)

### Why TPSM33620-Q1 Instead of LMR33630CDDA

- LMR33630 needs 3.8V minimum — fails during start-stop cranking (battery sags to 3-6V)
- LM43602-Q1 (3.5V min) was considered but requires external inductor + 12 components
- TPSM33620 integrates inductor + FETs + boot cap — only 5 external caps + 1 resistor
- AEC-Q100 qualified, CISPR 25 Class 5 compliant, DRSS spread spectrum
- Trade-off: 4.5V minimum input (vs 3.5V for LM43602) — mitigated by 1000uF hold-up cap

### Why SMBJ20A Instead of SMBJ33A

- SMBJ33A (originally in pin map) has 33V standoff / ~53V clamp — exceeds LMR33630's 36V max
- SMBJ20A has 20V standoff / ~32V clamp — safely under 36V max of any regulator considered
- 20V standoff is well above 14.5V normal operating voltage — no conduction during normal operation

### Why 1000uF Hold-Up Cap (Not 470uF)

- Initially calculated 470uF based on 15ms cold crank assumption
- User found TI article stating worst-case cold crank is **50ms** at 2.8V (ISO 16750-2)
- At 470uF: cap drops to 3.2V after 50ms — below TPSM33620's 4.5V minimum (FAIL)
- At 1000uF: cap drops to 6.0V after 50ms — comfortably above 4.5V minimum (PASS)

### Why Single Ground Net

- Honda and other OEMs separate ECU ground from chassis ground at the **vehicle wiring** level
- On the **PCB** level, ground should be a single unbroken plane (layer 2 of 4-layer stackup)
- Splitting grounds on PCB creates ground loops and makes EMI worse
- Physical placement + solid ground plane achieves the same noise isolation as separate ground wires

---

## 10. Reference Documents

| Document | Location / URL |
|----------|---------------|
| TPSM33620-Q1 datasheet | https://www.ti.com/lit/gpn/tpsm33620-q1 |
| F280049C datasheet | https://www.ti.com/lit/gpn/tms320f280049c |
| DRV8873H datasheet | https://www.ti.com/lit/gpn/drv8873 |
| AS5147U datasheet | `researchdata/AS5147U_AS5247U_DS000639_4-00.pdf` |
| TCAN3404 datasheet | https://www.ti.com/lit/gpn/tcan3404-q1 |
| LMR33630 datasheet (old, replaced) | http://www.ti.com/lit/ds/symlink/lmr33630.pdf |
| TI automotive transients article | User was reading this — covers cold crank, load dump, reverse battery |
| WEBENCH BOM export | `WBBOMDesign2.csv` in user's Downloads |
