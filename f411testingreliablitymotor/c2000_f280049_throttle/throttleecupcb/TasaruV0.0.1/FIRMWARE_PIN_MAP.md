# Tasaru V0.0.1 — Firmware Pin Map & Hardware Reference

> **Last Updated:** 2026-04-03
> **MCU:** TMS320F280049C (TQFP-100)
> **Board:** Cruise Control Actuator ECU (self-driving, start-stop compatible)
> **Assembly:** Hot plate reflow — all SMD

---

## Power Rails

| Rail | Voltage | Source | Notes |
|------|---------|--------|-------|
| `+VBAT` | 9-14V (survives 2.8V–60V transients) | Battery via F1 (3A PTC fuse) + D2 (SS34 Schottky) + D1 (SMBJ20A TVS) + C24 (1000uF hold-up) | Protected automotive power |
| `+3V3` | 3.3V | TPSM33620-Q1 power module (fixed 3.3V, 2A, 4.5V–36V input) | All logic power — rides through start-stop cranking |
| `VDD` | 3.3V | Direct from +3V3 | MCU digital core |
| `VDDA` | 3.3V | +3V3 through 60ohm ferrite bead (L2) | MCU analog supply — isolated from digital noise |
| `VDDIO` | 3.3V | Direct from +3V3 | MCU I/O banks |
| `VDDIO_SW` | 3.3V | +3V3 through 220ohm ferrite bead (L3) | MCU flash pump — TI required |
| `GND` | 0V | Common ground | Single ground net everywhere |

### Input Protection Chain

```
BAT+ → F1 (3A PTC, 1812) → D2 (SS34, SMA) → D1 (SMBJ20A, SMB) → C24 (1000uF/25V) → U5 VIN
```

| Ref | Part | Value/Rating | Package | Protects Against |
|-----|------|-------------|---------|-----------------|
| F1 | PTC Resettable Fuse | 3A hold / 6A trip | 1812 | Overcurrent / short circuit |
| D2 | SS34 Schottky | 3A / 40V | SMA | Reverse battery polarity |
| D1 | SMBJ20A TVS | 20V standoff / ~32V clamp | SMB | Load dump overvoltage (ISO 7637-2 Pulse 5) |
| C24 | Electrolytic | 1000uF / 25V | 10x12mm radial | Cold crank hold-up (rides through 50ms sag to 2.8V) |

### Buck Regulator (U5) — TPSM33620-Q1

| Ref | Part | Value | Package | Purpose |
|-----|------|-------|---------|---------|
| U5 | TPSM336203QRDNRQ1 | Fixed 3.3V / 2A | QFN 4.5x3.5mm (HotRod) | Integrated buck: FETs + inductor + boot cap inside |
| C18 | MLCC | 10uF | 0805 X5R 50V | Input bulk cap |
| C19 | MLCC | 0.1uF | 0402 X7R 50V | Input HF bypass |
| C21 | MLCC | 2.2uF | 0603 X5R 16V | VCC internal LDO bypass |
| C22 | MLCC | 22uF | 1206 X5R 10V | Output cap 1 |
| C23 | MLCC | 22uF | 1206 X5R 10V | Output cap 2 |
| R5 | Resistor | 100k | 0402 | PGOOD pullup to +3V3 |

> **U5 pin connections:** VIN ← protected +VBAT; EN ← tie to VIN (always on); MODE/SYNC ← GND (auto PFM); VCC ← C21; PGOOD ← R5 to +3V3 → MCU GPIO; GND/EP ← ground with thermal vias

### Cold Crank Survival Math

| Scenario | Battery | Cap voltage after sag | TPSM33620 min (4.5V) | Result |
|----------|---------|----------------------|----------------------|--------|
| Warm crank (start-stop) | 5-6V, 5ms | 8.45V | > 4.5V | **Safe** |
| Cold crank (ISO worst case) | 2.8V, 50ms | 6.0V | > 4.5V | **Safe** |
| Load dump | 60V spike | Clamped to 32V by D1 | < 36V max | **Safe** |
| Reverse battery | -12V | Blocked by D2 | N/A | **Safe** |

---

## ADC Configuration

### ADC Voltage Limits

| Parameter | Value | Notes |
|-----------|-------|-------|
| **ADC input range** | **0V to 3.3V** | ABSOLUTE MAX — inputs MUST be clamped to 3.3V |
| **Resolution** | 12-bit (4096 counts) | 0.806mV per LSB at 3.3V reference |
| **Reference mode** | Internal 3.3V | Set via ANAREFCTL register (default) |
| **ACQPS minimum** | 75ns (15 SYSCLK @ 100MHz) | For low impedance sources through 0ohm resistors |

### ADC Module Differences — Edge Cases

| Feature | ADC-A | ADC-B | ADC-C |
|---------|-------|-------|-------|
| Temperature sensor | No | **Yes (ch B14, internal)** | No |
| Internal test mux (VDDCORE, VDDA) | **Yes (INTERNALTESTCTL)** | No | No |
| DAC pin sharing | **A0=DACA, A1=DACB** | B15=DACA | C15=DACA |
| CMPSS7 connection | — | **B0 → CMPSS7** | — |

> **FIRMWARE WARNINGS:**
> - NEVER enable DACA output (DACOUTEN) — it shares pin with MOT_IPROPI1 (A0)
> - NEVER enable DACB output — it shares pin with A1 (now spare, was SOL_SENSE)
> - NEVER write to INTERNALTESTCTL register — it overrides ADC-A channel select
> - Consider: CMPSS7 on B0 for hardware overcurrent trip (free HW safety net on IPROPI2)
> - Consider: Sample ADC-B ch B14 for free MCU die temperature monitoring

### VREF Wiring

| Pin | Name | Connects To |
|-----|------|-------------|
| 25 | VREFHIA | 2.2uF cap (C35) to pin 27 (VREFLOA) |
| 24 | VREFHIB/VREFHIC | 2.2uF cap (C36) to pin 26 (VREFLOB/C) |
| 27 | VREFLOA | GND |
| 26 | VREFLOB/VREFLOC | GND |

> **IMPORTANT:** Caps go ACROSS the reference pair (VREFHI to VREFLO), not just to GND.

### ADC Input Assignments — Distributed Across All 3 ADCs

| Signal | Net Label | MCU Pin | ADC | Channel | Source | Max Voltage | Clamping Needed? |
|--------|-----------|---------|-----|---------|--------|-------------|-----------------|
| Motor current (primary) | `MOT_IPROPI1` | 23 | **ADC-A** | A0 | DRV8873H pin 10, via 0ohm series resistor + 1nF filter cap | ~2.0V max at 10A | No — within 3.3V |
| Motor current (redundant) | `MOT_IPROPI2` | 41 | **ADC-B** | B0 | DRV8873H pin 12, via 0ohm series resistor + 1nF filter cap | ~2.0V max at 10A | No — within 3.3V |
| Brake sense | `BRK_SENSE` | 19 | **ADC-C** | C0 | 150k/33k voltage divider from 9-14V brake signal + 0.1µF filter + TVS | 2.60V max at 14.4V | No — divider clamps it |
| Solenoid current sense | `SOL_CS` | 29 | **ADC-C** | C1 | TPS1H100B CS pin 14, via 4.7kΩ series resistor (RCS=500Ω to GND) | 3.0V normal, 4.9V fault (clamped by 4.7k) | 4.7kΩ limits fault current to 0.28mA |

> **Design Decision:** Critical signals split across ADC-A, ADC-B, and ADC-C. No single ADC failure loses more than one safety function.
> **ACQPS Note:** SOL_CS and BRK_SENSE have series resistance (4.7kΩ and 27kΩ Thevenin). Set ACQPS ≥ 30 (300ns) for these channels.

### ADC Redundancy Matrix

| ADC Failure | Motor Current | Brake | Solenoid | Safe? |
|-------------|--------------|-------|----------|-------|
| ADC-A dies | Still have ADC-B (IPROPI2) | Still have ADC-C | Still have ADC-C (SOL_CS) | YES |
| ADC-B dies | Still have ADC-A (IPROPI1) | Still have ADC-C | Still have ADC-C (SOL_CS) | YES |
| ADC-C dies | Still have both | Lost — fallback to digital GPIO | Lost — TPS1H100B still protects internally | YES (degraded) |

### Spare ADC Breakout Pads (for future use)

| MCU Pin | ADC Channel | Breakout Label |
|---------|-------------|----------------|
| 9 | A2 | SPARE_ADC_A2 |
| 22 | A1 | SPARE_ADC_A1 (was SOL_SENSE, freed by TPS1H100B) |
| 35 | A5 | SPARE_ADC_A5 |
| 8 | B3/VDAC | SPARE_DAC_B3 |

> **Note:** ADC-C1 (pin 29) is now assigned to `SOL_CS` — no longer spare.

---

## SPI Bus — AS5147U Magnetic Encoder

| Signal | Net Label | MCU Pin | GPIO | Peripheral | Mux | Notes |
|--------|-----------|---------|------|------------|-----|-------|
| MOSI | `SPI_MOSI` | — | GPIO8 | SPIA_SIMO | 7 | Command/register writes |
| Clock | `SPI_CLK` | — | GPIO9 | SPIA_CLK | 7 | Max 10MHz for AS5147U |
| MISO | `SPI_MISO` | — | GPIO10 | SPIA_SOMI | 7 | 14-bit position data + CRC |
| Chip Select | `SPI_CS` | — | GPIO11 | GPIO (manual CS) | 0 | Active low — drive high when not communicating |

> **AS5147U Notes:**
> - 3.3V operation: VDD and VDD3V3 tied together
> - SPI Mode 1 (CPOL=0, CPHA=1)
> - CRC verification on every read (firmware must validate)
> - AEC-Q100 / ASIL D self-diagnostics available

---

## Motor Driver — DRV8873H (PH/EN Mode)

| Signal | Net Label | MCU Pin | GPIO | Peripheral | Mux | Notes |
|--------|-----------|---------|------|------------|-----|-------|
| PWM (speed) | `MOT_PWM` | — | GPIO0 | EPWM1_A | 1 | Duty cycle = motor speed |
| Phase (direction) | `MOT_PH` | — | GPIO1 | GPIO output | 0 | HIGH = forward, LOW = reverse |
| Sleep (active low) | `MOT_nSLEEP` | — | GPIO3 | GPIO output | 0 | Must be HIGH for operation, pullup to 3.3V |
| Fault (active low) | `MOT_nFAULT` | — | GPIO5 | GPIO input | 0 | LOW = fault condition — read in ISR |
| Current sense 1 | `MOT_IPROPI1` | 23 | — | ADC-A ch A0 | — | See ADC section |
| Current sense 2 | `MOT_IPROPI2` | 41 | — | ADC-B ch B0 | — | See ADC section |

> **DRV8873H Hard-Wired Config (no firmware needed):**
> - MODE (pin 3) → GND (PH/EN mode)
> - SR (pin 4) → GND (fast slew rate)
> - nITRIP (pin 5) → GND (default trip point)
> - nOL (pin 6) → GND (default open load detect)
> - DISABLE (pin 1) → driven by safety MOSFET circuit (Q2 via Q_BRK_A brake override)

---

## Solenoid Driver — TPS1H100B-Q1 (Smart High-Side Switch)

| Signal | Net Label | MCU Pin | GPIO | Peripheral | Notes |
|--------|-----------|---------|------|------------|-------|
| Control input | `SOL_GATE` | — | GPIO7 | GPIO output | Through 4.7kΩ series resistor to IN (pin 3) |
| Current sense | `SOL_CS` | 29 | — | ADC-C ch C1 | Through 4.7kΩ from CS (pin 14), RCS=500Ω to GND |
| Diagnostics enable | `DIAG_EN` | — | — | Tied to 3.3V | Through 4.7kΩ series resistor (always enabled) |

### TPS1H100B Pin Connections

| Pin | Name | Connection | Component Values |
|-----|------|------------|-----------------|
| 1 | NC | Float | — |
| 2 | GND | GND via R_GND (1kΩ) \|\| D_GND (BAS21) | Inductive load GND network |
| 3 | IN | `SOL_GATE` (GPIO7) via R_SER1 (4.7kΩ); also Q_BRK_B collector | 500kΩ internal pulldown = defaults OFF |
| 4 | NC | Float | — |
| 5,6,7 | OUT | All tied → SOL+ (actuator connector) | High-side output to solenoid |
| 8,9,10 | VS | All tied → +BATT_PROTECTED | C_VS1 (10µF) + C_VS2 (0.1µF) decoupling to GND |
| 11 | NC | Float | — |
| 12 | DIAG_EN | 3.3V via R_SER2 (4.7kΩ) | Always HIGH = full diagnostics active |
| 13 | CL | R_CL (620Ω) to GND | Sets current limit at ~4A |
| 14 | CS | R_CS (500Ω) to GND; tap via R_SER3 (4.7kΩ) to MCU ADC (SOL_CS) | Current sense: V_CS = I_LOAD × k_ILIS × R_CS |
| Pad | Thermal | GND via same network as pin 2 | Thermal vias underneath |

### GND Network (Pin 2 + Thermal Pad)

```
IC GND ──┬── R_GND (1kΩ) ──┬── Board GND
         │                   │
         └── D_GND (BAS21) ──┘
```

> Parallel diode provides fast clamp path for inductive flyback; resistor provides DC ground.

### IN Pin Brake Override Circuit

```
                                              TPS1H100B
                                            ┌───────────┐
SOL_GATE (GPIO7) ── R_SER1 (4.7kΩ) ──┬──── │ IN (pin 3)│
                                      │     │           │
                                 Q1B collector  500kΩ internal pulldown
                                      │     │           │
                                 Q1B emitter │          │
                                      │     └───────────┘
                                     GND        GND
```

> **No 3.3V pullup.** The IN pin defaults LOW via the internal 500kΩ pulldown.
> If MCU dies/resets → GPIO floats → IN = LOW → solenoid OFF (fail-safe).
> If brake pressed → Q1B ON → IN = LOW → solenoid OFF (hardware override).
> R_SER1 isolates MCU from Q1B and protects GPIO from transients.

### Built-In Protection (no external parts needed)

| Fault | TPS1H100B Response | Firmware Indicator |
|-------|-------------------|-------------------|
| Short to GND | Current limit + thermal shutdown | SOL_CS reads >4A equivalent |
| Short to battery | Output clamped | SOL_CS reads abnormal |
| Open load | Detected via DIAG_EN | SOL_CS reads ~0 when SOL_GATE = HIGH |
| Overtemperature | Auto-shutdown, auto-retry | SOL_CS drops to 0 during thermal event |
| Reverse battery | Internal protection | — |
| Load dump (ISO 7637-2) | Survives pulses to 40V+ | — |
| Inductive flyback | Internal clamp diode | No external flyback diode needed |

### Solenoid Current Sense Scaling

```
V_CS = I_LOAD × k_ILIS × R_CS
     = I_LOAD × (450µA/A typical) × 500Ω
     = I_LOAD × 0.225 V/A

At 3A solenoid: V_CS = 0.675V → ADC count = 0.675/3.3 × 4096 = 838
At 4A limit:    V_CS = 0.9V   → ADC count = 0.9/3.3 × 4096 = 1117
```

---

## CAN Bus — PTCAN3404DRQ1

| Signal | Net Label | MCU Pin | GPIO | Peripheral | Mux | Notes |
|--------|-----------|---------|------|------------|-----|-------|
| CAN TX | `CAN_TX` | 99 | GPIO31 | CAN-A TX | 1 | 500 kbps |
| CAN RX | `CAN_RX` | 98 | GPIO30 | CAN-A RX | 1 | 500 kbps |

> **BUG FIX (2026-04-03):** Previous version had GPIO30/31 swapped. Verified against pin_map.h: GPIO30 = CANA_RX (mux 1), GPIO31 = CANA_TX (mux 1).

> **CAN Config:**
> - Baud: 500 kbps
> - RX Mailbox ID: 0x100
> - TX Mailbox ID: 0x101
> - PTCAN3404: SHDN (pin 5) → HIGH for normal, STB (pin 8) → LOW for normal
> - 120ohm termination resistor on CAN_H/CAN_L (if end of bus)

---

## UART Debug — SCI-A

| Signal | Net Label | MCU Pin | GPIO | Peripheral | Notes |
|--------|-----------|---------|------|------------|-------|
| UART TX | `UART_TX` | 1 | GPIO28 | SCI-A TX | 115200 baud, telemetry every 200ms |
| UART RX | `UART_RX` | 100 | GPIO29 | SCI-A RX | 115200 baud |

---

## Safety / Control Signals

| Signal | Net Label | MCU Pin | GPIO | Mux | Direction | Notes |
|--------|-----------|---------|------|-----|-----------|-------|
| Motor enable | `SAFE_DISABLE` | — | GPIO6 | 0 | Output | HIGH = motor runs (Q2 ON, DRV DISABLE pulled LOW); LOW/float = motor disabled (fail-safe) |
| Brake sense | `BRK_SENSE` | 19 | — | — | ADC Input | 150k/33k divider, 0.1µF filter, TVS protected |
| Solenoid gate | `SOL_GATE` | — | GPIO7 | 0 | Output | HIGH = solenoid ON (drives TPS1H100B IN through 4.7kΩ + NPN brake override) |
| Solenoid current sense | `SOL_CS` | 29 | — | — | ADC Input | TPS1H100B CS output, via 4.7kΩ series resistor |
| Power good | `PWR_PGOOD` | — | GPIO12 | 0 | Input | HIGH = 3.3V rail stable (TPSM33620-Q1, open-drain, 100k pullup to +3V3) |
| Motor fault | `MOT_nFAULT` | — | GPIO5 | 0 | Input | LOW = DRV8873H fault (overcurrent, thermal, etc.) |
| Motor sleep | `MOT_nSLEEP` | — | GPIO3 | 0 | Output | LOW = sleep mode, HIGH = active |

---

## JTAG Debug Header (2x5, 2.54mm)

| Header Pin | Net Label | MCU Pin | Notes |
|------------|-----------|---------|-------|
| 1 | `+3V3` | — | Target voltage sense for debug probe |
| 2 | `GND` | — | |
| 3 | `JTAG_TMS` | 62 (TMS) | |
| 4 | `GND` | — | |
| 5 | `JTAG_TCK` | 60 (TCK) | |
| 6 | `GND` | — | |
| 7 | `JTAG_TDO` | 61 (GPIO37_TDO) | Directly on U1B |
| 8 | `GND` | — | |
| 9 | `JTAG_TDI` | 63 (GPIO35_TDI) | Directly on U1B |
| 10 | `GND` | — | |

---

## Clock Source — SiTime SIT2024B

| Signal | Net Label | MCU Pin | Notes |
|--------|-----------|---------|-------|
| Oscillator output | `OSC_OUT` | 69 (X1) | 10 MHz MEMS, AEC-Q100, ±25ppm |

> **SIT2024BA-S2-XXE-10.000000**
> - SOT-23-5 package
> - Pin 1 (GND) → GND
> - Pin 2 (NC) → no connect
> - Pin 3 (OE) → +3V3 (always enabled)
> - Pin 4 (VDD) → +3V3 + 0.1uF decoupling cap
> - Pin 5 (OUT) → MCU X1 (pin 69)
> - GPIO18/X2 (pin 68) → tie to GND (not using external crystal mode)

---

## System / Config Pins (U1D)

| MCU Pin | Name | Connects To | Notes |
|---------|------|-------------|-------|
| 73 | VREGENZ | GND via 0ohm (populated) / +3V3 via 0ohm (DNP) | Internal 1.8V regulator enabled |
| 2 | XRS_N | 2.2k pullup to +3V3 | Active low reset — add tactile button to GND if desired |
| 49 | FLT1 | No-connect (X) | Internal pullup → default 3 wait states for 100MHz |
| 48 | FLT2 | No-connect (X) | Same |
| 69 | X1 | SIT2024B output | 10 MHz clock input |

---

## MCU Power Decoupling (U1E)

| Net | Pins | Decoupling |
|-----|------|------------|
| VDD | 4, 46, 71, 87 | 2x 10uF + 4x 0.1uF |
| VDDA | 11, 34 | 4x 2.2uF (through 60ohm ferrite from +3V3) |
| VDDIO | 3, 47, 70, 88 | 2x 10uF + 5x 0.1uF |
| VDDIO_SW | 80 | Through 220ohm ferrite from +3V3 |
| VSS | 5, 45, 72, 86 | All to GND |
| VSSA | 12, 33 | All to GND |
| VSS_SW | 82 | GND |

---

## PGA / Unused Analog Pins (U1C)

### PGA Grounds — All to GND
| Pin | Name |
|-----|------|
| 14 | PGA1_GND |
| 15 | PGA3_GND |
| 13 | PGA5_GND |
| 32 | PGA2_GND / PGA4_GND / PGA6_GND |
| 42 | PGA7_GND |

### No-Connect Pins (X)
PGA Inputs: 18, 30, 20, 31, 16, 28, 43
Unused ADC: 36 (A4), 6 (A6), 37 (A8), 38 (A9), 40 (B1), 7 (B2), 39 (B4), 21 (C2), 17 (C4), 44 (C14)

---

## GPIO Assignments Summary (U1A + U1B)

### Assigned — All GPIOs
| GPIO | Pin | Function | Mux | Net Label | Direction |
|------|-----|----------|-----|-----------|-----------|
| GPIO0 | — | EPWM1_A | 1 | `MOT_PWM` | Output (PWM) |
| GPIO1 | — | GPIO | 0 | `MOT_PH` | Output |
| GPIO3 | — | GPIO | 0 | `MOT_nSLEEP` | Output |
| GPIO5 | — | GPIO | 0 | `MOT_nFAULT` | Input |
| GPIO6 | — | GPIO | 0 | `SAFE_DISABLE` | Output |
| GPIO7 | — | GPIO | 0 | `SOL_GATE` | Output |
| GPIO8 | — | SPIA_SIMO | 7 | `SPI_MOSI` | Output |
| GPIO9 | — | SPIA_CLK | 7 | `SPI_CLK` | Output |
| GPIO10 | — | SPIA_SOMI | 7 | `SPI_MISO` | Input |
| GPIO11 | — | GPIO | 0 | `SPI_CS` | Output |
| GPIO12 | — | GPIO | 0 | `PWR_PGOOD` | Input |
| GPIO28 | 1 | SCI-A TX | 1 | `UART_TX` | Output |
| GPIO29 | 100 | SCI-A RX | 1 | `UART_RX` | Input |
| GPIO30 | 98 | CAN-A RX | 1 | `CAN_RX` | Input |
| GPIO31 | 99 | CAN-A TX | 1 | `CAN_TX` | Output |
| GPIO35 | 63 | JTAG TDI | — | `JTAG_TDI` | Input |
| GPIO37 | 61 | JTAG TDO | — | `JTAG_TDO` | Output |

### Reserved GPIOs (avoid using)
| GPIO | Reason |
|------|--------|
| GPIO2 | EPWM2_A — keep free for future second PWM channel |
| GPIO4 | EPWM3_A — keep free for future use |
| GPIO18 | X2 — tied to GND (crystal input, not using external crystal) |

### Remaining Free GPIOs (available for future expansion)
GPIO13, GPIO14, GPIO15, GPIO16, GPIO17, GPIO22, GPIO23, GPIO24, GPIO25, GPIO26, GPIO27, GPIO32, GPIO33, GPIO34, GPIO39, GPIO40, GPIO41, GPIO56, GPIO58

---

## Safety Checks (15 Total)

### Critical (must trigger safe state immediately)
1. Motor overcurrent — ADC-A (IPROPI1) or ADC-B (IPROPI2) exceeds threshold
2. Encoder failure — SPI CRC mismatch or AS5147U self-diagnostic fault
3. Position out of range — encoder reports value outside valid actuator range
4. Hardware brake override — brake pedal activates Q_BRK_A/Q_BRK_B, hardware overrides both motor DISABLE and solenoid IN
5. nFAULT active — DRV8873H reports overcurrent/thermal/short
6. PGOOD low — 3.3V rail unstable or lost

### Important (trigger safe state after confirmation)
7. CAN timeout — no valid command in >500ms
8. Watchdog timeout — firmware hung
9. Solenoid sense mismatch — SOL_GATE high but SOL_CS reads ~0 (open load) or SOL_GATE low but SOL_CS reads >0 (stuck on)
10. DISABLE state mismatch — firmware didn't command disable but DISABLE is active
11. Redundant current mismatch — IPROPI1 and IPROPI2 disagree significantly
12. Solenoid overcurrent — SOL_CS reads above 4A equivalent (TPS1H100B will auto-limit, firmware should log)

### Informational (log and report, no immediate action)
13. Temperature warning — DRV8873H nFAULT with thermal pre-warning
14. SPI retry count — encoder communication retries exceeded threshold
15. ADC calibration drift — reference voltage readings outside expected range

---

## Firmware Migration Notes (from BTS7960 prototype)

| Module | Status | Changes Needed |
|--------|--------|---------------|
| `main.c` | Minor updates | Init sequence changes for new peripherals |
| `throttle_config.h` | Major rewrite | All pin assignments, new constants, TPS1H100B scaling |
| `board.c` | Major rewrite | New GPIO mux config — GPIO0-12 + GPIO28-31 (see mux column above) |
| `can_io.c` | **Fix required** | GPIO30/31 were SWAPPED — GPIO30=RX, GPIO31=TX (not the other way) |
| `safety.c` | Extend | Add checks 9-15, integrate nFAULT, PGOOD, SOL_CS diagnostics |
| `motor_epwm.c` | **Full rewrite** | Dual ePWM → single EPWM1_A on GPIO0, PH/EN mode |
| `encoder_gpio.c` | **Replace entirely** | PWM pulse width → SPI driver for AS5147U |
| `adc_sense.c` | Major rewrite | Channels: A0 (IPROPI1), B0 (IPROPI2), C0 (BRK_SENSE), C1 (SOL_CS). Set ACQPS≥30 for C0/C1. |
| `sci_io.c` | Reusable | Same UART, same baud — no changes |
| `pid.c` | Reusable | Same algorithm, same constants (Kp=12, Ki=0.3, Kd=1.5) |
| **NEW:** `spi_encoder.c` | Write from scratch | AS5147U SPI driver with CRC validation (GPIO8-11, mux 7) |
| **NEW:** `drv8873h.c` | Write from scratch | nFAULT handler (GPIO5), nSLEEP control (GPIO3), IPROPI scaling |
| **NEW:** `solenoid.c` | Write from scratch | SOL_GATE (GPIO7) control, SOL_CS (ADC-C1) monitoring, open/short load diagnostics |
