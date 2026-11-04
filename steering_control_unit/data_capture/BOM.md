# EPS Data Logger: BOM v2

Checked against the Insight service manual (see [EPS_PINOUT.md](EPS_PINOUT.md) for sources).
Goal: the fewest parts that log the EPS **without being able to affect it**, built on Adafruit Feather boards.

| Related file | What it is |
|--------------|------------|
| [LOGGER_WIRING.svg](LOGGER_WIRING.svg) | Full system wiring diagram: every tap, its probe resistor, front end and Feather pin |
| [firmware/eps_logger](firmware/eps_logger/README.md) | Logger firmware: 1 kHz logging, one file per drive, lead diagnostics |
| [tools/decode_log.py](tools/decode_log.py) | Turns a `.BIN` log into CSV and prints a summary |
| [PROTO_BUILD.md](PROTO_BUILD.md) | Hand-build guide for section 3 on the FeatherWing Proto: layout, build order, bench tests |
| [STACK_MODES.md](STACK_MODES.md) | **Running with the bench stim wing:** power and grounding for logger-only, stim-only and both together, in the car and on a bench EPS |
| [AUX_LINK.md](AUX_LINK.md) | **Second microcontroller:** how another board (for example an OBD/IMA diag node) talks to the Feather over the stack's CAN bus: wiring, IDs 0x6C0–0x6CF, frames, interlocks |
| [hardware/eps_tap_wing](hardware/eps_tap_wing/README.md) | **Assembled option:** a KiCad FeatherWing carrying all of section 3, with plug-in terminals. It replaces the Proto wing and the loose parts. |

## What the manual changed from v1

| Finding (manual source) | v1 assumed | v2 does |
|-------------------------|------------|---------|
| **Vcc1 is battery voltage**, not 5 V (Fig. 9, DTC 17) | 5 V divider | Dropped from the core build (optional channel) |
| **NEP is a ~12 V pulse train** (~6 V average at idle; Fig. 9, DTC 23) | 5 V divider | 12 V-class divider |
| WLP, RT+ and IG1 are battery-level (Fig. 9) | — | 12 V-class dividers |
| VT3/VT6 are 1.5–3.5 V; **sensor output impedance isn't published**; ECU cross-checks both channels and sets EEPROM DTCs that disable assist (DTC 11/15/16) | 300 kΩ divider load | **1 MΩ at the probe + op-amp buffer.** Load is in the same class as the multimeter Honda's own procedure uses with the engine running. |
| Motor is driven by an **H-bridge inside the ECU**: both wires switch, current flows both ways, 40 A fuse (Fig. 11) | Unspecified clamp | **±50 A split-core Hall sensor** on the GRN (M−) wire, filtered to 250 Hz to reject PWM ripple |
| **EPS has no CAN** and no EPS data on the DLC (Fig. 11, OBDIIC&C sheet) | — | Analog taps are the only route. CAN is kept only to talk to the throttle ECU later. |
| All taps are at the ECU **behind the right kick panel** (Connector Physical Pinout #100/101) | — | One tap point, inside the cabin |
| A car-powered logger shares ground with the **40 A motor return** (Fig. 11) | UBEC from car 12 V | **Floating USB power bank** with a single ground reference at T/S GND (B9). No UBEC/TVS/fuse needed. |

**Board choice is confirmed: Feather M4 CAN.** The design needs 5 analog channels, and two of them (VT3/VT6) need sampling at the same instant. Pins A2/A3 (PB08/PB09) connect to **both** ADCs, so ADC0 and ADC1 can sample VT3 and VT6 together. The RP2040 Feather has only 4 analog pins and a single ADC with known DNL errors, so it's out.

---

## 1. Core boards (Adafruit PIDs and prices checked on adafruit.com, Oct 2026)

| Qty | Part | PID | Price |
|-----|------|-----|-------|
| 1 | **Feather M4 CAN Express** (ATSAME51J19, 120 MHz, dual 1 MSPS 12-bit ADC, USB-C) | 4759 | $24.95 |
| 1 | **Adalogger FeatherWing** (PCF8523 RTC + microSD) | 2922 | $8.95 |
| 1 | **FeatherWing Proto** (holds the front end) | 2884 | $4.95 |
| 1 | **FeatherWing Tripler Mini Kit** (female headers included, solder required) | 3417 | $8.50 |
| 1 | CR1220 coin cell (RTC backup) | 380 | $0.95 |
| 1 | microSD card, 8–32 GB, FAT32 | — | — |

The Adafruit subtotal is about **$48**.

## 2. Motor current sensor (not Adafruit)

| Qty | Part | Notes |
|-----|------|-------|
| 1 | **YHDC HSTS016L, ±50 A** split-core Hall sensor | 5 V supply. 2.5 V ± 0.625 V at ±50 A (12.5 mV/A), measures up to 75 A, DC–25 kHz, load ≥ 10 kΩ. **Clips around the wire, with no electrical contact.** Reseller specs vary, so check the datasheet for the unit you buy. |

## 3. Front end (all on the FeatherWing Proto)

| Qty | Part | Used for |
|-----|------|----------|
| 1 | **MCP6024-I/P** quad op-amp, PDIP-14 (rail-to-rail I/O, 1 pA bias, ±0.5 mV offset, 2.5–5.5 V), plus a DIP-14 socket | Buffers VT3, VT6, Vcc2, +B. Powered from the Feather's 3V3, so its outputs **can't exceed the ADC range**. |
| 4 | 1 MΩ 1 % | **Inline at the probe:** VT3, VT6, Vcc2, +B |
| 3 | 1.8 MΩ 1 % | Op-amp input to GND: VT3, VT6, Vcc2 (5 V → 3.21 V) |
| 1 | 150 kΩ 1 % | Op-amp input to GND: +B (25 V → 3.26 V) |
| 4 | 1 nF C0G | Op-amp inputs: anti-alias / RF filter, ~250 Hz on VT3/VT6/Vcc2 and ~1.2 kHz on +B |
| 5 | 100 kΩ | **Inline at the probe:** VSP, NEP, WLP, IG1, RT+ |
| 1 | 180 kΩ | VSP to GND (5 V → 3.2 V) |
| 4 | 27 kΩ | NEP, WLP, IG1, RT+ to GND (14.4 V → 3.06 V; still reads high at 11 V) |
| 5 | 1 nF | Digital inputs: noise filter |
| 5 | BAT85 Schottky (DO-35) | Digital inputs, clamped to 3V3 (cathode to 3V3) |
| 1 + 1 | 10 kΩ + 18 kΩ | Current sensor output divider (5 V max → 3.21 V) |
| 1 | 100 nF | Current channel at the ADC (~250 Hz, rejects motor PWM ripple) |
| 2 | 100 nF | Decoupling: op-amp, current sensor |
| 1 | 100 Ω | **Inline in the ground lead** at B9 |
| 2 | 6-position 3.5 mm screw terminal | Field wiring lands here |

RT+ is optional. If you skip it, drop one each of the 100 kΩ, 27 kΩ, 1 nF and BAT85.

## 4. Power

| Qty | Part | Notes |
|-----|------|-------|
| 1 | USB power bank with an **always-on / low-current mode** | The logger draws only ~60 mA, which many power banks treat as "nothing connected" and switch off |
| 1 | USB cable to USB-C | |

The power bank's 5 V reaches the Feather's **USB pin**, which also powers the current sensor. The logger **floats**: its only connection to car ground is the 100 Ω lead to B9. That means no ground loop through the EPS motor's 40 A return path.

> Don't connect a laptop that's charging from the car while logging, because that makes a ground loop. A laptop on battery is fine.

## 5. Tap kit

| Qty | Part | Notes |
|-----|------|-------|
| 10 | Back-probe pins, thin and insulated | VT3, VT6, Vcc2, T/S GND, +B, VSP, NEP, WLP, IG1, RT+ |
| 1 | 12-core shielded cable, ~1 m | Shield connects to logger GND **only at the logger end** |
| — | Heat-shrink | Covers each inline resistor **at the probe end**, so the whole cable sits behind its protection resistor |

---

## Per-channel circuits

**VT3, VT6, Vcc2 (0–5 V signals).** +B is the same, with 150 kΩ instead of 1.8 MΩ.
```
ECU pin ──[1 MΩ, at probe]── cable ──┬────────────── MCP6024 +IN      OUT ──► ADC pin
                                      ├──[1.8 MΩ]── GND               (−IN tied to OUT)
                                      └──[1 nF]──── GND
```

**Motor current**
```
HSTS016L clipped on GRN (M−) wire at connector A
   +5V ◄── Feather USB pin       GND ◄── logger GND
   OUT ──[10 kΩ]──┬── A4
                  ├──[18 kΩ]── GND
                  └──[100 nF]─ GND
```

**Digital: VSP (R2 = 180 kΩ); NEP, WLP, IG1, RT+ (R2 = 27 kΩ)**
```
ECU pin ──[100 kΩ, at probe]── cable ──┬── D pin
                                        ├──[R2]──── GND
                                        ├──[1 nF]── GND
                                        └──|>|──── 3V3   (BAT85, cathode to 3V3)
```

**Ground reference (one point only)**
```
ECU B9 (T/S GND) ──[100 Ω, at probe]── logger GND
```

## Logger pin map

Implemented in `firmware/eps_logger/config.h`.

| Feather pin | Signal | Notes |
|-------------|--------|-------|
| A0 | Vcc2 (buffered) | Used for ratiometric correction of VT3/VT6 |
| A1 | +B (buffered) | Battery sag under motor load |
| **A2** | **VT3** (buffered) | PB08, on both ADC0 and ADC1 |
| **A3** | **VT6** (buffered) | PB09, on both ADC0 and ADC1. Sampled at the same instant as VT3. |
| A4 | Motor current | |
| A5 | Spare | Vcc1 if a second op-amp is added |
| D5 | VSP | Edge timestamps → speed |
| D6 | NEP | Edge timestamps → rpm |
| D9 | WLP | Fault / assist-off state, also blink codes |
| D11 | RT+ (optional) | Relay state |
| D12 | IG1 | Opens a new log file at key-on and closes it at key-off, so the SD card isn't corrupted |
| D10 | SD CS | Adalogger |
| SDA/SCL | RTC | Adalogger |
| D4 | — | Free on the Feather M4 CAN (the CAN 5 V boost enable is internal pin PB13). The bench stim wing uses D4 and TX for its NEP/VSP outputs |

## Worst-case fault check

| Fault | Effect on the car |
|-------|-------------------|
| Logger cable shorted to ground or to 16 V anywhere past the probe | Torque lines: ≤ 16 µA through 1 MΩ, which is multimeter-class loading. Digital lines: ≤ 0.16 mA through 100 kΩ, negligible against their drivers. **EPS unaffected.** |
| Logger unpowered while connected | Same limits. Back-feed into the unpowered op-amp is ≤ 3.5 µA. |
| Ground probe slips onto a 12 V pin | The logger floats and every other lead has ≥ 100 kΩ inline, so no meaningful current flows. The 100 Ω only matters if the logger is accidentally grounded elsewhere, e.g. a car-charged laptop, where it limits the loop current. |
| A back-probe pin bridges two neighbouring terminals at the connector | This is the one fault the resistors can't cover, so prevent it: fit probes with the **ignition OFF**, use insulated pins, and continuity-check between neighbouring probes before key-on |
| Anything | The logger never cuts or interrupts an EPS wire, never touches SCS/DIAG-H, and never sources power into the car |

## First-session calibration

1. **Lead check:** with the probes fitted, key ON and engine OFF, send `c` over USB serial. The firmware compares every tap with the manual's values. Repeat with the engine running.
2. **Zeros:** key ON, engine OFF, wheels straight. Log VT3/VT6 at zero torque, Vcc2, and the current sensor offset (relay open, so 0 A). The firmware measures the current zero automatically at every key-on.
3. **Torque scale:** engine OFF, so no assist. Pull the rim with a spring scale at a known radius, in both directions, at a few forces. Torque = force × radius. Map VT3/VT6 against N·m and confirm whether they move in opposite directions.
4. **Speed and rpm:**
   - VSP pulses per km: check against GPS or a measured distance.
   - NEP pulses per rev: check against the tachometer at a steady rpm.

## Later options (not in the core build)

| Option | Parts |
|--------|-------|
| Vcc1 + extra spare channels | A second MCP6024, plus 1 MΩ + 150 kΩ per channel |
| Yaw rate / lateral g alongside torque | Adafruit LSM6DSOX (PID 4438, $11.95) plus a STEMMA QT-to-male-header cable (PID 4209, $0.95). The Feather M4 CAN has no STEMMA QT port. |
| Powered from the car | Use the [bench stim wing](../bench_stim_wing/README.md) (12 V in, protected, feeds the stack) with its ground wired to the **ECU end of A4 (PG)**: logger ground then sits at the ECU's own ground, the same node as B9. See [STACK_MODES.md](STACK_MODES.md). Don't power it from a USB car adapter grounded elsewhere |
