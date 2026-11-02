# EPS tap FeatherWing

A factory-assembled FeatherWing that holds the logger's whole front end, so the build has no loose resistors. It replaces the FeatherWing Proto and every part in section 3 of [BOM.md](../../BOM.md). The circuit is the same apart from the changes listed below.

**Rev 1.5: cheaper to build at JLCPCB**
- Divider values were re-picked from JLCPCB's **Basic** library, which loads with no setup fee; each Extended part type costs about $3. Extended types dropped from 8 to 2: the op-amp and the ADS1115.

  | Channel | Was | Now | Effect |
  |---|---|---|---|
  | VT3, VT6, VCC2 | 1M / 1.8M | 1M / 1M | 5 V → 2.50 V (was 3.21 V); resolution still ~0.016 %/step |
  | +B, X1–X4 | 1M / 150k | 1M / 100k | 36 V full scale (was 25 V); ~9 mV/step |
  | NEP, WLP, RT+, IG1 | 100k / 27k | 100k / 33k | 14.4 V → 3.57 V; still reads high at 9 V |
  | VSP | 100k / 180k | 100k / 100k | 5 V → 2.50 V |
  | Current, Vref | 10k / 18k | 10k / 47k | More of the ADC range used: better resolution |
  | Clamp diodes | BAT54WS | 1N4148WS | Basic part, lower leakage |
  | Op-amps | MCP6024 | MCP6004 | Same pins; about 1/10 the price |
- The BOM now carries a **JLCPCB Part #** column, so the upload matches exactly. The MCP6004 has no part number yet: pick MCP6004-I/ST at upload.
- `firmware/eps_logger/config.h` has a new `CFG_FRONT_END_TAP_WING` switch that sets the divider constants: 1 for this board, 0 for the hand-built Proto Wing.

**Rev 1.4: status LED, 4-layer board**
- **Red status LED** (D6, with R32 1 kΩ) on the **back**, next to the "LED" label. It's wired to Feather pin D13, which the logger firmware already uses for its status codes (`CFG_PIN_LED`), so it mirrors them with no firmware change. The two parts are 0805 so you can hand-solder them; or ask JLC for bottom-side assembly of just those two.
- **Now a 4-layer board.** With rev 1.3's extra parts plus the LED, the 2-layer board was full: the router always left one or two connections. With an inner GND plane and a third routing layer it routes completely, and the signal traces got **thicker (0.3 mm)**.
- Solder jumpers use plain rectangular pads. The rounded custom pads were being exported to the auto-router on the wrong layer.

**Rev 1.3: room to grow**
- **Four spare protected inputs, X1–X4,** for future signals: motor M+/M−, Vcc1, ground shift, or anything else that's a voltage. Each has the same protection as +B: a 1 MΩ series resistor with a bypass jumper, a 100 kΩ divider (0–36 V full scale) and a 1 nF filter. A second quad op-amp (U2) buffers them into an **ADS1115 16-bit ADC** (U3) on I2C at address 0x48.
- **J2 is now a 15-way JST XA** (2.5 mm pitch, positive locking latch). It's the same length as the old 11-way plug and carries the original 11 signals plus X1–X4.
- Passives went from 0603 to 0402 and the op-amps from SOIC-14 to TSSOP-14 to make room. Everything is still factory-assembled on one side.

**Rev 1.2:**
- All wiring is from the **back**: J2, J3 and the bypass jumpers are on the bottom side, and all 42 SMD parts moved to the top (still one-sided assembly).
- The wing is a "bottom Wing" (allowed by the Feather spec): it sits at the bottom of the stack, with the Feather plugged in on top.
- **HINDSIGHT** is on the back silkscreen, together with the terminal labels and "IGN OFF TO PLUG". The revision is on the top.

**Rev 1.1:**
- Follows the [Adafruit Feather specification](https://learn.adafruit.com/adafruit-feather/feather-specification).
- Each series resistor has a bypass solder jumper, so the probe resistors can live on the board or at the probe ends.
- Adds the current sensor's Vref channel on A5.
- The clamp terminal (J3) is now 4-way.

## Feather specification check

| Spec item | This wing |
|-----------|-----------|
| 0.9" × 2.0" board, 0.1" holes at each corner | 22.86 × 50.8 mm, four 2.5 mm holes 2.54 mm in from the edges |
| 16-pin strip on the bottom side, centred 1.0" from the left (USB) edge | Pins 1–16 at x = 6.35–44.45 mm (centre 25.4 mm), y = 21.59 mm |
| 12-pin strip on the top side, 1.2" from the left edge, 0.8" between strips | Pins 17–28 at x = 16.51–44.45 mm (centre 30.48 mm), y = 1.27 mm |
| Wings take 3.3 V from the Feather; at least 100 mA available | The op-amps and ADC draw about 2 mA from 3V3; the clamp draws ≤12 mA from USB |
| AREF, the free pin (right of TX), RX/TX, SPI and its CS: avoid | None of them are used |
| I2C pull-ups belong on Wings | The ADS1115 uses I2C; the Adalogger in the same stack already has the pull-ups |
| Reset button "ideally" on Wings | Not fitted: the OLED FeatherWing on top of the stack has one |
| Wings may be "bottom" Wings with the Feather on top | Used here, so the terminals face out of the back |

The board check script confirms the header pads line up with the Feather footprint every time the board is regenerated.

## Design rules

| Item | Value |
|------|-------|
| Layers | 4: **F.Cu** parts + signals, **In1.Cu** solid GND plane, **In2.Cu** signals + 3V3 + GND fill, **B.Cu** connectors + signals + GND fill |
| Signal traces | 0.3 mm. They carry microamps, so this is for robustness, not current |
| 3V3 / 5V traces | 0.3 mm. The 3V3 traces are short stubs and the USB 5 V carries ≤15 mA; 0.3 mm also fits into the 0.5 mm-pitch ADC pins |
| Status-LED traces | 0.2 mm (about 1.4 mA) |
| Clearance | 0.15 mm |
| Vias | 0.5 mm pad, 0.25 mm drill |

JLCPCB's standard minimum is 0.127 mm, so these widths have plenty of margin. A 4-layer board costs a little more than 2-layer, but it gives a solid ground plane under all the high-impedance inputs, which is better for noise. `make_pcb.py` still builds the 2-layer version if you set `EPS_LAYERS=2`, but with this many parts it won't fully route.

| File | What it is |
|------|------------|
| [eps_tap_wing.kicad_sch](eps_tap_wing.kicad_sch) / [schematic PDF](doc/eps_tap_wing_schematic.pdf) | Schematic. ERC: 0 errors, 0 warnings |
| [eps_tap_wing.kicad_pcb](eps_tap_wing.kicad_pcb) | Board, 50.8 × 22.86 mm (standard FeatherWing), 4 layers, fully routed. DRC: 0 errors, 0 unconnected, matches the schematic |
| [doc/render_top.png](doc/render_top.png), [doc/render_bottom.png](doc/render_bottom.png) | 3D renders |
| [fab/eps_tap_wing_bom_jlc.csv](fab/eps_tap_wing_bom_jlc.csv), [fab/eps_tap_wing_cpl_jlc.csv](fab/eps_tap_wing_cpl_jlc.csv) | Assembly BOM and pick-and-place, in JLCPCB's column format, SMD parts only |
| [gen/](gen/) | Generator scripts. The schematic and board are built from [gen/design.py](gen/design.py) |

## The stack

```
  OLED FeatherWing
  Adalogger FeatherWing
  Feather M4 CAN          (its male headers plug down into the wing)
  ─ female headers soldered on the wing's top side ─
  EPS tap wing            (SMD on top, facing the Feather; ~8 mm of header gap is plenty)
  ─ back side: J2 taps, J3 clamp, bypass jumpers, labels ── wire from here
```

The terminals point out of the back of the stack, so the harness never has to pass between boards. Leave room under the stack for the J2 plug and its cable (about 15 mm).

Solder joints you make yourself: the two female header rows on the top (28 joints) and the two connectors on the back (19 joints). Every resistor, capacitor, diode, both op-amps and the ADC arrive already fitted. The status LED and its resistor (0805, on the back) are either 2 more hand-soldered parts or bottom-side assembly.

## Connectors

**J2, EPS taps** (JST XA B15B-XASK-1, 2.5 mm pitch, locking; mating housing XAP-15V-1 with SXA-001T-P0.6 crimp contacts, or buy pre-crimped XA leads). Read the back silkscreen with the board face down: the labels sit under each pin, and pin 1 (VT3) is at the right-hand end.

| J2 pin | Silk | Car pin | On the wing |
|--------|------|---------|-------------|
| 1 | VT3 | B10 | 1 MΩ / 1 MΩ → buffer → A2 |
| 2 | VT6 | B8 | 1 MΩ / 1 MΩ → buffer → A3 |
| 3 | VCC2 | B2 | 1 MΩ / 1 MΩ → buffer → A0 |
| 4 | GND | B9 (T/S GND) | 100 Ω → logger GND, the only ground reference |
| 5 | SHLD | cable shield | logger GND; don't connect the shield at the car end |
| 6 | +B | A2 | 1 MΩ / 100 kΩ → buffer → A1 |
| 7 | IG1 | B6 | 100 kΩ / 33 kΩ → D12 |
| 8 | RT+ | B11 | 100 kΩ / 33 kΩ → D11 (optional: leave it empty and it reads "off") |
| 9 | WLP | B13 | 100 kΩ / 33 kΩ → D9 |
| 10 | NEP | B14 | 100 kΩ / 33 kΩ → D6 |
| 11 | VSP | B4 | 100 kΩ / 100 kΩ → D5 |
| 12–15 | X1–X4 | spare | 1 MΩ / 100 kΩ → U2 buffer → ADS1115 AIN0–AIN3 |

The torque-sensor lines sit at one end, behind GND and SHLD, so a stray strand can't put +B onto them. +B's other neighbour is IG1, which is also battery level. The digital lines follow the Feather's pin order, so their tracks don't cross. Crimp every contact properly and check each wire with a tug.

**J3, current clamp** (Phoenix MPT 0,5/4-2,54 fixed screw terminal, 2.54 mm pitch). A 3.5 mm plug-in 4-way terminal would sit over the Feather-spec corner mounting holes. The clamp has its own flying leads anyway. Pin 1 is the one nearest the 16-pin header edge, marked with a triangle.

| J3 pin | HSTS016L wire |
|--------|---------------|
| 1 | red: +5 V (from the Feather USB pin, i.e. the power bank) |
| 2 | black: 0 V, plus the sensor cable's shield |
| 3 | yellow: OUT → 10 k / 47 k / 100 nF → A4 |
| 4 | white: Vref → 10 k / 47 k / 100 nF → A5 (current = Vout − Vref, which cancels the sensor's reference drift) |

## Probe resistors: on the board or at the probe

Each car-facing series resistor has a **solder jumper** across it. The jumpers are the row of pads under J2 on the back, one under each terminal, and they ship **open**.

| Where you want the resistors | What to do |
|------------------------------|------------|
| **On the board** (default) | Nothing. Leave the jumpers open. Protect the cable as described below |
| **At the probe ends** (whole cable protected, as in [PROTO_BUILD.md](../../PROTO_BUILD.md)) | Bridge each jumper with a blob of solder. Then fit 1 MΩ (VT3, VT6, VCC2, +B, X1–X4), 100 kΩ (VSP, NEP, WLP, RT+, IG1) and 100 Ω (GND) inline at the probes |

| Jumper | Bypasses | J2 terminal |
|--------|----------|-------------|
| JP1 | R3 (1 MΩ) | VT3 |
| JP2 | R5 (1 MΩ) | VT6 |
| JP3 | R1 (1 MΩ) | VCC2 |
| JP4 | R11 (100 Ω) | GND |
| JP5 | R7 (1 MΩ) | +B |
| JP6–JP10 | R20, R18, R16, R14, R12 (100 kΩ) | IG1, RT+, WLP, NEP, VSP |
| JP11–JP14 | R24, R26, R28, R30 (1 MΩ) | X1–X4 |

Never bridge a jumper without fitting its probe resistor: that connects the car straight to the dividers.

The pin map is the one `firmware/eps_logger/config.h` already uses, so the firmware needs no changes for the original signals.

## Spare inputs X1–X4

| Signal type | Change on the board |
|-------------|---------------------|
| Up to 36 V (battery-level, motor terminals) | None: 1 MΩ / 100 kΩ gives 0–3.27 V at the ADC |
| 0–5 V sensor | Swap the 100 kΩ (R25, R27, R29, R31) for 1 MΩ: 5 V → 2.50 V, same as VT3 |

The ADS1115 reads all four channels at about 200 samples per second each, with a programmable gain (±0.256 V to ±4.096 V). That suits slow-to-medium signals; anything that must line up sample-for-sample with torque belongs on the Feather's own ADC. The firmware doesn't read U3 yet.

## Safety: the one trade-off

With the probe resistors on the wing, the cable between the car and the wing is no longer behind them. A cable fault, such as a crushed jacket or a strand bridging two terminals, now reaches the EPS directly instead of through ≥100 kΩ. To make up for that:

- Use one sheathed multicore cable (12-core shielded, as in BOM.md §5). Strap it down so it can't chafe, and keep it away from the pedal box and seat rails.
- Crimp ferrules on every wire and heat-shrink each back-probe pin.
- Fit the plug and probes with the **ignition OFF** (this is printed on the board). Continuity-check neighbouring probes before key-on, as in BOM.md's worst-case table.
- Run the firmware's `c` lead check with the engine off, then running.

Faults on the logger side of the terminals are covered just as in BOM.md: everything after J2 is ≥100 kΩ (1 MΩ on the torque lines) from the car.

## Changes from the BOM.md front end

| BOM.md part | Wing part | Why |
|-------------|-----------|-----|
| MCP6024-I/P (DIP-14) | MCP6004-I/ST (TSSOP-14) | Same pinout and rail-to-rail, 1 pA inputs; 1 MHz instead of 10 MHz (plenty) for about a tenth of the price |
| BAT85 (DO-35) | 1N4148WS (SOD-323) | JLC Basic part, same 3V3 clamp role; 25 nA leakage can't disturb the dividers |
| 2 × 6-way screw terminal | 15-way locking JST XA + 4-way screw terminal | Unplug the car harness in one go; 4 spare positions |
| — | X1–X4 inputs, second quad op-amp, ADS1115 | Room for future signals |
| — | 10 k / 47 k / 100 nF on the clamp's Vref → A5 | Cancels the current sensor's reference drift (needs a firmware update to use, see below) |
| — | 14 bypass solder jumpers | Choose between probe resistors on the board or at the probes |

All resistors are 0402 1 % parts and all capacitors are 0402 X7R (JLC Basic).

## Rebuilding the design

All scripts run with KiCad 10's bundled Python, from `gen/`:

```bash
"/c/Program Files/KiCad/10.0/bin/python.exe" make_sch.py
"/c/Program Files/KiCad/10.0/bin/kicad-cli.exe" sch export netlist -o ../eps_tap_wing.net ../eps_tap_wing.kicad_sch
sh route.sh
```

`route.sh` runs four steps:
1. `make_pcb.py place` places the parts and exports a Specctra DSN.
2. Freerouting routes the board.
3. `make_pcb.py finish` imports the routes and pours GND on both layers.
4. DRC with schematic parity runs, then `export_fab.py` writes the Gerbers, drill files, BOM and CPL.

Change the circuit or the placement in `design.py` and re-run. Hand-editing in KiCad is also fine, but then stop running the generators, because they would overwrite your edits.

## What's left

1. **Gerbers are ready** in `fab/eps_tap_wing_gerbers.zip`, with the assembly BOM and CPL next to them. **Nothing has been ordered or published.**
2. Before ordering:
   - Check the JLC CPL rotations in their viewer. Bottom-side parts are where offsets usually need a tweak.
   - Match each MPN to an in-stock LCSC part.
3. **Firmware:** `eps_logger` doesn't read A5 or the ADS1115 yet.
   - To use the Vref channel, compute current as A4 − A5 instead of A4 minus the key-on zero.
   - Reading X1–X4 means polling U3 over I2C (0x48) a few hundred times a second.
   - Both need new fields in the log record.

Routing used Freerouting 2.2.0 on a downloaded Temurin JRE 25 (not installed system-wide). To re-route: `JAVA=<path to java 25> sh gen/route.sh`. Freerouting results vary from run to run, so re-run it if a connection is left over.
