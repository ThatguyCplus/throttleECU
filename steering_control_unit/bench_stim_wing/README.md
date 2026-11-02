# Bench stimulus FeatherWing

A FeatherWing for the EPS logger stack that does four jobs:

1. **Powers the whole stack from 12 V** (bench supply or the car battery), so the logger needs no USB power bank.
2. **Powers the Toyota steering-angle sensor** (12 V, ~30 mA, switched and fused) and passes its CAN on to the Feather M4 CAN.
3. **Drives the EPS signals** NEP (rpm) and VSP (speed) and, with the wheels off the ground, VT3/VT6 (fake driver torque).
4. **Switches IG1** for an EPS that's out of the car, on a bench.

It sits **between the Adalogger and the OLED FeatherWing**, on stacking headers, and every cable connector is right-angle and exits a board edge. The brief is [HANDOFF.md](HANDOFF.md).

> **Status: routed and checked in KiCad 10.0.8, nothing ordered.** The board is fully routed, and the Gerbers, BOM and CPL are in [fab/](fab/). Read [Before ordering](#before-ordering) first: several part numbers, the relay pinout and the JLC rotations still need a human check.
>
> | KiCad 10.0.8 check | Result |
> |---|---|
> | ERC | **0 errors**. 20 warnings: symbols drawn from KiCad 7's library differ from KiCad 10's copies, and KiCad 10 renamed `G6K-2` |
> | DRC | **0 errors, 0 unconnected, 0 schematic-parity issues**. Warnings: footprints differ from KiCad 10's library copies, and some silk is small or under copper |

| Top (3D) | Back (3D) |
|---|---|
| ![top render](doc/render_top.png) | ![back render](doc/render_bottom.png) |

The 3D renders show bare pads because the KiCad image used has no 3D models.

| Top layers | Back layers (seen from below) |
|---|---|
| ![top](doc/top.png) | ![back](doc/back.png) |

## Safety model

The user asked for this to be overbuilt. **Faking VT3/VT6 makes the EPS motor steer by itself**: front wheels off the ground (or the rack on a bench rig), hands clear of the wheel.

| Layer | What it does |
|---|---|
| **Relays, fail to the car** | Every EPS line (VT3, VT6, NEP, VSP) goes through an Omron G6K relay. The **NC** contact connects the EPS to the car's own wire, and the **NO** contact connects it to this board. No power, a reset, a crash or a blown fuse all leave the EPS wired exactly as the car wires it. |
| **ARM loop (hard kill)** | Both relay coils and the IG1 output are powered only through the **ARM loop on J5 pins 7–8**. Wire a normally-closed e-stop or a key switch there. **Loop open = car wiring, whatever the firmware does.** The red ARM LED shows the loop is closed. |
| **Enable bits** | Each output also needs its own bit from the TCA9534 I2C expander. The expander powers up with every pin as an input, and 100 kΩ pull-downs hold every driver off until firmware sets the bit. |
| **Clamped torque range** | The DAC uses its 2.048 V internal reference and the op-amp gain is 2.2, so the torque outputs top out at **4.5 V** even at full scale. They're also clamped to 0–5 V, with a 470 Ω series resistor each. |
| **Inject on the EPS side only** | NEP and VSP also feed the gauges, ECM, MCM, TCM and climate control. Only ever break them at the EPS with a breakout harness: the relay then isolates the EPS from the car wire, and the car side never sees the stimulus. |

**Latching DTCs.** The torque-sensor and sensor-supply codes (11/15/16/17/18/67) latch until erased with the SCS procedure, so a bad VT3/VT6 fake bricks assist until you clear them. The relay swaps only VT3/VT6: the real sensor stays connected to Vcc1/Vcc2 and stays powered, so the supply checks still see a real load.

**Plausibility rules** (from the manual, also in [EPS_PINOUT.md](../data_capture/EPS_PINOUT.md)):
- Never send speed without rpm. At ≥ 6.2 mph with ≤ 280 rpm for 3 s, the lamp comes on with DTC 22.
- Ramp speed changes, because VSS jumps also set DTC 22.
- Don't hold ≥ 2,000 rpm at ≤ 1 mph for more than 6 minutes.

**Key ON, engine off: the 12 V battery isn't being charged.** The DC-DC converter doesn't charge it in that state (LiBCM FAQ), so all assist current comes from the battery. Keep bench sessions short, or put a charger on the battery.

## Stack and mechanics

```
  OLED FeatherWing
  ─ female side of this wing's stacking headers ─
  BENCH STIM WING        top: relays, inductor, J2/J3/J4, LEDs   (tallest part: relays, 5.2 mm)
                         back: logic, test pads, J5              (back parts <= 1.25 mm, J5 4.25 mm)
  ─ long male tails into the Adalogger's female headers ─
  Adalogger FeatherWing  (CR1220 holder and microSD socket on its top: ~2-4 mm)
  Feather M4 CAN         (USB-C at the left end, CAN terminal block at the right end)
  EPS tap wing           (bottom wing, connectors facing down)
```

Why this slot:
- **Feather ↔ Adalogger was ruled out.** The Feather M4 CAN's 3.5 mm CAN terminal block (right end) and USB-C (left end) sit right where this board's back and its edge connectors would be.
- **The Adalogger's top is low.** The tallest parts there are the CR1220 holder (x ≈ 3–17 mm) and the microSD socket (x ≈ 27–40 mm). That's why every back-side part on this wing is ≤ 1.25 mm tall, and J5 sits at the right end, where the Adalogger is bare.
- **The OLED's underside is bare,** so the relays (5.2 mm), the inductor (3 mm) and the 4.25 mm GH connectors fit under it, inside the ~8.5 mm stacking-header gap.

Use **Feather stacking headers** (female on top, long male tails) on this wing. **Check the gap with the real parts before soldering the headers.**

### Feather specification check

| Spec item | This wing |
|---|---|
| 0.9" × 2.0", 0.1" holes at each corner | 22.86 × 50.8 mm, four 2.5 mm holes 2.54 mm in from the edges |
| 16-pin and 12-pin header positions | Same footprint as the EPS tap wing; `make_pcb.py` asserts the pad positions every build |
| Power: take 3.3 V from the Feather | The expander and DAC run on the Feather's 3V3 (about 1 mA) |
| Powering the Feather from a wing | 5 V goes into the **USB pin through a Schottky** (D6). On the Feather M4 CAN the USB pin is VBUS straight off the USB-C connector, and the Feather's own MBR540 then feeds its regulator, so this is the normal path (checked against Adafruit's schematic) |
| Avoid AREF, RX/TX, SPI and its CS | AREF, RX and SPI are untouched. **TX is used** (VSP output): only D4 and RX/TX are left free in this stack, and NEP and VSP each need their own timer |
| I2C pull-ups on Wings | Not fitted: the Adalogger already has them |

### USB and 12 V at the same time

- **Our side:** the buck makes 5.05 V. After D6's drop the Feather's USB pin sees about **4.7 V**.
- **With a PC plugged in:** the PC's 5 V is higher, so the PC carries the stack and D6 stays off. This board never back-feeds the PC.
- **With 12 V gone:** D6 also stops the USB cable from powering this board's 5 V rail. The relays and op-amp go dead, and so the EPS is on the car wiring.

**A LiPo on the Feather.** If one is plugged into the Feather's JST, its charger draws up to ~200 mA from this 5 V rail. The buck is rated 0.6 A, so that still fits, but leave the LiPo off unless you want it charging.

**The current clamp's supply.** The HSTS016L on the tap wing takes its 5 V from the USB pin, so on 12 V alone it gets ~4.7 V. Its output is read as Vout − Vref, which cancels most of that offset, but **check its zero at key-on** the first time.

## Connectors

All connectors are right-angle, exit a board edge and are hand-soldered. The GH parts are SMD with locking latches: use JST GH housings (GHR-0xV-S) with SSHL-002T-P0.2 contacts, or buy pre-crimped GH leads.

**J2, 12 V IN** (top edge, JST PH S2B-PH-K, 2 mm). Pin 1 (marked +) is +12 V and pin 2 is GND. 2 A fuse.

**J3, SAS / CAN** (left end, JST GH 6-way). This is a pass-through: build one cable to the SAS, plus one pair from pins 5/6 to the Feather's CAN terminal.

| Pin | Net | Goes to |
|---|---|---|
| 1 | SAS 12V | SAS power (switched, 0.1 A PTC) |
| 2 | GND | SAS ground |
| 3 | CANH | SAS CAN-H |
| 4 | CANL | SAS CAN-L |
| 5 | CANH | Feather M4 CAN terminal **pin 3 (H)** |
| 6 | CANL | Feather M4 CAN terminal **pin 1 (L)** |

The Feather's terminal has a 120 Ω terminator behind a solder jumper (SJ2). The SAS bus here is two nodes, so close SJ2 unless the SAS end already terminates the bus.

**J4, TORQUE** (right end, top, JST GH 5-way). Break the torque sensor's VT3 and VT6 wires in a breakout harness: the CAR side goes to the sensor, the EPS side to the EPS.

| Pin | Net |
|---|---|
| 1 | VT3 EPS side (gearbox 7-pin 3 / ECU B10) |
| 2 | VT3 sensor side |
| 3 | GND |
| 4 | VT6 EPS side (gearbox 7-pin 1 / ECU B8) |
| 5 | VT6 sensor side |

**J5, ECM / IG1 / ARM** (right end, back, JST GH 8-way). The back silk marks "J5" with pins "1" and "8"; the pin names are below.

| Pin | Net | Notes |
|---|---|---|
| 1 | IG1 OUT | Switched 12 V, 0.5 A PTC. Bench EPS only (ECU B6) |
| 2 | NEP CAR | Car-side NEP wire (ECM) |
| 3 | NEP EPS | EPS B14 |
| 4 | GND | Separates the 12 V-level pins from the 5 V-level pins |
| 5 | VSP EPS | EPS B4 |
| 6 | VSP CAR | Car-side VSP wire (VSS) |
| 7 | ARM+ | 5 V out to the ARM loop (0.1 A PTC) |
| 8 | ARM | Back from the loop: powers the relay coils, enables IG1 |

**Loopback for free.** The tap wing's J2 reads NEP (D6), VSP (D5), VT3 (A2) and VT6 (A3). Tap the **EPS side** of the breakout harness and the logger records exactly what the EPS receives, including whether the relay switched.

## Circuit summary

| Block | Parts | Notes |
|---|---|---|
| 12 V input | F1 2 A fast fuse → D1 SS34 (reverse polarity) → D2 SMBJ24CA TVS → C1 10 µF/50 V | Everything on +12 V is rated ≥ 40 V; the TVS clamps a load dump to ~39 V |
| 5 V | U1 LMR16006 (40 V, 65 V transient, 0.6 A, 0.7 MHz), L1 22 µH, D4 catch diode, 56k/10k → 5.05 V | ~250 mA load: stack ~150 mA, relays 2 × 28 mA, LEDs |
| Feather feed | D6 PMEG6010 → Feather USB pin | See [USB and 12 V at the same time](#usb-and-12-v-at-the-same-time) |
| SAS power | Q1 Si2319 high side, Q2 2N7002, F2 0.1 A PTC | Expander P0. **JP1** (open as shipped): bridge for "SAS always on", for use with the unmodified logger firmware |
| IG1 out | Q3 Si2319, Q4 + Q5 in series (IG1_EN **and** ARM), F4 0.5 A PTC | Expander P1 |
| Relays | K1 (VT3/VT6), K2 (NEP/VSP): G6K-2F-Y 5 V, Q6/Q7 drivers, 1N4148WS flyback | Expander P2, P3. Coils fed from ARM |
| NEP | D4 (PA14) → Q8 → open drain, **4.7 kΩ to 12 V**, 100 Ω series | Like the ECM's open-collector output |
| VSP | TX (PB16) → Q9 → open drain, **4.7 kΩ to 5 V**, 100 Ω series | Like the VSS |
| Torque | U2 MCP4728 (I2C 0x60, 2.048 V ref, gain ×1) → U3 MCP6004 on 5 V, gain 2.2 → 470 Ω → BAT54S clamp | DAC A = VT3, B = VT6; C and D are spare outputs on test pads |
| Expander | U4 TCA9534 (I2C **0x20**) | P0 SAS, P1 IG1, P2 K1, P3 K2 (outputs); P4 ARM sense, P5 12 V sense (inputs); P6/P7 on test pads |

**Pulse output polarity.** GPIO high pulls the output low (open drain), and the idle level is high. Both D4 and TX sit on independent timers: PA14 on TC3/WO0 or TCC1/WO2, PB16 on TC6/WO0 or TCC0/WO4. The rates are low (NEP 1.5 pulses/rev: 22.5 Hz at 900 rpm; VSP ~1.24 Hz per mph, **unverified**), so plain `micros()` timing also works.

**I2C addresses** on the shared bus: RTC 0x68, OLED 0x3C, ADS1115 0x48, **MCP4728 0x60, TCA9534 0x20**. There are no clashes.

### LEDs

Top side, in one row above the relays. Left to right: **5V, SAS, IG1, ARM, TQ, SIG, NEP, VSP**; the legend is also on the back silk. The 12 V LED is in the top-left corner. TQ and SIG light when their relay is energised. NEP and VSP blink with the pulse outputs.

### Test pads (back)

The pads are numbered 1–15 on the back silk; names don't fit at JLC's 1 mm minimum text height.

| TP | Net | TP | Net | TP | Net |
|---|---|---|---|---|---|
| 1 | GND | 6 | NEP out | 11 | AUX D (DAC D) |
| 2 | +12 V | 7 | VSP out | 12 | ARM |
| 3 | +5 V | 8 | VT3 out | 13 | Expander P6 |
| 4 | +3V3 | 9 | VT6 out | 14 | Expander P7 |
| 5 | USB pin | 10 | AUX C (DAC C) | 15 | GND |

## Design rules

| Item | Value |
|---|---|
| Layers | 4: **F.Cu** parts + signals + GND pour, **In1.Cu** solid GND, **In2.Cu** signals + GND pour, **B.Cu** parts + signals + GND pour |
| Signal tracks | 0.15 mm (mA and below) |
| Power tracks | +12 V path and IG1 0.5 mm; +5 V, USB, buck SW, SAS and ARM 0.4 mm; 3V3 0.3 mm |
| Clearance | 0.127 mm; **0.21 mm from tracks to plated through-hole pads** (JLCPCB's 0.2 mm pad-to-trace rule) |
| GND plane | In1 carries no signal tracks (the router treats it as a power plane) |
| Silkscreen | All text >= 1.0 mm high with a 0.15 mm stroke (JLCPCB minimum) |
| JLCPCB rules | [bench_stim_wing.kicad_dru](bench_stim_wing.kicad_dru), the 4-layer 1 oz set from [Cimos/kicad-druid](https://github.com/Cimos/kicad-druid) (MIT). KiCad loads it as the project's custom rules |
| Vias | 0.45/0.2 mm signal, 0.6/0.3 mm power |
| Assembly | **Both sides** (SMD top and back). The back carries only parts ≤ 1.25 mm tall |

## Files

| File | What it is |
|---|---|
| [bench_stim_wing.kicad_sch](bench_stim_wing.kicad_sch) / [schematic PDF](doc/bench_stim_wing_schematic.pdf) | Schematic (generated) |
| [bench_stim_wing.kicad_pcb](bench_stim_wing.kicad_pcb) | Board, 50.8 × 22.86 mm (FeatherWing), 4 layers |
| [doc/](doc/) | Layer renders of the top and back (KiCad 7's CLI has no 3D render) |
| [fab/](fab/) | Gerbers + drill (zipped), JLC BOM and CPL. SMD parts only; connectors, headers, JP1 and test pads are excluded |
| [gen/](gen/) | Generators. Everything comes from [gen/design.py](gen/design.py) (parts and nets) and [gen/placement.py](gen/placement.py) (positions, silk, hand stubs). [gen/bench_stim_wing.ses](gen/bench_stim_wing.ses) is the accepted Freerouting result |

## Rebuilding

The board was generated in a Linux cloud session. The generators ran on **KiCad 7.0.11**, the version Ubuntu ships. Sign-off ran on **KiCad 10.0.8**, which [gen/setup_kicad10.sh](gen/setup_kicad10.sh) installs from the official `kicad/kicad` Docker image. That image comes through Google's Docker Hub mirror, because KiCad's Launchpad PPA is blocked there and Docker Hub rate-limits anonymous pulls. The script runs KiCad 10 through `chroot`, which needs root. Routing used **Freerouting 1.9.0** on Java 21, under `xvfb-run`. The files are written in KiCad 7's format. KiCad 10 opens them and asks to upgrade on save. Upgrading is fine, but then the generators would overwrite your edits, so pick one way of working.

```bash
cd gen
sh route.sh --keep-routing   # rebuild everything from the committed routing (bench_stim_wing.ses)
sh route.sh                  # full re-route with Freerouting (a different result every run)
```

`route.sh` expects `kicad-cli` and a Python that can `import pcbnew` (`PY=...`). A full re-route also needs the Freerouting 1.9 jar (`JAR=...`). On Windows with KiCad 10, point `PY` at KiCad's bundled `python.exe`, and set `KICAD_SYMBOL_DIR` / `KICAD_FOOTPRINT_DIR` if the libraries aren't found.

**How it got to 0 unconnected:**
- **Freerouting version.** 2.0.1 ignored its pass limit and never finished, so routing uses 1.9.0 with `-oit 100`. Without that flag the post-route optimiser runs for hours.
- **Finer signal rules.** Freerouting left 11 connections open at 0.2/0.15 mm, so signal tracks went down to 0.15 mm with 0.127 mm clearance.
- **Hand fixes for the last two connections** ([placement.py](gen/placement.py) `MANUAL_ROUTES`). KiCad 7 exports some back-side pads to Freerouting on the wrong layer, which left these two open. [find_via.py](gen/find_via.py) searches for clear via spots:
  - C5 gets a short stub and a via up to the BUCK_SW track.
  - A BUCK_EN segment is re-run at y = 15.4 mm.
  - R25's GND pad gets a via to the In1 plane. **That via is in the pad.** It's fine for a 100 k pull-down, but ask JLC to plug/tent vias, or nudge it in KiCad 10.
- **Net classes live in [bench_stim_wing.kicad_pro](bench_stim_wing.kicad_pro),** so KiCad 10 opens the board with the same rules.
- **One signal on In1.** Freerouting ran one track (IG1_G, DC) on In1. That leaves a small slot in the GND plane there, which is harmless for these low-speed signals.
- **What DRC still reports.** The remaining entries are cosmetic: silk labels smaller than 0.8 mm or partly under copper, and J2's outline reaching the board edge, which is deliberate since it's an edge connector. The rest are library-table warnings from the build machine.

`check_netlist.py` checks every pin in the exported netlist against `design.py`. When `kicad10-cli` is installed, `route.sh` also runs KiCad 10's ERC, its DRC with schematic parity, and the 3D renders.

## Before ordering

1. **ERC and DRC already pass in KiCad 10.0.8** (see Status). Open the board in your own KiCad 10 once anyway. If you save it there, KiCad upgrades the file and the generators would overwrite your edits.
2. **Check the relay pinout** against Omron's G6K datasheet: coil 1/8, COM 3/6, NC 2/7, NO 4/5. That pinout came from KiCad's G6K-2 symbol. If NC and NO are swapped, the board fails the wrong way.
3. **Check stock on the LCSC numbers.** Every part in the BOM now has an LCSC number, each matched to its listing in November 2026 (see [JLCPCB parts](#jlcpcb-parts)). Stock moves, so re-check at upload. Use the listed alternatives if a part has run out.
4. **Check every CPL rotation** in JLC's preview, on both sides. Watch especially the diodes, LEDs, SOT-23s and the relay, since a polarity error there matters.
5. **Check the stack gap** with the real stacking headers, relays and the OLED.
6. **Confirm LMR16006's FB voltage (0.765 V)** against the datasheet. The output is 5.05 V with 56k/10k.

## JLCPCB parts

All 35 SMD line items have LCSC numbers in [fab/bench_stim_wing_bom_jlc.csv](fab/bench_stim_wing_bom_jlc.csv).

| Part | LCSC | Notes |
|---|---|---|
| LMR16006YQDDCRQ1 buck | C544521 | Extended. TI rates it **40 V, 65 V transient**. The TVS clamps at ~39 V |
| MCP4728-E/UN DAC | C108207 | Extended |
| MCP6004-I/ST op-amp | C185811 | Extended. **Low stock** when checked; alternative is UMW MCP6004T-I/ST, C32713110 (same TSSOP-14 pinout) |
| TCA9534PWR expander | C783615 | Extended |
| Si2319CDS-T1-GE3 P-FET | C146287 | Vishay. Avoid the "-JSM" copy, C18191323, which has a different rating |
| G6K-2F-Y DC5 relay | C231389 | In stock. C1560486 is the same MPN but listed unavailable |
| SWPA4030S220MT 22 µH | C83472 | Isat 1.3 A |
| SMBJ24CA TVS | C1973089 | Bourns. Every SMBJ24CA listing found is Extended |
| PMEG6010CEH Schottky | C110797 | Stock may be 0; alternative is PMEG6010CEH-QX, C5311567 |
| SS34 Schottky | C8678 | Basic, but marked "not recommended for new designs"; alternative is FOSAN C5119706 |
| BAT54S | C171786 | |
| 2 A fuse, 1206 | C3105 | Littelfuse 0466002.NRHF, the halogen-free version of 0466002.NR |
| 0.1 A PTC, 60 V | C2153714 | Littelfuse 1206L010/60WR |
| 0.5 A PTC, 30 V | C7542947 | LUTE 1206L050/30NR. No LCSC listing found for the Littelfuse 1206L050/30WR |
| 10 µF 50 V 1206 | C13585 | |
| 4.7 µF 50 V 1206 | C51205 | |
| 10 µF 25 V 0805 | C15850 | Basic |
| 100 nF 50 V 0603 | C14663 | Basic. Showed out of stock on one LCSC page |
| 0402 resistors (100R, 470R, 1k, 10k, 12k, 22k, 33k, 100k) | C25076, C25117, C11702, C25744, C25752, C25768, C25779, C25741 | UNI-ROYAL, Basic |
| 56k 0402 | C25796 | UNI-ROYAL. **Extended**, not Basic |
| 4.7k 0603 | C23162 | Basic |
| 2N7002 | C8545 | Basic |
| 1N4148WS | C2128 | Basic |
| LEDs 0603 (red, yellow, blue, green) | C2286, C72038, C72041, C72043 | Basic per a community library listing; not confirmed on JLC's own pages |

**About 13 Extended part types,** at about $3 each in setup fees. Swapping R3 56k for a Basic value would mean recomputing the 5 V divider. Not worth it.

## Firmware (next)

Nothing is written yet. A bench sketch (or a bench mode in `eps_logger`) needs:
- Serial commands for rpm and mph. The OLED buttons share D5/D6/D9 with the tap wing inputs, so they can't be used.
- Ramp limits and the plausibility guards: rpm ≥ ~800 whenever speed > 0, and a time limit on high rpm at zero speed.
- The expander set up as P0–P3 outputs (low), then P4/P5 read as inputs.
- **Refuse to energise K1 unless ARM reads closed**, and drop K1 on any serial timeout.
- Logging of what was commanded, next to what the tap wing measured.

`eps_logger/config.h` says D4 is the CAN boost enable. Adafruit's Feather M4 CAN schematic shows boost enable and CAN standby are internal pins PB13 and PB12, and D4 (PA14) only goes to the header. Fix that comment when the firmware is touched.
