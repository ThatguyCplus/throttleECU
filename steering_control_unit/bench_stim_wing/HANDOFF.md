# Bench stimulus wing: handoff

A brief for a fresh (cloud) session. Nothing in this folder exists yet except this file.

## The ask, in the user's words

> make a dev board that can drive the needed signals while it takes a 12v input that can power on the collector along with powering on the sensor angle sensor (it needs 12v to work and draws 0.03 amps) make it work with the adc capture thing we made, house everything in the proto wing board footprint. Keep in mind that the adc board we made has the connector facing down so this one will probably need to be on side connectors.

In plain terms, the board must:
1. **Take a 12 V input** (bench supply or the car battery), protected for automotive use.
2. **Power the data collector** (the logger stack: Feather M4 CAN + Adalogger + OLED + EPS tap wing) from that 12 V, so there's no USB power bank.
3. **Power the Toyota steering angle sensor (SAS)**: 12 V at about 30 mA, switched and fused.
4. **Drive the EPS bench signals:** NEP and VSP to turn the EPS on and set its speed, and optionally VT3/VT6 (fake driver torque).
5. **Stack with the existing EPS tap wing** in the same Feather stack, on the standard **FeatherWing footprint (0.9" × 2.0", 22.86 × 50.8 mm)**.
6. **Use side-exit (right-angle) connectors**, because the board sits mid-stack (see "Stack position").

Folder rule from the user: keep this work in **`steering_control_unit/bench_stim_wing/`**, apart from the other boards. Don't edit `data_capture/hardware/eps_tap_wing/`. Never edit the throttle ECU folder (read-only).

Standing constraints: **don't order anything**, and ask before publishing, pushing, or downloading tools or libraries.

---

## Read these first

| File | What's in it |
|---|---|
| `steering_control_unit/data_capture/EPS_PINOUT.md` | EPS connector pinout, Honda voltages, DTC list, plausibility rules, and the **Pulse scaling** section (NEP/VSP rates) |
| `steering_control_unit/data_capture/hardware/eps_tap_wing/README.md` | The tap wing: Feather pins it uses, stack order, J2/J3 pinout, design rules, JLC workflow |
| `steering_control_unit/data_capture/hardware/eps_tap_wing/gen/` | The generator flow to copy: `design.py` → `make_sch.py` → netlist → `make_pcb.py place` → Freerouting → `make_pcb.py finish` → DRC → `export_fab.py` (JLC BOM/CPL) |
| `steering_control_unit/data_capture/firmware/eps_logger/config.h` | Logger pin map, `CFG_NEP_PULSES_PER_REV`, `CFG_VSP_PULSES_PER_MILE` |
| `steering_control_unit/toyota_sas/feather_sas_oled/` | SAS reader (CAN 500k, ID 0x25) |
| Honda manual: https://github.com/doppelhub/Insight-Service-Manual | `Green Service Manual/electrical_power_steering.pdf` Fig. 8–11 |

> **Repo state when this was written:** `steering_control_unit/data_capture/` was **untracked** (never committed), on local branch `feature/c2000-f280049c-port`, which was 2 commits ahead of origin. If the files above are missing in the cloud checkout, ask the user to commit and push them, and to point the session at that branch.

---

## Facts already settled (don't re-derive)

### EPS signals to drive
| Signal | EPS pin | Real source and level | Rate |
|---|---|---|---|
| **NEP** (engine speed) | B14 BLU | ECM open-collector, pull-up to **~12 V**; EPS input has a pull-down | **1.5 pulses per engine rev**: 900 rpm = 22.5 Hz, 2,000 = 50 Hz, 6,000 = 150 Hz (MuddersMIMA `engine_signals.h`) |
| **VSP** (vehicle speed) | B4 BLU/WHT | VSS open-collector, **0–5 V**; EPS input has a pull-down | **~4,450 pulses/mile, about 1.24 Hz per mph. UNVERIFIED** (5 pulses per tyre rev × 165/65R14) |
| **VT3, VT6** (torque) | B10, B8 (gearbox 7-pin: 3, 1) | Inductive sensor IC, about **1.5–3.5 V** when steering, ~2.5 V centre | The two channels probably move in opposite directions around 2.5 V. **Not confirmed**: log a real capture first |
| **IG1** | B6 YEL | Ignition, battery voltage | Needed only on a bench EPS that's out of the car. Low current |

- **NEP alone should make the EPS close its motor relay** with the engine off. RT+ (B11) goes high and the lamp (WLP B13) goes out. Confirm this on the first run.
- **Plausibility rules** (from the manual):
  - At ≥ 6.2 mph with ≤ 280 rpm for 3 s: lamp on, DTC 22. So never send speed without rpm.
  - At ≥ 2,000 rpm and ≤ 1 mph for more than 6 min: lamp on.
  - VSS jumps also set DTC 22, so ramp speed changes.
  - DTCs 22 and 23 clear by themselves once the signals look normal again.
  - **DTCs 11/15/16/17/18/67 latch** until erased with the SCS procedure. These are the torque-sensor and sensor-supply codes, so faking VT3/VT6 badly bricks assist until you clear them.
- **Shared lines.** VSP feeds the gauges, ECM, MCM, TCM and climate control. NEP feeds the tach, MCM and TCM. **Inject only on the EPS side of a breakout harness**, never onto the car's shared wires.
- With the key ON and the engine off, the **DC-DC converter does not charge the 12 V battery** (LiBCM FAQ). Assist current comes from the battery. That's not this board's problem, but put it in the README.

### Feather stack and pins
- **Stack today (top to bottom):** OLED FeatherWing / Adalogger FeatherWing / Feather M4 CAN / **EPS tap wing** (a "bottom wing": its connectors and labels are on its **back**, facing down).
- **Stack position for this board.** The bottom slot is taken. Below the tap wing would block its downward-facing J2/J3 and probe jumpers. So this board goes **mid-stack**: between the Feather and the Adalogger, or between the Adalogger and the OLED. Both faces are covered by other boards, so **every external connector must be right-angle and exit through a board edge.** Decide which slot, and check the header heights.
- **Feather pins already used (don't reuse):**
  - A0–A5: tap wing analog.
  - D5 (VSP in), D6 (NEP in), D9 (WLP in), D11 (RT+ in), D12 (IG1 in), D13 (status LED). D5, D6 and D9 are also the OLED buttons C, B and A.
  - D10: SD card chip select. SCK/MOSI/MISO: SD.
  - SDA/SCL: shared I2C bus.
- **I2C addresses in use:** RTC 0x68, OLED 0x3C, ADS1115 0x48. An **MCP4728 DAC defaults to 0x60** (free).
- **Free pins for the NEP/VSP outputs:** RX/TX (D0/D1) and possibly D4. Config.h says D4 is the CAN boost enable, but an earlier check found the Feather M4 CAN uses internal pins 40 (CAN standby) and 41 (boost enable). **Verify D4 against the Feather M4 CAN schematic before using it.** NEP/VSP need a timer output, so check the SAMD51 TC/TCC mux for whichever pin you pick.
- **Loopback.** The tap wing's NEP (D6) and VSP (D5) inputs will read whatever this board drives. That's a free self-check: wire the stim outputs into the same harness nodes that J2 taps.

### Power
- The HSTS016L current clamp on the tap wing takes **+5 V from the Feather's USB pin** (tap wing J3 pin 1, ≤12 mA). If this board makes 5 V and feeds the Feather's USB pin, the clamp gets powered too.
- **How to feed the Feather from a wing** (USB pin vs BAT pin, and the diode path against a plugged-in USB cable): check the Feather M4 CAN schematic. Don't back-feed the PC's USB port; use a Schottky or ideal-diode OR.
- **Budget, roughly:**
  - Feather M4 CAN, Adalogger and OLED: ~100–150 mA at 5 V.
  - Clamp: 12 mA.
  - CAN transceiver: the Feather has its own boost.
  - SAS: 30 mA at 12 V.
  - Stim op-amp and DAC: a few mA.
  - So a **≥ 500 mA 5 V buck** gives plenty of headroom.
- **Automotive input:**
  - Reverse-polarity protection (P-FET or ideal diode).
  - TVS (e.g. SMBJ24A class).
  - Fuse or polyfuse.
  - A buck rated for **≥ 36–40 V in** (load dump).
  - MuddersMIMA's CAR002 is a reminder that a plain LDO "cannot handle automotive 12V Vin".

---

## Proposed circuit (from the earlier discussion; refine it)

| Block | Circuit | Notes |
|---|---|---|
| 12 V in | Side connector → fuse → reverse-polarity FET → TVS → bulk cap | LED: "12V IN" |
| 5 V rail | Buck (≥ 40 V in, ≥ 500 mA) → Schottky/ideal-diode OR → Feather USB pin; also feeds the stim op-amp and the VSP pull-up | LED: "5V" |
| SAS power | 12 V → high-side switch or P-FET (GPIO-enabled, default OFF) → polyfuse ~100 mA → side connector | LED: "SAS PWR". The SAS's CAN H/L go to the Feather M4 CAN's own CAN terminal, or pass through a connector on this board to it |
| IG1 out (bench EPS only) | 12 V → high-side switch, fused ~1 A | Optional, default OFF. LED |
| NEP out | GPIO (timer) → 2N7002 → 4.7 kΩ pull-up to **12 V** | LED on the gate side (blinks with rpm) |
| VSP out | GPIO (timer) → 2N7002 → 4.7 kΩ pull-up to **5 V** | LED |
| VT3, VT6 out | MCP4728 (12-bit quad DAC, I2C 0x60) → **5 V rail-to-rail op-amp**, scaled to 0.5–4.5 V, with series R and a clamp at the output | The tap wing's MCP6004 runs on 3.3 V and **can't reach 3.5 V**. Pick a 5 V part (check JLC Basic/Extended) |
| Spare DAC ×2 | Same buffer | e.g. fake Vcc2 |
| CAR/BENCH + kill | A physical switch so outputs **fail back to the real sensor/harness**, plus a hard kill. Outputs default high-Z/off at reset | Safety requirement, not optional (see below) |

Of the parts, the MOSFETs, resistors, caps and polyfuse should all be JLC Basic. Expect Extended fees (about $3 each) for the MCP4728, the 5 V op-amp, the buck and maybe the high-side switch. The user wants JLC-friendly part picks (Basic where possible).

## Safety requirements
- **Faking VT3/VT6 makes the EPS motor steer by itself.**
  - Use it only with the front wheels off the ground, or with the rack/column on a bench rig.
  - Keep hands away from the wheel.
  - Never drive over the live sensor. Break the lines with a breakout harness; the CAR/BENCH switch returns them to the real sensor.
- **Unplugging the real torque sensor may trip the Vcc1/Vcc2 supply checks** (DTCs 17/18, which latch). Find out from the manual or a capture whether a dummy load is needed on Vcc1/Vcc2.
- **All outputs must power up OFF or high-Z**, and stay that way until firmware explicitly enables them.
- The user wants this **overbuilt for safety**: when unsure, add the protection.

## Layout preferences (the user's standing rules for every board)
- **Indicator LEDs:** power, status and comms LEDs on most chips, labelled on the silk. Keep them off safety-critical nets.
- **Datasheet layout:** decoupling and timing parts at the pins, as each datasheet asks.
- **Silkscreen:** as much as fits, on both sides, including a back-side pinout and legend.
- **Signal-flow clusters:** incoming, then outgoing, then probe points.
- **Test pads:** a test-pad row with its own GND pad.
- **Compact:** a compact board, 4-layer is fine. Match the tap wing's stack-up (In1 solid GND).
- **Feather spec:** follow it. The tap wing README's spec table and the corner-hole and header positions apply unchanged.

## Firmware (after the board)
- New sketch, e.g. `bench_stim_wing/firmware/`. Or a bench mode in `eps_logger`; ask the user.
- **Signal generation:** set rpm and mph. Build in ramp limits and the plausibility guards (rpm ≥ ~800 whenever speed > 0; a time limit on high rpm at zero speed).
- **Controls:** the OLED buttons are shared with the D5/D6/D9 inputs, so use serial commands instead.
- **Logging:** log what was commanded alongside what the tap wing measured.

## Tooling notes for a cloud session
- The `gen/` scripts hard-code Windows paths (KiCad 10 at `C:\Program Files\KiCad\10.0`). Freerouting 2.2.0 needs **Java 25** and is run with `-de dsn -do ses -mp 100 --gui.enabled=false`.
- On Linux you'll need KiCad 10 (`kicad-cli` and the `pcbnew` Python module) and a JRE 25. **Ask the user before downloading or installing them.**
- **Freerouting gotchas that were learned the hard way:**
  - Custom-shape pads on back-side footprints export on the wrong layer. Use plain pads.
  - Trust the log's "N unrouted", not the summary line.
  - Wide power traces won't enter 0.5 mm-pitch pads.
- **JLC exports:**
  - TSSOP packages need a −90° rotation offset in the CPL.
  - Hand-soldered parts (connectors, headers) get `in_bom False` in the schematic, and are excluded from the BOM and position files.

## Done looks like
- `bench_stim_wing/` has its own `gen/` scripts, schematic, board and README.
- ERC and DRC are clean, with schematic parity.
- JLC BOM/CPL and Gerbers are in `fab/`, with renders in `doc/`.
- Stack fit is confirmed against the tap wing: header heights, connector edges, and clearance to the Feather's CAN terminal and USB.
- Nothing is ordered or published.

## Open questions for the user
1. Which stack slot: Feather↔Adalogger, or Adalogger↔OLED?
2. Is IG1-out needed (a bench EPS out of the car), or is this only for the EPS in the car?
3. Should the SAS's CAN pass through this board, or go straight to the Feather's CAN terminal?
4. Separate bench firmware, or a mode inside `eps_logger`?
