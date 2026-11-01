# Icarus hardware: schematic rev 0.2

The KiCad 10 hierarchical schematic is **generated from code**. Edit `gen/design.py`, then rebuild:

```
cd gen
python make_sch.py
```

Then from `hardware/`:

```
kicad-cli sch erc --severity-all -o erc.rpt icarus.kicad_sch
kicad-cli sch export pdf -o doc/icarus_schematic.pdf icarus.kicad_sch
```

Don't hand-edit the `.kicad_sch` files; the next build overwrites them.

## Status (2026-11-01)

| Check | Result |
|---|---|
| ERC (`--severity-all`) | **0 violations** |
| Netlist vs `design.py` | 224 / 224 nets identical (KiCad netlist export, compared pin by pin) |
| Parts | 400 across 7 sheets |
| Safety analysis | SPFM 99.99 %, LFM 98.35 %, PMHF 0.064 FIT (placeholder data; `../docs/07_FMEDA_FTA.md`) |
| PCB | Not started |

## Sheets

| Sheet | Contents |
|---|---|
| `power` | Fuse, reverse-battery P-FET, TVS and hold-up. Then **two independent chains**, each LMR38020-Q1 5 V → TPS7B8133-Q1 3.3 V → TPS3702-Q1 5 V window |
| `ch_a` | Control F280049CPZQR, TPS3850-Q1 window watchdog, enable AND gate (GPIO · ERRORSTS · SUP_OK), TCAN3404-Q1, SiT2024B clock, JTAG, UART, LED |
| `ch_b` | Monitor F280049CPZQR (INTOSC), the same supervision, a PWM charge-pump dynamic enable, and the driver-indicator output |
| `overlay` | CH-A analog: 1 MΩ buffered taps, DAC RC slew limit, the Δ authority attenuator (k = 0.109 → ±0.18 V, TBD-DATA), VT6 inverter, summers, readback scaling |
| `guard` | CH-C: buffers, difference amps (D3, D6, sum), window ladder, TLV1704-Q1 comparators, 74LVC1G74 trip latch, self-test and reset FETs |
| `relays` | Two TPS1H100-Q1 high sides, cascaded G6K-2 relays K1/K2, Zener fast-release clamps, CH-B low-side FETs, CH-C guard FET, coil readback |
| `io` | 20-pin vehicle connector (placeholder), CAN ESD and split termination, a separate per-channel divider for every input, VBAT/5V/Vcc2/ground-offset monitors |

## Rev 0.2 changes (from the safety analysis, `../docs/06_SAFETY_ELEMENT.md`)

- **R1, J1 re-pinned.** The torque lines now sit in a GND-guarded block (pins 6–11). The old CAN_L-next-to-VT3_SNS single-point fault is gone.
- **R2, guard referenced to CH-B's expected Δ.** New group on the `guard` sheet: CH-B DACA → RC → k attenuator → REF3/REF6, with REF3 read back on ADC pin 6. VMID_B and the window ladder (28k / 3.01k / 4.02k / 3.01k / 28k) now run from +3V3_B.

## Decisions made while drawing

- **Watchdog changed to TPS3850-Q1.** The TPS3851-Q1 listed in the concept is timeout-only, so it can't catch a watchdog serviced too early. With the TPS3850 preset (CWD 10 kΩ to VDD, SET0 = SET1 = 1), the window is 1.85 ms closed and 11 ms timeout. Its SENSE pin also monitors 3.3 V for over- and undervoltage.
- **MCU pin mux** (SPRS945 Table 5-6), the same on both channels:

  | Function | Pins |
  |---|---|
  | CAN-A | GPIO30/31 |
  | FSI TX (to the other channel) | GPIO6/7 |
  | FSI RX | GPIO12/13 |
  | SCI-A | GPIO28/29 |
  | ERRORSTS | GPIO24, into the enable AND gate |
  | eCAP inputs (VSP / NEP) | GPIO16/17 |
  | DAC-A (CH-A only) | pin 23 |

- **Reverse-battery P-FET** follows the TI SLVA139 orientation: drain to battery, source to load. The throttle board's netlist has SQJ147 **source** on the fused battery side. Check that on the throttle design; I haven't changed it.

## Open items before layout

1. **TBD-DATA values** (from the logger): Δmax / k, slew τ, guard thresholds, hold-cap value.
2. **Relay.** G6K-2F-Y is a stand-in. Pick an automotive-qualified DPDT with gold contacts, and consider a different vendor for K1 and K2.
3. **Vehicle connector.** Choose the part and pinout. Then build the Y-harness with Honda connector-B halves.
4. **Confirm against datasheets:**
   - TPS3702 variant for 5 V;
   - F280049C VREG output capacitors and X1 single-ended clock wiring;
   - TPS7B8133 current budget;
   - TPS1H100 CL/CS values for the coil current.
5. **Charge-pump gate voltage** (about 2.7 V from a 3.3 V PWM) against the BSS138 threshold, at temperature.
6. **Test points**, then the PCB: four layers, using the throttle `LAYOUT.md` rules.

## Files

| Path | What |
|---|---|
| `gen/design.py` | Every part, pin and net. **The source of truth** |
| `gen/make_sch.py` | Generator: auto-placement, labels, sheets, library and tables |
| `icarus.kicad_sym`, `icarus.pretty/` | Vendor symbols and footprints copied (read-only) from the throttle ECU project, plus custom TPS3850, SQJ147 and relay pin types |
| `vendor_sym/` | The original vendor symbol files the library is built from |
| `doc/icarus_schematic.pdf` | Plot of all sheets |
| `erc.rpt` | Last ERC report |
