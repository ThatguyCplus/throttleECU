# Icarus — dual-channel steering torque-overlay unit (Honda Insight EPS)

Icarus adds a **bounded, double-monitored steering torque** by intercepting the OEM EPS torque-sensor signals (VT3/VT6). The OEM EPS still drives the motor. If anything goes wrong, Icarus drops out and the car steers exactly as stock.

**Status:** concept phase (ISO 26262 Parts 3–4 drafts). No schematic, PCB or firmware yet.

## Core safety idea: a deliberately overbuilt interceptor

- **Fail-silent hardware.** With no power or no enable, NC relay contacts put the OEM sensor straight through to the EPS.
- **Three independent shut-off channels:**
  - CH-A, the control F280049C;
  - CH-B, the monitor F280049C;
  - CH-C, an analog comparator guard with no software, which latches off.

  Any one of them removes the overlay.
- **Two cascaded relays.** The overlay reaches the EPS only if both are energized. Each coil needs a high side from one MCU and a low side from the other.
- **No shared regulators.** Each channel has its own buck, LDO, supervisor and window watchdog.
- **Authority limited by resistors.** The largest torque offset is fixed in analog hardware, so software can't exceed it.
- **Antisymmetric offset by circuit.** One DAC plus a hardware inverter keeps the EPS's VT3/VT6 average check satisfied. That check's DTCs latch until erased.

## Documents

| File | ISO 26262 | Contents |
|---|---|---|
| [docs/00_PROJECT_BRIEF.md](docs/00_PROJECT_BRIEF.md) | — | The improved project prompt: goals, reuse, constraints, limits |
| [docs/01_ITEM_DEFINITION.md](docs/01_ITEM_DEFINITION.md) | Part 3 §5 | Functions, boundary, interfaces, modes, assumptions |
| [docs/02_HARA.md](docs/02_HARA.md) | Part 3 §6 | Hazardous events, S/E/C, ASIL, safety goals SG1–SG5 |
| [docs/03_FUNCTIONAL_SAFETY_CONCEPT.md](docs/03_FUNCTIONAL_SAFETY_CONCEPT.md) | Part 3 §7 | FSR-01…16, decomposition B(D)+B(D), safe states |
| [docs/04_TECHNICAL_SAFETY_CONCEPT.md](docs/04_TECHNICAL_SAFETY_CONCEPT.md) | Part 4 §6 | Block diagram, hardware/software safety mechanisms, timing, metrics, DFA |
| [docs/05_HARDWARE_BASIS.md](docs/05_HARDWARE_BASIS.md) | Part 5 input | Parts (reused vs new), rails, MCU resource budget |

## Related folders (read-only references)

| Folder | Used for |
|---|---|
| `../data_capture/` | EPS pinout, signal levels and DTC behaviour. The logger measures the TBD-DATA values |
| `../toyota_sas/` | Steering-angle sensor on CAN |
| `../../f411testingreliablitymotor/c2000_f280049_throttle/` | Parts, layout rules and `safety.c` patterns. **Do not modify** |

## Planned layout

```
Icarus/
  docs/        ISO 26262 work products (this phase)
  hardware/    KiCad project (next: schematic)
  firmware/    ch_a_control/, ch_b_monitor/ (CCS projects, after the schematic)
  test/        HIL bench plans using the data logger and the VSP/NEP generator
```

## Next steps

1. **Measure the TBD-DATA values with the logger:** VT3/VT6 against driver torque, the antisymmetry check (A1), and the relay-gap tolerance (A2).
2. **Freeze Δmax, the relay part and the interception harness**, then draw the schematic in `hardware/`.
3. **Run the FMEDA** (SPFM/LFM/PMHF) on the schematic and iterate the safety mechanisms.
4. **Write the firmware** for both channels, then run the HIL bench, then do vehicle tests on private ground.

Compliance with ISO 26262 also needs a safety plan, confirmation reviews and an independent assessment. These documents are inputs to that process, not a substitute for it.
