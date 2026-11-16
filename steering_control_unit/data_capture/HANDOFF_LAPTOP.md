# Laptop handoff: EPS bench NEP test (2026-11-15)

Point a Claude Code session on the laptop at this file: "read steering_control_unit/data_capture/HANDOFF_LAPTOP.md and continue".

## Where things are

- **Branch:** `feature/c2000-f280049c-port` on `origin` (github.com/ThatguyCplus/throttleECU).
- **Get it on the laptop:**
  ```bash
  git fetch origin
  git checkout feature/c2000-f280049c-port
  git pull
  ```
  If the laptop clone has local changes, commit them or use a separate worktree first. Don't stash or reset.
- **Firmware:** `steering_control_unit/data_capture/firmware/eps_bench_nep/` (`.ino` + `README.md` with the full wiring table and run steps).
- **PC tools:** `steering_control_unit/data_capture/tools/feather_log.py` (serial logger), `merge_hantek_feather.py` (lines the Hantek CSV up with the Feather log).

## State

- The sketch builds for **Feather M4 CAN** (42 KB, 8 % flash). **Not flashed yet:** the Feather is on the laptop (COM11), not the desktop.
- The NEP driver is now an **XY-MOS module** instead of the NPN. Drive polarity is the same (D4 high pulls NEP low), so only comments changed.

## XY-MOS wiring (as built)

| From | To |
|---|---|
| Feather D4 | XY-MOS TRIG |
| XY-MOS trigger GND, VIN− | T/S GND (B9) |
| XY-MOS OUT+ | +12 V (EPS supply) |
| +12 V → 10 kΩ | XY-MOS OUT− = **NEP node** |
| NEP node | EPS **B14** and Hantek CH3 |
| Feather D13 | Hantek CH4 (sync) |
| Feather GND | T/S GND, every Hantek probe ground |
| Feather CAN-H / CAN-L | Toyota SAS, 120 Ω at each end |

## To do on the laptop

1. **Flash.** Arduino IDE: board *Adafruit Feather M4 CAN (SAME51)*, port COM11, Upload. Libraries: Adafruit CAN, Adafruit SH110X, Adafruit GFX. Or:
   ```bash
   arduino-cli compile --upload -p COM11 --fqbn adafruit:samd:adafruit_feather_m4_can steering_control_unit/data_capture/firmware/eps_bench_nep
   ```
   If the upload can't find the board, double-tap reset (green NeoPixel) and retry; the port number may change in bootloader mode.
2. **Check the startup message.** Serial monitor at 115200 should print `# eps_bench_nep: NEP on D4 -> XY-MOS, sync on D13, SAS 0x25 @ 500k`. `?` prints status.
3. **Check NEP before connecting B14.** NEP node at idle should read about 12 V. Send `r900`: scope CH3 should show about 22.5 Hz, swinging near 0 V to above 8 V. `r0` stops it and the node returns to 12 V.
4. **Run the logged test:** the steps in `firmware/eps_bench_nep/README.md` ("Build and run"): Scope1008 roll mode + Rec, `python steering_control_unit/data_capture/tools/feather_log.py COM11 bench_feather.log`, then merge.

## Rules that still apply

- Never send speed (VSP) without rpm (DTC 22). VSP isn't driven here, so the EPS sees 0 mph, which gives full assist.
- Don't hold ≥ 2,000 rpm at 0 mph for 6 min. The firmware drops to 900 rpm after 5 min.
- In the car, inject NEP only on the EPS side of B14; that wire also feeds the gauges, ECM, MCM and TCM.

## Why this test matters for Icarus

The rev 2.0 safety case still lacks Δmax and the fault-tolerant time interval (FTTI), and cutting the hardware speed stage relies on the EPS cutting assist with road speed (Icarus `HANDOFF.md`, "Rev 2.0 decision"). Next bench measurement after NEP works: inject a fixed VT offset and record motor current or rack torque against offset size at several VSP values **and with VSP removed**, to find what the EPS does on VSP loss.
