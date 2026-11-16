# eps_bench_nep: wake the EPS on the bench and log it with the Hantek

The Feather M4 CAN fakes engine speed (NEP) so the EPS turns its assist on, reads the Toyota steering angle sensor over CAN, and toggles a sync pin. The Hantek 1008B records VT3, VT6, NEP and the sync pin in [Scope1008](https://github.com/ThatguyCplus/scope1008) roll mode. Afterwards [merge_hantek_feather.py](../../tools/merge_hantek_feather.py) puts both on one timeline.

VSP is not driven, so the EPS sees 0 mph, which gives full assist. Never send speed without rpm (DTC 22), and don't hold ≥ 2,000 rpm at 0 mph for 6 min. The firmware drops back to 900 rpm after 5 min at ≥ 2,000.

## Wiring (breadboard)

| From | Through | To |
|---|---|---|
| Feather D4 | 10 kΩ | NPN base (2N3904 / 2N2222 / BC547). Emitter to GND |
| +12 V (same supply as EPS IG1/+B) | 10 kΩ | NPN collector = **NEP node** |
| NEP node | — | EPS **B14** (NEP) and Hantek CH3 |
| Feather D13 | — | Hantek **CH4** (sync) |
| VT3 B10 / VT6 B8 | — | Hantek CH1 / CH2 (no divider needed, 1 MΩ input) |
| T/S GND B9 | — | Feather GND, NPN emitter, **every** Hantek probe ground |
| Feather CAN-H / CAN-L | — | SAS, 120 Ω at each end of the bus. SAS on its own 12 V |

- The transistor copies the ECM's open-collector output: idle high (12 V through the pull-up), pulled low on each pulse.
- Check the NEP node on CH3: it should swing about 0 V to above 8 V. The EPS input has its own pull-down; if the high level is low, put a second 10 kΩ in parallel with the pull-up (5 kΩ).
- **In the car**, inject only on the EPS side of B14 with a breakout. That wire also feeds the gauges, ECM, MCM and TCM.
- Feather and Hantek both plug into the same PC, so their grounds meet there too. Run a laptop on battery, or make sure the bench supply's negative isn't tied to mains earth somewhere else.

## Build and run

```bash
"C:/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe" compile --fqbn adafruit:samd:adafruit_feather_m4_can steering_control_unit/data_capture/firmware/eps_bench_nep
```
Libraries: Adafruit CAN, Adafruit SH110X, Adafruit GFX.

1. Start Scope1008 with the probes disconnected, so it calibrates zero. Then connect the probes.
2. Enable CH1–CH4, switch to **Roll**, press **● Rec**.
3. Start the Feather logger:
   ```bash
   python steering_control_unit/data_capture/tools/feather_log.py COM7 bench_feather.log
   ```
   Type `r900` and Enter to start NEP at 900 rpm, `r0` to stop, `z` to zero the SAS, `?` for status. OLED buttons: A on/off, B next preset (900/1200/1500/2000), C zero SAS.
4. Stop both after the test, then merge:
   ```bash
   python steering_control_unit/data_capture/tools/merge_hantek_feather.py scope.csv bench_feather.log merged.csv --sync CH4
   ```
   It reports how many sync edges matched and the timing error. Expect about the scope's sample period (2–3 ms at 440 Sa/s) or better.

The merged CSV keeps every scope column and adds `sas_deg`, `sas_dps`, `sas_zeroed_deg` (blank when the SAS is stale) and `nep_rpm`.

## Serial format

| Line | Meaning |
|---|---|
| `N,<us>,<rpm>` | NEP command changed |
| `A,<us>,<deg>,<dps>,<zeroed_deg>` | SAS frame, timestamped in the CAN interrupt |
| `S,<us>,<level>,<n>` | Sync edge on D13, timestamped as the pin changed. Intervals are pseudo-random 200–1000 ms |
| `# ...` | Status and comments |
