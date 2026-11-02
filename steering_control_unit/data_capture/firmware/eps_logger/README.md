# eps_logger firmware

Listen-only 1 kHz logger for the Honda Insight G1 EPS. The hardware is described in [BOM.md](../../BOM.md) and [LOGGER_WIRING.svg](../../LOGGER_WIRING.svg).

| File | What's in it |
|------|--------------|
| `eps_logger.ino` | Sampling interrupt, SD logging, serial commands, LED |
| `config.h` | Pins, resistor values, rates. **Change this file when the hardware changes.** |
| `log_format.h` | On-card record and header layout, shared with `tools/decode_log.py` |
| `lead_diag.h` | Lead diagnostics against the Honda manual's values. Portable C++. |
| `fast_adc.h` | SAMD51 register-level ADC: VT3 and VT6 converted together on ADC1 + ADC0 |

## Build

- **Board package:** "Adafruit SAMD Boards". Board manager URL: `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json`
- **Board:** Adafruit Feather M4 CAN (SAME51), FQBN `adafruit:samd:adafruit_feather_m4_can`
- **Libraries:** "SdFat - Adafruit Fork" and "RTClib" (pulls in Adafruit BusIO)

```bash
arduino-cli core install adafruit:samd --additional-urls https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
```
```bash
arduino-cli lib install "SdFat - Adafruit Fork" RTClib
```
```bash
arduino-cli compile --fqbn adafruit:samd:adafruit_feather_m4_can steering_control_unit/data_capture/firmware/eps_logger
```

If the board doesn't show a port for upload, double-tap reset to enter the bootloader.

## Use

Powered from a USB power bank, the logger floats. With the [bench stim wing](../../../bench_stim_wing/README.md) it can run from 12 V instead. In that case wire the 12 V return to the ECU end of A4: see [STACK_MODES.md](../../STACK_MODES.md).

1. **On the bench (USB only):** open a serial monitor at 115200.
   - `s` shows card, RTC and ADC status.
   - `t<unix seconds>` sets the clock.
   - `r` records without IG1, so you can check a file decodes.
2. **In the car:**
   1. Fit the back-probes with the **ignition OFF**.
   2. Key ON, engine OFF: run `c`.
   3. Start the engine and run `c` again. Every row should read PASS or an expected INFO.
3. **Driving:** key ON opens `EPSnnnnn.BIN`, and key OFF for 3 s closes it.
4. **Decoding:** `python tools/decode_log.py EPS00001.BIN` writes a CSV and prints a summary.

Commands (one per line): `c` install check · `l` live values at 10 Hz · `s` status · `r` record toggle · `t<unix>` set clock.

| LED (D13) | Meaning |
|-----------|---------|
| short blip every 2 s | waiting for key ON |
| solid | logging |
| 2 Hz blink | logging, a lead diagnostic is active (see `c` or the serial log) |
| fast blink | SD card missing or failed (retried every 5 s while the key is ON) |

## Lead diagnostics

Every tap has a pull-down on the Proto Wing, so an open lead reads 0 V. The checks below compare each signal with what Honda's manual says it should read (`electrical_power_steering.pdf`, Fig. 9). Active flags go into every record and are printed when they change.

| Flag | Fires when | Usual cause |
|------|------------|-------------|
| VBAT_MISSING | +B < 1 V (A2 is always hot) | +B lead or **ground lead** open |
| VBAT_RANGE | +B outside 10.5–15.5 V, key ON | Weak battery / DC-DC, or bad lead |
| VCC2_MISSING | Vcc2 ~0 V with the engine running | Vcc2 lead open |
| VCC2_RANGE | Vcc2 outside 4.5–5.5 V | EPS unit or sensor supply problem |
| VT3_OPEN / VT6_OPEN | Signal ~0 V while Vcc2 is good | That lead is open |
| VT3_RANGE / VT6_RANGE | Outside 1.0–4.0 V (manual: 1.5–3.5 V) | Sensor fault, or lead touching another pin |
| IMOT_SENSOR | Clamp output < 0.5 V or > 4.5 V | Clamp unpowered or unplugged |
| IMOT_ZERO | \|I\| > 2 A while the relay is open | Clamp offset drift, or clamp on the wrong wire |
| IG1_OPEN | IG1 reads OFF while Vcc2/NEP show the car is on | IG1 lead open |
| WLP_ON_RUN | Lamp ON for 30 s with the engine running | Real EPS fault (assist is off), or WLP lead open |
| RT_OPEN | Relay drive OFF while running with the lamp OFF | RT+ lead open |
| NEP_MISSING | Relay ON but no engine pulses for 3 s | NEP lead open, or auto-stop |
| ADC_SEQ | ADC1 cross-check failed | VT3/VT6 now sampled ~6 µs apart instead of together |
| SD_FAULT | Card write failed or records dropped | Card full, slow or removed |

## Not yet verified on hardware

- The full sketch hasn't been compiled. The Adafruit SAMD core isn't installed on the dev machine yet.
  - The portable headers compile cleanly on ARM with `-Wall`.
  - Every SAMD51 register name was checked against Microchip's device headers.
  - The decoder passes a synthetic-file test.
- First bench run should confirm:
  - `s` reports "VT3/VT6 simultaneous".
  - A recorded file decodes with no gaps.
  - The RTC is found. The Adalogger has its own 10 kΩ I2C pull-ups, which the bare Feather M4 CAN lacks.
