# Running the EPS tap wing and the bench stim wing: every combination

Two FeatherWings share one Feather M4 CAN:

- **[EPS tap wing](hardware/eps_tap_wing/README.md)** (the logger front end). It only listens: every lead to the car goes through ≥ 100 kΩ.
- **[Bench stim wing](../bench_stim_wing/README.md)**. It powers the stack from 12 V, powers the SAS, and can drive NEP, VSP, VT3/VT6 and IG1 through fail-safe relays.

They are designed to run **alone or together, in the car or on a bench EPS**. The one thing that changes between those cases is **where the stack's ground connects to the car**. Get that right and everything else is plug-and-play.

## The stack

```
  OLED FeatherWing          male headers
  bench stim wing           stacking headers (only when fitted)
  Adalogger FeatherWing     stacking headers
  Feather M4 CAN            stacking headers
  EPS tap wing              female headers on top (it's a "bottom wing")
```

- Each board in the middle needs **Feather stacking headers** (Adafruit PID 2830: long male tails below, female sockets on top).
- **Without the stim wing,** the OLED plugs straight into the Adalogger.
- **Without the tap wing,** the Feather can sit at the bottom on plain male headers.
- Either wing can be fitted or left out. Neither needs the other.

## Why ground matters

- **The logger measures relative to its own ground (GND).** The torque sensor's outputs (VT3/VT6, about 1.5–3.5 V) are referenced to **T/S GND (B9)**. B9 is tied to the ECU's ground inside the ECU.
- **The ECU's ground leaves on A4 (PG, BLK, to G451).** That wire also carries the **40 A motor return**, so the far end of it, or any other chassis point, can sit tenths of a volt away from the ECU's ground while the motor works.
- **Logger alone:** the stack **floats**, on a USB power bank. Its only tie to the car is the tap wing's 100 Ω resistor (R11) to B9. There's no loop and no offset.
- **Logger with the stim wing on 12 V:** the stack's ground *is* the stim wing's 12 V return. That return **must land at the ECU end of A4**. Then logger GND, the ECU's ground and B9 are all within a few mV. R11 carries almost nothing, and every reading stays true.
- **If the 12 V return goes anywhere else** (a chassis bolt, the battery, the cigarette socket), motor current puts that drop between logger GND and B9. It shows up as an offset on **every** analog channel, VT3/VT6 included, and current flows through R11.

## The four configurations

| | 1. Logger only, in the car | 2. Logger + stim wing, in the car | 3. Stim wing only (car or bench) | 4. Logger + stim wing, bench EPS |
|---|---|---|---|---|
| **Stack** | Feather + Adalogger + tap wing (+ OLED) | Full stack | Feather + stim wing (+ Adalogger, OLED) | Full stack |
| **Power** | USB power bank in the Feather's USB-C | 12 V into stim J2 (fused feed, IG or battery) | 12 V into stim J2 | Bench supply into stim J2 |
| **Stack ground** | **Floating.** Only R11 → B9 | **Stim J2 GND → ECU end of A4 (PG)**, spliced within ~10 cm of the connector | Stim J2 GND → ECU end of A4 (in the car) or the bench supply's − | Bench supply − and stim J2 GND both at the EPS's A4 terminal |
| **Tap wing J2 GND (pin 4)** | B9 (as designed) | B9 (as designed) | — | B9 |
| **Stim J4.3 / J5.4 GND pins** | Leave unwired | **Leave unwired** (the reference already comes through J2 GND) | Leave unwired | Optional: the EPS's A4 terminal |
| **Stim ARM loop (J5.7–8)** | — | e-stop / key switch | e-stop / key switch | e-stop / key switch |
| **SAS** | Not powered (needs the stim wing) | J3, powered by the stim wing; its ground is J3.2 only | J3 | J3 |
| **IG1** | Tap reads the car's IG1 | Tap reads the car's IG1. Stim J5.1 unwired | Bench only: J5.1 → EPS B6 | Stim J5.1 → EPS B6; the tap reads that same wire |
| **Laptop on USB** | Battery-powered laptop only (a mains charger grounds the stack) | Battery-powered laptop, or a USB isolator | Any | Any, if the bench supply floats; otherwise a USB isolator |

**The stim wing installed but not powered** (in-line in the harness, J2 unplugged) is safe in configuration 1:
- Its relays rest on **NC**, so the EPS sees the car's own wires.
- Relay contacts are isolated from the coils.
- The NO side (op-amps, clamps) is cut off.
- Its I2C chips run from the Feather's 3V3. R40/R44 keep the DAC from back-feeding the unpowered op-amp.
- As long as J4.3/J5.4 stay unwired, the logger still floats.

**Don't plug the power bank in while the stim wing supplies 12 V** in configurations 2 or 4. Nothing breaks: the stim wing feeds the USB pin through a diode at ~4.7 V and the power bank floats. It just isn't needed.

## Signal compatibility, checked

| Signal | Stim wing drives | Tap wing reads | OK? |
|---|---|---|---|
| NEP | Open drain, 4.7 kΩ to 12 V | 100 kΩ / 33 kΩ → D6, clamped to 3.3 V | High ≈ 3.0–3.6 V, low ≈ 0 V ✓ |
| VSP | Open drain, 4.7 kΩ to 5 V | 100 kΩ / 100 kΩ → D5 | High ≈ 2.5 V (SAMD51 threshold ≈ 1.8 V) ✓ |
| VT3 / VT6 | 0–4.5 V via 470 Ω | 1 MΩ / 1 MΩ, buffered → A2/A3 | 2 MΩ load: under 0.05 % error ✓ |
| IG1 | 12 V via 0.5 A PTC | 100 kΩ / 33 kΩ → D12 | ✓ |
| Current clamp supply | Feather USB pin ≈ 4.7 V (from the stim wing) | J3 pin 1 | HSTS016L is ratiometric, and the logger re-zeroes at key-on ✓ |

**Tap the EPS side.** With the stim wing in the harness, fit the tap wing's J2 leads on the **EPS side** of the breakout. The log then records what the EPS actually receives, faked or real, and shows which relay state was active.

## Feather resources

| Resource | Tap wing / logger | Stim wing |
|---|---|---|
| Pins | A0–A5, D5, D6, D9, D11, D12, D13, SDA/SCL | D4, TX, SDA/SCL, 3V3, USB |
| Shared | SPI + D10 (Adalogger SD) | — |
| I2C | ADS1115 0x48 (+ RTC 0x68, OLED 0x3C) | TCA9534 0x20, MCP4728 0x60 |
| Timers | **TC3** (1 kHz sample tick), EIC on D5/D6 | NEP on D4 (PA14): use **TCC1/WO2 or TCC2/WO0**, *not* TC3. VSP on TX (PB16): **TC6/WO0 or TCC0/WO4** |
| Serial | USB only | USB only. TX is a GPIO here, so Serial1 is unavailable |

**No pin, address or timer clashes.** Firmware that combines both (for example a bench mode inside `eps_logger`) must keep TC3 for the sample tick and generate NEP/VSP on TCC1/TCC2 and TC6/TCC0.

## Before the first powered run with both

1. Fit the stack. Check the stim wing's relays (5.2 mm) clear the OLED, and its back clears the Adalogger's coin cell.
2. 12 V off. Wire J2 GND to the ECU end of A4. Check with an ohmmeter:
   - **J2 GND to B9: under 1 Ω.** That's the ECU's internal tie.
   - **J2 GND to chassis: low, but via A4 only.**
3. Power up with the ARM loop **open**. The 12V and 5V LEDs should light and the logger should boot.
4. Run the logger's `c` check with the key ON and the engine OFF. +B, Vcc2 and VT3/VT6 should read the same as with the power bank. Any steady offset means the ground isn't at the A4 star point.
