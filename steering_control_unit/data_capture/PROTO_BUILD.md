# Building the front end on a FeatherWing Proto

This builds the logger's input protection and scaling by hand, on the Adafruit FeatherWing Proto (PID 2884). The circuit is the one in [BOM.md](BOM.md) and [LOGGER_WIRING.svg](LOGGER_WIRING.svg). The pin map matches `firmware/eps_logger/config.h`, so the firmware needs no changes.

The idea in one line: **every wire from the car meets a big resistor first, at the probe end.** By the time a signal reaches the Proto wing, it's only a few microamps. The wing then scales it to 0–3.3 V and clamps it so the Feather can't see more than 3.3 V.

---

## 1. Parts (through-hole)

**On the Proto wing**

| Qty | Part | Used for |
|-----|------|----------|
| 1 | MCP6024-I/P quad op-amp (DIP-14) + DIP-14 socket | Buffers VT3, VT6, Vcc2, +B |
| 3 | 1.8 MΩ 1 % | Lower divider resistor: VT3, VT6, Vcc2 |
| 1 | 150 kΩ 1 % | Lower divider resistor: +B |
| 1 | 180 kΩ | Lower divider resistor: VSP |
| 4 | 27 kΩ | Lower divider resistor: NEP, WLP, RT+, IG1 |
| 9 | 1 nF ceramic, C0G/NP0 | Filters: 4 analog + 5 digital |
| 5 | BAT85 Schottky diode | Clamps the digital inputs to 3.3 V |
| 1 + 1 | 10 kΩ + 18 kΩ | Current sensor divider |
| 1 | 100 nF ceramic | Current channel filter |
| 2 | 100 nF ceramic | Decoupling: op-amp, current sensor supply |
| *(optional)* 1 + 1 | 10 kΩ + 18 kΩ | Current sensor Vref → A5 (same divider as the current output) |
| — | Hookup wire, 26–30 AWG solid | Jumpers on the wing |
| 1 set | Feather stacking headers (PID 2830), or the Tripler | So the wing fits in the stack |

**At the probe end, NOT on the wing** (heat-shrunk inline, right behind each back-probe pin)

| Qty | Part | Lead |
|-----|------|------|
| 4 | 1 MΩ 1 % | VT3, VT6, Vcc2, +B |
| 5 | 100 kΩ | VSP, NEP, WLP, IG1, RT+ |
| 1 | 100 Ω | B9 (T/S GND), the only ground lead |

Keeping these at the probe means the whole cable is protected: a crushed or chafed cable can't load the EPS.

## 2. The circuits

**Analog: VT3, VT6, Vcc2, +B (buffered)**
```
probe [1 MΩ] ── cable ──► wing ──┬──────────── op-amp +IN
                                 ├─[1.8 MΩ]─ GND   (150 kΩ for +B)
                                 └─[1 nF]─── GND
op-amp −IN tied to OUT ──► Feather analog pin
```

| Signal | Op-amp section | +IN | −IN = OUT | Feather pin |
|--------|----------------|-----|-----------|-------------|
| VT3 | A | pin 3 | pins 2 + 1 | **A2** |
| VT6 | B | pin 5 | pins 6 + 7 | **A3** |
| Vcc2 | C | pin 10 | pins 9 + 8 | **A0** |
| +B | D | pin 12 | pins 13 + 14 | **A1** |

The op-amp takes power from pin 4 to the Feather's **3V** and pin 11 to **GND**, with 100 nF from pin 4 to pin 11 right at the socket. Because it runs from 3.3 V, its output physically can't exceed the ADC range.

**Digital: VSP, NEP, WLP, RT+, IG1**
```
probe [100 kΩ] ── cable ──► wing ──┬─────────────── Feather pin
                                   ├─[R2]──── GND   (180 kΩ for VSP, 27 kΩ for the rest)
                                   ├─[1 nF]── GND
                                   └─BAT85──► 3V    (band/cathode to 3V)
```

| Signal | R2 | Feather pin |
|--------|----|-------------|
| VSP | 180 kΩ | **D5** |
| NEP | 27 kΩ | **D6** |
| WLP | 27 kΩ | **D9** |
| RT+ | 27 kΩ | **D11** |
| IG1 | 27 kΩ | **D12** |

**Current sensor (HSTS016L ±50 A)**
```
red    ── Feather USB pin (5 V from the power bank)
black  ── GND
yellow ──[10 kΩ]──┬── A4
                  ├─[18 kΩ]── GND
                  └─[100 nF]─ GND
white  ──[10 kΩ]──┬── A5          (optional Vref channel, same divider)
                  └─[18 kΩ]── GND
shield ── GND (at the wing only)
```
Add 100 nF between USB and GND near where the sensor's power wires land.

**Ground:** the B9 lead (through its 100 Ω at the probe) lands on wing **GND**. Nothing else in the car connects to logger ground.

## 3. Layout on the Proto wing

The wing is small: about 30 parts in 50 × 23 mm. Mount the resistors **standing up** (one lead folded back over the body) to save room.

```
 ┌───────────────── 12-pin header row (D5 D6 D9 ... D11 D12, USB) ────────────────┐
 │  [digital zone]  5 channels in a row: R2 │ 1nF │ BAT85, each next to its D pin  │
 │                                                                                │
 │  [field wire entry: bare wires or a 0.1" header strip along the middle]        │
 │                                                                                │
 │  [analog zone]  1.8M/150k + 1nF per channel  [ MCP6024 socket ]  [10k/18k/100nF]│
 └──────────── 16-pin header row (3V  GND  A0 A1 A2 A3 A4 A5) ─────────────────────┘
```

- Put the **op-amp socket near A0–A3** on the 16-pin side, so the outputs are one short jumper away.
- Put each **digital channel next to its D pin** on the 12-pin side.
- Run a **ground bus**: one bare wire along the board, with every lower resistor, capacitor and the op-amp pin 11 landing on it. Do the same for **3V** (diode cathodes and op-amp pin 4).
- **Field wires:** solder them straight to the wing with a cable tie through the corner holes for strain relief. Or fit a 0.1" header strip and a matching housing so the harness unplugs. Label every wire.

## 4. Build order

1. Fit the **stacking headers** first, then check the wing still plugs into the Feather/Adalogger stack.
2. Ground bus and 3V bus wires.
3. Op-amp **socket** (not the chip yet) + 100 nF decoupling.
4. Analog dividers and filters, then the jumpers to A0–A3.
5. Digital channels: R2, 1 nF, BAT85 (**band towards 3V**), jumper to the D pin.
6. Current sensor divider and filter, then A4 (and A5 if you're using Vref).
7. Inspect under a light for solder bridges, especially around the header rows.

## 5. Test on the bench before it goes near the car

Put the wing in the stack (op-amp still out of its socket), load `firmware/oled_test` and power from USB.

1. **Shorts:** with power off, measure 3V to GND on the wing. It should not read 0 Ω.
2. **Power on, op-amp out:** the 3V bus should read 3.3 V and the USB pin about 5 V.
3. **Fit the MCP6024** (notch towards pin 1) with power off. All of A0–A3 should now read about 0 V on the OLED with nothing connected.
4. **Analog channels:** feed 5.0 V from a bench supply through a spare 1 MΩ, standing in for the probe resistor, into each analog input. Expect:
   - VT3 / VT6 / Vcc2 → **3.21 V** on A2 / A3 / A0
   - +B with 12.0 V in → **1.57 V** on A1 (and 14.4 V → 1.88 V)
5. **Digital channels:** feed 12 V through a spare 100 kΩ. A multimeter on the D pin should read **2.55 V** (27 kΩ channels) or about **3.4 V**, clamped by the BAT85 (VSP). With 5 V in, VSP should read **3.21 V**.
6. **Current sensor:** plug it in with nothing through the jaws. A4 should read about **1.61 V** (2.5 V × 0.643). Run a known current through it and check it moves about **8 mV per amp**.

When all of that checks out, load `eps_logger` and run its `c` lead check in the car, ignition ON and engine OFF, as BOM.md describes.

## 6. Things to watch

- **OLED wing buttons:** A/B/C connect D9, D6 and D5 to GND, the same pins as WLP, NEP and VSP. That's safe for the car, since the 100 kΩ probe resistors are in the way. But pressing a button while logging corrupts those three signals, so don't press them during a drive.
- **Cable shield** goes to GND at the wing end only.
- Fit all probes with the **ignition OFF**, and continuity-check neighbouring probes before key-on.
