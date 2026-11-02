# EPS (Electrical Power Steering): Car-Side Reference

Vehicle: **Honda Insight, 1st gen (2000–06)**, M/T and CVT.
This file covers the car side: where the parts are, the pins, the voltages Honda specifies, and how the EPS behaves. The logger hardware is in [BOM.md](BOM.md).

## Sources

All in the service manual repo <https://github.com/doppelhub/Insight-Service-Manual/tree/master>:

| Document | What came from it |
|----------|-------------------|
| `Green Service Manual/electrical_power_steering.pdf` | Fig. 8–9 (p.15–16): I/O tables with expected voltages. Fig. 10–11 (p.17–18): circuit diagram including the ECU internals. DTC list and troubleshooting. Indicator behaviour. ECU removal (p.134). |
| `Electrical/Connector Physical Pinout.pdf` | Items 35 (EPS motor), 100 (EPS control unit), 101 (EPS motor relay), 125 (torque sensor): connector colours and locations |
| `Electrical/Schematic.pdf` | Page 45 (= [eps_wiring_diagram.jpg](eps_wiring_diagram.jpg)), Circuit E93 (vehicle speed), Circuit F44 (engine speed) |
| `OBDIIC&C Crib Sheet US.pdf` | Confirms the community DLC tool exposes no EPS parameters |

---

## Where everything is

| Part | Connector | Location |
|------|-----------|----------|
| **EPS control unit** | **A: 4P natural** + **B: 14P green** | **Under the right-side kick panel** (passenger footwell), on the wiper sub-harness |
| EPS motor relay | 4P brown/natural | Under the right kick panel, next to the control unit |
| EPS torque sensor | 7P black | Left rear of the engine compartment, on the steering gearbox |
| EPS motor | 2P gray | Left rear of the engine compartment, on the steering gearbox |
| Data link connector (DLC) | 16P | Under the dash: passenger side ('00), driver side ('01–'06) |
| Fuse 15 (40 A) EPS +B | — | Under-hood fuse/relay box, fed from the No. 2 (80 A) main fuse |
| Fuse 6 (7.5 A) IG1 | — | Under-dash fuse/relay box, also feeds the EPS indicator bulb |

**Everything the logger needs is at the control unit connectors behind the right kick panel**, inside the dry cabin. Don't tap at the gearbox end; it's wet, hot and moving.

---

## Connector A (4P, natural): motor power

Viewed from the wire side of the female terminals:
```
[1][2]
[3][4]
```

| Pin | Wire | Signal | Function | Honda measurement (pin to ground) |
|-----|------|--------|----------|----------------------------------|
| A1 | GRN | M− | Motor minus (H-bridge output) | — |
| A2 | WHT | +B | Motor power, from Fuse 15 (40 A) | Battery voltage, always |
| A3 | RED | M+ | Motor plus (H-bridge output), runs through the motor relay | — |
| A4 | BLK | PG | Power ground, to G451 | — |

## Connector B (14P, green): sensors and signals

Viewed from the wire side of the female terminals (pin 12 is not used):
```
[1][2][3]  lock  [4][5][6]
[7][8][9][10] x [11] / [13][14]
```

| Pin | Wire | Signal | Function | Honda measurement (pin to ground) |
|-----|------|--------|----------|----------------------------------|
| B1 | WHT | Vcc1 | Torque sensor supply 1 (**12 V class**) | Engine running: **battery voltage**. Ign OFF: 0 V |
| B2 | YEL | Vcc2 | Torque sensor supply 2 (5 V) | Engine running: **about 5 V**. Ign OFF: 0 V |
| B3 | BRN | SCS | Service check input (DLC pin 9) | — **Do not tap** |
| B4 | BLU/WHT | VSP | Vehicle speed pulses from the VSS, 4 pulses per rev | Spin a front wheel: **0–5 V pulses** |
| B5 | LT BLU | DIAG-H | Honda diagnostic line (DLC pin 14) | — **Do not tap** |
| B6 | YEL | IG1 | Ignition power, Fuse 6 | Ign ON: battery voltage. OFF: 0 V |
| B7 | GRY/BLU | SG | Shield ground of the torque sensor cable | — |
| B8 | RED | VT6 | Torque sensor signal | Engine running, wheel turned: **about 3.5–1.5 V** |
| B9 | BRN | T/S GND | Torque sensor ground | — |
| B10 | BLU | VT3 | Torque sensor signal | Engine running, wheel turned: **about 3.5–1.5 V** |
| B11 | PNK/BLU | RT+ | Motor relay coil drive (relay off on fault) | Engine running: battery voltage. Ign OFF: 0 V |
| B13 | YEL/RED | WLP | EPS warning lamp drive (low side) | Lamp ON: 0 V. Lamp OFF: battery voltage |
| B14 | BLU | NEP | Engine speed pulses from ECM A19 | Idling: **about 6 V** on a meter (average of a ~12 V pulse train) |

---

## What's inside the control unit (Fig. 11)

- **Motor drive:** +B (A2) goes through an internal fail-safe relay to a **4-MOSFET H-bridge**. M+ (A3) and M− (A1) are the bridge outputs, and the low side returns through PG (A4) to G451.
  - Both motor wires switch (PWM), so neither is ever ground.
  - Motor current flows in both directions, one per steering direction.
- **Motor wiring:**
  - M+ path: A3 RED → relay pin 3 → relay contact → pin 1 WHT/RED → motor connector pin 1.
  - M− path: A1 GRN → motor connector pin 2.
  - **The colours change at the gray motor connector.** On the gearbox side, Motor+ is BLK and Motor− is RED.
- **Relay coil:** RT+ (B11) → relay pin 2 → pin 4 → BLK → G402. The ECU pulls RT+ up to battery voltage to close the relay.
- **Sensor supplies:** Vcc1 (B1) is fed from 12 V and Vcc2 (B2) from the ECU's own 5 V regulator, each through a resistor.
- **Grounds:** T/S GND (B9) and SG (B7) are tied to the ECU's ground internally.
- **VSP (B4) and NEP (B14):** inputs with pull-downs. The VSS and the ECM each drive their line with an open-collector output and their own pull-up.
- **WLP (B13):** open-collector low-side driver. The bulb pulls it up to IG1.
- **SCS (B3):** pulled up inside the ECU. Grounding it puts the ECU into DTC blink/clear mode.
- **DIAG-H (B5):** pulled-up single wire with an open-collector transmitter. This is Honda's HDS line.

## Torque sensor

- **Type:** inductive. Two coils on the pinion shaft feed an integrated circuit, which is powered by Vcc1 (battery) and Vcc2 (5 V).
- **Outputs:** VT3 and VT6 are about **1.5–3.5 V** while steering.
- **Redundancy:** the ECU checks each channel on its own (DTC 11, DTC 15) and also their **average** (DTC 16). That suggests the two channels move in opposite directions around ~2.5 V. **Verify this on the first capture.**
- **7P black connector at the gearbox:**

  | Pin | Wire | Signal |
  |-----|------|--------|
  | 1 | RED | VT6 |
  | 2 | YEL | Vcc2 |
  | 3 | BLU | VT3 |
  | 4 | BRN | T/S GND |
  | 6 | WHT | Vcc1 |
  | 5, 7 | — | not used |

  The cable shield (GRY/BLU) goes to ECU pin B7.
- **Powered at ignition ON:** the manual's DTC-clear procedure reads a 45° steering input with the engine stopped. The motor relay only closes after the engine starts, so **static torque calibration with the engine off is safe** (no assist).

## Shared signals: taps must not load these

- **VSP** (BLU/WHT, circuit E93): VSS → gauge assembly, ECM, MCM, TCM (CVT), climate control unit, EPS.
- **NEP** (BLU, circuit F44): ECM A19 → gauge assembly (tachometer), MCM, TCM, test-tachometer connector, EPS.

## Pulse scaling

| Signal | Scale | Source | Status |
|---|---|---|---|
| **NEP** | **1.5 pulses per engine rev** (3 per 2 revs). Hz = rpm × 1.5 / 60: idle 900 rpm = 22.5 Hz, 2,000 rpm = 50 Hz, 6,000 rpm = 150 Hz | [MuddersMIMA](https://github.com/doppelhub/MuddersMIMA) `engine_signals.h`, which reads the same F44 line (its schematic labels it `NEP_12V`) | Confirmed in working code; still worth one check against the tach |
| **VSP** | **About 4,450 pulses/mile** (≈ 1.24 Hz per mph): 6.2 mph ≈ 7.7 Hz, 60 mph ≈ 74 Hz | 5 pulses per tyre rev ([EcoModder, quoting Insight Central](https://ecomodder.com/forum/268041-post24.html)) × ~889 revs/mile for a 165/65R14. Other Hondas use about 4,000/mile | **Unverified.** Calibrate with a measured roll or GPS |

The logger keeps raw edges on the card. These figures (`CFG_NEP_PULSES_PER_REV`, `CFG_VSP_PULSES_PER_MILE` in `config.h`) only scale the serial readouts.

**Bench note:** with the key ON and the engine off, the DC-DC converter doesn't charge the 12 V battery (per the [LiBCM](https://github.com/doppelhub/Honda_Insight_LiBCM) technical FAQ). Any EPS assist driven by a faked NEP comes from the 12 V battery alone.

## No digital access to EPS data

- The control unit has **no CAN**.
- Its only data link is DIAG-H to Honda's HDS tool.
- The community OBDIIC&C crib sheet lists no EPS parameters.

**Analog taps are the only way to log torque, current and assist state.**

---

## EPS behaviour that matters when reading logs

- **Indicator:** on at key-on, off once the engine starts. On a fault it stays on and **assist turns off** (manual steering).
- **Indicator with no fault:** it also comes on when:
  - the engine is at ≥ 2,000 rpm with the car at ≤ 1 mph for more than 6 min, or
  - the engine is at ≤ 280 rpm with the car at ≥ 6.2 mph for 3 s.
- **Thermal protection:** repeated hard steering, especially when stationary, makes the ECU **gradually reduce motor current**. The lamp does **not** come on. Full assist comes back gradually, within about **14 minutes**.
- **DTC 23 (engine speed signal):** assist comes back once speed is ≥ 1 mph.
- **When the lamp clears:**
  - DTCs 22, 23, 46, 47, 50, 64, 66 and 68 clear the lamp by themselves once the system is normal again.
  - The other codes clear at the next key cycle.
  - DTCs **11, 15, 16, 17, 18 and 67 keep the lamp on until the code is erased.**
- **Code storage:** DTCs live in EEPROM, so **disconnecting the battery doesn't clear them.** A tap that disturbs the torque sensor could therefore leave you with no assist until the code is cleared.
- **Reading codes:**
  1. Wheels straight, engine off.
  2. Short SCS (DLC pin 9) to body ground before turning the ignition ON.
  3. Read the long/short blinks on the EPS lamp.
- **Clearing codes:** short SCS the same way, then follow the manual's 45°-left steering sequence. Two blinks means the code was erased.
- **Logger bonus:** with WLP logged, the logger can record these blink codes too.

### DTC list

| DTC | Problem |
|-----|---------|
| 11 / 15 / 16 | Torque sensor VT3 / VT6 / VT3+VT6 average |
| 17 / 18 | Torque sensor supply Vcc1 (12 V) / Vcc2 (5 V) |
| 22 | Vehicle speed vs engine speed plausibility, or VSS jumps |
| 23 | Engine speed signal circuit |
| 37 | ECU internal (motor voltage input circuit) |
| 41 | Motor voltage |
| 42, 45 | Motor driven current |
| 43 | Motor current excessively high |
| 46, 47 | Power relay stuck ON / OFF |
| 50 | ECU CPU |
| 62 | Fail-safe relay stuck ON |
| 64 | Battery voltage too low (fail-safe relay contact / motor voltage drop) |
| 66 | Motor driven voltage |
| 67 | Torque sensor interface circuit |
| 68 | Interlock circuit |

## If you ever unplug the control unit

The manual's procedure is:
1. Disconnect the battery negative first.
2. Afterwards, re-enter the radio code, set the clock and do the ECM idle learn.
3. Pull Fuse 15. If the IMA battery gauge shows no segments, hold 3,500–4,000 rpm until it shows ≥ 3 segments.
4. Refit Fuse 15.

**Back-probing the connectors avoids all of this.** That's the reason the logger taps that way.

---

## Tap list

The front end for each tap is in [BOM.md](BOM.md) and drawn in [LOGGER_WIRING.svg](LOGGER_WIRING.svg).

| Signal | Pin | Level (manual) | Class | Tap? |
|--------|-----|----------------|-------|------|
| VT3 | B10 BLU | 1.5–3.5 V | Precision analog | **Yes** |
| VT6 | B8 RED | 1.5–3.5 V | Precision analog | **Yes** |
| Vcc2 | B2 YEL | ~5 V | Precision analog (ratiometric reference) | **Yes** |
| T/S GND | B9 BRN | 0 V | Logger ground reference | **Yes** (single point) |
| +B | A2 WHT | Battery | Analog, 12 V class | **Yes** |
| Motor current | A1 GRN wire | ±tens of A | Clamp-on Hall, no electrical contact | **Yes** |
| VSP | B4 BLU/WHT | 0–5 V pulses | Digital/timer | **Yes** |
| NEP | B14 BLU | ~0–12 V pulses | Digital/timer | **Yes** |
| WLP | B13 YEL/RED | 0 V lamp on / battery lamp off | Digital | **Yes** |
| IG1 | B6 YEL | Battery when key ON | Digital (opens/closes log files) | **Yes** |
| RT+ | B11 PNK/BLU | Battery when relay on | Digital | Optional |
| Vcc1 | B1 WHT | Battery | Analog | Optional (needs a second op-amp) |
| SG | B7 GRY/BLU | — | Shield drain | **No.** Don't use as logger ground |
| SCS | B3 BRN | — | Diagnostic mode input | **Never** |
| DIAG-H | B5 LT BLU | — | HDS bus | **Never** |
| M+ / M− / PG | A3 / A1 / A4 | Switched, 40 A class | Power | **Never electrically.** Clamp only |
