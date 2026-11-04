# AUX link: a second microcontroller on the stack's CAN bus

The stack has no free pins and the stim wing has no board area left. A second
microcontroller (for example the OBD/IMA "diag node") therefore talks to the
Feather M4 CAN over the **CAN bus the stack already has**: the SAS bus that
passes through the stim wing's J3. No new hardware is needed on either wing.

Status: **spec only.** Neither end is written yet.

## Why CAN and not I2C or UART

| Option | Verdict |
|---|---|
| **CAN (chosen)** | Already wired (J3 pins 5/6 → the Feather's terminal). Differential, so it doesn't care about the few tenths of a volt between grounds in a car. **No ground wire needed**, so the [grounding rule](STACK_MODES.md#why-ground-matters) still holds. 500 kbit/s, frames arrive timestamped within ~1 ms |
| I2C (STEMMA QT) | The Feather M4 CAN has no STEMMA QT port and the stim wing has no room for one. Off-board I2C also needs a shared ground and is fragile on long car cables |
| UART | The only free Feather pin is RX. TX already drives VSP |

## Wiring

- **Bus:** CAN-H and CAN-L only. No ground wire, no power wire. The second board powers itself (for example from the DLC, pin 16 / pin 4, through its own fuse and regulator).
- **Where to join:** splice into the lead from stim wing **J3 pins 5/6** (CANH/CANL) to the Feather's terminal (pin 3 H, pin 1 L). Keep any stub under 30 cm.
- **Termination:** exactly **two** 120 Ω terminators on the whole bus, ideally at its two physical ends. With everything unpowered, **CAN-H to CAN-L should read about 60 Ω**. 120 Ω means one terminator is missing; 40 Ω means there's one too many.
  - The Feather's terminator is solder jumper SJ2.
  - If the second board sits at the far end of the bus, close its terminator and open whichever other one isn't at an end.
- **Ground offset:** CAN transceivers tolerate −2 V to +7 V common mode. The stack's ground (ECU end of A4, or floating via R11 to B9) and the DLC ground (G402/G101) are well inside that.

Compatible boards: anything with a CAN controller and 3.3 V or 5 V transceiver, for example Adafruit's RP2040 CAN Bus Feather (MCP2515), a second Feather M4 CAN, or an ESP32 with a TJA1051/SN65HVD230.

## Bus rules

- **500 kbit/s**, standard 11-bit IDs, the same as the Toyota SAS (ID 0x25).
- The AUX link uses **IDs 0x6C0–0x6CF** only. That stays clear of the SAS (0x25) and of OBD diagnostic IDs (0x7DF, 0x7E0–0x7EF).
- Multi-byte values are **little-endian**.
- Bus load: SAS plus all frames below at their stated rates is under 10 % of 500 kbit/s.

## Frames

AUX = the second microcontroller. FTH = the Feather M4 CAN (logger / stim firmware).

| ID | From → to | Rate | Bytes |
|---|---|---|---|
| 0x6C0 | AUX → FTH | 1 Hz | Heartbeat. 0: protocol version (1). 1: status bits (b0 K-line OK, b1 H-line OK, b2 METSCI seen, b3 BATTSCI seen, b4 SCS active). 2–5: uptime, ms (u32). 6: last error code. 7: rolling counter |
| 0x6C1 | AUX → FTH | 5 Hz | METSCI (MCM → BCM). 0: assist/regen byte (0x40 none, 0x41–0x54 assist 1–20, 0x21–0x34 regen 1–20). 1: SoC bars byte (0x20–0x34). 2: latest B3 data. 3: latest B4 data. 4: bad-checksum count (saturating). 7: counter |
| 0x6C2 | AUX → FTH | 10 Hz | BATTSCI (BCM → MCM), raw. 0: pack V ÷ 2. 1–2: SoC, 0.1 % (u16). 3–4: current counts (u16, as sent: 2048 − 20 × A). 5: max temp byte. 6: min temp byte. 7: counter |
| 0x6C3 | AUX → FTH | 10 Hz | ECM (K-line). 0–1: rpm (u16). 2: vehicle speed, km/h. 3: coolant, °C + 40. 4: battery, 0.1 V. 5: TPS, %. 6: MAP, kPa. 7: counter |
| 0x6C4 | AUX → FTH | on request | DTC report. 0: module (1 ECM, 2 MCM, 3 EPS, 4 ABS, 5 SRS, 6 TCM). 1: total count. 2: index of the first code in this frame. 3–7: up to 5 codes, 1 byte each (Honda blink codes). Repeated until all codes are sent |
| 0x6C8 | FTH → AUX | on demand | Command. 0: command (1 read DTCs, 2 clear DTCs, 3 SCS on, 4 SCS off, 5 run EPS erase sequence). 1: module (same numbering as 0x6C4). 2–3: token (u16; for commands 2, 3 and 5 it must equal the token from the latest 0x6C9 "armed" reply). 7: counter |
| 0x6C9 | AUX → FTH | per command | Reply. 0: command echoed. 1: result (0 OK, 1 refused: interlock, 2 refused: bad token, 3 timeout, 4 not supported, 5 armed). 2–3: token. 4: detail |

Rule for 0x6C3: AUX fills only what it can read and sends **0xFF** in any unfilled byte (0xFFFF for rpm).

## Interlocks (enforced on AUX, not trusted from FTH)

- **Clearing codes and SCS** (commands 2, 3, 5) need a two-step handshake: FTH sends the command once with token 0. AUX replies "armed" (5) with a fresh random token, valid for 10 s. FTH must then send the same command with that token.
- AUX refuses (result 1) unless **engine rpm = 0 and vehicle speed = 0**, when it can read them. On the bench, with no ECM, it refuses unless its own bench jumper is fitted.
- **SRS: never.** Module 5 is accepted for reading only; clearing it always returns "not supported" (4).
- Command 5 (EPS erase) follows the manual's sequence: SCS on, ignition on, two "45° left and hold, then centre" steps within the 4 s windows, two blinks = erased. AUX only times the steps and watches the EPS lamp; in the car the driver turns the wheel when prompted, and on the bench the stim firmware moves VT3/VT6 instead.
- Losing the 0x6C8 stream does nothing. **SCS turns itself off after 60 s** unless FTH re-sends "SCS on".

## Logger side

- The logger records every 0x6C0–0x6CF frame it receives, stamped with its own clock, to a separate `aux.csv` on the SD card, next to the existing log.
- The logger must keep **TC3** for its 1 kHz sample tick (see [STACK_MODES](STACK_MODES.md#feather-resources)). CAN receive is interrupt-driven on the SAME51's own CAN peripheral and uses no timer.
