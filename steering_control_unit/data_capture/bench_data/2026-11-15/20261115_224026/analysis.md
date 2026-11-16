# Raw bench session 20261115_224026

60 s free steering, NEP 900 rpm (EPS assist on) from 2.0 to 61.7 s. Hantek roll at 800 Sa/s
per channel: CH1 VT3, CH2 VT6, CH3 NEP, CH4 D13 sync. Feather: SAS (CAN 0x25), rpm, all CAN.
Alignment: 105 sync edges, 0.98 ms rms. NEP 22.22 Hz, 0 bad periods out of 1318.

## Torque sensor (VT3, VT6)

- Rest level (wheel still): VT3 2.550 V, VT6 2.569 V. VT6 sits 20 mV above VT3.
- VT6 = 1.003 x VT3 + 0.011 V, correlation 0.9994, residual 9 mV rms. The two outputs move
  together with the same gain, not mirrored.
- Noise at rest: about 6 mV rms on each (high-pass above ~20 Hz).
- Range seen: VT3 1.27 to 3.22 V. Largest swings were -1.28 V (pushing against the left end
  around -595 deg, 52 to 57 s) and +0.67 V.
- Sign: VT rises when turning right (+ angle), falls when turning left.

## Torque vs steering rate (assist on)

Median of (VT3+VT6)/2 minus rest, by SAS rate:

| rate (deg/s) | turning right | turning left |
|---|---|---|
| 20 to 100 | +0.155 V | -0.275 V |
| 100 to 300 | +0.210 V | -0.325 V |
| 300 and faster | +0.255 V | -0.380 V |

- Friction-like: the signal jumps to about +/-0.2 to 0.3 V as soon as the wheel moves and only
  grows slowly with speed (see torque_vs_rate.png).
- Left needs about 1.5x the signal of right at every speed. Either the sensor zero is offset,
  the bench rack drags more one way, or the hand effort differed.
- Torque leads steering rate by about 50 ms.
- 98 % of the torque signal energy is below 5 Hz; nothing notable above 20 Hz.

## CAN

| ID | rate | content |
|---|---|---|
| 0x25 | 83.8/s, no jitter | SAS angle; bytes 0, 1, 4, 5, 7 change (byte 7 counts) |
| 0x423 | 1.0/s | constant `20` |
| 0x4C6 | 1.1/s | constant `0700030000000000` |

Nothing on the bus reacted to NEP or steering apart from the SAS.

## Suggested next captures

1. Same moves with NEP off (assist off): the difference in VT signal is the assist effect.
2. Slow, steady turns both ways at about 50 deg/s with hands-off pauses, to pin down the
   left/right asymmetry and the zero offset.
3. If you can, a known torque (spring scale on a spoke) held still both ways, to turn volts
   into Nm.
