# 05 — Hardware basis: parts and resources

| | |
|---|---|
| Revision | 0.1 draft, 2026-11-01 |
| Reference board | Throttle ECU **Tasaru V0.0.1** (`f411testingreliablitymotor/c2000_f280049_throttle/throttleecupcb/TasaruV0.0.1`), read-only |

## 1. Parts list (block level)

| Block | Part | Qty | Reused from throttle? | Notes |
|---|---|---|---|---|
| MCU CH-A | **F280049CPZQR** (TMS320F280049C-Q1, LQFP-100 PZ) | 1 | Same die and footprint; the throttle uses F280049CPZS | The **PZQR** suffix is the TÜV-listed functional-safety part (datasheet Table 3-1). PZS is quality-managed only |
| MCU CH-B (rev 0.7) | **TMS5700432BPZQQ1** (TMS570LS0432-Q1, LQFP-100 PZ, 14 × 14 mm) | 1 | No: diverse on purpose | Arm Cortex-R4F, dual core in lockstep, ECC flash/RAM, CPU/RAM self-test, ESM nERROR. Core 1.2 V (1.14–1.32 V, ≤ 135 mA, 145 mA in LBIST), I/O + flash + ADC 3.3 V. No DAC, no FSI. Pins from SPNS186C |
| CH-B core supply (rev 0.7) | TLV73312PQDRVRQ1 (TLV733P-Q1, WSON-6) from +5V0_B, + TPS3702CX12QDDCRQ1 into nPORRST | 2 | No | DRV pinout differs from the SOT-23 one (OUT 1, GND 3, EN 4, IN 6). 0.55 W worst case in the thermal pad |
| Clock CH-A | SiT2024B 10 MHz MEMS | 1 | Yes (U6) | CH-B (rev 0.7) uses a 16 MHz quartz crystal, load caps to KELVIN_GND only: different clock technology |
| CAN | TCAN3404-Q1 (SOIC-8) | 2 | Yes (U7) | One per channel, same bus. 120 Ω termination by solder jumper (as throttle R27/JP) |
| CAN ESD | ESD2CAN24-Q1 or similar | 2 | New | Throttle has none on CAN |
| Buck | LMR38020-Q1 set to **5.0 V** + 10 µH | 2 | Yes (U5, L1), output changed from 3.3 V | One per channel (SM-16). A feeds the overlay op-amps; B feeds the CH-C guard |
| LDO | **TPS7B8233-Q1**, 3.3 V, 300 mA, WSON-6 2×2 mm (DRV) | 2 | New | TPS7B8133-Q1 (150 mA) was too tight: F280049C IDDIO max is 113 mA (SPRS945) plus TCAN3404 and the clock. Same DRV pinout |
| Supervisor | TPS3702CX50 (5 V window, SET = VDD → 4.80 / 5.20 V) | 2 | Yes (U9), ×2 | One per channel. **Correction (rev 0.6):** an AEC-Q100 TPS3702-Q1 does exist (SBVS261C; TPS3702CX50QDDCRQ1, same DDC package and pinout). Now used for all three |
| Watchdog | TPS3850G33-Q1 (window WDT + 3V3 OV/UV supervisor, VSON-10) | 2 | New | One per MCU. TPS3851-Q1 was rejected: timeout-only, it cannot catch early service |
| Enable logic | SN74LVC1G11-Q1 (3-input AND) ×2, SN74LVC1G08-Q1 (2-input AND) ×1 | 3 | New | 1G11: GPIO · ERRORSTS · SUP_OK per channel. 1G08: 2oo2 clear of the guard latch (R7) |
| Dynamic enable | Charge pump (2× BAS21 + 2 caps) → N-MOSFET | 2 | BAS21, MMBT2222A/N-FET reused | CH-B low side, one for each coil |
| Coil high side | TPS1H100-Q1 | 2 | Yes (U2), ×2 | One per coil, driven by CH-A. Current sense to CH-A ADC |
| Guard coil FET | Automotive N-MOSFET, logic level | 1 | New | CH-C, in series with both coil returns |
| Relays K1, K2 | DPDT signal relay, 12 V coil, gold contacts | 2 | New | Cascaded (SM-14). Consider different vendors for K1 and K2. See TSC §7.3 |
| Coil clamp | Diode + 18 V Zener (SMBJ / BZX84-Q) | 2 sets | Partly | Fast release |
| CH-C guard | TLV1704-Q1 quad comparator ×2, window references, SR latch (SN74LVC1G74-Q1 type), key-off reset | 1 set | New | No software (SM-15) |
| Overlay analog | TLV9062-Q1 / TLV9064-Q1, 5 V RRIO | 2–3 | New | Buffers, summers, Δ inverter. Power-off-safe inputs behind 1 MΩ |
| Reverse battery | SQJ147ELP P-FET | 1 | Yes (U1) | |
| TVS | SMBJ20A | 1 | Yes (D2) | |
| Fuse | Mini blade 5 A + PTC | 1+1 | Yes (F2, F1) | Icarus load is about 200 mA plus the coil, so a lower rating than throttle |
| Hold-up | 470–2200 µF electrolytic | 1 | Yes (C22 type) | Size it for the IMA auto-stop restart dip |
| Input dividers | 100 k / 33 k (12 V), 100 k / 100 k (5 V), 1 MΩ VT taps | — | Values from the EPS tap wing (JLC Basic) | VSP, NEP, WLP, RT+, IG1, disengage ×2 |
| Debug | 2× 2×5 JTAG headers, UART pads | 2 | Yes (J1 style) | One per MCU |
| Connector | TE family as throttle J2 (173862-1), pin count TBD; vehicle side: Y-harness with Honda connector-B halves | 1 | Family reused | Mating part numbers TBD |
| Passives | 0.1 µF / 2.2 µF / 10 µF / 22 µF MLCC; 0 Ω / 1 k / 2.2 k / 4.7 k / 10 k / 100 k resistors | — | Yes | Same value strategy as throttle HANDOFF §4 |

## 2. Rails

| Rail | Source | Load | Supervised by |
|---|---|---|---|
| VBAT_P | Fuse → rev-FET → TVS | Coil (about 30–60 mA), buck | TPS1H100 sense, CH-A/CH-B ADC dividers |
| +5V0_A | LMR38020-Q1 (A) | LDO A, overlay op-amps | TPS3702 A (OV and UV) |
| +5V0_B | LMR38020-Q1 (B) | LDO B, CH-C guard comparators and latch | TPS3702 B (OV and UV) |
| +3V3_A | TPS7B81-Q1 (A), from +5V0_A | CH-A MCU, SiT2024, TCAN3404 A, TPS3850 A | TPS3850 A (3V3), TPS3702 A (5V0) |
| +3V3_B | TPS7B81-Q1 (B), from +5V0_B | CH-B MCU, TCAN3404 B, TPS3850 B | TPS3850 B (3V3), TPS3702 B (5V0) |
| VDDA (each MCU) | 3V3_x through a ferrite (as throttle FB1 60 Ω) | ADC, DAC | Internal ADC self-test |

Check the TPS7B81-Q1 current rating against the F280049C datasheet's worst-case IDD at 100 MHz with all peripherals on. If the margin is thin, step up to a 300 mA automotive LDO.

## 3. MCU resource budget (per channel)

| Resource | CH-A (control) | CH-B (monitor) |
|---|---|---|
| ADC inputs | VT3/VT6 raw, VT3/VT6 out, Δ readback, TPS1H100 sense, 5V0, 3V3_A, VBAT, Vcc2 = **10** | VT3/VT6 raw, VT3/VT6 out, Δ readback, coil low-side node, 5V0, 3V3_B, VBAT, Vcc2 = **10** |
| DAC | DACA → Δ (DACB spare, or test signal for the proof-test) | DACs unused (CH-B never drives Δ) |
| eCAP / XBAR inputs | VSP, NEP | VSP, NEP |
| GPIO in | WLP, RT+, IG1, disengage NO + NC, SUP_OK (WDO + OV/UV, read back) | Same set |
| GPIO out | EN_A, WDI_A, status LED | PWM_EN_B (charge pump), WDI_B, driver indicator (low side) |
| Comms | CANA (TCAN A), SCI-B ↔ CH-B (rev 0.7, GPIO12/13), SCI debug | DCAN1 (TCAN B), SCI/LIN ↔ CH-A (pins 94/95), cTI-20 JTAG |

The F28004x has 3 ADCs (more than 20 channels on the 100-pin package), 2 buffered DAC outputs, 2 CAN, 1 FSI TX + RX. Both channels fit with spare pins. Confirm the exact pin mux, and which pins DACA/DACB come out on, from the datasheet pin tables before the schematic.

## 4. Board

| Item | Value |
|---|---|
| Layers | 4, same stack-up as the throttle: F.Cu signals; In1 solid GND; In2 power; B.Cu signals |
| Layout zones | **Noisy:** connector, protection, buck, coil driver. **Quiet:** two MCU islands, one per channel, with the overlay analog block between them |
| Grounding | Single ground net, as on the throttle |
| Spacing | Creepage between VBAT_P / coil copper and the VT analog path ≥ 1 mm |
| Coating | Conformal coat after test |
| Target size | About 80 × 60 mm, to be confirmed after placement |
