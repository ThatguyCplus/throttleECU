# 05 — Hardware basis: parts and resources

| | |
|---|---|
| Revision | 0.1 draft, 2026-11-01 |
| Reference board | Throttle ECU **Tasaru V0.0.1** (`f411testingreliablitymotor/c2000_f280049_throttle/throttleecupcb/TasaruV0.0.1`), read-only |

## 1. Parts list (block level)

| Block | Part | Qty | Reused from throttle? | Notes |
|---|---|---|---|---|
| MCU CH-A, CH-B | **F280049CPZQR** (TMS320F280049C-Q1, LQFP-100 PZ) | 2 | Same die and footprint; the throttle uses F280049CPZS | The **PZQR** suffix is the TÜV-listed functional-safety part (datasheet Table 3-1). PZS is quality-managed only |
| Clock CH-A | SiT2024B 10 MHz MEMS | 1 | Yes (U6) | CH-B runs on internal INTOSC2 for clock diversity |
| CAN | TCAN3404-Q1 (SOIC-8) | 2 | Yes (U7) | One per channel, same bus. 120 Ω termination by solder jumper (as throttle R27/JP) |
| CAN ESD | ESD2CAN24-Q1 or similar | 2 | New | Throttle has none on CAN |
| Buck | LMR38020-Q1 set to **5.0 V** + 10 µH | 2 | Yes (U5, L1), output changed from 3.3 V | One per channel (SM-16). A feeds the overlay op-amps; B feeds the CH-C guard |
| LDO | TPS7B81-Q1, 3.3 V fixed | 2 | New | One per channel, for supply independence |
| Supervisor | TPS3702-Q1 window | 2 | Yes (U9), ×2 | One per channel: 5V0 + that channel's 3V3 |
| Watchdog | TPS3850G33-Q1 (window WDT + 3V3 OV/UV supervisor, VSON-10) | 2 | New | One per MCU. TPS3851-Q1 was rejected: timeout-only, it cannot catch early service |
| Enable logic | SN74LVC1G11-Q1 (3-input AND) | 1 | New | CH-A: GPIO · WDO_A · PG_A |
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
| Comms | CANA (TCAN A), FSI TX/RX ↔ CH-B, SCI debug | CANA (TCAN B), FSI TX/RX ↔ CH-A, SCI debug |

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
