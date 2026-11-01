# 04 — Technical safety concept (ISO 26262-4, clause 6)

| | |
|---|---|
| Revision | 0.1 draft, 2026-11-01 |
| Based on | `03_FUNCTIONAL_SAFETY_CONCEPT.md`, FSR-01…16 |

## 1. Block diagram

**Design philosophy: deliberately overbuilt.** No single fault, and no single *latent + single* fault pair, may leave an overlay connected. This holds across a relay, a regulator, an MCU, a watchdog, an op-amp or a resistor. There are three independent ways to remove the overlay, and each can do it alone:

1. **CH-A**, the control MCU;
2. **CH-B**, the monitor MCU;
3. **CH-C**, an analog guard with no software at all.

```
 POWER — one regulator chain per channel, nothing shared except the protected battery input
 VBAT ─ F1 blade ─ SQJ147ELP rev-FET ─ SMBJ20A ─ hold-up ─ VBAT_P ─┬─ LMR38020-Q1 (A) ─ +5V0_A ─ TPS7B81-Q1 ─ +3V3_A ─ CH-A
                                                                   ├─ LMR38020-Q1 (B) ─ +5V0_B ─ TPS7B81-Q1 ─ +3V3_B ─ CH-B
                                                                   └─ +5V0_A → overlay op-amps;  +5V0_B → CH-C guard comparators
 TPS3702-Q1 window on +5V0_A, +3V3_A → PG_A        TPS3702-Q1 window on +5V0_B, +3V3_B → PG_B
 TPS3850-Q1 window WDT + 3V3 OV/UV on CH-A → WDO_A   TPS3850-Q1 window WDT + 3V3 OV/UV on CH-B → WDO_B

 ENABLES — each relay coil needs a high side from one channel AND a low side from the other
 EN_A_HW = GPIO_EN_A · WDO_A · PG_A · GUARD_OK            (SN74LVC1G11-Q1 + guard) → K1 high side (TPS1H100 A), K2 high side (TPS1H100 B)
 EN_B_HW = PUMP(PWM_EN_B) · WDO_B · PG_B · GUARD_OK       (dynamic)                → K1 low side, K2 low side
 GUARD_OK (CH-C, latching until key-off) also drives its own FET in series with both coils.

 SIGNAL PATH — two changeover relays in cascade; the overlay reaches the EPS only if BOTH are energized
                              ┌───────────────── NC ┐                  ┌─────────── NC ┐
  Torque sensor VT3 ─┬────────┤                     ├ K1a COM ── NO ───┤               ├ K2a COM ─┬─ hold C ─ EPS B10
                     │        └ overlay VT3 ── NO ──┘                  └ sensor VT3 ───┘          └─ taps → CH-A, CH-B, CH-C
                     ├─ 1 MΩ ─ buffer ─ summer(+Δ) = overlay VT3
                     └─ 1 MΩ taps → CH-A, CH-B, CH-C (sensor side)
  (VT6 identical on K1b / K2b, summer(−Δ), EPS B8)

  CH-A DACA ─ RC slew limit ─ Δ gain (sets Δmax) ─┬─► +Δ to VT3 summer
                                                  └─ inverter ─► −Δ to VT6 summer

  CH-A ⇄ FSI (CRC, counter) ⇄ CH-B         CAN: TCAN3404-Q1 (CH-A)  +  TCAN3404-Q1 (CH-B), same bus
  VSP / NEP / WLP / RT+ / IG1 / disengage (2 contacts) → separate dividers per channel
```

**Relays.** K1 and K2 are DPDT signal relays with gold-clad contacts for dry switching; contact **a** = VT3, **b** = VT6. The EPS sees the overlay only when K1 **and** K2 are both energized. If either drops, the EPS gets the raw sensor. A welded contact in one relay is therefore still covered by the other relay, and SM-08 / SM-09 detect it.

**CH-C analog guard.** Window comparators (TLV1704-Q1 class) on +5V0_B compare EPS-side against sensor-side for VT3 and for VT6. They trip if either:

- |VT_out − VT_raw| > Δmax + margin, or
- |(VT3_out + VT6_out) − (VT3_raw + VT6_raw)| > ε_sum.

A trip latches GUARD_OK low (a flip-flop or SR latch reset only by key-off) and cuts every coil path. CH-C has no software, clock or firmware, so it is independent of the MCUs' systematic faults.

## 2. Hardware safety mechanisms

| SM | Mechanism | Covers | Implements |
|---|---|---|---|
| SM-01 | **NC relay contacts restore the OEM path** whenever the coil is not energized | Every MCU fault, power loss | FSR-01 |
| SM-02 | **Series high-side (CH-A) + low-side (CH-B) coil switches.** Either one de-energizes K1 | 1oo2 shutdown | FSR-01, -09 |
| SM-03 | **Hardware AND of each channel's enable with its window watchdog and supply-good** | MCU hang, runaway, early or late service, rail fault | FSR-10, -11 |
| SM-04 | **Dynamic enable on CH-B.** The FET gate is fed by a charge pump from a PWM, so a stuck-high or stuck-low pin gives OFF | GPIO stuck-at faults | FSR-01 |
| SM-05 | **Analog authority limit.** Δ = k·(V_DAC − V_mid). The DAC can't leave 0–3.3 V, so \|Δ\| ≤ 1.65·k = Δmax | Any software or MCU error | FSR-02 |
| SM-06 | **Single DAC with a hardware inverter for VT6.** Antisymmetry comes from the circuit, not from two software paths | Asymmetric offset that would trip the EPS average check | FSR-03 |
| SM-07 | **RC slew limit on Δ** (τ TBD-DATA, about 10 ms) | Steps and oscillation | FSR-05 |
| SM-08 | **EPS-side readback on both channels.** Checks EPS-side − sensor-side = commanded Δ ± ε, and that VT3_out + VT6_out = VT3_raw + VT6_raw | Op-amp, resistor, DAC and relay faults, including a welded contact | FSR-03, -04 |
| SM-09 | **Relay proof-test at key-on.** Energize with Δ = 0, confirm both contacts moved by readback with a test offset in a safe window, then release. Also checks coil current (TPS1H100 sense) and the low-side node | Latent welded or open relay, coil-driver faults | FSR-12 |
| SM-10 | **1 MΩ isolation on every sensor-side tap, plus power-off-safe op-amp inputs** | Loading the OEM signal when Icarus is unpowered or faulty | FSR-14 |
| SM-11 | **Hold capacitor at relay COM** (value TBD, ≤ 10 nF, verify against EPS input filtering) | Floating input during the break-before-make gap | FSR-13 |
| SM-12 | **Fast-release coil clamp** (diode + Zener, not a bare diode) | Long relay drop-out time | FTTI |
| SM-13 | **Overvoltage and reverse protection** (TVS, rev-FET), plus TPS3702 OV windows | Load dump, reverse battery, regulator OV | FSR-11 |
| SM-14 | **Two cascaded changeover relays (K1, K2).** The overlay needs both energized; each coil is cross-driven (high side from one channel, low side from the other) | A welded relay contact, a single coil-driver short | FSR-01 |
| SM-15 | **CH-C analog guard.** Window comparators on the out − raw difference and on the sum, latched trip, own supply (+5V0_B), cuts all coils | Errors in both MCUs at once, including common-cause software/silicon faults | FSR-02, -03, -04 |
| SM-16 | **Separate regulator chain per channel** (buck + LDO + supervisor) | Common-cause supply over- or undervoltage | FSR-11, DFA |
| SM-17 | **Guard self-test at key-on.** CH-A injects a test Δ above the guard threshold with K2 released (EPS still on raw sensor); CH-B confirms GUARD_OK tripped; a key-off/on resets it (or a test-reset path usable only in INIT) | Latent CH-C failure | LFM |

## 3. Software safety mechanisms (both channels unless noted)

| SM | Mechanism | Channel |
|---|---|---|
| SW-01 | E2E check on commands: CRC-8/16, rolling counter, 50 ms timeout, range and rate check | A, B |
| SW-02 | Engage envelope: VSP speed and CAN speed agree within ±10 %, inside the allowed window; EPS lamp OFF; RT+ ON; Vcc2 4.75–5.25 V; IG1 ON; no latched fault | A, B |
| SW-03 | Driver override: \|VT3_raw − VT6_raw\| converted to driver torque > T_override (TBD-DATA) → ramp-out | A, B |
| SW-04 | Cross-channel check over FSI: state, Δ command, sequence counter, CRC. The monitor recomputes the allowed Δ envelope, not the control law (diverse) | A, B |
| SW-05 | Readback comparison (SM-08) at ≥ 1 kHz. Mismatch debounce ≤ 5 ms | A, B |
| SW-06 | MCU self-tests per the F28004x safety manual: flash ECC, RAM ECC/parity, CPU and PIE tests, DCC clock check, missing-clock detect, ADC self-test (VREFLO/VREFHI conversions), DAC loop-back | A, B |
| SW-07 | Patterns from the throttle ECU's `safety.c`: POST RAM march, config range check, RAM canary, fault bitmap, latched safe state | A, B |
| SW-08 | Watchdog serviced only from the end of the verified main cycle (window), never from an ISR | A, B |

## 4. Fault-reaction timing (target)

| Step | Time |
|---|---|
| Fault detection (1 kHz checks, ≤ 5 ms debounce) | ≤ 10 ms |
| Enable removal (GPIO → AND gate / charge-pump collapse) | ≤ 1 ms |
| Relay release with Zener clamp | ≤ 5 ms (check against the datasheet) |
| CH-C guard trip path (comparator → latch → coil FET), in parallel with the above | < 1 ms + relay release |
| **Total fault-reaction time** | **≤ 20 ms**, against an FTTI of ≥ 100 ms (TBD-DATA) |

Watchdog-detected faults depend on the watchdog window. TPS3850-Q1 preset (CWD 10 kΩ to VDD, SET0 = SET1 = 1): 1.85 ms closed window, 11 ms timeout. Service every ~5 ms.

## 5. Hardware metric targets (ISO 26262-5, for SG1/SG2 at ASIL D)

| Metric | Target | How |
|---|---|---|
| SPFM | ≥ 99 % | FMEDA after the schematic. SM-01…SM-09 are the main coverage |
| LFM | ≥ 90 % | Key-on proof-test (SM-09), POST (SW-06/07) |
| PMHF | < 10 FIT | FMEDA with failure rates from SN 29500 / IEC 62380 or the vendors' FIT data |

## 6. Dependent-failure analysis (initial)

| Coupling factor | Risk | Measure |
|---|---|---|
| Shared protected battery input | Load dump or reverse battery hits both chains | TVS, rev-FET, fuse. Beyond that point each channel has its own buck, LDO and supervisor (SM-16). Losing either chain drops its enables, which is safe |
| Same MCU type (systematic) | Common silicon or compiler bug | Different clock sources (CH-A external SiT2024, CH-B internal INTOSC2); diverse monitor algorithm (envelope check vs control law); separate builds and memory maps; use only TI-qualified tools |
| Shared CAN bus | Corrupt or replayed commands | E2E protection; each channel has its own transceiver; the monitor validates independently |
| Shared PCB and connector | Contamination, a cracked board | Conformal coat, creepage between the VBAT and analog areas, connector with locking and seals |
| Same relay type for K1 and K2 | Both weld from the same overload or contamination | Dry-circuit switching only (µA, no load to weld); key-on proof-test of each relay on its own (SM-09); consider two different relay vendors |
| Same MCU firmware defect in CH-A and CH-B | Both accept a bad overlay | CH-C guard (SM-15) has no software |
| EMC | Both channels upset together | Watchdog windows, E2E, readback, layout rules carried over from the throttle ECU (`LAYOUT.md`) |

## 7. Open technical decisions

1. **Δmax and k (TBD-DATA).** Map VT3/VT6 against steering torque with the logger, then pick a Δmax that a 5th-percentile driver can overpower at all speeds.
2. **Hold-capacitor value and relay gap (A2).** Bench-test with the logger before freezing.
3. **Relay part.** It must have a coil-suppression option, a dry-circuit rating, and automotive qualification. Candidates to check: Panasonic TX2/TQ2 series, Omron G6K-2.
4. **Interception point.** Recommended: a Y-harness at EPS connector B in the cabin. The alternative is the gearbox 7P connector, which is wet and hot, so not recommended.
5. **Second F280049C vs a diverse monitor MCU.** Using the same part keeps parts and tools common, and its weaker independence is covered by §6. Revisit after the FMEDA.
