# 01 — Item definition (ISO 26262-3, clause 5)

| | |
|---|---|
| Item | **Icarus**: steering torque-overlay unit for the 1st-gen Honda Insight (ZE1) EPS |
| Revision | 0.1 draft, 2026-11-01 |
| Status | Concept. No hardware built. |

## 1. Purpose and function

Icarus lets an external planner (lane keeping / automated steering, issued over CAN) add a **bounded steering torque** on top of the driver's input. It does this by shifting the torque-sensor voltages that the OEM EPS control unit reads. The EPS then applies assist as if the driver had applied that torque.

| ID | Function | Notes |
|---|---|---|
| F1 | **Pass-through.** The OEM torque sensor connects directly to the EPS. | Default and safe state; hardware NC relay contacts, no power needed |
| F2 | **Torque overlay.** EPS inputs = sensor signals + antisymmetric offset ±Δ. | Δ is commanded over CAN, limited in software *and* by analog hardware |
| F3 | **Driver override.** Release when driver torque > threshold, or on the disengage switch. | Ramps out, then pass-through |
| F4 | **Monitoring.** Two independent channels check commands, signals, EPS state, speed and each other. | Either channel can force F1 |
| F5 | **Status reporting** on CAN, plus a driver indicator output. | Engaged / available / fault |
| F6 | **Diagnostics.** POST, relay proof-test at key-on, fault memory. | |

Not in scope: driving the EPS motor, changing the EPS ECU firmware, and the planner (lane detection, path planning). The planner is an external item whose output is treated as untrusted.

## 2. Item boundary and interfaces

```
                    ┌──────────────────────── Icarus ────────────────────────┐
 Torque sensor ────►│ VT3/VT6 raw in ──► overlay path ──► relay ──► VT3/VT6 out│────► EPS ECU (B10/B8)
 (gearbox, 7P)      │                         ▲            │ NC = straight     │
                    │  Vcc1, Vcc2, T/S GND, SG pass straight through (never cut)│◄──── EPS ECU (B1/B2/B9/B7)
 EPS ECU WLP, RT+ ─►│ status taps (high impedance)                              │
 VSS / ECM ────────►│ VSP, NEP taps (high impedance, no load on shared lines)   │
 Battery, IG1 ─────►│ power, ignition                                           │
 Vehicle CAN ◄─────►│ commands in, status out, SAS angle in                     │
 Disengage switch ─►│ driver input (dual contact)                               │
                    └──────────────────────────────────────────────────────────┘
```

| Interface | Signal | Level / type | Direction | Source |
|---|---|---|---|---|
| I1 | VT3, VT6 (sensor side) | about 1.5–3.5 V analog, moving opposite ways around about 2.5 V (to verify) | In | Torque sensor |
| I2 | VT3, VT6 (EPS side) | Same range | Out | Icarus → EPS ECU B10/B8 |
| I3 | Vcc1 (12 V class), Vcc2 (5 V), T/S GND, SG | Pass-through only; Icarus senses Vcc2 | Through | EPS ECU |
| I4 | WLP (lamp, low = ON), RT+ (relay drive) | 0 / 12 V | In (tap) | EPS ECU B13/B11 |
| I5 | VSP | 0–5 V open-collector pulses, about 4,450 pulses/mile (unverified) | In (tap) | VSS, circuit E93 |
| I6 | NEP | about 12 V open-collector pulses, 1.5 pulses/engine rev | In (tap) | ECM A19, circuit F44 |
| I7 | +B, GND, IG1 | 12 V automotive, ISO 16750-2 profiles | In | Fused feed |
| I8 | CAN | ISO 11898-2, 500 kbit/s | Both | Planner, SAS (Toyota, ID 0x25) |
| I9 | Disengage switch | Two contacts (NO + NC) for plausibility | In | Driver |
| I10 | Status indicator | Low-side driver | Out | Driver display |

Pin numbers, wire colours and expected voltages: `../../data_capture/EPS_PINOUT.md`.

**Physical location:** behind the right kick panel (dry cabin), wired as a Y-harness at EPS connector B. The torque-sensor lines are routed through Icarus; every other pin of connector B passes straight through.

## 3. Operating modes

| Mode | Relay | Overlay | Entered when |
|---|---|---|---|
| OFF | De-energized (pass-through) | — | No power / IG1 off |
| INIT | Pass-through | — | Key-on: POST, relay proof-test, cross-check |
| STANDBY | Pass-through | — | All checks OK, no valid engage request |
| ENGAGED | Energized (overlay path) | Δ ≤ limit | Valid request, both channels agree, preconditions met |
| RAMP-OUT | Energized → pass-through | Δ ramps to 0, then relay drops | Driver override, request withdrawn, non-critical fault |
| SAFE | Pass-through, latched for the ignition cycle | — | Critical fault in either channel |

## 4. Assumptions (to be verified; each one is a requirement on the vehicle or the user)

| ID | Assumption | Verify by |
|---|---|---|
| A1 | The EPS reads VT3/VT6 as a differential torque, and its DTC 16 checks their **average**. Antisymmetric ±Δ keeps the average unchanged. | Logger capture of VT3/VT6 while steering |
| A2 | The EPS input filtering tolerates the relay's break-before-make gap (≤ 1–3 ms) without setting a DTC, with the hold capacitor fitted. | Bench test with the logger and repeated relay cycling |
| A3 | Driver torque can overpower the maximum overlay at all speeds. | Measured steering-wheel force at the hardware limit |
| A4 | Planner commands are untrusted (QM). Icarus provides all of the safety integrity. | Design rule |
| A5 | The EPS ECU itself is unmodified and keeps its own protections (thermal derating, lamp, DTCs). | — |
| A6 | Driver supervision is required at all times (SAE level 2, hands-on). | Operating rules and driver monitoring |
| A7 | Operating temperature −40 to +85 °C (cabin). | Part selection: AEC-Q100 grade 1 or better |

## 5. Legal and operating constraints

- Development and testing happen on private ground.
- Public-road use of a modified steering system is outside the scope of this engineering work, and is the owner's responsibility under local law.
