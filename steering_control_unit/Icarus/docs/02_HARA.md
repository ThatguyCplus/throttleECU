# 02 — Hazard analysis and risk assessment (ISO 26262-3, clause 6)

| | |
|---|---|
| Revision | 0.1 draft, 2026-11-01 |
| Status | Preliminary. Needs review by someone independent of the design. |

**Rules followed:**
- Ratings assume the item has **no safety mechanisms of its own**. The HARA rates the function, not the design.
- Severity, exposure and controllability use the standard S0–S3, E0–E4 and C0–C3 classes. The ASIL comes from the standard's S×E×C table.
- Controllability values need confirming in vehicle tests. Each one marked **(C-test)** is a validation target.

## 1. Malfunctions considered

| ID | Malfunction at item level |
|---|---|
| M1 | Unintended overlay torque: wrong direction, too large, or with no request |
| M2 | Overlay torque that can't be overridden, or keeps going after driver override / disengage |
| M3 | Loss of OEM power assist caused by Icarus: open or invalid VT signals, or a latching EPS DTC |
| M4 | Unannounced loss of overlay while engaged: the planner believes it is steering but isn't |
| M5 | Overlay oscillating or too fast (high-frequency torque) |

## 2. Operational situations

| ID | Situation | Exposure basis |
|---|---|---|
| OS1 | Highway or rural road, > 50 mph, straight or gentle curves, other traffic present | Most driving time (E4) |
| OS2 | Urban, 15–50 mph, junctions, pedestrians | Frequent (E4) |
| OS3 | Tight curve or lane change at speed | Several times per drive (E3) |
| OS4 | Parking / manoeuvring, < 6 mph | Every drive, short duration (E3) |
| OS5 | Stationary, ignition ON (bench and garage tests) | E2 for road use |

## 3. Hazard ratings

| HE | Malfunction | Situation | Hazardous event | S | E | C | ASIL |
|---|---|---|---|---|---|---|---|
| HE1 | M1 | OS1 | Sudden steering pull at highway speed → lane departure into oncoming traffic or a barrier | S3 | E4 | C3 (C-test) | **D** |
| HE2 | M1 | OS2 | Steering pull in the city → collision with a vehicle or pedestrian | S3 | E4 | C2 (C-test) | **C** |
| HE3 | M1 | OS3 | Pull that adds to a curve or lane change → loss of control | S3 | E3 | C3 (C-test) | **C** |
| HE4 | M1 | OS4 | Unexpected wheel motion while parking | S1 | E3 | C1 | QM |
| HE5 | M2 | OS1 | Driver can't correct a wrong pull at speed | S3 | E4 | C3 | **D** |
| HE6 | M3 | OS3 | Sudden loss of assist mid-curve: heavy steering, driver under-steers | S2 | E3 | C2 | **A** |
| HE7 | M3 | OS4 | No assist while parking: heavy but controllable | S0–S1 | E3 | C1 | QM |
| HE8 | M4 | OS1 | Planner silently stops steering, and the hands-on driver must take over | S3 | E4 | C1 (hands-on, A6) | **B** |
| HE9 | M5 | OS1 | Oscillating torque: wheel shake and a distracted or startled driver | S2 | E4 | C2 | **B** |

**Notes:**
- HE8 is rated with a hands-on driver (assumption A6). If the system were ever used hands-off, C becomes C3 and the ASIL becomes D.
- HE1 and HE5 lead to the top ASIL. The **hardware authority limit** in the technical concept is what later makes the C-test argument convincing. The ASIL isn't lowered by it; the HARA stays independent of the item's own mechanisms.

## 4. Safety goals

| SG | Safety goal | ASIL | Safe state | FTTI |
|---|---|---|---|---|
| **SG1** | Prevent overlay torque that the driver did not or could not expect, in direction, size or timing (HE1–HE3) | **D** | Pass-through (overlay electrically removed) | **TBD-DATA**, placeholder 100 ms |
| **SG2** | Ensure the driver can always override, and the overlay ends on override or disengage (HE5) | **D** | Pass-through | TBD-DATA, placeholder 100 ms |
| **SG3** | Prevent loss of OEM assist caused by Icarus (HE6) | **A** | Pass-through *with valid signals* (no open input, no latching DTC) | Must not happen at all, and set no DTC |
| **SG4** | Prevent unannounced loss of overlay while engaged (HE8) | **B** | Ramp-out **plus** a driver warning within 200 ms | TBD-DATA |
| **SG5** | Prevent oscillating or excessively fast overlay torque (HE9) | **B** | Pass-through | TBD-DATA |

**Deriving the FTTI (TBD-DATA):**
1. Measure steering-wheel torque against EPS output at the hardware limit Δmax.
2. Model the vehicle's lateral response at 70 mph.
3. FTTI = the time until lateral offset passes 0.3 m (half the margin inside a lane), minus driver reaction allowance.

Expect it to land at 100–300 ms. Icarus targets a fault-reaction time of **≤ 20 ms** whatever the result, so the margin stays wide.
