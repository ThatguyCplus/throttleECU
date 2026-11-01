# Icarus — project brief (the improved prompt)

This is the original request rewritten as a brief that a designer (human or AI) can work from without guessing.

---

## Original request

> Start building out an ISO 26262 compliant, robust, double-monitored steering control PCB with safeties in place. It lives in a subfolder titled Icarus. Look at, but don't change, the throttle ECU folder. The MCU, CAN controller and most parts should be similar to the throttle ECU (F280049C).

## Improved brief

**Goal.** Design **Icarus**, a steering control unit for the 1st-gen Honda Insight. It adds a commanded steering torque by **intercepting the OEM EPS torque-sensor signals** (VT3/VT6). The OEM EPS control unit, motor and relay stay in charge of the motor. Icarus never drives the EPS motor itself.

**Safety intent.** Develop it along the ISO 26262 lifecycle (Part 3 concept → Part 4 system → Part 5 hardware → Part 6 software), targeting **ASIL D for the top safety goal** through **ASIL decomposition into two independent channels**: a control channel and a monitor channel. Each channel uses a TI F280049C. Its functional-safety-compliant part number is certified for hardware integrity up to ASIL B.

**Non-negotiable safety properties.**
1. **Fail-silent by hardware.** With no power, a reset, a watchdog fault, or *either* channel saying "stop", the OEM torque sensor connects straight through to the EPS. Icarus then has no electrical influence.
2. **2-out-of-2 to act, 1-out-of-2 to stop.** Injection needs both channels and both watchdogs to agree. Any single one can remove it.
3. **Bounded authority in analog hardware.** The largest torque offset Icarus can inject is set by resistors, not software. It must be small enough for any driver to overpower.
4. **Driver always wins.** Driver torque above a threshold, or any driver-disengage input, releases control within the fault-tolerant time.
5. **Never cause a latching EPS fault.** Injected signals keep the VT3/VT6 relationship the EPS checks (DTCs 11/15/16 latch until erased). Transitions must not leave the EPS inputs floating.

**Reuse from the throttle ECU (`f411testingreliablitymotor/c2000_f280049_throttle`, read-only).**
- MCU F280049C (upgraded to the **-Q1 functional-safety part, F280049CPZQR**, same PZ 100-pin package and footprint)
- CAN transceiver TCAN3404-Q1
- LMR38020-Q1 buck, TPS3702-Q1 supervisor, SiT2024B MEMS clock
- SQJ147ELP reverse-battery FET, SMBJ20A TVS, mini-blade fuse
- TPS1H100-Q1 high-side switch (here it drives the bypass-relay coil, with current-sense diagnostics)
- 0402/0603 passive values, 4-layer stack-up, TE connector family, JTAG header style
- The safety firmware patterns in `throttleECU/safety.c`: POST, RAM canary, fault bitmaps, safe-state entry

**Inputs Icarus uses.**
- OEM torque sensor VT3/VT6, raw.
- EPS-side VT3/VT6 readback: what the EPS actually sees.
- EPS Vcc1/Vcc2, so it knows the EPS is powered.
- EPS lamp WLP and motor relay RT+, so it knows the EPS state.
- VSP and NEP pulses, for a hardware speed source independent of CAN.
- CAN: commands from the planner, plus steering angle from the Toyota SAS already used in `steering_control_unit/toyota_sas`.
- IG1 ignition and a driver disengage switch.

Pinouts and voltage levels come from `steering_control_unit/data_capture/EPS_PINOUT.md`.

**What "done" looks like for the concept phase (this folder, now).**
- `01_ITEM_DEFINITION.md`: item boundary, functions, interfaces, assumptions
- `02_HARA.md`: hazards, situations, S/E/C, ASIL, safety goals
- `03_FUNCTIONAL_SAFETY_CONCEPT.md`: functional safety requirements, safe states, FTTI, decomposition
- `04_TECHNICAL_SAFETY_CONCEPT.md`: hardware/software architecture, safety mechanisms, diagnostic-coverage targets, dependent-failure notes
- `05_HARDWARE_BASIS.md`: block-level parts list (reused vs new), rails, MCU resource budget

**Next phases (not started).**
- KiCad schematic → FMEDA (SPFM/LFM/PMHF) → layout.
- Firmware for both channels.
- HIL bench using the data logger and the VSP/NEP generator.
- Vehicle validation on private ground.

**Constraints.**
- Don't modify anything in the throttle ECU folder.
- Don't order or publish anything without asking.
- Downloads need explicit permission.
- Calibration values that need real data (torque gradient, authority limit, FTTI) stay marked **TBD-DATA** until the logger has measured them.

**Honest limits.** A PCB and documents can't make a product "ISO 26262 compliant" by themselves. Compliance also needs:
- a safety plan and a confirmation-measure process (reviews, audit, assessment by someone independent of the design);
- tool qualification;
- a safety case.

This folder produces the engineering work products in a form that process can use. Public-road use of an aftermarket steering interceptor raises legal and liability questions separate from the engineering.
