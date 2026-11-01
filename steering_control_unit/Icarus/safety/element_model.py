"""Icarus functional safety element model (rev 0.1).

The item is broken into hardware elements. Each element has:
- its parts, pulled from ../hardware/gen/design.py so part counts follow the schematic;
- failure modes, with a share of the element failure rate;
- the effect of each mode on SG1 (unintended or out-of-envelope overlay torque at the EPS);
- the safety mechanisms that detect or control it.

analyze.py turns this into the FMEDA (SPFM/LFM/PMHF) and the fault-tree cut sets.

Failure rates are PLACEHOLDERS: typical generic values for automotive parts at a
cabin mission profile. Replace them with SN 29500 / IEC 61709 calculations or vendor
FIT data before using any metric for a safety case.
Diagnostic-coverage claims follow the low / medium / high (60 / 90 / 99 %) convention
of ISO 26262-5 Annex D. Each one needs a justification (test or analysis) later.

Effect classes, relative to SG1:
  direct  violates SG1 on its own unless a safety mechanism detects it and reacts within the FTTI
  latent  can't violate SG1 on its own, but removes a barrier, so a second fault could
  safe    no SG1 contribution. It may still matter for SG3 (loss of assist) or SG4 (silent dropout)
"""

MISSION_HOURS = 10_000        # vehicle operating lifetime assumed for latent-fault exposure
KEY_CYCLE_HOURS = 2           # interval between key-on tests (proof test, guard self-test)

# --- safety mechanisms: id -> (description, DC, kind) -------------------------
# kind "detect": detects a direct fault and reaches pass-through within the fault-reaction time
# kind "latent": test that finds a latent fault before it can combine with a second one
SM = {
    "SM-08A": ("CH-A readback: EPS-side minus sensor-side vs commanded Delta, sum check", 0.99, "detect"),
    "SM-08B": ("CH-B readback via its own buffers and diff amps (D3, D6, sum)", 0.99, "detect"),
    "SM-15": ("CH-C analog guard: window comparators, latched, cuts coil return", 0.99, "detect"),
    "SW-04": ("CH-B independently checks the CAN command against CH-A's Delta over FSI", 0.99, "detect"),
    "SW-01": ("E2E on CAN commands (CRC, counter, timeout), checked in both channels", 0.99, "detect"),
    "SM-03": ("Window watchdog + SUP_OK + ERRORSTS gate the channel enable", 0.90, "detect"),
    "SM-16": ("Per-channel OV/UV supervisors (TPS3702, TPS3850 SENSE)", 0.90, "detect"),
    "SM-20": ("Guard D3/D6 windows referenced to CH-B's own DAC output of the expected Delta (R2): "
              "CH-C catches a wrong CH-A command in hardware", 0.90, "detect"),
    "SM-09": ("Key-on relay proof-test: each relay alone, both contacts, by readback", 0.99, "latent"),
    "SM-17": ("Key-on guard self-test: forced trip via GUARD_TEST, latch + GNODE readback", 0.90, "latent"),
    "SW-04L": ("Cross-channel plausibility of readbacks (A vs B every cycle)", 0.90, "latent"),
    "SW-06": ("MCU self-tests per F28004x safety manual (ECC, CPU, DCC, ADC, DAC loop-back)", 0.90, "latent"),
    "SM-03L": ("Key-on watchdog test (deliberate missed service) + ENHW/GATE monitors", 0.90, "latent"),
    "SM-19": ("Key-on coil-driver test: each high side, low side and guard FET switched alone, "
              "coil nodes read back (CS, LSN, GNODE)", 0.90, "latent"),
}


def FM(name, share, effect, sms=(), defeats=(), note=""):
    return dict(name=name, share=share, effect=effect, sms=list(sms), defeats=list(defeats), note=note)


# --- elements -----------------------------------------------------------------
# select: list of (sheet, group prefix) from design.py; libs_only / libs_skip filter by symbol lib id
ELEMENTS = [
    dict(id="E01", name="Input protection (shared)", ch="shared", select=[("power", "Input protection")],
         fms=[FM("Open / fuse blown / FET open: no power", 0.6, "safe", note="Pass-through (SM-01)"),
              FM("TVS or FET short: fuse blows", 0.3, "safe"),
              FM("TVS open (latent loss of load-dump clamp)", 0.1, "safe",
                 note="Damage from an unclamped dump fails toward power loss; check in DFA")]),
    dict(id="E02", name="Supply chain A (buck, LDO, 5 V window)", ch="A", select=[("power", "Supply chain A")],
         fms=[FM("Output lost or low", 0.6, "safe", note="Enable drops: pass-through"),
              FM("Output overvoltage", 0.1, "direct", ["SM-16", "SM-08B", "SM-15"],
                 note="Could corrupt the CH-A overlay; B and C are on the other chain"),
              FM("Ripple / noise into the overlay path", 0.1, "direct", ["SM-08A", "SM-08B", "SM-15"]),
              FM("Supervisor fails to flag OV/UV", 0.2, "latent", ["SW-06"], defeats=["SM-16"])]),
    dict(id="E03", name="Supply chain B (buck, LDO, 5 V window)", ch="B", select=[("power", "Supply chain B")],
         fms=[FM("Output lost or low", 0.6, "safe"),
              FM("Output overvoltage / noise: corrupts CH-B or the guard", 0.2, "latent", ["SW-04L", "SM-17"],
                 defeats=["SM-08B", "SM-15", "SM-20"]),
              FM("Supervisor fails to flag OV/UV", 0.2, "latent", ["SW-04L"], defeats=["SM-16"])]),
    dict(id="E04", name="CH-A MCU, clock, decoupling", ch="A",
         select=[("ch_a", "MCU A"), ("ch_a", "MCU A supplies")],
         extra=[("ch_a", "FSI to CH-B", "icarus:SIT2024B")],
         fms=[FM("Wrong Delta command (within the hardware envelope)", 0.3, "direct", ["SW-04", "SM-20"],
                 note="Uncommanded pull, bounded by SM-05"),
              FM("Hang / no output", 0.3, "safe", note="Watchdog drops the enable"),
              FM("Enable GPIO stuck active", 0.1, "latent", ["SM-03L"], defeats=["SM-03"]),
              FM("Readback ADC path fault masks a real error", 0.1, "latent", ["SW-04L", "SW-06"], defeats=["SM-08A"]),
              FM("No effect on SG1 (debug, unused peripherals)", 0.2, "safe")]),
    dict(id="E05", name="CH-A supervision (TPS3850, AND gate, pull-ups)", ch="A",
         select=[("ch_a", "Reset, window watchdog A")],
         fms=[FM("Fails to drop the enable", 0.4, "latent", ["SM-03L"], defeats=["SM-03"]),
              FM("Spurious disable / reset", 0.4, "safe"),
              FM("Other", 0.2, "safe")]),
    dict(id="E06", name="CH-B MCU and decoupling", ch="B",
         select=[("ch_b", "MCU B"), ("ch_b", "MCU B supplies")],
         fms=[FM("Monitor accepts a bad overlay (silent pass)", 0.3, "latent", ["SW-04L", "SW-06"],
                 defeats=["SM-08B", "SW-04"]),
              FM("Hang: PWM stops, low side opens", 0.3, "safe"),
              FM("False trip", 0.1, "safe"),
              FM("No effect on SG1", 0.3, "safe")]),
    dict(id="E07", name="CH-B supervision + charge-pump enable", ch="B",
         select=[("ch_b", "Reset, window watchdog B"), ("ch_b", "Dynamic enable B")],
         fms=[FM("Low-side gate held on without PWM (pump diode short + leakage)", 0.2, "latent", ["SM-19"],
                 note="One of three series coil switches"),
              FM("Watchdog / gate fails to drop the enable", 0.3, "latent", ["SM-03L"], defeats=["SM-03"]),
              FM("Spurious disable", 0.4, "safe"),
              FM("Other", 0.1, "safe")]),
    dict(id="E08", name="Overlay analog path (Delta, inverter, summers)", ch="A",
         select=[("overlay", "References and Delta"), ("overlay", "Summers")],
         fms=[FM("Overlay wrong or beyond Delta_max at the EPS (op-amp rail, resistor open, attenuator open)",
                 0.5, "direct", ["SM-08A", "SM-08B", "SM-15"]),
              FM("Overlay lost / stuck at raw sensor", 0.2, "safe", note="SG4: detected by readback"),
              FM("No effect", 0.3, "safe")]),
    dict(id="E09", name="CH-A readback taps and scaling", ch="A", select=[("overlay", "EPS-side readback")],
         fms=[FM("Reading wrong: false trip", 0.5, "safe"),
              FM("Reading plausibly wrong: masks a real error", 0.2, "latent", ["SW-04L"], defeats=["SM-08A"]),
              FM("Other", 0.3, "safe")]),
    dict(id="E10", name="CH-C guard (buffers, diff amps, ladder, comparators, latch)", ch="C",
         select=[("guard", "Buffers"), ("guard", "Difference amps"), ("guard", "Window thresholds"),
                 ("guard", "Trip latch")],
         fms=[FM("Guard cannot trip", 0.4, "latent", ["SM-17"], defeats=["SM-15"]),
              FM("Spurious trip", 0.4, "safe"),
              FM("Other", 0.2, "safe")]),
    dict(id="E19", name="Expected-Delta reference (CH-B DAC -> REF3/REF6, R2)", ch="B",
         select=[("guard", "Expected-Delta reference")],
         fms=[FM("Reference wrong: guard false-trips", 0.5, "safe"),
              FM("Reference follows a wrong CH-A Delta (masks it)", 0.1, "latent", ["SW-04L"], defeats=["SM-20"],
                 note="Needs CH-B to compute the same wrong value; CH-A compares REF3M_B with its own Delta"),
              FM("Other", 0.4, "safe")]),
    dict(id="E11", name="CH-B readback scaling", ch="B", select=[("guard", "CH-B ADC scaling")],
         fms=[FM("Reading plausibly wrong: masks a real error", 0.2, "latent", ["SW-04L"], defeats=["SM-08B"]),
              FM("Reading wrong: false trip", 0.5, "safe"),
              FM("Other", 0.3, "safe")]),
    dict(id="E12", name="Coil drivers (2x TPS1H100, 2x low side, guard FET, clamps)", ch="A/B/C",
         select=[("relays", "K1 coil drive"), ("relays", "K2 coil drive"), ("relays", "CH-C guard FET")],
         fms=[FM("One series switch stuck on", 0.35, "latent", ["SM-19"],
                 note="Two more series switches still release the coil"),
              FM("Release slowed (clamp short)", 0.05, "latent", ["SM-19"]),
              FM("Switch stuck off / open", 0.45, "safe"),
              FM("Other", 0.15, "safe")]),
    dict(id="E13", name="Signal relays K1, K2", ch="shared", select=[("relays", "Relays:")],
         libs_only={"Relay:G6K-2"},
         fms=[FM("Contact stuck on NO (welded / mechanical)", 0.2, "latent", ["SM-09"],
                 note="The other relay still restores pass-through"),
              FM("Contact open / high resistance: EPS input floats", 0.4, "safe", note="SG3: EPS DTC, see finding"),
              FM("Pole-to-pole short: VT3 = VT6", 0.05, "safe", note="SG3: torque reads zero"),
              FM("Coil open / short", 0.35, "safe")]),
    dict(id="E14", name="EPS-side node (hold caps)", ch="shared", select=[("relays", "Relays:")],
         libs_only={"Device:C"},
         fms=[FM("Short: VT_EPS to GND", 0.4, "safe", note="SG3: EPS DTC"),
              FM("Open / drift", 0.6, "safe")]),
    dict(id="E15", name="CAN interfaces (2x TCAN3404, ESD, termination)", ch="A/B",
         select=[("io", "CAN bus")], extra=[("ch_a", "FSI to CH-B", "icarus:TCAN3404"),
                                            ("ch_b", "FSI to CH-A", "icarus:TCAN3404")],
         fms=[FM("Corrupted or replayed command accepted", 0.05, "direct", ["SW-01"]),
              FM("Bus dominant / babbling / silent", 0.75, "safe", note="Timeout: ramp-out"),
              FM("Other", 0.2, "safe")]),
    dict(id="E16", name="Car-side inputs (dividers, clamps, wetting)", ch="A/B",
         select=[("io", "Digital inputs"), ("io", "Disengage switch")],
         fms=[FM("Wrong speed / EPS-state / disengage reading in one channel", 0.3, "latent", ["SW-04L"],
                 note="Each channel has its own divider; A vs B vs CAN speed cross-check"),
              FM("Clamp / divider short: input reads stuck", 0.3, "latent", ["SW-04L"]),
              FM("Other", 0.4, "safe")]),
    dict(id="E17", name="Analog monitors (VBAT, 5V, Vcc2, ground offset)", ch="A/B", select=[("io", "Analog monitors")],
         fms=[FM("Monitor wrong", 0.4, "latent", ["SW-04L"]), FM("Other", 0.6, "safe")]),
]

# Connector: adjacent-pin shorts are modelled per pair from the J1 pin order in design.py.
CONNECTOR_PIN_FIT = 0.3           # per pin, open
CONNECTOR_PAIR_SHORT_FIT = 0.1    # per adjacent pair, short (contamination, crimp whisker)
VT_LINES = {"VT3_SNS", "VT6_SNS", "VT3_EPS", "VT6_EPS"}
DYNAMIC = {"CAN_H", "CAN_L", "VSP_IN", "NEP_IN", "IND_OUT", "WLP_IN", "RT_IN", "DIS_NO_IN", "DIS_NC_IN"}
STATIC = {"GND", "VBAT_IN", "IG1_IN", "VCC2_TAP", "TSGND_TAP"}


def classify_pair(a, b):
    """Effect of a short between two adjacent harness pins."""
    s = {a, b}
    if not s & VT_LINES:
        return "safe", "No torque line involved"
    if s in ({"VT3_SNS", "VT3_EPS"}, {"VT6_SNS", "VT6_EPS"}):
        return "safe", "Bypasses the overlay on that line; readback flags the loss (SG4)"
    if s <= VT_LINES:
        return "safe", "VT3 tied to VT6: torque reads ~0 or DTC (SG3)"
    other = (s - VT_LINES).pop()
    if other in DYNAMIC:
        return "direct", f"{other} pulses couple into the torque line upstream of every check: spurious torque"
    return "safe", f"Torque line forced to {other}: out of range, EPS DTC (SG3)"
