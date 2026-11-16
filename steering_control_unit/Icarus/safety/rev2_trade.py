"""Rev 2.0 architecture trade study: block-level FMEDA for SG1 (ASIL D), variant by variant.

    python rev2_trade.py            -> prints the table used in docs/11_REV2_DESIGN_REVIEW.md

Unlike analyze.py, this does not read design.py: rev 2.0 has no schematic yet. Element failure rates are
the same placeholder convention as analyze.py (generic FIT per part type, cabin mission profile), summed per
block by hand below. The point is to compare architectures on the same footing, not to produce a safety-case
number. The maths (SPFM, LFM, PMHF with a dual-point term) is the same as analyze.py.
"""
import math

MISSION_HOURS = 10_000
KEY_CYCLE_HOURS = 2
TARGET = dict(SPFM=0.99, LFM=0.90, PMHF=10.0)

# id -> (description, DC, kind). "detect": direct fault reaches pass-through in time. "latent": key-on / periodic test.
SM = {
    "SM-08A": ("CH-A readback of EPS-side vs sensor-side and commanded delta, VT3+VT6 sum check", 0.99, "detect"),
    "SM-08B": ("CH-B readback on its own buffers, compared with CH-B's OWN expected delta (computed from its own "
               "CAN copy, SAS, speed), plus the sum check", 0.99, "detect"),
    "SM-26A": ("CH-A self-diagnostics (F28004x safety manual: ECC, CPU/PIE tests, DCC, ERRORSTS) into PMIC-A's "
               "MCU error monitor, PMIC-A Q&A watchdog", 0.90, "detect"),
    "SM-PA": ("PMIC-A rail OV/UV monitors (own bandgap) -> reset + ENDRV low", 0.99, "detect"),
    "SM-PB": ("PMIC-B rail OV/UV monitors (own bandgap) -> reset + ENDRV low", 0.99, "detect"),
    "SW-01": ("E2E on CAN (CRC, counter, timeout), both channels", 0.99, "detect"),
    "SM-15": ("CH-C analog guard (rev 1.0), window comparators, latched", 0.99, "detect"),
    "SM-20": ("CH-C windows referenced to CH-B's expected delta (rev 1.0 R2)", 0.90, "detect"),
    "SM-24": ("LiPo boost OV cutoff (rev 1.0)", 0.90, "detect"),
    # latent tests
    "SM-09": ("Key-on relay proof test, each relay alone, by readback", 0.99, "latent"),
    "SM-19": ("Key-on coil-switch test: each switch alone, coil nodes read back by both MCUs", 0.90, "latent"),
    "SM-03L": ("Key-on enable-path test: ENDRV and GPIO paths dropped one at a time, coil node read back", 0.90, "latent"),
    "SM-PBIST": ("PMIC analog + logic BIST at power-up, ENDRV read-back (ENDRV_ERR), MCU-forced watchdog fail", 0.90,
                 "latent"),
    "SW-04L": ("Cross-channel plausibility of every reading, A vs B, every cycle", 0.90, "latent"),
    "SW-06": ("MCU self-tests at start-up (PBIST/LBIST on TMS570, F28004x STL)", 0.90, "latent"),
    "SM-25": ("TMS570 lockstep cores + ECC + ESM nERROR into PMIC-B", 0.99, "latent"),
    "SM-17": ("Key-on guard self-test (rev 1.0)", 0.99, "latent"),
    # docs/12 self-test plan
    "ST-COIL": ("Coil-switch test with coil-current shunts + node readback, each switch alone, observed by both "
                "MCUs, key-on and post-run", 0.99, "latent"),
    "ST-REL": ("Relay proof test with a real test delta (EPS motor relay off): each relay alone must block, "
               "both must pass, release time < 5 ms (clamp check)", 0.99, "latent"),
    "ST-RB": ("Known-delta test: both readbacks must measure the injected delta and the OVL pre-relay taps", 0.99,
              "latent"),
    "ST-EN": ("Enable-path test: GPIO alone, ENDRV alone, per switch, in PMIC DIAGNOSTIC state, coil current read "
              "by both MCUs", 0.99, "latent"),
    "ST-ARM": ("Arm gesture each trip: driver press must open the NC coil-return loop and close the NO contact", 0.99,
               "latent"),
}


def FM(name, share, effect, sms=(), defeats=()):
    return dict(name=name, share=share, effect=effect, sms=list(sms), defeats=list(defeats))


def base_rev2(one_relay=False, credit_cha_selftest=True, guard=False, selftest=False):
    E = _base_rev2(one_relay, credit_cha_selftest, guard)
    return apply_selftest(E) if selftest else E


def _base_rev2(one_relay=False, credit_cha_selftest=True, guard=False):
    cha = ["SM-08B", "SM-26A"] if credit_cha_selftest else ["SM-08B"]
    if guard:
        cha.append("SM-20")
    ov_det = ["SM-08A", "SM-08B"] + (["SM-15"] if guard else [])
    E = [
        dict(id="E01", name="Input protection (fuse, ideal diode, TVS)", fit=23.5,
             fms=[FM("Open / no power", 0.6, "safe"), FM("Short: fuse blows", 0.3, "safe"),
                  FM("TVS open (latent loss of clamp)", 0.1, "safe")]),
        dict(id="E02", name="ESD series pairs on VT lines", fit=4.0,
             fms=[FM("One diode short/open", 0.85, "safe"),
                  FM("Both diodes of a pair leak: one VT line shifts", 0.15, "direct", ov_det)]),
        dict(id="E03", name="PMIC-A TPS65381A-Q1 + inductor + caps", fit=30.0,
             fms=[FM("Rail lost / reset", 0.5, "safe"),
                  FM("Rail OV or noise into the overlay op-amps", 0.15, "direct", ["SM-PA"] + ov_det),
                  FM("Monitor / ENDRV fails to act", 0.15, "latent", ["SM-PBIST"], defeats=["SM-PA", "SM-26A"]),
                  FM("Other", 0.2, "safe")]),
        dict(id="E04", name="PMIC-B TPS65381A-Q1 + core FET + inductor + caps", fit=32.0,
             fms=[FM("Rail lost / reset", 0.5, "safe"),
                  FM("Rail OV / noise corrupts CH-B", 0.15, "latent", ["SM-PB", "SW-04L"], defeats=["SM-08B"]),
                  FM("Monitor / ENDRV fails to act", 0.15, "latent", ["SM-PBIST"], defeats=["SM-PB"]),
                  FM("Other", 0.2, "safe")]),
        dict(id="E05", name="CH-A F280049C-Q1 + MEMS clock + decoupling", fit=116.4,
             fms=[FM("Wrong delta command inside the hardware envelope", 0.3, "direct", cha),
                  FM("Hang / no output: watchdog drops ENDRV", 0.3, "safe"),
                  FM("Enable GPIO stuck active", 0.1, "latent", ["SM-03L"], defeats=[]),
                  FM("Readback ADC fault masks a real error", 0.1, "latent", ["SW-04L", "SW-06"], defeats=["SM-08A"]),
                  FM("No SG1 effect", 0.2, "safe")]),
        dict(id="E06", name="CH-B TMS570LS0432-Q1 + crystal + decoupling", fit=115.2,
             fms=[FM("Monitor silently passes a bad overlay", 0.3, "latent", ["SW-04L", "SW-06", "SM-25"],
                     defeats=["SM-08B"]),
                  FM("Hang: watchdog drops ENDRV", 0.3, "safe"), FM("False trip", 0.1, "safe"),
                  FM("No SG1 effect", 0.3, "safe")]),
        dict(id="E07", name="Overlay analog (DAC buffer, VMID, resistor-weighted summers)", fit=22.0,
             fms=[FM("Overlay wrong / beyond delta_max", 0.5, "direct", ov_det),
                  FM("Overlay lost (SG4)", 0.2, "safe"), FM("No effect", 0.3, "safe")]),
        dict(id="E08", name="CH-A readback scaling", fit=5.2,
             fms=[FM("Plausibly wrong: masks", 0.2, "latent", ["SW-04L"], defeats=["SM-08A"]),
                  FM("Wrong: false trip", 0.5, "safe"), FM("Other", 0.3, "safe")]),
        dict(id="E09", name="CH-B readback buffers (TLV9064-Q1) and scaling", fit=12.0,
             fms=[FM("Plausibly wrong: masks", 0.2, "latent", ["SW-04L"], defeats=["SM-08B"]),
                  FM("Wrong: false trip", 0.5, "safe"), FM("Other", 0.3, "safe")]),
        dict(id="E10", name="Coil drivers: 2 high side, 2 low side, cross-driven, ENDRV AND GPIO, clamps", fit=20.0,
             fms=[FM("One series switch stuck on", 0.35, "latent", ["SM-19"]),
                  FM("Release slowed (clamp short)", 0.05, "latent", ["SM-19"]),
                  FM("Stuck off", 0.45, "safe"), FM("Other", 0.15, "safe")]),
        dict(id="E11", name="Signal relay(s)", fit=20.0 if one_relay else 40.0,
             fms=[FM("Contact stuck on NO", 0.2, "latent", ["SM-09"], defeats=["*ALL*"] if one_relay else []),
                  FM("Contact open (100k bypass keeps the EPS on the sensor)", 0.4, "safe"),
                  FM("Pole-to-pole short (SG3)", 0.05, "safe"), FM("Coil open/short", 0.35, "safe")]),
        dict(id="E12", name="EPS-side hold caps + 100k bypass", fit=2.0, fms=[FM("All", 1.0, "safe")]),
        dict(id="E13", name="CAN (2x TCAN3404-Q1, ESD, termination)", fit=22.0,
             fms=[FM("Corrupted / replayed command accepted", 0.05, "direct", ["SW-01"]),
                  FM("Bus fault: timeout, ramp-out", 0.75, "safe"), FM("Other", 0.2, "safe")]),
        dict(id="E14", name="Car-side inputs + analog monitors, per channel", fit=16.7,
             fms=[FM("Wrong reading in one channel", 0.4, "latent", ["SW-04L"]), FM("Other", 0.6, "safe")]),
        dict(id="E15", name="Vehicle connector (AMPSEAL 23, R1 pin order: no pulsed pin next to a VT pin)", fit=11.7,
             fms=[FM("Pin open / static short (SG3 only)", 1.0, "safe")]),
        dict(id="E99", name="LEDs, test pads, debug footprints", fit=30.0, fms=[FM("All", 1.0, "safe")]),
    ]
    if guard:
        E.append(dict(id="E16", name="CH-C guard + expected-delta reference (rev 1.0)", fit=66.2,
                      fms=[FM("Guard cannot trip", 0.35, "latent", ["SM-17"], defeats=["SM-15", "SM-20"]),
                           FM("Spurious trip", 0.45, "safe"), FM("Other", 0.2, "safe")]))
    return E


def apply_selftest(E):
    """docs/12: stronger key-on / post-run tests and the new monitoring hardware."""
    swap = {"SM-19": "ST-COIL", "SM-09": "ST-REL", "SM-03L": "ST-EN"}
    for el in E:
        for f in el["fms"]:
            f["sms"] = [swap.get(x, x) for x in f["sms"]]
            if f["effect"] == "latent" and "masks" in f["name"]:
                f["sms"].append("ST-RB")
            if f["name"].startswith("Release slowed"):
                f["sms"] = ["ST-REL", "ST-COIL"]
    E.append(dict(id="E17", name="Hardware disengage loop (switch NC in the coil return, pull-up, TVS)", fit=4.0,
                  fms=[FM("NC loop shorted (switch welded or harness to GND): hardware kill lost", 0.3, "latent",
                          ["ST-ARM"]), FM("Loop open: relays drop (availability)", 0.6, "safe"),
                       FM("Other", 0.1, "safe")]))
    E.append(dict(id="E18", name="New monitors: coil shunts, OVL taps, relay NTC, seat sense, lamp readback",
                  fit=8.0, fms=[FM("Plausibly wrong reading masks a test", 0.2, "latent", ["SW-04L"]),
                                FM("Wrong reading: false trip", 0.5, "safe"), FM("Other", 0.3, "safe")]))
    return E


def fmeda(E, scale=1.0, dc_cap=1.0):
    def dc_of(ids, kind):
        ids = [s for s in ids if SM[s][2] == kind]
        if not ids:
            return 0.0
        return min(1 - math.prod(1 - min(SM[s][1], dc_cap) for s in ids), 0.999)

    rows = []
    for el in E:
        assert abs(sum(f["share"] for f in el["fms"]) - 1) < 1e-9, el["id"]
        for f in el["fms"]:
            lam = el["fit"] * f["share"] * scale
            dc = dc_of(f["sms"], "detect" if f["effect"] == "direct" else "latent")
            r = dict(el=el["id"], mode=f["name"], fit=lam, effect=f["effect"], sms=f["sms"], dc=dc,
                     defeats=f["defeats"], spf=0.0, rf=0.0, lat=0.0)
            if f["effect"] == "direct":
                r["spf" if dc == 0 else "rf"] = lam if dc == 0 else lam * (1 - dc)
            elif f["effect"] == "latent":
                r["lat"] = lam * (1 - dc)
            rows.append(r)
    tot = sum(r["fit"] for r in rows)
    spf, rf, lat = (sum(r[k] for r in rows) for k in ("spf", "rf", "lat"))
    dpf = 0.0
    for L in (r for r in rows if r["effect"] == "latent" and r["defeats"]):
        tau = (1 - L["dc"]) * MISSION_HOURS + L["dc"] * KEY_CYCLE_HOURS
        for D in (r for r in rows if r["effect"] == "direct"):
            lost = set(D["sms"]) if "*ALL*" in L["defeats"] else set(L["defeats"]) & set(D["sms"])
            if lost:
                resid = D["fit"] * (1 - dc_of([s for s in D["sms"] if s not in lost], "detect"))
                dpf += L["fit"] * tau * resid * 1e-9
    return dict(tot=tot, SPFM=1 - (spf + rf) / tot, LFM=1 - lat / (tot - spf - rf), PMHF=spf + rf + dpf, dpf=dpf)


# rev 1.0 numbers come from analyze.py on the rev 1.0 schematic (docs/07), not re-modelled here
REV10 = {"Baseline": (0.99988, 0.9892, 0.088), "FIT x3, DC capped at 90 %": (0.9994, 0.9853, 1.233)}

VARIANTS = [
    ("Rev 2.0 proposed (2 relays, no CH-C, no LiPo)", dict()),
    ("Rev 2.0 + self-test plan (docs/12)", dict(selftest=True)),
    ("Rev 2.0 + self-test plan, no credit for CH-A self-diagnostics", dict(selftest=True, credit_cha_selftest=False)),
    ("Rev 2.0 + CH-C guard kept", dict(guard=True)),
    ("Rev 2.0 with ONE relay", dict(one_relay=True)),
    ("Rev 2.0, no credit for CH-A self-diagnostics", dict(credit_cha_selftest=False)),
    ("Rev 2.0 + CH-C, no credit for CH-A self-diagnostics", dict(credit_cha_selftest=False, guard=True)),
]
CASES = [("Baseline", {}), ("FIT x3", dict(scale=3)), ("DC capped at 90 %", dict(dc_cap=0.9)),
         ("FIT x3, DC capped at 90 %", dict(scale=3, dc_cap=0.9))]

if __name__ == "__main__":
    print("| Variant | Case | Total (FIT) | SPFM | LFM | PMHF (FIT) | ASIL D? |\n|---|---|---|---|---|---|---|")
    for name, (s, l, p) in REV10.items():
        print(f"| Rev 1.0 as drawn (analyze.py) | {name} | 731 | {s*100:.2f} % | {l*100:.2f} % | {p:.3f} | yes |")
    for vname, kw in VARIANTS:
        E = base_rev2(**kw)
        for cname, ckw in CASES:
            m = fmeda(E, **ckw)
            good = m["SPFM"] >= TARGET["SPFM"] and m["LFM"] >= TARGET["LFM"] and m["PMHF"] < TARGET["PMHF"]
            print(f"| {vname} | {cname} | {m['tot']:.0f} | {m['SPFM']*100:.2f} % | {m['LFM']*100:.2f} % | "
                  f"{m['PMHF']:.3f} | {'yes' if good else '**no**'} |")
