"""FMEDA + fault-tree cut sets for the Icarus safety element (SG1, ASIL D).

    python analyze.py      -> ../docs/07_FMEDA_FTA.md and fmeda.csv

Reads the parts from ../hardware/gen/design.py and the element model from element_model.py.
Every number is only as good as the placeholder failure rates in element_model.py.
"""
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "hardware", "gen"))
import design  # noqa: E402
import element_model as M  # noqa: E402

TARGET = {"SPFM": 0.99, "LFM": 0.90, "PMHF": 10.0}      # ASIL D (ISO 26262-5 Tables 4, 5, 6)


def fit(p):
    lib, v = p["lib"], p["value"]
    if lib == "Device:C":
        return 1.0 if any(k in v for k in ("10uF", "22uF")) else 0.5
    table = {"Device:R": 0.2, "Device:C_Polarized": 5, "Device:L": 0.5, "Device:FerriteBead_Small": 0.2,
             "Device:Fuse": 1, "Jumper:SolderJumper_2_Open": 0.1, "Connector:TestPoint": 0.05, "Device:LED": 1, "Device:D_Schottky": 0.5,
             "Diode:1N4148WS": 0.5, "Diode:BAS21": 0.5, "Diode:BZX84Cxx": 1, "Device:D_Zener": 2,
             "Power_Protection:NUP2105L": 1, "Transistor_FET:Q_NMOS_GSD": 2, "icarus:SQJ147ELP": 5,
             "Amplifier_Operational:TLV9064": 8, "Comparator:LM339": 6, "74xGxx:74LVC1G11": 2, "74xGxx:74LVC1G08": 2,
             "74xGxx:74AUC1G74": 3, "Power_Supervisor:TPS3702": 5, "icarus:TPS3850": 5,
             "Regulator_Linear:TPS7B8133DRV": 8, "icarus:LMR38020": 15, "icarus:F280049C": 100,
             "icarus:TCAN3404": 10, "icarus:TPS1H100": 10, "icarus:SIT2024B": 5, "Relay:G6K-2": 20,
             "Connector:Conn_ARM_JTAG_SWD_10": 1, "Connector_Generic:Conn_01x03": 1,
             "Device:Polyfuse": 2, "Battery_Management:MCP73833-xxx-MF": 10, "Regulator_Switching:TPS61092": 15,
             "Power_Management:LM66100DCK": 5, "Analog_Switch:TS5A3159DBV": 3, "74xGxx:74LVC1G123": 3,
             "Device:Thermistor_NTC": 1, "Connector_Generic:Conn_01x02": 1,
             "icarus:TMS570LS0432": 100, "icarus:TLV73312_DRV": 8, "Device:Crystal_GND24": 5,   # rev 0.7 CH-B
             "Connector_Generic:Conn_02x10_Odd_Even": 1}
    if lib == "Connector_Generic:Conn_01x20":
        return 0.0                                  # modelled per pin / pair below
    return table[lib]


def combined_dc(sm_ids, kind):
    """Independent mechanisms in parallel; capped at 99.9 %."""
    miss = math.prod(1 - M.SM[s][1] for s in sm_ids if M.SM[s][2] == kind)
    return min(1 - miss, 0.999) if sm_ids else 0.0


# --- assign parts to elements -----------------------------------------------
assigned = {}
for el in M.ELEMENTS:
    refs = []
    for sheet, gp in el["select"]:
        for p in design.PARTS:
            if p["sheet"] == sheet and (p["group"] or "").startswith(gp):
                if "libs_only" in el and p["lib"] not in el["libs_only"]:
                    continue
                if p["lib"] in el.get("libs_skip", ()):
                    continue
                refs.append(p)
    for sheet, gp, lib in el.get("extra", []):
        refs += [p for p in design.PARTS if p["sheet"] == sheet and (p["group"] or "").startswith(gp) and p["lib"] == lib]
    refs = list({p["ref"]: p for p in refs}.values())      # group prefixes may overlap within one element
    for p in refs:
        assert p["ref"] not in assigned, f"{p['ref']} in two elements"
        assigned[p["ref"]] = el["id"]
    el["parts"] = refs
    el["fit"] = sum(fit(p) for p in refs)
    assert abs(sum(f["share"] for f in el["fms"]) - 1) < 1e-6, el["id"]
rest = [p for p in design.PARTS if p["ref"] not in assigned and p["lib"] != "Connector_Generic:Conn_01x20"]
M.ELEMENTS.append(dict(id="E99", name="Not SG1-relevant (debug headers, LEDs, FSI series R, CAN STB, indicator)",
                       ch="-", parts=rest, fit=sum(fit(p) for p in rest),
                       fms=[M.FM("All modes", 1.0, "safe")]))

# connector element, built from the J1 pin order
j1 = next(p for p in design.PARTS if p["lib"] == "Connector_Generic:Conn_01x20")
pins = j1["units"][1]
order = [pins[str(i)] for i in range(1, len(pins) + 1)]
pairs = []
for i in range(len(order) - 1):
    eff, why = M.classify_pair(order[i], order[i + 1])
    pairs.append((i + 1, i + 2, order[i], order[i + 1], eff, why))
n_direct_pairs = sum(1 for p in pairs if p[4] == "direct")
conn_fit = M.CONNECTOR_PIN_FIT * len(order) + M.CONNECTOR_PAIR_SHORT_FIT * len(pairs)
conn_fms = [M.FM("Pin open", M.CONNECTOR_PIN_FIT * len(order) / conn_fit, "safe", note="Floating input: EPS DTC (SG3)")]
for a, b, na, nb, eff, why in pairs:
    conn_fms.append(M.FM(f"Short J1.{a}-{b} ({na} / {nb})", M.CONNECTOR_PAIR_SHORT_FIT / conn_fit, eff, note=why))
M.ELEMENTS.append(dict(id="E18", name="Vehicle connector J1 (pins and adjacent-pin shorts)", ch="shared",
                       parts=[j1], fit=conn_fit, fms=conn_fms))

# --- FMEDA ------------------------------------------------------------------
def fmeda(scale=1.0, dc_cap=1.0, mcu_scale=1.0, no_r2=False):
    """Rows + metrics. scale multiplies every FIT, mcu_scale the MCUs, dc_cap caps each mechanism's DC."""
    def dc_of(sm_ids, kind):
        miss = math.prod(1 - min(M.SM[s][1], dc_cap) for s in sm_ids if M.SM[s][2] == kind)
        return min(1 - miss, 0.999) if sm_ids else 0.0

    rows = []
    for el in M.ELEMENTS:
        k = scale * (mcu_scale if el["id"] in ("E04", "E06") else 1.0)
        for f in el["fms"]:
            lam = el["fit"] * f["share"] * k
            f = dict(f)
            if no_r2:                                    # comparison: the rev 0.1 guard without SM-20
                f["sms"] = [s for s in f["sms"] if s != "SM-20"]
            dc = dc_of(f["sms"], "detect" if f["effect"] == "direct" else "latent")
            r = dict(el=el["id"], element=el["name"], mode=f["name"], fit=lam, effect=f["effect"], sms=f["sms"],
                     dc=dc, spf=0.0, rf=0.0, lat=0.0, note=f["note"], defeats=f["defeats"])
            if f["effect"] == "direct":
                r["spf" if dc == 0 else "rf"] = lam if dc == 0 else lam * (1 - dc)
            elif f["effect"] == "latent":
                r["lat"] = lam * (1 - dc)
            rows.append(r)
    tot = sum(r["fit"] for r in rows)
    spf, rf, lat = (sum(r[k] for r in rows) for k in ("spf", "rf", "lat"))
    # dual point: a latent fault removes a mechanism, then a direct fault meets fewer barriers
    dual = []
    for L in (r for r in rows if r["effect"] == "latent" and r["defeats"]):
        tau = (1 - L["dc"]) * M.MISSION_HOURS + L["dc"] * M.KEY_CYCLE_HOURS
        for D in (r for r in rows if r["effect"] == "direct"):
            lost = set(L["defeats"]) & set(D["sms"])
            if lost:
                resid = D["fit"] * (1 - dc_of([s for s in D["sms"] if s not in lost], "detect"))
                dual.append((L, D, L["fit"] * tau * resid * 1e-9))
    dpf = sum(x[2] for x in dual)
    met = dict(tot=tot, spf=spf, rf=rf, lat=lat, dpf=dpf, SPFM=1 - (spf + rf) / tot,
               LFM=1 - lat / (tot - spf - rf), PMHF=spf + rf + dpf)
    return rows, met, dual


rows, met, dual = fmeda()
tot, spf, rf, lat, dpf = (met[k] for k in ("tot", "spf", "rf", "lat", "dpf"))
SPFM, LFM, PMHF = met["SPFM"], met["LFM"], met["PMHF"]
mpf_total = sum(r["fit"] for r in rows if r["effect"] == "latent")
SENS = [("Baseline (placeholders)", {}), ("All FIT x3", dict(scale=3)), ("MCU FIT x5 (500 FIT each)", dict(mcu_scale=5)),
        ("Every DC claim capped at 90 %", dict(dc_cap=0.90)), ("FIT x3 and DC capped at 90 %", dict(scale=3, dc_cap=0.90)),
        ("Without R2 (guard not referenced to CH-B)", dict(no_r2=True)),
        ("Without R2, FIT x3 and DC capped at 90 %", dict(no_r2=True, scale=3, dc_cap=0.90))]
sens = [(name, fmeda(**kw)[1]) for name, kw in SENS]

# higher-order cut sets the FMEDA's single + dual terms don't cover
direct_raw = sum(r["fit"] for r in rows if r["effect"] == "direct")
weld = next(r for r in rows if r["el"] == "E13" and r["mode"].startswith("Contact stuck"))["fit"] / 2
tau_w = (1 - combined_dc(["SM-09"], "latent")) * M.MISSION_HOURS + combined_dc(["SM-09"], "latent") * M.KEY_CYCLE_HOURS
sw = next(r for r in rows if r["el"] == "E12" and r["mode"].startswith("One series"))["fit"] / 5
tau_s = (1 - combined_dc(["SM-19"], "latent")) * M.MISSION_HOURS + combined_dc(["SM-19"], "latent") * M.KEY_CYCLE_HOURS
cutsets = [
    ("CS1", 1, "Overlay fault beyond Delta_max, missed by CH-A, CH-B and CH-C readback",
     sum(r["rf"] for r in rows if r["el"] == "E08")),
    ("CS2", 1, "Adjacent-pin short couples a pulsed signal into a torque line (J1)",
     sum(r["spf"] + r["rf"] for r in rows if r["el"] == "E18")),
    ("CS3", 1, "Corrupted CAN command passes E2E in both channels", sum(r["rf"] for r in rows if r["el"] == "E15")),
    ("CS4", 2, "CH-A wrong Delta + CH-B silently passes it (dual point)",
     sum(x[2] for x in dual if x[0]["el"] == "E06" and x[1]["el"] == "E04")),
    ("CS5", 2, "Latent readback / guard / supervisor loss + overlay fault (dual point, all others)",
     dpf - sum(x[2] for x in dual if x[0]["el"] == "E06" and x[1]["el"] == "E04")),
    ("CS6", 3, "K1 AND K2 welded on NO + any direct fault (no reaction path left)",
     weld * tau_w * weld * tau_w * direct_raw * 1e-18),
    ("CS7", 4, "Three series coil switches of one relay stuck on + any direct fault, per relay x2",
     2 * (sw * tau_s) ** 3 * direct_raw * 1e-27),
]

# --- outputs ----------------------------------------------------------------
with open(os.path.join(HERE, "fmeda.csv"), "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["Element", "Name", "Failure mode", "FIT", "Effect on SG1", "Safety mechanisms", "DC",
                "SPF FIT", "Residual FIT", "Latent FIT", "Note"])
    for r in rows:
        w.writerow([r["el"], r["element"], r["mode"], f"{r['fit']:.4f}", r["effect"], " ".join(r["sms"]),
                    f"{r['dc']:.3f}", f"{r['spf']:.4f}", f"{r['rf']:.4f}", f"{r['lat']:.4f}", r["note"]])


def ok(v, t, higher=True):
    return "PASS" if (v >= t if higher else v < t) else "**FAIL**"


L = []
L.append("# 07: FMEDA and fault-tree analysis, SG1 (generated)\n")
L.append("> Generated by `safety/analyze.py` from `hardware/gen/design.py` and `safety/element_model.py`. "
         "Don't edit by hand. **Failure rates are placeholders** (see `element_model.py`), so these metrics "
         "show the architecture's *shape*, not a safety-case number.\n")
L.append(f"Safety goal **SG1** (ASIL D): no unintended or out-of-envelope overlay torque at the EPS. "
         f"Mission profile: {M.MISSION_HOURS:,} operating hours. Key-on tests every {M.KEY_CYCLE_HOURS} h. "
         f"Design rev {design.REV}, {len(design.PARTS)} parts.\n")
L.append("## 1. Hardware metrics\n")
L.append("| Metric | Value | ASIL D target | Result |\n|---|---|---|---|")
L.append(f"| SPFM | {SPFM * 100:.2f} % | ≥ 99 % | {ok(SPFM, TARGET['SPFM'])} |")
L.append(f"| LFM | {LFM * 100:.2f} % | ≥ 90 % | {ok(LFM, TARGET['LFM'])} |")
L.append(f"| PMHF | {PMHF:.3f} FIT | < 10 FIT | {ok(PMHF, TARGET['PMHF'], higher=False)} |")
L.append(f"\nλ total {tot:.1f} FIT · single-point {spf:.4f} · residual {rf:.4f} · multi-point {mpf_total:.1f} "
         f"(latent {lat:.3f}) · dual-point term {dpf:.5f} FIT\n")

L.append("## 2. Elements\n")
L.append("| Id | Element | Channel | Parts | λ (FIT) | SPF + RF (FIT) | Latent (FIT) |\n|---|---|---|---|---|---|---|")
for el in M.ELEMENTS:
    er = [r for r in rows if r["el"] == el["id"]]
    L.append(f"| {el['id']} | {el['name']} | {el['ch']} | {len(el['parts'])} | {el['fit']:.1f} | "
             f"{sum(r['spf'] + r['rf'] for r in er):.4f} | {sum(r['lat'] for r in er):.4f} |")

L.append("\n## 3. Failure modes (safety-relevant only; the full table is in `safety/fmeda.csv`)\n")
L.append("| El | Failure mode | λ (FIT) | Effect | Mechanisms | DC | Residual / latent (FIT) |\n|---|---|---|---|---|---|---|")
for r in rows:
    if r["effect"] == "safe":
        continue
    L.append(f"| {r['el']} | {r['mode']} | {r['fit']:.3f} | {r['effect']} | {', '.join(r['sms']) or '**none**'} | "
             f"{r['dc'] * 100:.1f} % | {r['spf'] + r['rf'] + r['lat']:.4f} |")

L.append("\n## 4. Vehicle connector: adjacent-pin short analysis\n")
L.append("| Pins | Nets | Effect | Why |\n|---|---|---|---|")
for a, b, na, nb, eff, why in pairs:
    L.append(f"| {a}-{b} | {na} / {nb} | {'**SPF**' if eff == 'direct' else eff} | {why} |")

L.append("\n## 5. Fault tree: top event SG1, minimal cut sets (block level)\n")
L.append("```\nSG1 violated\n└── OR\n    ├── CS1  overlay fault ∧ ¬SM-08A ∧ ¬SM-08B ∧ ¬SM-15     (residual)\n"
         "    ├── CS2  J1 pulsed-signal short into a VT line          (single point)\n"
         "    ├── CS3  CAN corruption ∧ ¬SW-01(A) ∧ ¬SW-01(B)          (residual)\n"
         "    ├── CS4  CH-A wrong Δ ∧ CH-B silent pass                (dual point)\n"
         "    ├── CS5  latent barrier loss ∧ covered direct fault     (dual point)\n"
         "    ├── CS6  K1 weld ∧ K2 weld ∧ any direct fault           (triple)\n"
         "    └── CS7  3 series coil switches stuck ∧ direct fault    (quadruple)\n```\n")
L.append("| Cut set | Order | Description | Rate (FIT) | Share of PMHF |\n|---|---|---|---|---|")
for cid, order_, desc, val in cutsets:
    L.append(f"| {cid} | {order_} | {desc} | {val:.2e} | {val / PMHF * 100 if PMHF else 0:.1f} % |")

L.append("\n## 5a. Sensitivity to the placeholder data\n")
L.append("| Case | SPFM | LFM | PMHF (FIT) | Meets ASIL D? |\n|---|---|---|---|---|")
for name, m in sens:
    good = m["SPFM"] >= TARGET["SPFM"] and m["LFM"] >= TARGET["LFM"] and m["PMHF"] < TARGET["PMHF"]
    L.append(f"| {name} | {m['SPFM'] * 100:.2f} % | {m['LFM'] * 100:.2f} % | {m['PMHF']:.3f} | {'yes' if good else '**no**'} |")
top = sorted(rows, key=lambda r: -(r["spf"] + r["rf"]))[:5]
L.append("\n## 6. Largest contributors to SPF + residual\n")
L.append("| El | Failure mode | FIT |\n|---|---|---|")
for r in top:
    L.append(f"| {r['el']} | {r['mode']} | {r['spf'] + r['rf']:.4f} |")
toplat = sorted(rows, key=lambda r: -r["lat"])[:5]
L.append("\n## 7. Largest latent contributors\n")
L.append("| El | Failure mode | Latent FIT | Test |\n|---|---|---|---|")
for r in toplat:
    L.append(f"| {r['el']} | {r['mode']} | {r['lat']:.4f} | {', '.join(r['sms'])} |")
out = os.path.normpath(os.path.join(HERE, "..", "docs", "07_FMEDA_FTA.md"))
with open(out, "w", encoding="utf-8", newline="\n") as f:
    f.write("\n".join(L) + "\n")
print(f"SPFM {SPFM * 100:.3f} %  LFM {LFM * 100:.2f} %  PMHF {PMHF:.4f} FIT  "
      f"(SPF {spf:.4f}, RF {rf:.4f}, DPF {dpf:.5f})  total {tot:.1f} FIT  direct J1 pairs {n_direct_pairs}")
for cid, o, d, v in cutsets:
    print(cid, o, f"{v:.3e}", d)
for name, m in sens:
    print(f"  {name:32s} SPFM {m['SPFM']*100:.2f}  LFM {m['LFM']*100:.2f}  PMHF {m['PMHF']:.3f}")
print("wrote", out)
