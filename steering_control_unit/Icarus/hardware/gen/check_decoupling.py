"""Decoupling and timing-part placement check (rev 0.5). Run with KiCad's Python from hardware/:
    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" gen/check_decoupling.py [board]

Decoupling: for every IC supply-input pin (pin type power_in), the pad-edge gap to the nearest
capacitor on that net. The F280049C and TMS570 are judged per pin (SPRS945: a cap at each VDDIO / VDD / VDDA pin);
other ICs by their closest supply pin, since a SET or SENSE pin tied to the rail needs no cap of its own.
Timing / feedback: the named parts' distance to their IC pin. Flags anything over LIMIT ("as close as
possible" in the datasheets; 2 mm copper edge to copper edge is the bar used here)."""
import os
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import design  # noqa: E402
import make_pcb  # noqa: E402

LIMIT = 2.0
PARTS = {p["ref"]: p for p in design.PARTS}
# (IC lib, pin, net-of-part, what the datasheet layout section wants there)
TIMING = [
    ("icarus:TPS3850", "2", "CWD_", "CWD timing resistor at the CWD pin (TPS3850 layout)"),
    ("icarus:LMR38020", "4", "RT_", "RT resistor at the RT pin"),
    ("icarus:LMR38020", "5", "FB_", "FB divider at the FB pin, away from SW"),
    ("icarus:LMR38020", "7", "BOOT_", "BOOT cap across BOOT/SW"),
    ("74xGxx:74LVC1G123", "6", "OS_C", "Cext between Cext and RextCext"),
    ("74xGxx:74LVC1G123", "7", "OS_RC", "Rext at RextCext"),
    ("icarus:F280049C", "69", "X1_A", "oscillator series R at X1"),
    ("icarus:TMS570LS0432", "14", "OSCIN_B", "crystal at OSCIN"),
    ("icarus:TMS570LS0432", "16", "OSCOUT_B", "crystal at OSCOUT"),
    ("Battery_Management:MCP73833-xxx-MF", "6", "CHG_PROG", "PROG resistor at PROG"),
    ("Regulator_Switching:TPS61092", "9", "LBI", "LBI divider at LBI"),
    ("icarus:TPS1H100", "13", "CL", "current-limit resistor at CL"),
    ("icarus:TPS1H100", "14", "CSR", "current-sense resistor at CS"),
]


def mm(v):
    return pcbnew.ToMM(v)


def dist(a, b):
    """Copper-edge gap between two pads (bounding boxes), mm."""
    ra, rb = a.GetBoundingBox(), b.GetBoundingBox()
    dx = max(0.0, mm(max(ra.GetLeft(), rb.GetLeft()) - min(ra.GetRight(), rb.GetRight())))
    dy = max(0.0, mm(max(ra.GetTop(), rb.GetTop()) - min(ra.GetBottom(), rb.GetBottom())))
    return (dx * dx + dy * dy) ** 0.5


def main(path):
    board = pcbnew.LoadBoard(path)
    _c, _p, ptypes = make_pcb.read_netlist()
    fps = {f.GetReference(): f for f in board.GetFootprints()}
    caps = {}
    for ref, f in fps.items():
        p = PARTS.get(ref)
        if p and p["lib"] == "Device:C":
            nets = {pd.GetNetname() for pd in f.Pads()}
            if "GND" in nets and len(nets) == 2:
                rail = (nets - {"GND"}).pop()
                caps.setdefault(rail, []).append(f)
    rows, bad = [], 0
    for ref, f in sorted(fps.items()):
        p = PARTS.get(ref)
        if not p or p["lib"].startswith(("Device:", "Diode:", "Connector", "Transistor", "Jumper", "Relay")):
            continue
        for pad in f.Pads():
            net = pad.GetNetname()
            if net not in caps or ptypes.get((ref, pad.GetNumber())) != "power_in":
                continue
            best = min(((dist(pad, cp), c.GetReference())
                        for c in caps[net] for cp in c.Pads() if cp.GetNetname() == net))
            rows.append((ref, p["value"], pad.GetNumber(), net, best[1], best[0]))
    per_ic = {}
    for ref, val, pin, net, cref, d in rows:
        k = (ref, val, net)
        per_pin = PARTS[ref]["lib"] in ("icarus:F280049C", "icarus:TMS570LS0432")   # a cap at every supply pin
        if k not in per_ic or (d > per_ic[k][0] if per_pin else d < per_ic[k][0]):
            per_ic[k] = (d, cref)
    print(f"{'IC':6s} {'part':18s} {'supply':12s} pin->cap (mm; F280049C: worst pin)")
    for (ref, val, net), (d, cref) in sorted(per_ic.items()):
        flag = "  <-- over" if d > LIMIT else ""
        bad += d > LIMIT
        print(f"{ref:6s} {val:18s} {net:12s} {d:5.2f} ({cref}){flag}")
    print()
    for lib, pin, prefix, why in TIMING:
        for ref, f in sorted(fps.items()):
            if PARTS.get(ref, {}).get("lib") != lib:
                continue
            pad = next(pd for pd in f.Pads() if pd.GetNumber() == pin)
            net = pad.GetNetname()
            if not net.split("/")[-1].startswith(prefix):
                continue
            d, oref = min((dist(pad, op), o.GetReference())
                          for o in fps.values() if o is not f for op in o.Pads() if op.GetNetname() == net)
            flag = "  <-- over" if d > LIMIT else ""
            bad += d > LIMIT
            print(f"{ref:6s} pin {pin:3s} {net:10s} {d:5.2f} mm to {oref:6s} {why}{flag}")
    print(f"\n{bad} over {LIMIT} mm")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", design.PROJECT + ".kicad_pcb"))
