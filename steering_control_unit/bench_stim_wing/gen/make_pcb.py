"""Generate bench_stim_wing.kicad_pcb from the schematic's netlist + placement.PLACE.

Needs KiCad's pcbnew Python module (KiCad 7 or later):
    python3 make_pcb.py place    # parts + outline + planes, writes route/<project>.dsn
    (run Freerouting on the .dsn -> .ses, see route.sh)
    python3 make_pcb.py finish   # import .ses, pour GND, save, DRC report
    python3 make_pcb.py check    # placement only: DRC courtyard/edge report
"""
import os
import re
import sys

import pcbnew

import design
from sexpr import Str, dump, parse

HERE = os.path.dirname(os.path.abspath(__file__))
PRJ = os.path.normpath(os.path.join(HERE, ".."))
PCB = os.path.join(PRJ, design.PROJECT + ".kicad_pcb")
NET = os.path.join(PRJ, design.PROJECT + ".net")
DSN = os.path.join(HERE, "route", design.PROJECT + ".dsn")
SES = os.path.join(HERE, "route", design.PROJECT + ".ses")
DRC = os.path.join(HERE, "route", "drc.rpt")
FP_ROOT = os.environ.get("KICAD_FOOTPRINT_DIR") or next(
    (p for p in ["/usr/share/kicad/footprints", r"C:\Program Files\KiCad\10.0\share\kicad\footprints"]
     if os.path.isdir(p)), "")
OX, OY = 100.0, 80.0          # board origin on the page
LAYERS = 4

# Net classes: (name, track mm, nets). Default is 0.2 mm for signals.
NETCLASSES = [
    ("PWR12", 0.5, ["V12_RAW", "V12_F", "+12V", "IG1_SW", "IG1_OUT"]),
    ("PWR5", 0.4, ["+5V", "VUSB", "BUCK_SW", "SAS_SW", "SAS_12V", "ARM_SRC", "ARM"]),
    ("PWR3", 0.3, ["+3V3"]),
]
TRACK_SIG = 0.2
CLEARANCE = 0.15


def mm(v):
    return pcbnew.FromMM(v)


def pt(x, y):
    return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


def read_netlist():
    tree = parse(open(NET, encoding="utf-8").read())
    get = lambda node, key: next(x for x in node if isinstance(x, list) and x[0] == key)
    comps = {}
    for c in get(tree, "components")[1:]:
        fields = {}
        for f in next((x for x in c if isinstance(x, list) and x[0] == "fields"), [None])[1:]:
            if len(f) > 2:
                fields[str(f[1][1])] = str(f[2])
        comps[str(get(c, "ref")[1])] = dict(value=str(get(c, "value")[1]),
                                          footprint=str(get(c, "footprint")[1]),
                                          uuid=str(get(c, "tstamps")[1]), fields=fields)
    pads = {}
    for n in get(tree, "nets")[1:]:
        name = str(get(n, "name")[1]).lstrip("/")
        for node in n:
            if isinstance(node, list) and node[0] == "node":
                pads[(str(get(node, "ref")[1]), str(get(node, "pin")[1]))] = name
    return comps, pads


def segment(board, layer, a, b, width=0.1):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(a)
    s.SetEnd(b)
    s.SetLayer(layer)
    s.SetWidth(mm(width))
    board.Add(s)


def arc(board, layer, center, start, angle_deg, width=0.1):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_ARC)
    s.SetCenter(center)
    s.SetStart(start)
    s.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(angle_deg, pcbnew.DEGREES_T))
    s.SetLayer(layer)
    s.SetWidth(mm(width))
    board.Add(s)


def outline(board):
    w, h, r = design.BOARD_W, design.BOARD_H, 2.54
    E = pcbnew.Edge_Cuts
    segment(board, E, pt(r, 0), pt(w - r, 0))
    segment(board, E, pt(w, r), pt(w, h - r))
    segment(board, E, pt(w - r, h), pt(r, h))
    segment(board, E, pt(0, h - r), pt(0, r))
    arc(board, E, pt(r, r), pt(0, r), 90)
    arc(board, E, pt(w - r, r), pt(w - r, 0), 90)
    arc(board, E, pt(w - r, h - r), pt(w, h - r), 90)
    arc(board, E, pt(r, h - r), pt(r, h), 90)


def text(board, s, x, y, size=1.0, layer=pcbnew.F_SilkS, angle=0, thick=0.15, justify=None):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s)
    t.SetPosition(pt(x, y))
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    t.SetTextThickness(mm(thick))
    t.SetTextAngleDegrees(angle)
    if justify == "left":
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
    elif justify == "right":
        t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_RIGHT)
    if layer in (pcbnew.B_SilkS, pcbnew.B_Fab):
        t.SetMirrored(True)
    board.Add(t)


def rules(board):
    ds = board.GetDesignSettings()
    ds.SetCopperLayerCount(LAYERS)
    ds.m_TrackMinWidth = mm(0.15)
    ds.m_MinClearance = mm(CLEARANCE)
    ds.m_ViasMinSize = mm(0.5)
    ds.m_MinThroughDrill = mm(0.25)
    ds.m_CopperEdgeClearance = mm(0.25)    # Feather header pads sit 0.27 mm from the edge
    ds.m_HoleClearance = mm(0.25)
    ds.m_TextSize[pcbnew.LAYER_CLASS_SILK] = ds.m_TextSize[pcbnew.LAYER_CLASS_SILK]
    ds.m_MinSilkTextHeight = mm(0.5)
    ds.SetAuxOrigin(pt(0, design.BOARD_H))   # bottom-left corner: origin for Gerbers and CPL


def setup_netclasses(board, nets):
    ns = board.GetDesignSettings().m_NetSettings
    dflt = ns.m_DefaultNetClass
    dflt.SetTrackWidth(mm(TRACK_SIG))
    dflt.SetClearance(mm(CLEARANCE))
    dflt.SetViaDiameter(mm(0.5))
    dflt.SetViaDrill(mm(0.25))
    for name, width, members in NETCLASSES:
        nc = pcbnew.NETCLASS(name)
        nc.SetTrackWidth(mm(width))
        nc.SetClearance(mm(CLEARANCE))
        nc.SetViaDiameter(mm(0.6 if width >= 0.4 else 0.5))
        nc.SetViaDrill(mm(0.3 if width >= 0.4 else 0.25))
        ns.m_NetClasses[name] = nc
        for m in members:
            nets[m].SetNetClass(nc)


def flip(fp):
    try:
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
    except (AttributeError, TypeError):
        fp.Flip(fp.GetPosition(), False)


def make_footprint_lib():
    """Feather headers + mounting holes only, and a plain-pad solder jumper."""
    lib = os.path.join(PRJ, design.PROJECT + ".pretty")
    os.makedirs(lib, exist_ok=True)
    src = os.path.join(FP_ROOT, "Module.pretty", "Adafruit_Feather_WithMountingHoles.kicad_mod")
    tree = parse(open(src, encoding="utf-8").read())
    drop = {"fp_line", "fp_arc", "fp_circle", "fp_rect", "fp_poly", "fp_text", "model"}
    tree = [x for x in tree if not (isinstance(x, list) and x[0] in drop)]
    tree[1] = Str("Feather_Wing_Headers")
    for x in tree:
        if isinstance(x, list) and x[0] == "descr":
            x[1] = Str("FeatherWing header pads (16 + 12 pin) and 4 mounting holes; no outline or courtyard")
    with open(os.path.join(lib, "Feather_Wing_Headers.kicad_mod"), "w", encoding="utf-8", newline="\n") as f:
        f.write(dump(tree) + "\n")
    # Plain rectangular pads: custom-shape pads export to Freerouting on the wrong layer.
    src = os.path.join(FP_ROOT, "Jumper.pretty", "SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm.kicad_mod")
    tree = parse(open(src, encoding="utf-8").read())
    tree[1] = Str("SolderJumper_Plain")
    with open(os.path.join(lib, "SolderJumper_Plain.kicad_mod"), "w", encoding="utf-8", newline="\n") as f:
        f.write(dump(tree) + "\n")
    with open(os.path.join(PRJ, "fp-lib-table"), "w", encoding="utf-8", newline="\n") as f:
        f.write('(fp_lib_table\n\t(lib (name "%s")(type "KiCad")'
                '(uri "${KIPRJMOD}/%s.pretty")(options "")(descr "Bench stim wing footprints"))\n)\n'
                % (design.PROJECT, design.PROJECT))


def fp_dir(lib):
    if lib == design.PROJECT:
        return os.path.join(PRJ, lib + ".pretty")
    return os.path.join(FP_ROOT, lib + ".pretty")


def zone(board, layer, net, outline_pts=None, priority=0):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net)
    z.SetLocalClearance(mm(0.25))
    z.SetMinThickness(mm(0.2))
    z.SetThermalReliefGap(mm(0.3))
    z.SetThermalReliefSpokeWidth(mm(0.4))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetAssignedPriority(priority)
    z.SetIsFilled(False)
    poly = z.Outline()
    poly.NewOutline()
    pts = outline_pts or [(0, 0), (design.BOARD_W, 0), (design.BOARD_W, design.BOARD_H), (0, design.BOARD_H)]
    for x, y in pts:
        poly.Append(mm(OX + x), mm(OY + y))
    board.Add(z)
    return z


def build(route=True):
    import placement
    make_footprint_lib()
    comps, pads = read_netlist()
    board = pcbnew.CreateEmptyBoard()
    board.SetCopperLayerCount(LAYERS)
    rules(board)
    nets = {}
    for name in sorted(set(pads.values())):
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        nets[name] = ni
    setup_netclasses(board, nets)

    for ref, c in sorted(comps.items()):
        lib, name = c["footprint"].split(":")
        fp = pcbnew.FootprintLoad(fp_dir(lib), name)
        assert fp, c["footprint"]
        fp.SetParent(board)
        fp.SetReference(ref)
        fp.SetValue(c["value"])
        fp.SetPath(pcbnew.KIID_PATH("/" + c["uuid"]))
        fp.SetFPIDAsString(c["footprint"])
        x, y, rot, side = placement.PLACE[ref]
        fp.SetPosition(pt(x, y))
        fp.SetOrientationDegrees(rot)
        for k, v in c["fields"].items():
            if k not in ("Footprint", "Datasheet", "Description"):
                fp.SetProperty(k, v)
        if side == "B":
            flip(fp)
        fp.Reference().SetVisible(False)   # silk labels the functions instead; assembly uses the CPL
        fp.Value().SetVisible(False)
        if ref in design.HAND_SOLDERED:
            fp.SetExcludedFromBOM(True)
            fp.SetExcludedFromPosFiles(True)
        for pad in fp.Pads():
            net = pads.get((ref, pad.GetNumber()))
            if net:
                pad.SetNet(nets[net])
        board.Add(fp)

    outline(board)
    # In1: solid GND plane (placed before the DSN export so Freerouting drops vias to it).
    zone(board, pcbnew.In1_Cu, nets["GND"])
    placement.silkscreen(board, text, pcbnew)
    board.SetFileName(PCB)
    pcbnew.SaveBoard(PCB, board)
    check_pins(board)
    return board


def check_pins(board):
    """The wing must line up with the Feather: check a few header pads."""
    fp = board.FindFootprintByReference("J1")
    want = {"1": (6.35, 21.59), "16": (44.45, 21.59), "17": (44.45, 1.27), "28": (16.51, 1.27)}
    for pad in fp.Pads():
        if pad.GetNumber() in want:
            p = pad.GetPosition()
            got = (round(pcbnew.ToMM(p.x) - OX, 2), round(pcbnew.ToMM(p.y) - OY, 2))
            assert got == want[pad.GetNumber()], (pad.GetNumber(), got)


def drc(board, keep=None):
    os.makedirs(os.path.dirname(DRC), exist_ok=True)
    pcbnew.WriteDRCReport(board, DRC, pcbnew.EDA_UNITS_MILLIMETRES, True)
    txt = open(DRC, encoding="utf-8").read()
    blocks = re.split(r"\n(?=\[)", txt)
    hits = [b for b in blocks if b.startswith("[") and (keep is None or any(k in b for k in keep))]
    summary = re.findall(r"\*\* Found (\d+) (.*?) \*\*", txt)
    return hits, summary


def dsn_classes():
    """Put the power nets into their DSN classes. KiCad 7 only assigns net classes
    by pattern (not reachable from Python), so the exporter lists every net under
    kicad_default; move them so Freerouting uses the wide tracks."""
    txt = open(DSN, encoding="utf-8").read()
    head, sep, rest = txt.partition("(class kicad_default")
    dflt_end = rest.index("(circuit")
    members = rest[:dflt_end].split()
    for name, _w, nets in NETCLASSES:
        members = [m for m in members if m not in nets]
        rest = rest.replace(f"(class {name}\n", f"(class {name} " + " ".join(nets) + "\n", 1)
    rest = " " + " ".join(members) + "\n      " + rest[dflt_end:]
    open(DSN, "w", encoding="utf-8").write(head + sep + rest)


def place():
    board = build()
    os.makedirs(os.path.dirname(DSN), exist_ok=True)
    if not pcbnew.ExportSpecctraDSN(board, DSN):
        raise SystemExit("DSN export failed")
    dsn_classes()
    print("placed", len(board.GetFootprints()), "footprints; wrote", PCB, "and", DSN)


def check():
    board = build()
    hits, summary = drc(board, keep=["courtyard", "Courtyard", "edge", "Edge", "hole", "silk"])
    for h in hits:
        print(h.strip().replace("\n", " | ")[:300])
    print(summary)


def resilk(board):
    """Replace the board's free silk text with placement.silkscreen (no re-route needed)."""
    import placement
    for d in list(board.GetDrawings()):
        if isinstance(d, pcbnew.PCB_TEXT) and d.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
            board.Remove(d)
    placement.silkscreen(board, text, pcbnew)


def finish():
    board = pcbnew.LoadBoard(PCB)
    if not pcbnew.ImportSpecctraSES(board, SES):
        raise SystemExit("SES import failed")
    resilk(board)
    gnd = board.FindNet("GND")
    for z in list(board.Zones()):
        if not z.GetIsRuleArea() and z.GetLayer() != pcbnew.In1_Cu:
            board.Remove(z)
    for layer in (pcbnew.F_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
        zone(board, layer, gnd)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)
    hits, summary = drc(board)
    print("routed + poured:", len(board.GetTracks()), "tracks/vias")
    print(summary)
    for h in hits[:40]:
        print(h.strip().replace("\n", " | ")[:300])


def redsn():
    """Export the routed board (tracks kept, pours removed) for another Freerouting pass."""
    board = pcbnew.LoadBoard(PCB)
    for z in list(board.Zones()):
        if not z.GetIsRuleArea() and z.GetLayer() != pcbnew.In1_Cu:
            board.Remove(z)
    if not pcbnew.ExportSpecctraDSN(board, DSN):
        raise SystemExit("DSN export failed")
    dsn_classes()
    print("re-exported", DSN)


if __name__ == "__main__":
    {"place": place, "finish": finish, "check": check, "redsn": redsn}[sys.argv[1]]()
