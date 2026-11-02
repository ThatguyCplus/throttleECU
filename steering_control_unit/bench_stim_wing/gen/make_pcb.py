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
SES = os.path.join(HERE, design.PROJECT + ".ses")    # the accepted routing, committed
DRC = os.path.join(HERE, "route", "drc.rpt")
PLACED = os.path.join(HERE, "route", "placed.kicad_pcb")   # finish always starts from this
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
TRACK_SIG = 0.15
CLEARANCE = 0.127


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
    ds.m_TrackMinWidth = mm(0.127)
    ds.m_MinClearance = mm(CLEARANCE)
    ds.m_ViasMinSize = mm(0.45)
    ds.m_MinThroughDrill = mm(0.2)
    ds.m_CopperEdgeClearance = mm(0.25)    # Feather header pads sit 0.27 mm from the edge
    ds.m_HoleClearance = mm(0.25)
    ds.m_TextSize[pcbnew.LAYER_CLASS_SILK] = ds.m_TextSize[pcbnew.LAYER_CLASS_SILK]
    ds.m_MinSilkTextHeight = mm(0.5)
    ds.SetAuxOrigin(pt(0, design.BOARD_H))   # bottom-left corner: origin for Gerbers and CPL


def write_project():
    """KiCad 7+ keeps net classes in the .kicad_pro: write them there so DRC (and
    KiCad 10 when you open the board) uses the same rules the router used."""
    import json
    def cls(name, width, via, drill):
        return {"name": name, "clearance": CLEARANCE, "track_width": width, "via_diameter": via,
                "via_drill": drill, "microvia_diameter": 0.3, "microvia_drill": 0.1,
                "diff_pair_width": 0.2, "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25,
                "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)", "schematic_color": "rgba(0, 0, 0, 0.000)",
                "wire_width": 6, "bus_width": 12}
    classes = [cls("Default", TRACK_SIG, 0.45, 0.2)] + [
        cls(n, w, 0.6 if w >= 0.4 else 0.45, 0.3 if w >= 0.4 else 0.2) for n, w, _ in NETCLASSES]
    patterns = [{"netclass": n, "pattern": m} for n, _w, members in NETCLASSES for m in members]
    pro = {"meta": {"filename": design.PROJECT + ".kicad_pro", "version": 1},
           "net_settings": {"classes": classes, "meta": {"version": 3}, "net_colors": None,
                            "netclass_assignments": None, "netclass_patterns": patterns}}
    with open(os.path.join(PRJ, design.PROJECT + ".kicad_pro"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(pro, f, indent=2)
        f.write("\n")


def setup_netclasses(board, nets):
    ns = board.GetDesignSettings().m_NetSettings
    dflt = ns.m_DefaultNetClass
    dflt.SetTrackWidth(mm(TRACK_SIG))
    dflt.SetClearance(mm(CLEARANCE))
    dflt.SetViaDiameter(mm(0.45))
    dflt.SetViaDrill(mm(0.2))
    for name, width, members in NETCLASSES:
        nc = pcbnew.NETCLASS(name)
        nc.SetTrackWidth(mm(width))
        nc.SetClearance(mm(CLEARANCE))
        nc.SetViaDiameter(mm(0.6 if width >= 0.4 else 0.45))
        nc.SetViaDrill(mm(0.3 if width >= 0.4 else 0.2))
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
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
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
    write_project()
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
        fp.SetLocalClearance(0)            # e.g. MSOP-10 carries 0.2 mm; the board rule (0.127) applies instead
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
    import shutil
    shutil.copyfile(PCB, PLACED)
    print("placed", len(board.GetFootprints()), "footprints; wrote", PCB, "and", DSN)


def check():
    board = build()
    hits, summary = drc(board, keep=["courtyard", "Courtyard", "edge", "Edge", "hole", "silk"])
    for h in hits:
        print(h.strip().replace("\n", " | ")[:300])
    print(summary)


def import_ses(board):
    """Read Freerouting's .ses (wires + vias) into the board. KiCad 7's
    ImportSpecctraSES only works inside the editor, so this does it directly."""
    tree = parse(open(SES, encoding="utf-8").read())
    find = lambda node, key: [x for x in node if isinstance(x, list) and x[0] == key]
    routes = find(tree, "routes")[0]
    res = find(routes, "resolution")[0]
    scale = {"um": 1e-3, "mm": 1.0, "mil": 0.0254}[str(res[1])] / float(res[2])   # -> mm
    layers = {board.GetLayerName(l): l for l in range(pcbnew.PCB_LAYER_ID_COUNT)}
    # finish() starts from the placed board, which has no tracks yet
    n_seg = n_via = 0
    for net in find(find(routes, "network_out")[0], "net"):
        ni = board.FindNet(str(net[1]))
        for w in find(net, "wire"):
            path = find(w, "path")[0]
            layer, width = layers[str(path[1])], float(path[2]) * scale
            xy = [float(v) * scale for v in path[3:]]
            pts = [pcbnew.VECTOR2I(mm(xy[i]), mm(-xy[i + 1])) for i in range(0, len(xy), 2)]
            for a, b in zip(pts, pts[1:]):
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(a)
                t.SetEnd(b)
                t.SetWidth(mm(width))
                t.SetLayer(layer)
                t.SetNet(ni)
                board.Add(t)
                n_seg += 1
        for v in find(net, "via"):
            m = re.search(r"_(\d+):(\d+)_um", str(v[1]))
            dia, drill = (int(m.group(1)) / 1000, int(m.group(2)) / 1000) if m else (0.5, 0.25)
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(pcbnew.VECTOR2I(mm(float(v[2]) * scale), mm(-float(v[3]) * scale)))
            via.SetWidth(mm(dia))
            via.SetDrill(mm(drill))
            via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            via.SetNet(ni)
            board.Add(via)
            n_via += 1
    print(f"imported {n_seg} track segments and {n_via} vias from {os.path.basename(SES)}")


def manual_routes(board):
    """Hand-placed stubs and vias from placement.MANUAL_ROUTES, for the last few
    connections Freerouting leaves open. They belong to this routing result:
    re-check them (DRC) whenever the board is re-routed."""
    import placement
    tracks = list(board.GetTracks())        # one snapshot (KiCad 7 SWIG: don't re-list after edits)
    near = lambda p, x, y: abs(pcbnew.ToMM(p.x) - OX - x) < 0.02 and abs(pcbnew.ToMM(p.y) - OY - y) < 0.02
    for item in getattr(placement, "MANUAL_ROUTES", []):
        kind, net = item[0], board.FindNet(item[1])
        if kind == "delete":
            _, name, x1, y1, x2, y2 = item
            hit = [t for t in tracks if t.GetNetname() == name and not isinstance(t, pcbnew.PCB_VIA)
                   and ((near(t.GetStart(), x1, y1) and near(t.GetEnd(), x2, y2))
                        or (near(t.GetStart(), x2, y2) and near(t.GetEnd(), x1, y1)))]
            assert len(hit) == 1, ("manual delete matched", len(hit), item)
            board.Remove(hit[0])
            tracks.remove(hit[0])
        elif kind == "via":
            _, _, x, y = item
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pt(x, y))
            v.SetWidth(mm(0.45))
            v.SetDrill(mm(0.2))
            v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            v.SetNet(net)
            board.Add(v)
        else:
            _, _, layer, x1, y1, x2, y2, w = item
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pt(x1, y1))
            t.SetEnd(pt(x2, y2))
            t.SetWidth(mm(w))
            t.SetLayer(getattr(pcbnew, layer))
            t.SetNet(net)
            board.Add(t)


def resilk(board):
    """Replace the board's free silk text with placement.silkscreen (no re-route needed)."""
    import placement
    for d in list(board.GetDrawings()):
        if isinstance(d, pcbnew.PCB_TEXT) and d.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
            board.Remove(d)
    placement.silkscreen(board, text, pcbnew)


def finish():
    import shutil
    shutil.copyfile(PLACED, PCB)
    board = pcbnew.LoadBoard(PCB)
    # silk comes from the placed board (place re-applies placement.silkscreen)
    import_ses(board)
    manual_routes(board)
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
