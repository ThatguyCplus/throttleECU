"""Generate eps_tap_wing.kicad_pcb from the schematic's netlist + design.PLACE.

Run with KiCad's bundled Python (needs pcbnew):
    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" make_pcb.py place   # parts + outline, writes .dsn
    (run Freerouting on the .dsn -> .ses, see route.sh)
    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" make_pcb.py finish  # import .ses, pour GND, save
"""
import os
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
FP_ROOT = r"C:\Program Files\KiCad\10.0\share\kicad\footprints"
OX, OY = 100.0, 80.0          # board origin on the page
POWER = {"GND", "+3V3", "+5V"}
# Track widths (mm); overridable so a routing sweep can try alternatives.
TRACK_SIG = float(os.environ.get("EPS_TRACK_SIG", "0.3"))
TRACK_PWR = float(os.environ.get("EPS_TRACK_PWR", "0.3"))   # 3V3/5V stubs; GND is the In1 plane
FINE_EXPANSION = os.environ.get("EPS_FINE_EXPANSION", "0") == "1"   # expansion nets at 0.2 mm
# Fitted by hand, not by the assembler: headers, connectors, and the back-side
# status LED (avoids paying for a second, bottom-side assembly pass).
HAND_SOLDERED = {"J1", "J2", "J3", "D6", "R32"}
LAYERS = int(os.environ.get("EPS_LAYERS", "4"))                     # copper layers: 2 or 4


def mm(v):
    return pcbnew.FromMM(v)


def pt(x, y):
    return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


def read_netlist():
    tree = parse(open(NET, encoding="utf-8").read())
    get = lambda node, key: next(x for x in node if isinstance(x, list) and x[0] == key)
    comps = {}
    for c in get(tree, "components")[1:]:
        comps[str(get(c, "ref")[1])] = dict(value=str(get(c, "value")[1]),
                                          footprint=str(get(c, "footprint")[1]),
                                          uuid=str(get(c, "tstamps")[1]),
                                          fields={str(f[1][1]): str(f[2]) for f in get(c, "fields")[1:]
                                                  if len(f) > 2 and str(f[1][1]) not in ("Footprint",)})
    pads = {}
    for n in get(tree, "nets")[1:]:
        name = str(get(n, "name")[1])
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
    # KiCad arcs run clockwise on screen for a positive angle (y down).
    arc(board, E, pt(r, r), pt(0, r), 90)
    arc(board, E, pt(w - r, r), pt(w - r, 0), 90)
    arc(board, E, pt(w - r, h - r), pt(w, h - r), 90)
    arc(board, E, pt(r, h - r), pt(r, h), 90)


def text(board, s, x, y, size=1.0, layer=pcbnew.F_SilkS, angle=0, thick=0.15):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s)
    t.SetPosition(pt(x, y))
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    t.SetTextThickness(mm(thick))
    t.SetTextAngleDegrees(angle)
    if layer in (pcbnew.B_SilkS, pcbnew.B_Fab):
        t.SetMirrored(True)
    board.Add(t)


def rules(board):
    ds = board.GetDesignSettings()
    ds.SetCopperLayerCount(LAYERS)
    ds.m_TrackMinWidth = mm(0.15)
    ds.m_MinClearance = mm(0.15)
    ds.m_ViasMinSize = mm(0.5)
    ds.m_MinThroughDrill = mm(0.25)
    ds.m_CopperEdgeClearance = mm(0.25)    # Feather header pads sit 0.27 mm from the edge
    ds.m_HoleClearance = mm(0.25)
    ds.SetAuxOrigin(pt(0, design.BOARD_H))   # bottom-left corner: origin for Gerbers and CPL
    nc = ds.m_NetSettings.GetDefaultNetclass()
    nc.SetTrackWidth(mm(TRACK_SIG))
    nc.SetClearance(mm(0.15))
    nc.SetViaDiameter(mm(0.5))
    nc.SetViaDrill(mm(0.25))
    pwr = pcbnew.NETCLASS("Power")
    pwr.SetTrackWidth(mm(TRACK_PWR))
    pwr.SetClearance(mm(0.15))
    pwr.SetViaDiameter(mm(0.5))
    pwr.SetViaDrill(mm(0.25))
    ds.m_NetSettings.SetNetclass("Power", pwr)
    for n in POWER:
        ds.m_NetSettings.SetNetclassPatternAssignment(n, "Power")
    # Expansion and LED nets (microamps to ~1.5 mA): 0.2 mm, so they fit between
    # the J2 pins; every original channel keeps the 0.25 mm default.
    fine = pcbnew.NETCLASS("Expansion")
    fine.SetTrackWidth(mm(0.2))
    fine.SetClearance(mm(0.15))
    fine.SetViaDiameter(mm(0.5))
    fine.SetViaDrill(mm(0.25))
    ds.m_NetSettings.SetNetclass("Expansion", fine)
    fine_nets = ["/D13_LED", "/LED_A"]
    if FINE_EXPANSION:
        fine_nets += ["/TAP_X*", "/IN_X*", "/AIN_X*", "/SDA", "/SCL"]
    for pattern in fine_nets:
        ds.m_NetSettings.SetNetclassPatternAssignment(pattern, "Expansion")


def flip(fp):
    try:
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
    except (AttributeError, TypeError):
        fp.Flip(fp.GetPosition(), False)


def make_footprint_lib():
    """Feather headers + mounting holes only (the stock footprint also draws the
    Feather outline and a courtyard covering the whole board)."""
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

    # Bypass solder jumper without a courtyard: they sit at the J2 pitch (2.5 mm)
    # and are bare pads, so there is no body for a courtyard to protect.
    src = os.path.join(FP_ROOT, "Jumper.pretty", "SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm.kicad_mod")   # plain pads: custom shapes export to the DSN on the wrong layer
    tree = parse(open(src, encoding="utf-8").read())
    tree = [x for x in tree if not (isinstance(x, list) and x[0] in ("fp_line", "fp_rect", "fp_poly")
                                    and any(isinstance(y, list) and y[:1] == ["layer"] and "CrtYd" in str(y[1])
                                            for y in x))]
    tree[1] = Str("SolderJumper_Bypass")
    with open(os.path.join(lib, "SolderJumper_Bypass.kicad_mod"), "w", encoding="utf-8", newline="\n") as f:
        f.write(dump(tree) + "\n")
    with open(os.path.join(PRJ, "fp-lib-table"), "w", encoding="utf-8", newline="\n") as f:
        f.write('(fp_lib_table\n\t(version 7)\n\t(lib (name "eps_tap_wing")(type "KiCad")'
                '(uri "${KIPRJMOD}/eps_tap_wing.pretty")(options "")(descr "EPS tap wing footprints"))\n)\n')


def fp_dir(lib):
    if lib == design.PROJECT:
        return os.path.join(PRJ, lib + ".pretty")
    return os.path.join(FP_ROOT, lib + ".pretty")


def place():
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

    for ref, c in sorted(comps.items()):
        lib, name = c["footprint"].split(":")
        fp = pcbnew.FootprintLoad(fp_dir(lib), name)
        fp.SetParent(board)
        fp.SetReference(ref)
        fp.SetValue(c["value"])
        fp.SetPath(pcbnew.KIID_PATH("/" + c["uuid"]))
        fp.SetSheetname("/")
        fp.SetSheetfile(design.PROJECT + ".kicad_sch")
        fp.SetFPIDAsString(c["footprint"])
        x, y, rot, side = design.PLACE[ref]
        fp.SetPosition(pt(x, y))
        fp.SetOrientationDegrees(rot)
        for k, v in c["fields"].items():
            fp.SetField(k, v)
            fp.GetField(k).SetVisible(False)
        if side == "B":
            flip(fp)
        fp.Reference().SetVisible(False)   # silk names the terminals; assembly uses the CPL file
        if ref in HAND_SOLDERED:
            fp.SetExcludedFromBOM(True)        # keeps them out of the assembly BOM/CPL
            fp.SetExcludedFromPosFiles(True)
        for pad in fp.Pads():
            net = pads.get((ref, pad.GetNumber()))
            if net:
                pad.SetNet(nets[net])
        board.Add(fp)

    outline(board)
    if LAYERS == 4:
        # In1 is a solid GND plane, placed before the DSN export so Freerouting
        # drops a via to it instead of running GND traces. In2 is a third
        # routing layer (signals and 3V3); it gets a GND pour after routing.
        zone(board, pcbnew.In1_Cu, nets["GND"])
    # Silkscreen. The back is the wiring side and the face you see when installed
    # (the Feather sits on top of this wing), so labels and branding go there.
    B = pcbnew.B_SilkS
    text(board, "HINDSIGHT", 10.0, 2.1, 1.2, layer=B, thick=0.2)
    text(board, "IGN OFF TO PLUG", 10.0, 4.2, 0.8, layer=B, thick=0.12)
    for i, (_, short, _src) in enumerate(design.J2_PINS):
        x0 = max(design.J2_X0 + design.J2_PITCH * i, 4.5)   # VT3 label clears the corner hole
        text(board, short, x0, 18.4, 0.8, layer=B, angle=90, thick=0.12)
    text(board, "CLAMP", 45.0, 18.4, 0.8, layer=B, thick=0.12)
    text(board, "LED", 33.0, 4.2, 0.8, layer=B, thick=0.12)
    text(board, "EPS TAP " + design.REV, 10.0, 2.2, 0.8, thick=0.12)   # top: revision

    board.SetFileName(PCB)
    pcbnew.SaveBoard(PCB, board)
    check_pins(board)
    os.makedirs(os.path.dirname(DSN), exist_ok=True)
    if not pcbnew.ExportSpecctraDSN(board, DSN):
        raise SystemExit("DSN export failed")
    print("placed", len(comps), "footprints; wrote", PCB, "and", DSN)


def check_pins(board):
    """The wing must line up with the Feather: check a few header pads."""
    fp = board.FindFootprintByReference("J1")
    want = {"1": (6.35, 21.59), "16": (44.45, 21.59), "17": (44.45, 1.27), "28": (16.51, 1.27)}
    for pad in fp.Pads():
        if pad.GetNumber() in want:
            p = pad.GetPosition()
            got = (round(pcbnew.ToMM(p.x) - OX, 2), round(pcbnew.ToMM(p.y) - OY, 2))
            assert got == want[pad.GetNumber()], (pad.GetNumber(), got)


def zone(board, layer, net):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net)
    z.SetLocalClearance(mm(0.25))
    z.SetMinThickness(mm(0.2))
    z.SetThermalReliefGap(mm(0.3))
    z.SetThermalReliefSpokeWidth(mm(0.4))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetIsFilled(False)
    poly = z.Outline()
    poly.NewOutline()
    for x, y in [(0, 0), (design.BOARD_W, 0), (design.BOARD_W, design.BOARD_H), (0, design.BOARD_H)]:
        poly.Append(mm(OX + x), mm(OY + y))
    board.Add(z)


def finish():
    board = pcbnew.LoadBoard(PCB)
    if not pcbnew.ImportSpecctraSES(board, SES):
        raise SystemExit("SES import failed")
    gnd = board.FindNet("GND")
    # Feather GND header pin: solid to the pour (a thermal there ends up with one spoke).
    for pad in board.FindFootprintByReference("J1").Pads():
        if pad.GetNetname() == "GND":
            pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
    for z in list(board.Zones()):
        if not z.GetIsRuleArea() and z.GetLayer() in (pcbnew.F_Cu, pcbnew.B_Cu):
            board.Remove(z)                 # inner planes (4-layer) stay
    for z in list(board.Zones()):
        if not z.GetIsRuleArea() and z.GetLayer() == pcbnew.In2_Cu:
            board.Remove(z)
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu) + ((pcbnew.In2_Cu,) if board.GetCopperLayerCount() == 4 else ()):
        zone(board, layer, gnd)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)
    print("routed + poured:", len(board.GetTracks()), "tracks/vias")


def mark_hand_soldered():
    """Apply HAND_SOLDERED attributes to the existing routed board (no re-route)."""
    board = pcbnew.LoadBoard(PCB)
    for ref in sorted(HAND_SOLDERED):
        fp = board.FindFootprintByReference(ref)
        fp.SetExcludedFromBOM(True)
        fp.SetExcludedFromPosFiles(True)
    pcbnew.SaveBoard(PCB, board)
    print("marked", ", ".join(sorted(HAND_SOLDERED)), "as hand-soldered")


def redsn():
    """Export the routed board (tracks kept, pours removed) so Freerouting can
    continue from it and close any connections the first pass left open."""
    board = pcbnew.LoadBoard(PCB)
    for z in list(board.Zones()):
        if not z.GetIsRuleArea():
            board.Remove(z)
    if not pcbnew.ExportSpecctraDSN(board, DSN):
        raise SystemExit("DSN export failed")
    print("re-exported", DSN)


if __name__ == "__main__":
    {"place": place, "finish": finish, "redsn": redsn, "mark": mark_hand_soldered}[sys.argv[1]]()
