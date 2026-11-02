"""Generate icarus.kicad_pcb (4 layers) from the schematic netlist + a floorplan.

Run with KiCad's bundled Python (needs pcbnew), from hardware/:
    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" gen/make_pcb.py place    # parts, outline, rules, DSN
    (Freerouting: see gen/route.sh)
    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" gen/make_pcb.py finish   # import SES, pours, fill

Floorplan (board W x H, default 130 x 100 mm, origin top-left; regions sized by regions()). It follows the safety architecture:
CH-A and CH-B live in separate halves, the CH-A analog sits next to CH-A, the CH-C guard next
to CH-B, and the noisy power/relay parts stay by the connector on the left edge.

Stack-up: see STACK below (SGSS, SGPS or SGSSPS).
"""
import json
import os
import re
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import design  # noqa: E402
import placer  # noqa: E402
import silk  # noqa: E402
from sexpr import Str, dump, parse  # noqa: E402

PRJ = os.path.normpath(os.path.join(HERE, ".."))
PCB = os.path.join(PRJ, design.PROJECT + ".kicad_pcb")
NET = os.path.join(PRJ, design.PROJECT + ".net")
DSN = os.path.join(HERE, "route", design.PROJECT + ".dsn")
SES = os.path.join(HERE, "route", design.PROJECT + ".ses")
FP_ROOT = r"C:\Program Files\KiCad\10.0\share\kicad\footprints"
OX, OY = 100.0, 50.0                  # board origin on the page
# Placement configs (ICARUS_CONFIG). "single" is rev 0.5 as routed: one side, 152 x 115 mm.
# The 100 x 100 mm configs (rev 0.6 exploration) place on both sides:
# - All through-hole parts (J1, fuse holder, hold-up electrolytic) sit in one edge STRIP that both
#   sides keep clear. Everything else is SMD, so a region can sit on either side.
# - A bottom-side cluster is mirrored as a unit, so every cap stays at its pin.
# - LEDs and test pads always stay on top (visible, probeable).
# Each side has its own columns (left to right, top view); RULES map design groups to regions.
SIGNAL_FLOW = [["entry", "pass", "inputs"], ["supply", "coil", "backup"], ["ch_a", "ch_b"],
               ["overlay", "guard", "measure"]]
CONFIGS = {
    "single": dict(W=152.0, H=115.0, strip=False, top=SIGNAL_FLOW, bottom=[],
                   gaps=(0.6, 1.0, 1.2, 1.5)),
    # The four below came out of gen/search_floorplan.py (best fit per engineering theme at 100 x 100).
    # floaters=True: test pads and LEDs sit next to the signal they show, always on top (place_floaters).
    # A: analog and supplies on top. In -> pass-through -> supplies + overlay -> guard on top, the
    #    switchers and the precision analog both next to the In1 plane; MCUs, coil drive, backup and
    #    monitors underneath. Best fit of the search (23.8 mm spare width).
    # anchor_rot: U15 (TMS570, CH-B) turned 180° puts its GIOA inputs and N2HET pins on the side
    # facing the input conditioning; at 0° WLP_B / DISNC_B were walled in by the crystal (rev 0.7)
    # overlay above supply: the overlay is CH-A analog, so it sits over CH-A (U10). Over U15 its
    # In2 traffic walled in U15's pin field (3 nets open after Freerouting + rrr); the supplies,
    # with few signals, now sit over U15 instead
    "A": dict(W=100.0, H=100.0, strip=True, floaters=True, spread=True, stack="SGSS",
              top=[["entry", "pass", "inputs"], ["overlay", "supply"], ["guard"]],
              bottom=[["coil", "backup"], ["ch_a", "ch_b"], ["measure"]],
              gaps=(0.4, 0.8, 1.0, 1.2), anchor_rot={"U15": 180}),
    # B: MCUs on top: decoupling and the oscillator next to the In1 plane, JTAG headers on top, MCU
    #    status LEDs right at their pins. Supplies, coils, backup, input conditioning, guard underneath.
    "B": dict(W=100.0, H=100.0, strip=True, floaters=True,
              top=[["entry", "pass"], ["ch_a", "ch_b"], ["overlay", "measure"]],
              bottom=[["inputs", "supply", "coil"], ["backup", "guard"]],
              gaps=(0.4, 0.8, 1.0, 1.2)),
    # C: power on top. Supplies, coil drive and backup (all the heat and the through-hole thermal vias)
    #    on top with the guard; MCUs, input conditioning, overlay and monitors underneath.
    "C": dict(W=100.0, H=100.0, strip=True, floaters=True,
              top=[["entry", "pass", "supply"], ["coil", "backup"], ["guard"]],
              bottom=[["inputs", "ch_a"], ["ch_b", "overlay", "measure"]],
              gaps=(0.4, 0.8, 1.0, 1.2)),
    # D: channel split. CH-A, its overlay and the coil high sides on top; CH-B, its supplies and the
    #    guard it references (CH-C runs on the B rail) underneath.
    "D": dict(W=100.0, H=100.0, strip=True, floaters=True,
              top=[["entry", "pass", "coil"], ["backup", "ch_a"], ["overlay", "measure"]],
              bottom=[["inputs"], ["supply", "ch_b"], ["guard"]],
              gaps=(0.4, 0.8, 1.0, 1.2)),
}
CONFIG = os.environ.get("ICARUS_CONFIG", "single")
CFG = dict(CONFIGS[CONFIG])
for _side in ("top", "bottom"):                # quick experiments: ICARUS_TOP / ICARUS_BOTTOM = JSON column lists
    if os.environ.get("ICARUS_" + _side.upper()):
        CFG[_side] = json.loads(os.environ["ICARUS_" + _side.upper()])
W = float(os.environ.get("ICARUS_W", CFG["W"]))
H = float(os.environ.get("ICARUS_H", CFG["H"]))
if os.environ.get("ICARUS_GAPS"):               # e.g. "0.4,0.8,2.5,2.0": spend spare board on routing channels
    CFG["gaps"] = tuple(float(v) for v in os.environ["ICARUS_GAPS"].split(","))
SMALL_GAP, GAP, BLOCK_GAP, COL_GAP = CFG["gaps"]   # 0402-class / medium / between clusters / between regions
IC_GAP = 2.5                          # extra fan-out room around large ICs (single-sided config)
# Stack-up, one letter per copper layer from F.Cu down: S = signal, G = GND plane, P = split power plane.
#   "SGSS"   (rev 0.5): F sig | In1 GND | In2 sig | B sig
#   "SGPS"   F sig | In1 GND | In2 power | B sig: two signal layers, F vertical / B horizontal
#   "SGSSPS" (rev 0.6): F sig | In1 GND | In2 sig | In3 sig | In4 power | B sig. Both outer layers sit next
#            to a plane (parts on both sides), and there are four signal layers.
# Signal layers alternate vertical / horizontal from F.Cu down; the router gets that as its preferred
# direction. Planes are LT_POWER: the router never puts tracks there. The power plane is split into
# one island per rail in RAILS (power_islands); every SMD pad on a rail gets its own via down to it.
STACK = os.environ.get("ICARUS_STACK", CFG.get("stack", "SGSS"))
_CU = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.In3_Cu, pcbnew.In4_Cu]
_LAY = _CU[:len(STACK) - 1] + [pcbnew.B_Cu]
N_LAYERS = len(STACK)
SIG = [l for l, r in zip(_LAY, STACK) if r == "S"]
GND_PLANES = [l for l, r in zip(_LAY, STACK) if r == "G"]
PWR_PLANES = [l for l, r in zip(_LAY, STACK) if r == "P"]
PLANES = GND_PLANES + PWR_PLANES
DIRECTION = {l: ("vertical" if i % 2 == 0 else "horizontal") for i, l in enumerate(SIG)}
RAILS = ["+3V3_A", "+3V3_B", "+5V0_A", "+5V0_B", "VBAT_P"]

# Floorplan by signal flow, left to right. Beside the connector, in its pin order: what comes IN
# (power entry + CAN, pins 1-5), the unbroken PASS-THROUGH path (relay NC contacts, pins 6-11) and the
# vehicle INPUTS (pins 12-20). Then what goes OUT (coil drive, the CH-A overlay) with the supplies and
# backup, then the two MCUs, then what MEASURES (the CH-C guard, the monitors). Test pads sit in the
# matching cluster (single) or in one "tp" region of three rows on top (double-sided configs).
COLUMNS = CFG["top"]
SIDE = {r: "top" for col in CFG["top"] for r in col}
SIDE.update({r: "bottom" for col in CFG["bottom"] for r in col})
TP_REGION = "tp" if "tp" in SIDE else None
RULES = [   # (sheet, group prefix, region); first match wins
    ("power", "Input protection", "entry"), ("io", "CAN bus", "entry"),
    ("relays", "Relays:", "pass"), ("relays", "Indicators", "pass"),
    ("io", "Digital inputs", "inputs"), ("io", "Disengage", "inputs"), ("ch_b", "Driver indicator", "inputs"),
    ("test", "Test points: incoming", TP_REGION or "inputs"),
    ("power", "", "supply"),
    ("relays", "", "coil"), ("test", "Test points: outgoing", TP_REGION or "coil"),
    ("overlay", "", "overlay"), ("backup", "", "backup"), ("ch_a", "", "ch_a"), ("ch_b", "", "ch_b"),
    ("guard", "", "guard"), ("io", "Analog monitors", "measure"), ("test", "Test points: measured", TP_REGION or "measure"),
]
CONN_X = 13.0 if CFG["strip"] else 9.5   # left edge strip: J1 (+ fuse holder and electrolytic in strip configs)
TITLE_STRIP = 1.8                     # silk title strip at the top of each region
REGIONS = {}                          # region -> (x0, y0, x1, y1), filled by regions()


def placed_as(p):
    """A part as the placer sees it: design.place_with moves it into another group's cluster and region."""
    g = p.get("place_with")
    if not g:
        return p
    return dict(p, group=g, sheet=next(q["sheet"] for q in design.PARTS if q["group"] == g))


def region_of(p):
    """Floorplan region for a design part (RULES). Indicator LEDs whose cluster went to the bottom
    come back up to the config's led_home region: an LED under the board is no use."""
    g = p["group"] or ""
    r = next(r for sh, pre, r in RULES if p["sheet"] == sh and g.startswith(pre))
    if g.startswith("Indicators") and SIDE.get(r) == "bottom":
        return CFG["led_home"]
    return r


# Vehicle connector on the left edge, pin 1 at the top (found by symbol, not designator). In strip
# configs the fuse holder sits above it (next to pin 1, VBAT_IN) and the hold-up electrolytic below.
VEHICLE_J = next(p["ref"] for p in design.PARTS if p["lib"] == "Connector_Generic:Conn_01x20")
FUSE_HOLDER = next(p["ref"] for p in design.PARTS if p["fp"].startswith("Fuse:Fuseholder"))
HOLDUP = next(p["ref"] for p in design.PARTS if p["fp"].startswith("Capacitor_THT"))
STRIP = [(FUSE_HOLDER, 90), (VEHICLE_J, 0), (HOLDUP, 0)] if CFG["strip"] else []   # stacked top to bottom
FIXED = {r: None for r, _rot in STRIP} if STRIP else {VEHICLE_J: (3.6, round((H - 19 * 2.54) / 2, 2), 0)}
OVERFLOW = []
MOUNT_HOLES = [(3.5, 3.5), (W - 3.5, 3.5), (3.5, H - 3.5), (W - 3.5, H - 3.5)]

POWER = {"VBAT_P", "+5V0_A", "+5V0_B", "+5V0_A_SRC", "VBK5", "VLI", "COIL1_HI", "COIL2_HI"}
HIGH_CURRENT = {"VBAT_IN", "VBAT_F", "VBAT_FP", "SW_A", "SW_B", "SWB", "BAT_P_IN"}
# Hand-fitted after reflow (THT or large): kept out of the assembly CPL
HAND_SOLDERED_LIBS = ("Connector_PinHeader", "Capacitor_THT", "Fuse:Fuseholder", "Connector_JST",
                      "TestPoint")


def mm(v):
    return pcbnew.FromMM(v)


def pt(x, y):
    return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


def read_netlist():
    tree = parse(open(NET, encoding="utf-8").read())
    get = lambda node, key: next(x for x in node if isinstance(x, list) and x[0] == key)
    comps = {}
    for c in get(tree, "components")[1:]:
        sp = get(c, "sheetpath")
        comps[str(get(c, "ref")[1])] = dict(
            value=str(get(c, "value")[1]), footprint=str(get(c, "footprint")[1]),
            path=str(get(sp, "tstamps")[1]) + str(get(c, "tstamps")[1]),
            sheetname=str(get(sp, "names")[1]),
            fields={str(f[1][1]): str(f[2]) for f in get(c, "fields")[1:]
                    if len(f) > 2 and str(f[1][1]) not in ("Footprint", "Datasheet")})
    pads, ptypes = {}, {}
    for n in get(tree, "nets")[1:]:
        name = str(get(n, "name")[1])
        for node in n:
            if isinstance(node, list) and node[0] == "node":
                key = (str(get(node, "ref")[1]), str(get(node, "pin")[1]))
                pads[key] = name
                pt_ = next((x for x in node if isinstance(x, list) and x[0] == "pintype"), None)
                ptypes[key] = str(pt_[1]) if pt_ else ""
    return comps, pads, ptypes


def fp_dir(lib):
    return os.path.join(PRJ, "icarus.pretty") if lib == "icarus" else os.path.join(FP_ROOT, lib + ".pretty")


def clean_vendor_library():
    """Clean the vendor (UltraLibrarian) footprints once, in icarus.pretty itself, so board and
    library agree. File-level (S-expressions), because pcbnew's DRAWINGS container breaks on
    some of these footprints. Idempotent.
    - drop literal 'Designator..' silk texts and repeated texts (duplicate pin-1 marks); the courtyard
      covers the remaining pin-1 marks
    - replace the courtyard (some are open shapes plus a stray crosshair) by a closed rectangle
      around the pads and the fab outline, +0.25 mm
    - thermal-via pads named 'V' take the number of the SMD pad they sit in (net + parity)"""
    lib = fp_dir("icarus")
    for f in sorted(os.listdir(lib)):
        if not f.endswith(".kicad_mod"):
            continue
        path = os.path.join(lib, f)
        tree = parse(open(path, encoding="utf-8").read())
        layer_of = lambda n: next((str(x[1]) for x in n if isinstance(x, list) and x[0] == "layer"), "")
        sub = lambda n, k: next((x for x in n if isinstance(x, list) and x[0] == k), None)
        keep, seen = [], set()
        for n in tree:
            if isinstance(n, list) and n[0] == "fp_text" and "Designator" in str(n[2]):
                continue
            if isinstance(n, list) and n[0] == "fp_text":      # vendor files repeat their pin-1 "*" texts
                at = sub(n, "at")
                key = (str(n[2]), layer_of(n), float(at[1]), float(at[2]))
                if key in seen:
                    continue
                seen.add(key)
            if isinstance(n, list) and n[0].startswith("fp_") and layer_of(n) == "F.CrtYd":
                continue
            keep.append(n)
        tree = keep
        xs, ys, smd = [], [], []
        for n in tree:
            if isinstance(n, list) and n[0] == "pad":
                at, size = sub(n, "at"), sub(n, "size")
                x, y = float(at[1]), float(at[2])
                rot = float(at[3]) if len(at) > 3 else 0
                w, h = float(size[1]), float(size[2])
                if rot % 180:
                    w, h = h, w
                xs += [x - w / 2, x + w / 2]
                ys += [y - h / 2, y + h / 2]
                if n[2] == "smd" and str(n[1]):
                    smd.append((str(n[1]), x - w / 2, y - h / 2, x + w / 2, y + h / 2))
            elif isinstance(n, list) and n[0] in ("fp_line", "fp_rect") and layer_of(n) == "F.Fab":
                for k in ("start", "end"):
                    p = sub(n, k)
                    xs.append(float(p[1]))
                    ys.append(float(p[2]))
            elif isinstance(n, list) and n[0] == "fp_text" and layer_of(n) == "F.SilkS":
                at = sub(n, "at")                  # pin-1 marks: inside the courtyard, so nothing sits on them
                xs += [float(at[1]) - 0.5, float(at[1]) + 0.5]
                ys += [float(at[2]) - 0.6, float(at[2]) + 0.6]
        for n in tree:
            if isinstance(n, list) and n[0] == "pad" and n[2] == "thru_hole" and not str(n[1]).isdigit():
                at = sub(n, "at")
                x, y = float(at[1]), float(at[2])
                host = next((h for h in smd if h[1] <= x <= h[3] and h[2] <= y <= h[4]), None)
                if host:
                    n[1] = Str(host[0])
        m = 0.25
        tree.append(["fp_rect", ["start", round(min(xs) - m, 3), round(min(ys) - m, 3)],
                     ["end", round(max(xs) + m, 3), round(max(ys) + m, 3)],
                     ["layer", Str("F.CrtYd")], ["width", 0.05], ["fill", "none"]])
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(dump(tree) + "\n")


def courtyard(fp):
    """(dx0, dy0, dx1, dy1) of the courtyard relative to the footprint origin, unrotated."""
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    bb = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False)
    p = fp.GetPosition()
    return (pcbnew.ToMM(bb.GetLeft() - p.x), pcbnew.ToMM(bb.GetTop() - p.y),
            pcbnew.ToMM(bb.GetRight() - p.x), pcbnew.ToMM(bb.GetBottom() - p.y))


def net_name(n):
    return n.split("/")[-1]


def rules(board):
    ds = board.GetDesignSettings()
    ds.SetCopperLayerCount(N_LAYERS)
    ds.m_TrackMinWidth = mm(0.15)
    ds.m_MinClearance = mm(0.15)
    ds.m_ViasMinSize = mm(0.45)
    ds.m_MinThroughDrill = mm(0.2)
    ds.m_CopperEdgeClearance = mm(0.3)
    ds.m_HoleClearance = mm(0.25)
    ds.SetAuxOrigin(pt(0, H))           # bottom-left: origin for Gerbers and CPL
    nc = ds.m_NetSettings.GetDefaultNetclass()
    nc.SetTrackWidth(mm(0.2))
    nc.SetClearance(mm(0.15))
    nc.SetViaDiameter(mm(0.5))
    nc.SetViaDrill(mm(0.25))
    for name, width, nets, clr in (("Power", 0.3, POWER, 0.15), ("HighCurrent", 0.6, HIGH_CURRENT, 0.15)):
        c = pcbnew.NETCLASS(name)
        c.SetTrackWidth(mm(width))
        c.SetClearance(mm(clr))
        c.SetViaDiameter(mm(0.6))
        c.SetViaDrill(mm(0.3))
        ds.m_NetSettings.SetNetclass(name, c)
        for n in nets:
            ds.m_NetSettings.SetNetclassPatternAssignment("*" + n, name)


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
    r, E = 3.0, pcbnew.Edge_Cuts
    segment(board, E, pt(r, 0), pt(W - r, 0))
    segment(board, E, pt(W, r), pt(W, H - r))
    segment(board, E, pt(W - r, H), pt(r, H))
    segment(board, E, pt(0, H - r), pt(0, r))
    arc(board, E, pt(r, r), pt(0, r), 90)
    arc(board, E, pt(W - r, r), pt(W - r, 0), 90)
    arc(board, E, pt(W - r, H - r), pt(W, H - r), 90)
    arc(board, E, pt(r, H - r), pt(r, H), 90)


def text(board, s, x, y, size=1.2, layer=pcbnew.F_SilkS, thick=0.18):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s)
    t.SetPosition(pt(x, y))
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    t.SetTextThickness(mm(thick))
    if layer in (pcbnew.B_SilkS, pcbnew.B_Fab):
        t.SetMirrored(True)
    board.Add(t)


def zone(board, layer, net, priority=0, clearance=0.25):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net)
    z.SetAssignedPriority(priority)
    z.SetLocalClearance(mm(clearance))
    z.SetMinThickness(mm(0.2))
    z.SetThermalReliefGap(mm(0.3))
    z.SetThermalReliefSpokeWidth(mm(0.4))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetIsFilled(False)
    poly = z.Outline()
    poly.NewOutline()
    for x, y in [(0.3, 0.3), (W - 0.3, 0.3), (W - 0.3, H - 0.3), (0.3, H - 0.3)]:
        poly.Append(mm(OX + x), mm(OY + y))
    board.Add(z)
    return z


def _skyline(items, width, obstacles=()):
    """items: [(key, w, h[, turnable[, sens]])] -> ({key: (x, y, turned)}, used_w, used_h). Bottom-left
    skyline packing, largest first: each item goes where its top edge ends lowest (then leftmost),
    turned 90 degrees if that ends lower and the item allows it. sens(turned) gives an item's
    sensitive rects (its through-hole thermal vias) relative to its corner; only those must stay off
    obstacles (the other side's pads). Items without sens ignore obstacles."""
    sky = [(0.0, 0.0, width)]                     # (x, y, w) segments, left to right
    pos, used_w, used_h = {}, 0.0, 0.0
    for item in sorted(items, key=lambda t: (-max(t[1], t[2]), -min(t[1], t[2]))):
        key, w0, h0 = item[:3]
        best = None
        for w, h, turned in [(w0, h0, False)] + ([(h0, w0, True)] if len(item) > 3 and item[3] else []):
            sens = item[4](turned) if len(item) > 4 and item[4] and obstacles else []
            xs = [s[0] for s in sky]
            if sens:                              # also try each segment's right end
                xs += [s[0] + s[2] - w for s in sky]
                xs = sorted({round(x, 3) for x in xs if -1e-6 <= x <= width - w + 1e-6})
            for sx in xs:
                if sx + w > width + 1e-6:
                    break
                top = max(sy for qx, sy, qw in sky if qx < sx + w - 1e-6 and sx < qx + qw - 1e-6)
                while sens:                       # slide down until the via patches clear the other side's pads
                    hit = [o[3] - r[1] for r in sens for o in obstacles
                           if sx + r[0] < o[2] and o[0] < sx + r[2] and top + r[1] < o[3] and o[1] < top + r[3]]
                    if not hit:
                        break
                    top = max(hit)
                if best is None or (top + h, sx) < (best[0] + best[2], best[1]):
                    best = (top, sx, h, w, turned)
        if best is None:                          # wider than the region: start at the left edge
            best = (max(s[1] for s in sky), 0.0, h0, w0, False)
        y, x, h, w, turned = best
        pos[key] = (x, y, turned)
        used_w, used_h = max(used_w, x + w), max(used_h, y + h)
        new = []
        for sx, sy, sw in sky:                    # cut [x, x + w] out of the skyline, insert the item top
            if sx + sw <= x + 1e-6 or sx >= x + w - 1e-6:
                new.append((sx, sy, sw))
                continue
            if sx < x:
                new.append((sx, sy, x - sx))
            if sx + sw > x + w:
                new.append((x + w, sy, sx + sw - x - w))
        new.append((x, y + h, w))
        new.sort()
        sky = []
        for s in new:                             # merge equal-height neighbours
            if sky and abs(sky[-1][1] - s[1]) < 1e-6 and abs(sky[-1][0] + sky[-1][2] - s[0]) < 1e-6:
                sky[-1] = (sky[-1][0], sky[-1][1], sky[-1][2] + s[2])
            else:
                sky.append(s)
    return pos, used_w, used_h


def _gap(fp):
    cx0, cy0, cx1, cy1 = courtyard(fp)
    a = (cx1 - cx0) * (cy1 - cy0)
    return SMALL_GAP if a < 8 else (GAP if a < 15 else 1.2)


def cluster_block(cl, fps):
    """A placer.Cluster as a packable block: (w, h, place(x, y), name, turnable, sens)."""
    x0, y0, x1, y1 = cl.bbox()
    m = BLOCK_GAP / 2
    pth = [(lx + a, ly + b, lx + c, ly + d) for ref, (lx, ly, rot) in cl.pos.items()
           for a, b, c, d in cl.geo[ref].rot[rot]["pth"]]

    def sens(turned, flip):
        """The block's through-hole pads (+0.4 mm) relative to its corner, as placed by put()."""
        bw = ((y1 - y0) if turned else (x1 - x0)) + 2 * m
        out = []
        for a, b, c, d in pth:
            r = (m + b - y0, m + x1 - c, m + d - y0, m + x1 - a) if turned else \
                (m + a - x0, m + b - y0, m + c - x0, m + d - y0)
            if flip:
                r = (bw - r[2], r[1], bw - r[0], r[3])
            out.append((r[0] - 0.4, r[1] - 0.4, r[2] + 0.4, r[3] + 0.4))
        return out

    def put(x, y, turned=False, flip=False):
        for ref, (lx, ly, rot) in cl.pos.items():
            if turned:                            # +90 degrees, as pcbnew turns a footprint: (x, y) -> (y, -x)
                fps[ref].SetOrientationDegrees((rot + 90) % 360)
                fps[ref].SetPosition(pt(x + m + ly - y0, y + m + x1 - lx))
            else:
                fps[ref].SetOrientationDegrees(rot)
                fps[ref].SetPosition(pt(x + m + lx - x0, y + m + ly - y0))
        if flip:                                  # bottom side: mirror the whole cluster about its own centre line,
            cx = x + ((y1 - y0) if turned else (x1 - x0)) / 2 + m   # so every pad keeps its neighbour
            for ref in cl.pos:
                fps[ref].Flip(pt(cx, 0), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    return (x1 - x0 + 2 * m, y1 - y0 + 2 * m, put, cl.name, True, sens if pth else None)


TP_ROW, TP_PER_ROW = 7.8, 10


def tp_block(refs, fps, title):
    """Test pads in rows (pitch = courtyard + 0.3 mm), with room under each for its vertical net-name label."""
    cx0, _cy0, cx1, _cy1 = courtyard(fps[refs[0]])
    TP_PITCH = max(2.4, cx1 - cx0 + 0.3)
    n = len(refs)
    rows = (n + TP_PER_ROW - 1) // TP_PER_ROW
    per = min(n, TP_PER_ROW)

    def put(x, y, _turned=False, _flip=False):
        for i, ref in enumerate(refs):
            fps[ref].SetPosition(pt(x + 0.8 + TP_PITCH * (i % TP_PER_ROW + 0.5),
                                    y + 0.8 + 1.2 + TP_ROW * (i // TP_PER_ROW)))
    return (per * TP_PITCH + 1.6, rows * TP_ROW + 1.6, put, title, False)


def _pack_height(blocks, w):
    return _skyline([(i, b[0], b[1], b[4]) for i, b in enumerate(blocks)], w)[2] + TITLE_STRIP


def regions(blocks):
    """Size the floorplan from the packer itself, per side: each column gets the narrowest width at
    which its regions, skyline-packed, fit the column height; leftover width is shared out in
    proportion, and each region gets its packed height plus a share of the column's spare height."""
    short = []
    for side in ("top", "bottom"):
        if CFG[side]:
            short += _regions(blocks, CFG[side], side)
    if short:
        raise SystemExit("\n".join(short))


def _regions(blocks, columns, side):
    def span(col):
        top, bot = (7.6, H - 7.6) if col is columns[-1] else (2.5, H - 2.5)   # right column + outline clear the M3 holes
        return top, bot - top - COL_GAP * (len(col) - 1)

    x_free = W - CONN_X - 1.5 - COL_GAP * (len(columns) - 1)
    widths, short = [], []
    for col in columns:
        lo = max(min(b[0], b[1]) if b[4] else b[0] for s in col for b in blocks[s])
        hi = x_free
        fits = lambda w: sum(_pack_height(blocks[s], w) for s in col) <= span(col)[1]
        if not fits(hi):
            short.append(f"{side}: column {col} does not fit H = {H} mm at any width")
            widths.append(x_free)
            continue
        if fits(lo):
            hi = lo
        while hi - lo > 0.25:
            mid = (lo + hi) / 2
            lo, hi = (lo, mid) if fits(mid) else (mid, hi)
        widths.append(hi)
    spare = x_free - sum(widths)
    print(f"{side:6s} columns need", " / ".join(f"{w:.1f}" for w in widths), f"mm, spare {spare:.1f} mm")
    if spare < 0 or short:
        return short + [f"{side}: board too small: columns need {sum(widths):.1f} mm of {x_free:.1f} mm width"]
    widths = [w + spare * w / sum(widths) for w in widths]
    x = CONN_X
    for col, cw in zip(columns, widths):
        top, free = span(col)
        hs = {s: _pack_height(blocks[s], cw) for s in col}
        extra = (free - sum(hs.values())) / len(col)
        y = top
        for s in col:
            rh = hs[s] + extra
            REGIONS[s] = (round(x, 2), round(y, 2), round(x + cw, 2), round(y + rh, 2))
            y += rh + COL_GAP
        x += cw + COL_GAP
    return []


SPREAD = float(os.environ.get("ICARUS_SPREAD", "0.5"))                       # share of a region's spare room _spread() hands to the gaps between clusters


def _spread(blocks, items, bpos, rw, rh, obstacles):
    """Spend a region's spare room on routing channels: scale every block's position (not its size)
    by kx, ky >= 1 so the blocks fill the region. Scaling positions by >= 1 can't create an overlap.
    Kept unscaled if a via patch would then land on an obstacle (the other side's pads)."""
    size = {i: ((b[1], b[0]) if bpos[i][2] else (b[0], b[1])) for i, b in enumerate(blocks)}
    kx = min([(rw - size[i][0]) / bpos[i][0] for i in bpos if bpos[i][0] > 1e-6] + [4.0])
    ky = min([(rh - size[i][1]) / bpos[i][1] for i in bpos if bpos[i][1] > 1e-6] + [4.0])
    # only half way: the rest of the spare room stays as gaps for the floaters (test pads, LEDs)
    kx, ky = 1.0 + max(0.0, kx - 1.0) * SPREAD, 1.0 + max(0.0, ky - 1.0) * SPREAD
    new = {i: (x * kx, y * ky, t) for i, (x, y, t) in bpos.items()}
    for i, (x, y, t) in new.items():
        sens = items[i][4](t) if items[i][4] and obstacles else []
        for r in sens:
            if any(x + r[0] < o[2] and o[0] < x + r[2] and y + r[1] < o[3] and o[1] < y + r[3] for o in obstacles):
                return bpos, max(p[1] + size[i][1] for i, p in bpos.items())
    return new, max(p[1] + size[i][1] for i, p in new.items())


def pack(blocks, region, sheet, keepouts=()):
    """Skyline-pack a region's blocks below its title strip. keepouts (board coordinates): the other
    side's pads, which this region's through-hole via patches must stay off."""
    x0, y0, x1, y1 = region
    oy = y0 + TITLE_STRIP
    flip = SIDE[sheet] == "bottom"
    local = [(k[0] - x0, k[1] - oy, k[2] - x0, k[3] - oy) for k in keepouts
             if k[0] < x1 and x0 < k[2] and k[1] < y1 and y0 < k[3]]
    items = [(i, b[0], b[1], b[4], (lambda t, f=b[5]: f(t, flip)) if len(b) > 5 and b[5] else None)
             for i, b in enumerate(blocks)]
    bpos, _uw, uh = _skyline(items, x1 - x0, local)
    if CFG.get("spread"):
        bpos, uh = _spread(blocks, items, bpos, x1 - x0, y1 - y0 - TITLE_STRIP, local)
    if uh + TITLE_STRIP > y1 - y0 + 0.01:
        OVERFLOW.append(f"region '{sheet}' needs {uh + TITLE_STRIP:.1f} mm of height, has {y1 - y0:.1f}")
    for i, b in enumerate(blocks):
        bx, by, turned = bpos[i]
        b[2](x0 + bx, y0 + TITLE_STRIP + by, turned, SIDE[sheet] == "bottom")
    return y0 + TITLE_STRIP + uh


OUT_TYPES = ("output", "power_out", "open_collector", "tri_state", "bidirectional")


def is_floater(p):
    """Test pads and indicator LEDs (with their resistors / blocking diodes): placed last, on top,
    next to the signal they show (place_floaters), in configs with floaters=True."""
    return CFG.get("floaters") and (p["lib"] == "Connector:TestPoint" or (p["group"] or "").startswith("Indicators"))


def hole_rects(board):
    """Bounding boxes (internal units) of the through-hole pads in SMD footprints (thermal-via patches)."""
    out = []
    for fp in board.GetFootprints():
        pads = list(fp.Pads())
        if not any(pd.GetAttribute() == pcbnew.PAD_ATTRIB_SMD for pd in pads):
            continue                              # real through-hole parts export all layers already
        for pd in pads:
            if pd.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                bb = pd.GetBoundingBox()
                out.append((bb.GetLeft(), bb.GetTop(), bb.GetRight(), bb.GetBottom()))
    return out


def via_keepouts(board, rects=None, clr=0.3):
    """Rule areas (no tracks, no vias; pours allowed) on the inner signal layers over every
    through-hole pad of an SMD footprint (thermal-via patches). KiCad's Specctra export describes
    those pads as copper on F.Cu / B.Cu only ("[TB]" padstacks), so without these the router runs
    inner-layer tracks straight through their drill holes. The GND pour still reaches the vias."""
    inner = [l for l in SIG if l not in (pcbnew.F_Cu, pcbnew.B_Cu)]
    if not inner:
        return 0
    ls = pcbnew.LSET()
    for l in inner:
        ls.AddLayer(l)
    rects = hole_rects(board) if rects is None else rects
    for x0, y0, x1, y1 in rects:
        z = pcbnew.ZONE(board)
        z.SetIsRuleArea(True)
        z.SetLayerSet(ls)
        z.SetDoNotAllowTracks(True)
        z.SetDoNotAllowVias(True)
        z.SetDoNotAllowPads(False)
        z.SetDoNotAllowZoneFills(False)
        z.SetDoNotAllowFootprints(False)
        poly = z.Outline()
        poly.NewOutline()
        c = mm(clr)
        for x, y in ((x0 - c, y0 - c), (x1 + c, y0 - c), (x1 + c, y1 + c), (x0 - c, y1 + c)):
            poly.Append(int(x), int(y))
        board.Add(z)
    return len(rects)


def place_floaters(fps, refs, parts, geo, gap, net_size, pads, ptypes):
    """Put every test pad and indicator LED next to the signal it shows, always on the top side.

    - Units: an LED with its resistor (and blocking diode, across a relay coil) is one unit, joined by
      the nets only floaters use. A test pad is a unit with room for its net-name label on its right.
    - Target: the pad that drives the unit's most specific outside net. Drivers come first (output /
      power_out / open-collector pin types), then other IC pins, connectors, passives. A GND test pad
      targets the middle of its cluster's other pads (incoming / outgoing / measured), so a scope
      ground is always near.
    - Spot: the free top-side spot nearest the target, searched in rings. If the signal's chip is on
      the bottom, that's directly above it, so the LED stays visible and the pad probeable.
      Keep-outs: top courtyards, the bottom side's through-hole thermal vias, the edge strip, the M3
      holes, the region titles and outlines."""
    from silk import Index
    fl = set(refs)
    net_refs = {}
    for (r, pd), n in pads.items():
        net_refs.setdefault(n, set()).add(r)
    parent = {r: r for r in fl}

    def find(r):
        while parent[r] != r:
            parent[r] = parent[parent[r]]
            r = parent[r]
        return r
    for n, rs in net_refs.items():
        if n != "GND" and rs <= fl and len(rs) > 1:
            first = next(iter(rs))
            for r in rs:
                parent[find(r)] = find(first)
    units = {}
    for r in sorted(fl, key=lambda r: [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", r)]):
        units.setdefault(find(r), []).append(r)

    def pad_xy(ref, num):
        for pd in fps[ref].Pads():
            if pd.GetNumber() == num:
                return pcbnew.ToMM(pd.GetPosition().x) - OX, pcbnew.ToMM(pd.GetPosition().y) - OY
        return None

    def rank(ref, num):
        lib = parts[ref]["lib"]
        if ptypes.get((ref, num)) in OUT_TYPES:
            return 0
        return 3 if placer.is_passive(parts[ref]) else (2 if lib.startswith("Connector") else 1)

    def target(unit):
        """(net, ref-in-unit on that net, (x, y) of the driving pad) or None for GND-only units."""
        best = None
        for r in unit:
            for num, n in geo[r].nets.items():
                if n == "GND" or net_refs[n] <= fl:
                    continue
                cands = [(rank(o, pd), o, pd) for (o, pd), nn in pads.items() if nn == n and o not in fl]
                if not cands:
                    continue
                k = (net_size[n], min(cands)[0])
                if best is None or k < best[0]:
                    rk, o, pd = min(cands)
                    best = (k, n, r, num, pad_xy(o, pd))
        return best

    # keep-outs (board mm, origin top-left)
    idx = Index()
    for ref, fp in fps.items():
        if ref in fl:
            continue
        if fp.IsFlipped():
            for pd in fp.Pads():
                if pd.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                    bb = pd.GetBoundingBox()
                    idx.add((pcbnew.ToMM(bb.GetLeft()) - OX - 0.6, pcbnew.ToMM(bb.GetTop()) - OY - 0.6,
                             pcbnew.ToMM(bb.GetRight()) - OX + 0.6, pcbnew.ToMM(bb.GetBottom()) - OY + 0.6))
            if not placer.is_passive(parts[ref]) and len(list(fp.Pads())) > 6:
                # stay off the footprint of a fine-pitch IC underneath: a floater right over its pins
                # leaves no room for the via down to them (rev 0.6 config A: 6 LED nets unroutable)
                cy = fp.GetCourtyard(pcbnew.B_CrtYd)
                if cy.OutlineCount():
                    bb = cy.BBox()
                    idx.add((pcbnew.ToMM(bb.GetLeft()) - OX, pcbnew.ToMM(bb.GetTop()) - OY,
                             pcbnew.ToMM(bb.GetRight()) - OX, pcbnew.ToMM(bb.GetBottom()) - OY))
            continue
        c0, c1, c2, c3 = courtyard(fp)
        q = fp.GetPosition()
        x, y, m = pcbnew.ToMM(q.x) - OX, pcbnew.ToMM(q.y) - OY, gap.get(ref, GAP) / 2
        idx.add((x + c0 - m, y + c1 - m, x + c2 + m, y + c3 + m))
    for hx, hy in MOUNT_HOLES:
        idx.add((hx - 4.0, hy - 4.0, hx + 4.0, hy + 4.0))
    for name, (x0, y0, x1, y1) in REGIONS.items():
        if SIDE.get(name) != "top":
            continue
        idx.add((x0, y0, x1, y0 + TITLE_STRIP))                   # title strip
        for a in ((x0 - 0.9, y0 - 0.9, x1 + 0.9, y0 - 0.1), (x0 - 0.9, y1 + 0.1, x1 + 0.9, y1 + 0.9),
                  (x0 - 0.9, y0 - 0.9, x0 - 0.1, y1 + 0.9), (x1 + 0.1, y0 - 0.9, x1 + 0.9, y1 + 0.9)):
            idx.add(a)                                              # silk outline band

    def shape(unit, anchor):
        """Local layout of a unit: [(variant rects, {ref: (x, y, rot)}, attach (x, y))], anchor at 0, 0."""
        order = {r: i for i, r in enumerate(unit)}
        cl = placer.Cluster("floater", anchor, geo, gap, order)
        if len(unit) > 1:
            placer._fill(cl, [r for r in unit if r != anchor], parts, net_size)
        x0, y0, x1, y1 = cl.bbox()
        if parts[anchor]["lib"] == "Connector:TestPoint":
            x1 += 5.6                                               # room for the net-name label
        out = []
        for turned in ((False,) if parts[anchor]["lib"] == "Connector:TestPoint" else (False, True)):
            if turned:                                              # +90: (x, y) -> (y, -x), as pcbnew turns
                pos = {r: (ly, -lx, (rot + 90) % 360) for r, (lx, ly, rot) in cl.pos.items()}
                rect = (y0, -x1, y1, -x0)
            else:
                pos = dict(cl.pos)
                rect = (x0, y0, x1, y1)
            out.append((rect, pos))
        return out

    placed = {}
    order = sorted(units.values(), key=lambda u: (parts[u[0]]["lib"] == "Connector:TestPoint",
                                                  all(n == "GND" for r in u for n in geo[r].nets.values())))
    for unit in order:
        t = target(unit)
        if t is None:                                               # GND test pad: middle of its cluster
            grp = parts[unit[0]]["group"]
            mates = [placed[r] for r in placed if parts[r]["group"] == grp]
            tx = sum(m[0] for m in mates) / len(mates) if mates else W / 2
            ty = sum(m[1] for m in mates) / len(mates) if mates else H / 2
            anchor, attach_num = unit[0], None
        else:
            _k, _n, anchor, attach_num, (tx, ty) = t
        best = None
        r = 0.0
        while r <= 50.0 and best is None:
            for dx, dy in placer._ring(round(r, 2)):
                for rect, pos in shape(unit, anchor):
                    ax, ay, arot = pos[anchor]
                    offs = (0.0, 0.0)
                    if attach_num:
                        offs = geo[anchor].rot[arot]["pads"][attach_num][0]
                    ox, oy = tx + dx - ax - offs[0], ty + dy - ay - offs[1]   # unit origin
                    g = (rect[0] + ox, rect[1] + oy, rect[2] + ox, rect[3] + oy)
                    if g[0] < CONN_X + 0.3 or g[1] < 0.8 or g[2] > W - 0.8 or g[3] > H - 0.8 or idx.hit(g):
                        continue
                    best = (g, pos, ox, oy)
                    break
                if best:
                    break
            r += 0.3
        if best is None:
            raise SystemExit(f"no free top-side spot within 50 mm for {unit}")
        g, pos, ox, oy = best
        idx.add(g)
        for ref, (lx, ly, rot) in pos.items():
            fps[ref].SetOrientationDegrees(rot)
            fps[ref].SetPosition(pt(ox + lx, oy + ly))
            placed[ref] = (ox + lx, oy + ly)
    dist = []
    for unit in order:
        t = target(unit)
        if t:
            x, y = placed[t[2]]
            dist.append(((x - t[4][0]) ** 2 + (y - t[4][1]) ** 2) ** 0.5)
    dist.sort()
    print(f"floaters: {len(units)} units ({len(fl)} parts) next to their signals; distance to the driving pin "
          f"median {dist[len(dist) // 2]:.1f} mm, max {dist[-1]:.1f} mm")


def place():
    clean_vendor_library()
    comps, pads, ptypes = read_netlist()
    board = pcbnew.CreateEmptyBoard()
    board.SetCopperLayerCount(N_LAYERS)
    rules(board)
    nets = {}
    for name in sorted(set(pads.values())):
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        nets[name] = ni
    by_region = {s: [] for s in SIDE}
    floaters = []
    order = {p["ref"]: i for i, p in enumerate(design.PARTS)}
    fps = {}
    for ref in sorted(comps, key=lambda r: order[r]):
        c = comps[ref]
        lib, name = c["footprint"].split(":")
        fp = pcbnew.FootprintLoad(fp_dir(lib), name)
        fp.SetParent(board)
        fp.SetExcludedFromBOM(False)              # match the symbols; export_fab drops bare test pads
        fp.SetReference(ref)
        fp.SetValue(c["value"])
        fp.SetPath(pcbnew.KIID_PATH(c["path"]))
        fp.SetSheetname(c["sheetname"])
        fp.SetFPIDAsString(c["footprint"])
        for k, v in c["fields"].items():
            fp.SetField(k, v)
            fp.GetField(k).SetVisible(False)
        fp.Reference().SetVisible(False)          # silk.apply() decides where references go
        fp.Value().SetVisible(False)
        if c["footprint"].startswith(HAND_SOLDERED_LIBS):
            fp.SetExcludedFromPosFiles(True)
        if c["footprint"].startswith("Fuse:Fuseholder"):
            fp.SetDuplicatePadNumbersAreJumpers(True)   # the two legs of each fuse clip are one piece of metal
        for pad in fp.Pads():
            net = pads.get((ref, pad.GetNumber()))
            if net:
                pad.SetNet(nets[net])
        board.Add(fp)
        fps[ref] = fp
        if ref in FIXED:
            if FIXED[ref]:
                x, y, rot = FIXED[ref]
                fp.SetOrientationDegrees(rot)
                fp.SetPosition(pt(x, y))
        elif is_floater(next(p for p in design.PARTS if p["ref"] == ref)):
            floaters.append(ref)
        else:
            by_region[region_of(placed_as(next(p for p in design.PARTS if p["ref"] == ref)))].append(ref)
    parts = {p["ref"]: placed_as(p) for p in design.PARTS}
    net_size = {}
    for n in pads.values():
        net_size[n] = net_size.get(n, 0) + 1
    pad_nets, power_pins = {}, {}
    for (r, pd), n in pads.items():
        pad_nets.setdefault(r, {})[pd] = n
        if ptypes.get((r, pd)) == "power_in":
            power_pins.setdefault(r, set()).add(pd)
    geo = {ref: placer.Geo(fps[ref], pad_nets.get(ref, {}), power_pins.get(ref, set()))
           for refs in list(by_region.values()) + [floaters] for ref in refs}
    gap = {ref: _gap(fps[ref]) for ref in geo}
    placer.ANCHOR_ROT = CFG.get("anchor_rot", {})
    blocks = {}
    for region, refs in by_region.items():
        tps = {}
        for r in refs:
            if parts[r]["lib"] == "Connector:TestPoint":
                tps.setdefault(parts[r]["group"], []).append(r)
        rest = [r for r in refs if parts[r]["lib"] != "Connector:TestPoint"]
        clusters = placer.build(rest, parts, geo, gap, net_size)
        blocks[region] = [cluster_block(cl, fps) for cl in clusters] + \
                         [tp_block(v, fps, g) for g, v in tps.items()]
        print(f"{region:8s} {len(clusters):2d} clusters, {sum(len(c.pos) for c in clusters):3d} parts"
              + (f", {sum(len(v) for v in tps.values())} test pads" if tps else ""))
    if STRIP:                                     # THT parts stacked in the edge strip, centred, clear of the M3 holes
        y = 7.6
        for ref, rot in STRIP:
            fp = fps[ref]
            fp.SetOrientationDegrees(rot)
            fp.SetPosition(pt(0, 0))
            fp.BuildCourtyardCaches()
            c0, c1, c2, c3 = courtyard(fp)
            fp.SetPosition(pt(CONN_X / 2 - 0.5 - (c0 + c2) / 2, y - c1))
            y += c3 - c1 + 1.0
        if y - 1.0 > H - 7.4:
            raise SystemExit(f"edge strip needs {y - 1.0:.1f} mm, has {H - 7.4:.1f}")
    regions(blocks)
    # The side WITHOUT through-hole thermal vias packs first. Then the via side packs, and only its
    # via-carrying clusters (coil drive, boost) must land their via patches clear of the first side's
    # pads; every other cluster packs freely.
    via_side = SIDE.get("coil", "top")
    if SIDE.get("backup", via_side) != via_side:
        raise SystemExit("coil and backup must share a side (both carry through-hole thermal vias)")
    first = "bottom" if via_side == "top" else "top"
    fill, keepouts = {}, []
    for side in (first, via_side):
        for region, bl in blocks.items():
            if SIDE[region] == side:
                fill[region] = pack(bl, REGIONS[region], region, keepouts)
        if side == first:
            for region in [r for r in SIDE if SIDE[r] == first]:
                for ref in by_region[region]:            # courtyards, not just pads: via copper under a part
                    fp = fps[ref]                         # body can still meet its exposed pad
                    cy = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd)
                    bb = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False)
                    keepouts.append((pcbnew.ToMM(bb.GetLeft()) - OX - 0.2, pcbnew.ToMM(bb.GetTop()) - OY - 0.2,
                                     pcbnew.ToMM(bb.GetRight()) - OX + 0.2, pcbnew.ToMM(bb.GetBottom()) - OY + 0.2))
            print(f"{len(keepouts)} {first}-side courtyards for the {via_side}-side via patches to avoid")
    if OVERFLOW:
        raise SystemExit("\n".join(OVERFLOW))
    if floaters:
        place_floaters(fps, floaters, parts, geo, gap, net_size, pads, ptypes)
    for i, (x, y) in enumerate(MOUNT_HOLES):
        fp = pcbnew.FootprintLoad(fp_dir("MountingHole"), "MountingHole_3.2mm_M3_Pad")
        fp.SetParent(board)
        fp.SetReference(f"H{i + 1}")
        fp.SetBoardOnly(True)
        fp.SetExcludedFromBOM(True)
        fp.SetExcludedFromPosFiles(True)
        fp.Reference().SetVisible(False)
        fp.SetPosition(pt(x, y))
        for pad in fp.Pads():
            pad.SetNet(nets["GND"])
        board.Add(fp)
    outline(board)
    # Pre-route GND fan-out: every GND SMD pad gets its own locked stub + via to the In1 plane
    # before Freerouting runs, so signals route around them (post-route fan-out left boxed-in pads).
    # Exposed pads that already carry thermal vias are skipped.
    targets = []
    for fp in board.GetFootprints():
        pth = {p.GetNumber() for p in fp.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH}
        for pad in fp.Pads():
            if pad.GetNetname() == "GND" and pad.GetAttribute() == pcbnew.PAD_ATTRIB_SMD \
                    and (pad.IsOnLayer(pcbnew.F_Cu) or pad.IsOnLayer(pcbnew.B_Cu)) and pad.GetNumber() not in pth:
                targets.append((fp.GetReference(), pad.GetNumber()))
    added, failed = fanout_gnd(radius_max=2.5, board=board, targets=targets, lock=True, stitch=False)
    print(f"pre-route GND fan-out: {added} of {len(targets)} pads, no room: {' '.join(failed) or 'none'}")
    for rail in (RAILS if PWR_PLANES else []):    # every rail SMD pad gets its own via down to its island
        rt = [(fp.GetReference(), pd.GetNumber()) for fp in board.GetFootprints() for pd in fp.Pads()
              if pd.GetNetname() == rail and pd.GetAttribute() == pcbnew.PAD_ATTRIB_SMD
              and (pd.IsOnLayer(pcbnew.F_Cu) or pd.IsOnLayer(pcbnew.B_Cu))]
        added, failed = fanout_gnd(radius_max=2.5, board=board, targets=rt, lock=True, stitch=False, net=rail)
        print(f"pre-route {rail} fan-out: {added} of {len(rt)} pads" + (f", no room: {' '.join(failed)}" if failed else ""))
    # Planes are LT_POWER layers, so the DSN tells Freerouting not to route signals there (pass 1 of
    # rev 0.4 put 5 m of tracks on a signal-type In1 and islanded the plane).
    for plane in GND_PLANES:
        board.SetLayerType(plane, pcbnew.LT_POWER)
        zone(board, plane, nets["GND"])            # reference plane(s) before the DSN export
    for plane in PWR_PLANES:
        board.SetLayerType(plane, pcbnew.LT_POWER)
        print("power islands on", board.GetLayerName(plane) + ":", power_islands(board, plane, RAILS, nets))
    ds = board.GetDesignSettings()
    ds.m_TentViasFront = ds.m_TentViasBack = True   # silk may cross vias; no bare via copper
    placed = silk.apply(board, OX, OY, W, H, REGIONS, TITLE_STRIP, VEHICLE_J, SIDE, not CFG.get("floaters"))
    print(f"silk: {placed['front']} front, {placed['back']} back texts; {placed['skipped']} didn't fit")
    print("inner-layer keep-outs over through-hole via pads:", via_keepouts(board))
    board.SetFileName(PCB)
    pcbnew.SaveBoard(PCB, board)
    os.makedirs(os.path.dirname(DSN), exist_ok=True)
    if not pcbnew.ExportSpecctraDSN(board, DSN):
        raise SystemExit("DSN export failed")
    dsn_router_settings(board, DSN)
    for region, y in fill.items():
        print(f"{region:8s} region bottom {REGIONS[region][3]:6.1f}  used to {y:6.1f}")
    print("placed", len(comps), "footprints; wrote", PCB, "and", DSN)
    placement_report(board)


def finish(ses=None):
    board = pcbnew.LoadBoard(PCB)
    if not pcbnew.ImportSpecctraSES(board, ses or SES):
        raise SystemExit("SES import failed")
    gnd = board.FindNet("GND")
    for z in [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetLayer() not in PLANES]:
        board.Remove(z)
    for layer in SIG:
        zone(board, layer, gnd)
    print("VBAT_IN corridor zones:", power_corridor(board))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)
    print("routed + poured:", len(board.GetTracks()), "tracks/vias")
    report(board)


def power_corridor(board, net="VBAT_IN", half=1.2):
    """Unfused battery feed: whatever layer the router used, add a B.Cu copper corridor (2 x half wide)
    along each of its tracks, solid to the pads. J1 pin 1 and the fuse clip are through-hole, so the
    outer-layer copper joins both ends. Sized for a downstream short until the 5 A blade fuse opens,
    which a 0.6 mm inner track is not."""
    import math
    n = board.FindNet(net)
    z = pcbnew.ZONE(board)
    z.SetLayer(pcbnew.B_Cu)
    z.SetNet(n)
    z.SetAssignedPriority(5)                      # above the GND pour
    z.SetLocalClearance(mm(0.25))
    z.SetMinThickness(mm(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    poly = z.Outline()
    made = 0
    for t in [t for t in board.GetTracks() if t.GetNetname() == net and not isinstance(t, pcbnew.PCB_VIA)]:
        ax, ay = pcbnew.ToMM(t.GetStart().x), pcbnew.ToMM(t.GetStart().y)
        bx, by = pcbnew.ToMM(t.GetEnd().x), pcbnew.ToMM(t.GetEnd().y)
        L = math.hypot(bx - ax, by - ay) or 1e-6
        ux, uy = (bx - ax) / L, (by - ay) / L
        nx, ny = -uy * half, ux * half
        ax, ay, bx, by = ax - ux * half, ay - uy * half, bx + ux * half, by + uy * half
        poly.NewOutline()
        for x, y in ((ax + nx, ay + ny), (bx + nx, by + ny), (bx - nx, by - ny), (ax - nx, ay - ny)):
            poly.Append(mm(x), mm(y))
        made += 1
    if made:
        poly.Simplify()                           # one outline: the union of the segment strips
        board.Add(z)
    return made


def power_islands(board, layer, rails, nets, cell=0.5):
    """Split power plane: the layer's copper is shared out among the rails as a Voronoi partition on a
    0.5 mm grid, seeded by each rail's vias and through-hole pads (so every via sits inside its own
    rail's island). A cell next to another rail's cell stays empty: islands are >= 0.5 mm apart.
    Cells near the edge or the M3 holes stay empty too. Each rail's island(s) become one zone."""
    seeds = []
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA) and t.GetNetname() in rails:
            seeds.append((pcbnew.ToMM(t.GetPosition().x) - OX, pcbnew.ToMM(t.GetPosition().y) - OY, t.GetNetname()))
    for fp in board.GetFootprints():
        for pd in fp.Pads():
            if pd.GetNetname() in rails and pd.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                seeds.append((pcbnew.ToMM(pd.GetPosition().x) - OX, pcbnew.ToMM(pd.GetPosition().y) - OY, pd.GetNetname()))
    nx, ny = int(W / cell), int(H / cell)
    lab = [[None] * ny for _ in range(nx)]
    for i in range(nx):
        cx = (i + 0.5) * cell
        if cx < 0.8 or cx > W - 0.8:
            continue
        for j in range(ny):
            cy = (j + 0.5) * cell
            if cy < 0.8 or cy > H - 0.8 or any((cx - hx) ** 2 + (cy - hy) ** 2 < 4.2 ** 2 for hx, hy in MOUNT_HOLES):
                continue
            lab[i][j] = min(seeds, key=lambda q: (q[0] - cx) ** 2 + (q[1] - cy) ** 2)[2]
    keep = [[lab[i][j] for j in range(ny)] for i in range(nx)]
    for i in range(nx):                               # one empty cell along every boundary between rails
        for j in range(ny):
            a = lab[i][j]
            for di, dj in ((1, 0), (0, 1), (1, 1), (1, -1)):
                b = lab[i + di][j + dj] if 0 <= i + di < nx and 0 <= j + dj < ny else None
                if a and b and a != b:
                    keep[i][j] = None
    made = {}
    for rail in rails:
        sps = pcbnew.SHAPE_POLY_SET()
        for j in range(ny):
            i = 0
            while i < nx:
                if keep[i][j] != rail:
                    i += 1
                    continue
                i0 = i
                while i < nx and keep[i][j] == rail:
                    i += 1
                x0, x1 = OX + i0 * cell, OX + i * cell
                y0, y1 = OY + j * cell, OY + (j + 1) * cell + 0.002
                sps.NewOutline()
                for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
                    sps.Append(mm(x), mm(y))
        if not sps.OutlineCount():
            continue
        sps.Simplify()
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(nets[rail])
        z.SetLocalClearance(mm(0.25))
        z.SetMinThickness(mm(0.25))
        z.SetThermalReliefGap(mm(0.3))
        z.SetThermalReliefSpokeWidth(mm(0.4))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.Outline().Append(sps)                   # a copy: SetOutline() keeps the raw pointer, and Python frees sps
        board.Add(z)
        made[rail] = sps.OutlineCount()
    return made


def dsn_router_settings(board, path):
    """Give Freerouting the stack's routing rules (its own autoroute_settings extension of the
    Specctra DSN): signal layers alternate vertical / horizontal from F.Cu down, against-direction
    track costs 3x, plane layers inactive, cheap vias into the planes."""
    rules = ""
    for l in SIG + PLANES:
        name = board.GetLayerName(l)
        if l in SIG:
            rules += (f"    (layer_rule {name}\n      (active on)\n      (preferred_direction {DIRECTION[l]})\n"
                      f"      (preferred_direction_trace_costs 1.0)\n      (against_preferred_direction_trace_costs 3.0)\n    )\n")
        else:
            rules += f"    (layer_rule {name}\n      (active off)\n    )\n"
    block = ("  (autoroute_settings\n    (fanout off)\n    (autoroute on)\n    (postroute on)\n    (vias on)\n"
             "    (via_costs 50)\n    (plane_via_costs 5)\n    (start_ripup_costs 100)\n    (start_pass_no 1)\n"
             + rules + "  )\n")
    s = open(path, encoding="utf-8").read()
    if "(autoroute_settings" not in s:             # after the layer definitions: the rules name the layers
        s = s.replace("\n    (boundary", "\n  " + block.rstrip("\n").replace("\n", "\n  ") + "\n    (boundary", 1)
        open(path, "w", encoding="utf-8").write(s)
    print("router: " + ", ".join(f"{board.GetLayerName(l)} {DIRECTION[l]}" for l in SIG))


def placement_report(board, cell=2.0):
    """Placement metrics, before routing (tscircuit: check placement and routing difficulty first).
    Density: courtyard area per side over the usable board. Routing difficulty: RUDY (rectangular
    uniform wire density): each non-GND net spreads its half-perimeter wire length evenly over its
    bounding box; summed per cell and divided by the cell's wiring capacity on the 3 signal layers
    (F, In2, B at a 0.35 mm track pitch). GND is excluded: it drops to the In1 plane through vias."""
    import math
    area = {"top": 0.0, "bottom": 0.0}
    for fp in board.GetFootprints():
        if fp.GetFPIDAsString().startswith("MountingHole"):
            continue
        cy = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd)
        if cy.OutlineCount():
            b = cy.BBox()
            a = pcbnew.ToMM(b.GetWidth()) * pcbnew.ToMM(b.GetHeight())
            tht = any(p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH for p in fp.Pads())
            area["bottom" if fp.IsFlipped() else "top"] += a
            if tht and not fp.IsFlipped():
                area["bottom"] += a               # through-hole parts block both sides
    usable = W * H
    nets = {}
    for fp in board.GetFootprints():
        for pd in fp.Pads():
            n = pd.GetNetname()
            if n and n != "GND" and not (PWR_PLANES and n in RAILS):
                nets.setdefault(n, []).append((pcbnew.ToMM(pd.GetPosition().x) - OX, pcbnew.ToMM(pd.GetPosition().y) - OY))
    nx, ny = int(math.ceil(W / cell)), int(math.ceil(H / cell))
    grid = [[0.0] * ny for _ in range(nx)]
    for pts in nets.values():
        if len(pts) < 2:
            continue
        x0, x1 = min(p[0] for p in pts), max(p[0] for p in pts)
        y0, y1 = min(p[1] for p in pts), max(p[1] for p in pts)
        w, h = max(x1 - x0, cell), max(y1 - y0, cell)
        dens = ((x1 - x0) + (y1 - y0)) / (w * h)  # mm of wire per mm2
        for i in range(max(0, int(x0 // cell)), min(nx, int((x0 + w) // cell) + 1)):
            for j in range(max(0, int(y0 // cell)), min(ny, int((y0 + h) // cell) + 1)):
                grid[i][j] += dens
    cap = len(SIG) / 0.35                         # mm of track per mm2 on the signal layers
    u = sorted(v / cap for col in grid for v in col)
    p95, peak = u[int(0.95 * (len(u) - 1))], u[-1]
    hot = sum(1 for v in u if v > 0.5) / len(u)
    print(f"density: top {100 * area['top'] / usable:.0f} %, bottom {100 * area['bottom'] / usable:.0f} % "
          f"(courtyards over {W:.0f} x {H:.0f} mm)")
    print(f"RUDY routing demand / capacity: 95th pct {p95:.2f}, peak {peak:.2f}, cells over 0.5: {100 * hot:.1f} %")
    return dict(top=area["top"] / usable, bottom=area["bottom"] / usable, p95=p95, peak=peak, hot=hot)


def report(board):
    """Per-layer usage; In1 must carry no tracks (it is the unbroken GND reference plane)."""
    seg, length, vias = {}, {}, 0
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            vias += 1
            continue
        name = board.GetLayerName(t.GetLayer())
        seg[name] = seg.get(name, 0) + 1
        length[name] = length.get(name, 0.0) + pcbnew.ToMM(t.GetLength())
    print("vias:", vias)
    for name in sorted(seg):
        print(f"  {name:7s} {seg[name]:5d} segments {length[name]:8.0f} mm")
    for plane in PLANES:
        if seg.get(board.GetLayerName(plane)):
            raise SystemExit(f"FAIL: tracks on {board.GetLayerName(plane)} (GND plane is broken up)")


def _seg_dist(p, a, b):
    """Distance from point p to segment ab (mm tuples)."""
    ax, ay, bx, by, px, py = a[0], a[1], b[0], b[1], p[0], p[1]
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    cx, cy = ax + t * dx, ay + t * dy
    return ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5


def _seg_seg_dist(a, b, c, d):
    return min(_seg_dist(a, c, d), _seg_dist(b, c, d), _seg_dist(c, a, b), _seg_dist(d, a, b)) \
        if not _segments_cross(a, b, c, d) else 0.0


def _segments_cross(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return (orient(a, b, c) * orient(a, b, d) < 0) and (orient(c, d, a) * orient(c, d, b) < 0)


def _rect_seg_dist(rect, a, b):
    """Distance between an axis-aligned rect (x0, y0, x1, y1) and segment ab."""
    x0, y0, x1, y1 = rect
    inside = lambda p: x0 <= p[0] <= x1 and y0 <= p[1] <= y1
    if inside(a) or inside(b):
        return 0.0
    edges = [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    return min(_seg_seg_dist(a, b, e0, e1) for e0, e1 in edges)


def _maze(board, net, a, la, b, lb, obs, holes, width, via_d, drill, clr, seg_clear, window=6.0, step=0.1):
    """Grid A* for one connection over F.Cu / In2.Cu / B.Cu with via transitions.
    Nodes are blocked by other-net copper (clearance + a diagonal margin); the result is
    merged into straight runs, re-checked exactly, and only then added to the board."""
    import heapq
    import math
    layers = list(SIG)
    x0, y0 = min(a[0], b[0]) - window, min(a[1], b[1]) - window
    nx = int((abs(a[0] - b[0]) + 2 * window) / step) + 1
    ny = int((abs(a[1] - b[1]) + 2 * window) / step) + 1
    near = [o for o in obs if (o[0] == "rect" and o[1][2] > x0 - 1 and o[1][0] < x0 + nx * step + 1
                               and o[1][3] > y0 - 1 and o[1][1] < y0 + ny * step + 1)
            or (o[0] == "circ" and x0 - 2 < o[1][0] < x0 + nx * step + 2 and y0 - 2 < o[1][1] < y0 + ny * step + 2)
            or (o[0] == "seg" and max(o[1][0][0], o[1][1][0]) > x0 - 2 and min(o[1][0][0], o[1][1][0]) < x0 + nx * step + 2
                and max(o[1][0][1], o[1][1][1]) > y0 - 2 and min(o[1][0][1], o[1][1][1]) < y0 + ny * step + 2)]
    need = width / 2 + clr + float(os.environ.get("ICARUS_MAZE_MARGIN", "0.05"))   # grid margin; exact re-check follows

    def dist(p, o):
        kind, g, _ls, w = o
        if kind == "rect":
            dx = max(g[0] - p[0], 0, p[0] - g[2])
            dy = max(g[1] - p[1], 0, p[1] - g[3])
            return (dx * dx + dy * dy) ** 0.5
        if kind == "circ":
            return ((p[0] - g[0]) ** 2 + (p[1] - g[1]) ** 2) ** 0.5 - w
        return _seg_dist(p, g[0], g[1]) - w

    free = {}
    for li, l in enumerate(layers):
        lo = [o for o in near if l in o[2]]
        for i in range(nx):
            for j in range(ny):
                p = (x0 + i * step, y0 + j * step)
                free[(li, i, j)] = all(dist(p, o) >= need for o in lo)

    def via_free(i, j):
        p = (x0 + i * step, y0 + j * step)
        return all(dist(p, o) >= via_d / 2 + clr for o in near) and \
            all(((p[0] - h[0]) ** 2 + (p[1] - h[1]) ** 2) ** 0.5 >= drill + 0.5 for h in holes)

    def cell(p):
        return int(round((p[0] - x0) / step)), int(round((p[1] - y0) / step))

    sa, sb = cell(a), cell(b)
    start, goal = (layers.index(la), *sa), (layers.index(lb), *sb)
    for n in (start, goal):
        free[n] = True                          # own pad / track end
    h = lambda n: math.hypot(n[1] - goal[1], n[2] - goal[2])
    openq, came, cost = [(h(start), 0.0, start)], {start: None}, {start: 0.0}
    moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    found = False
    while openq:
        _f, g, n = heapq.heappop(openq)
        if n == goal:
            found = True
            break
        if g > cost.get(n, 1e18):
            continue
        li, i, j = n
        nbrs = [((li, i + di, j + dj), c) for di, dj, c in moves]
        if via_free(i, j):
            nbrs += [((lk, i, j), 12.0) for lk in range(len(layers)) if lk != li]
        for m, c in nbrs:
            if not (0 <= m[1] < nx and 0 <= m[2] < ny) or not free.get(m, False):
                continue
            ng = g + c
            if ng < cost.get(m, 1e18):
                cost[m] = ng
                came[m] = n
                heapq.heappush(openq, (ng + h(m), ng, m))
    if not found:
        print(f"    maze: no path ({len(cost)} nodes explored, "
              f"{sum(1 for n in cost if n[0] != start[0])} off the start layer)")
        return False
    path = []
    n = goal
    while n:
        path.append(n)
        n = came[n]
    path.reverse()
    pt_of = lambda n: (x0 + n[1] * step, y0 + n[2] * step)
    # straight runs per layer; vias where the layer changes
    runs, vias = [], []
    seg_start = path[0]
    for k in range(1, len(path)):
        prev, cur = path[k - 1], path[k]
        if cur[0] != prev[0]:
            runs.append((seg_start, prev))
            vias.append(prev)
            seg_start = cur
            continue
        nxt = path[k + 1] if k + 1 < len(path) else None
        if nxt is None or nxt[0] != cur[0] or (nxt[1] - cur[1], nxt[2] - cur[2]) != (cur[1] - prev[1], cur[2] - prev[2]):
            runs.append((seg_start, cur))
            seg_start = cur
    segs = []
    for s, e in runs:
        p, q = pt_of(s), pt_of(e)
        if s == path[0]:
            p = a
        if e == path[-1]:
            q = b
        if p != q:
            segs.append((p, q, layers[s[0]]))
    if path[0][1:] != cell(a) or True:
        pass
    for p, q, l in segs:
        if not seg_clear(p, q, width / 2, l, obs):
            print(f"    maze: path found but segment {p}->{q} on {board.GetLayerName(l)} fails the exact check")
            return False
    for p, q, l in segs:
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(mm(p[0]), mm(p[1])))
        t.SetEnd(pcbnew.VECTOR2I(mm(q[0]), mm(q[1])))
        t.SetWidth(mm(width))
        t.SetLayer(l)
        t.SetNet(net)
        board.Add(t)
    for n in vias:
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(mm(pt_of(n)[0]), mm(pt_of(n)[1])))
        v.SetWidth(mm(via_d))
        v.SetDrill(mm(drill))
        v.SetNet(net)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        board.Add(v)
    print(f"    maze: {len(segs)} segments, {len(vias)} vias")
    return True


def bridge(clr=0.16, via_d=0.5, drill=0.25):
    """Close the few connections the router left open (from gen/route/drc.rpt): a straight
    track when both ends are on F.Cu, else a via at the inner-layer track end plus an F.Cu
    stub to the pad. Each candidate is committed only if it clears all other-net copper."""
    import re
    rpt = open(os.path.join(HERE, "route", "drc.rpt"), encoding="utf-8").read()
    board = pcbnew.LoadBoard(PCB)
    to = lambda v: (pcbnew.ToMM(v.x), pcbnew.ToMM(v.y))
    L3 = set(SIG)
    holes = [to(p.GetPosition()) for fp in board.GetFootprints() for p in fp.Pads()
             if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)]
    holes += [to(t.GetPosition()) for t in board.GetTracks() if isinstance(t, pcbnew.PCB_VIA)]

    def obstacles(net):
        obs = []
        for fp in board.GetFootprints():
            for pad in fp.Pads():
                if pad.GetNetname() == net:
                    continue
                bb = pad.GetBoundingBox()
                ls = L3 if pad.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH) else \
                    {l for l in L3 if pad.IsOnLayer(l)}
                obs.append(("rect", (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                                     pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())), ls, 0.0))
        for t in board.GetTracks():
            if t.GetNetname() == net:
                continue
            if isinstance(t, pcbnew.PCB_VIA):
                obs.append(("circ", to(t.GetPosition()), L3, pcbnew.ToMM(t.GetWidth(pcbnew.F_Cu)) / 2))
            else:
                obs.append(("seg", (to(t.GetStart()), to(t.GetEnd())), {t.GetLayer()}, pcbnew.ToMM(t.GetWidth()) / 2))
        return obs

    def seg_clear(a, b, hw, layer, obs):
        for kind, g, ls, w in obs:
            if layer not in ls:
                continue
            d = _rect_seg_dist(g, a, b) if kind == "rect" else \
                (_seg_dist(g, a, b) - w if kind == "circ" else _seg_seg_dist(a, b, g[0], g[1]) - w)
            if d < hw + clr:
                return False
        return True

    fixed, left = 0, 0
    for block in rpt.split("[unconnected_items]")[1:]:
        block = block.split("\n[")[0]
        items = re.findall(r"@\(([\d.]+) mm, ([\d.]+) mm\): (\w+)[^\[]*\[([^\]]+)\][^\n]* on (\S+)", block)
        if len(items) != 2:
            left += 1
            continue
        net = items[0][3]
        nobj = board.FindNet(net)
        obs = obstacles(net)
        ends = []
        for x, y, kind, _n, layer in items:
            p = (float(x), float(y))
            lid = board.GetLayerID(layer.rstrip(","))
            if kind == "Track":
                cands = [to(e) for t in board.GetTracks() if not isinstance(t, pcbnew.PCB_VIA)
                         and t.GetNetname() == net and t.GetLayer() == lid for e in (t.GetStart(), t.GetEnd())]
                p = min(cands, key=lambda q: (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2)
            ends.append((p, lid))
        (a, la), (b, lb) = ends
        width = float(os.environ.get("ICARUS_BRIDGE_W", "0.25"))   # 0.2 = the default signal width (fine-pitch escapes)
        done = False
        if la == lb and seg_clear(a, b, width / 2, la, obs):
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
            t.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
            t.SetWidth(mm(width))
            t.SetLayer(la)
            t.SetNet(nobj)
            board.Add(t)
            done = True
        elif la != lb:
            inner, outer = (a, b) if la != pcbnew.F_Cu else (b, a)
            via_ok = all(((inner[0] - h[0]) ** 2 + (inner[1] - h[1]) ** 2) ** 0.5 >= drill + 0.5 for h in holes)
            circ_ok = all(seg_clear(inner, inner, via_d / 2, l, obs) for l in L3)
            if via_ok and circ_ok and seg_clear(inner, outer, width / 2, pcbnew.F_Cu, obs):
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(pcbnew.VECTOR2I(mm(inner[0]), mm(inner[1])))
                v.SetWidth(mm(via_d))
                v.SetDrill(mm(drill))
                v.SetNet(nobj)
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                board.Add(v)
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(v.GetPosition())
                t.SetEnd(pcbnew.VECTOR2I(mm(outer[0]), mm(outer[1])))
                t.SetWidth(mm(width))
                t.SetLayer(pcbnew.F_Cu)
                t.SetNet(nobj)
                board.Add(t)
                done = True
        if not done:
            done = _maze(board, nobj, a, la, b, lb, obs, holes, width, via_d, drill, clr, seg_clear,
                         window=float(os.environ.get("ICARUS_MAZE_WINDOW", "6.0")))
        fixed += done
        left += not done
        print(f"  {net}: {'bridged' if done else 'NOT routed'}")
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)
    print(f"bridge: {fixed} closed, {left} left")


def fanout_gnd(radius_max=3.0, via_d=0.5, drill=0.25, stub_w=0.25, clr=0.18, board=None, targets=None,
               lock=False, stitch=True, net="GND"):
    """Give every GND pad that the pours can't reach a via to the In1 plane.

    Reads the unconnected GND pads from gen/route/drc.rpt, then for each searches rings
    around the pad for a via spot (0.5 mm via, checked on F/In2/B) and a straight F.Cu stub
    that clear all other-net copper by `clr`. Geometry is plain Python (rect/segment/circle)."""
    import math
    import re
    standalone = board is None
    if standalone:
        rpt = open(os.path.join(HERE, "route", "drc.rpt"), encoding="utf-8").read()
        want = set()
        for block in rpt.split("[unconnected_items]")[1:]:
            for num, ref in re.findall(r"[Pp]ad (\S+) \[GND\] of (\S+)", block.split("\n[")[0]):
                want.add((ref, num))
        board = pcbnew.LoadBoard(PCB)
    else:
        want = set(targets)
    gnd = board.FindNet(net)                      # GND, or a rail going down to its power-plane island
    to = lambda v: (pcbnew.ToMM(v.x), pcbnew.ToMM(v.y))
    # obstacles: (kind, geom, layers, half_width)
    obs = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetname() == net:
                continue
            bb = pad.GetBoundingBox()
            layers = {l for l in SIG if pad.IsOnLayer(l)}
            if pad.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
                layers = set(SIG)
            obs.append(("rect", (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                                 pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())), layers, 0.0))
    for t in board.GetTracks():
        if t.GetNetname() == net:
            continue
        if isinstance(t, pcbnew.PCB_VIA):
            obs.append(("circ", to(t.GetPosition()), set(SIG),
                        pcbnew.ToMM(t.GetWidth(pcbnew.F_Cu)) / 2))
        else:
            obs.append(("seg", (to(t.GetStart()), to(t.GetEnd())), {t.GetLayer()}, pcbnew.ToMM(t.GetWidth()) / 2))

    def clear_point(p, r, layers, near):
        for kind, g, ls, hw in near:
            if not (ls & layers):
                continue
            if kind == "rect":
                x0, y0, x1, y1 = g
                dx = max(x0 - p[0], 0, p[0] - x1)
                dy = max(y0 - p[1], 0, p[1] - y1)
                d = (dx * dx + dy * dy) ** 0.5
            elif kind == "circ":
                d = ((p[0] - g[0]) ** 2 + (p[1] - g[1]) ** 2) ** 0.5 - hw
            else:
                d = _seg_dist(p, g[0], g[1]) - hw
            if d < r + clr:
                return False
        return True

    def clear_stub(a, b, hw, near, own, layer=pcbnew.F_Cu):
        for kind, g, ls, w in near:
            if layer not in ls or g is own:
                continue
            if kind == "rect":
                d = _rect_seg_dist(g, a, b)
            elif kind == "circ":
                d = _seg_dist(g, a, b) - w
            else:
                d = _seg_seg_dist(a, b, g[0], g[1]) - w
            if d < hw + clr:
                return False
        return True

    added, failed = 0, []
    W0, H0 = OX, OY
    # every drilled hole as (x, y, radius): vias, PTH and NPTH pads
    new_vias = [to(t.GetPosition()) + (pcbnew.ToMM(t.GetDrillValue()) / 2,) for t in board.GetTracks()
                if isinstance(t, pcbnew.PCB_VIA)]
    new_vias += [to(p.GetPosition()) + (pcbnew.ToMM(max(p.GetDrillSize().x, p.GetDrillSize().y)) / 2,)
                 for fp in board.GetFootprints() for p in fp.Pads()
                 if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)]
    hole_ok = lambda p: all(((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2) ** 0.5 >= drill / 2 + q[2] + 0.3 + 0.1
                            for q in new_vias)
    for ref, num in sorted(want):
        fp = board.FindFootprintByReference(ref)
        pad = next(p for p in fp.Pads() if p.GetNumber() == num)
        c = to(pad.GetPosition())
        near = [o for o in obs if (o[0] == "rect" and o[1][0] - 6 < c[0] < o[1][2] + 6 and o[1][1] - 6 < c[1] < o[1][3] + 6)
                or (o[0] == "circ" and abs(o[1][0] - c[0]) < 6 and abs(o[1][1] - c[1]) < 6)
                or (o[0] == "seg" and min(_seg_dist(c, o[1][0], o[1][1]), 99) < 6)]
        bb = pad.GetBoundingBox()
        half = max(pcbnew.ToMM(bb.GetWidth()), pcbnew.ToMM(bb.GetHeight())) / 2
        lay = pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu   # bottom-side parts stub on B.Cu
        done = False
        r = half + via_d / 2 + 0.15
        while r <= radius_max + half and not done:
            for k in range(24):
                ang = 2 * math.pi * k / 24
                p = (c[0] + r * math.cos(ang), c[1] + r * math.sin(ang))
                if not (W0 + 0.8 < p[0] < W0 + W - 0.8 and H0 + 0.8 < p[1] < H0 + H - 0.8):
                    continue
                if hole_ok(p) and clear_point(p, via_d / 2, set(SIG), near) and \
                        clear_stub(c, p, stub_w / 2, near, None, lay):
                    new_vias.append(p + (drill / 2,))
                    v = pcbnew.PCB_VIA(board)
                    v.SetPosition(pcbnew.VECTOR2I(mm(p[0]), mm(p[1])))
                    v.SetWidth(mm(via_d))
                    v.SetDrill(mm(drill))
                    v.SetNet(gnd)
                    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                    v.SetLocked(lock)
                    board.Add(v)
                    t = pcbnew.PCB_TRACK(board)
                    t.SetStart(pad.GetPosition())
                    t.SetEnd(v.GetPosition())
                    t.SetWidth(mm(stub_w))
                    t.SetLayer(lay)
                    t.SetNet(gnd)
                    t.SetLocked(lock)
                    board.Add(t)
                    obs.append(("circ", p, set(SIG), -1e9))   # GND: not an obstacle
                    added += 1
                    done = True
                    break
            r += 0.2
        if not done:
            failed.append(f"{ref}.{num}")
    if not stitch:
        return added, failed
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    # Stitch every F.Cu GND pour fragment that isn't the main one: one via inside each island
    # ties it to the In1 plane (fragments are otherwise joined only through a few pads).
    stitched = 0
    gnd_vias = [to(t.GetPosition()) for t in board.GetTracks() if isinstance(t, pcbnew.PCB_VIA) and t.GetNetname() == "GND"]
    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetNetname() != "GND" or z.GetLayer() not in (pcbnew.F_Cu, pcbnew.B_Cu):
            continue
        polys = z.GetFilledPolysList(z.GetLayer())
        frags = []
        for i in range(polys.OutlineCount()):
            ch = polys.Outline(i)
            pts = [(pcbnew.ToMM(ch.CPoint(j).x), pcbnew.ToMM(ch.CPoint(j).y)) for j in range(ch.PointCount())]
            if len(pts) < 3:
                continue
            area = abs(sum(pts[j][0] * pts[j - 1][1] - pts[j - 1][0] * pts[j][1] for j in range(len(pts)))) / 2
            frags.append((area, pts))
        frags.sort(key=lambda f: -f[0])
        for area, pts in frags[1:]:                       # the largest fragment is the main pour
            if area < 0.6:
                continue
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            inside = lambda q: sum(((pts[j][1] > q[1]) != (pts[j - 1][1] > q[1])) and
                                   (q[0] < (pts[j - 1][0] - pts[j][0]) * (q[1] - pts[j][1]) /
                                    (pts[j - 1][1] - pts[j][1] + 1e-12) + pts[j][0])
                                   for j in range(len(pts))) % 2 == 1
            edge = lambda q: min(_seg_dist(q, pts[j - 1], pts[j]) for j in range(len(pts)))
            if any(inside(q) for q in gnd_vias):          # already stitched (re-run)
                continue
            near = [o for o in obs if (o[0] == "rect" and min(xs) - 3 < o[1][2] and o[1][0] < max(xs) + 3
                                        and min(ys) - 3 < o[1][3] and o[1][1] < max(ys) + 3)
                    or (o[0] == "circ" and min(xs) - 3 < o[1][0] < max(xs) + 3 and min(ys) - 3 < o[1][1] < max(ys) + 3)
                    or (o[0] == "seg" and min(xs) - 3 < max(o[1][0][0], o[1][1][0]) and min(o[1][0][0], o[1][1][0]) < max(xs) + 3
                        and min(ys) - 3 < max(o[1][0][1], o[1][1][1]) and min(o[1][0][1], o[1][1][1]) < max(ys) + 3)]
            best = None
            step = 0.15
            y = min(ys)
            while y <= max(ys) and best is None:
                x = min(xs)
                while x <= max(xs):
                    q = (x, y)
                    if inside(q) and edge(q) >= via_d / 2 and hole_ok(q) and \
                            clear_point(q, via_d / 2, set(SIG), near):
                        best = q
                        break
                    x += step
                y += step
            if best:
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(pcbnew.VECTOR2I(mm(best[0]), mm(best[1])))
                v.SetWidth(mm(via_d))
                v.SetDrill(mm(drill))
                v.SetNet(gnd)
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                board.Add(v)
                new_vias.append(best + (drill / 2,))
                stitched += 1
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)
    print(f"GND fan-out: {added} pad vias, {stitched} island stitches, {len(failed)} pads without room: "
          f"{' '.join(failed)}")


def thermals():
    """Pads DRC reports as starved_thermal (one spoke after routing) get a solid zone
    connection, then refill. Reads gen/route/drc.rpt; re-run DRC afterwards."""
    import re
    rpt = open(os.path.join(HERE, "route", "drc.rpt"), encoding="utf-8").read()
    hits = set()
    for block in rpt.split("[starved_thermal]")[1:]:
        m = re.search(r"[Pp]ad (\S+) \[[^\]]*\] of (\S+)", block.split("\n[")[0])
        if m:
            hits.add((m.group(2), m.group(1)))
    board = pcbnew.LoadBoard(PCB)
    for ref, num in sorted(hits):
        for pad in board.FindFootprintByReference(ref).Pads():
            if pad.GetNumber() == num:
                pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)
    print("solid zone connection on", len(hits), "pads:", ", ".join(f"{r}.{n}" for r, n in sorted(hits)))


def corridor():
    """Add the VBAT_IN B.Cu corridor to an already finished board."""
    board = pcbnew.LoadBoard(PCB)
    print("VBAT_IN corridor zones:", power_corridor(board))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)


def ripup_holes(clr=0.2):
    """Delete inner-layer track segments that pass too close to a through-hole pad's drill (the
    Specctra [TB] padstack problem, see via_keepouts) so bridge() can re-close those connections."""
    board = pcbnew.LoadBoard(PCB)
    holes = [(pcbnew.ToMM(pd.GetPosition().x), pcbnew.ToMM(pd.GetPosition().y),
              pcbnew.ToMM(max(pd.GetBoundingBox().GetWidth(), pd.GetBoundingBox().GetHeight())) / 2,
              pd.GetNetname())                    # the pad's copper ring (inner layers too), not just its drill
             for fp in board.GetFootprints() for pd in fp.Pads() if pd.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
    gone = []
    for t in list(board.GetTracks()):
        if isinstance(t, pcbnew.PCB_VIA) or t.GetLayer() in (pcbnew.F_Cu, pcbnew.B_Cu):
            continue
        a = (pcbnew.ToMM(t.GetStart().x), pcbnew.ToMM(t.GetStart().y))
        b = (pcbnew.ToMM(t.GetEnd().x), pcbnew.ToMM(t.GetEnd().y))
        hw = pcbnew.ToMM(t.GetWidth()) / 2
        if any(n != t.GetNetname() and _seg_dist((x, y), a, b) < r + hw + clr for x, y, r, n in holes):
            gone.append(t.GetNetname())
            board.Remove(t)
    pcbnew.SaveBoard(PCB, board)                  # no more pcbnew calls here: its SWIG types break after Remove()
    print(f"ripped up {len(gone)} inner-layer segments through drill holes: {sorted(set(gone))}")
    print("next: make_pcb.py keepouts (fresh process) adds the keep-outs and refills")


def keepouts():
    """Add the inner-layer via keep-outs to a finished board (once) and refill the pours."""
    board = pcbnew.LoadBoard(PCB)
    if any(z.GetIsRuleArea() for z in board.Zones()):
        print("keep-outs already present")
    else:
        print("inner-layer keep-outs over through-hole via pads:", via_keepouts(board))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)


def redsn():
    """Re-export the routed board (pours removed) so Freerouting can continue from it."""
    board = pcbnew.LoadBoard(PCB)
    um = lambda v: pcbnew.ToMM(v) * 1000.0                     # DSN units: µm, y up
    pads = [(um(bb.GetLeft()), -um(bb.GetBottom()), um(bb.GetRight()), -um(bb.GetTop()),
             None if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)
             else {board.GetLayerName(l) for l in SIG if p.IsOnLayer(l)})
            for fp in board.GetFootprints() for p in fp.Pads() for bb in [p.GetBoundingBox()]]   # before Remove()
    for z in [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetLayer() not in PLANES]:
        board.Remove(z)
    if not pcbnew.ExportSpecctraDSN(board, DSN):
        raise SystemExit("DSN export failed")
    dsn_router_settings(board, DSN)
    import dsn_merge_wires                        # chains of 2-point wires overflow Freerouting's combine()
    text, before, after = dsn_merge_wires.merge(open(DSN, encoding="utf-8").read(), pads)
    open(DSN, "w", encoding="utf-8").write(text)
    print("re-exported", DSN, f"({before} wires merged into {after})")


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "fanout":
        fanout_gnd(radius_max=float(sys.argv[2]) if len(sys.argv) > 2 else 3.0)
    elif cmd == "finish":
        finish(os.path.abspath(sys.argv[2]) if len(sys.argv) > 2 else None)
    else:
        {"place": place, "redsn": redsn, "thermals": thermals, "bridge": bridge, "corridor": corridor,
         "ripup": ripup_holes, "keepouts": keepouts}[cmd]()
