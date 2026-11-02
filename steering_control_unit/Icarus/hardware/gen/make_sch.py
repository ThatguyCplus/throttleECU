"""Generate the Icarus hierarchical schematic from design.py.

    python make_sch.py      (any Python 3.8+; KiCad 10 symbol libraries must be installed)

Every pin gets a short stub ending in a label (global when the net crosses sheets), a GND
symbol or a no-connect flag, so connectivity is exactly what design.py says. Parts are
auto-placed in rows, one row block per design group. Also writes icarus.kicad_sym (vendor
symbols copied from the throttle ECU project plus custom ones), the library tables and a
minimal project file.
"""
import copy
import json
import math
import os
import uuid

import design
from sexpr import Str, dump, find_symbol, parse

KICAD_SYMBOLS = r"C:\Program Files\KiCad\10.0\share\kicad\symbols"
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, ".."))
VENDOR = os.path.join(OUT, "vendor_sym")
NS = uuid.UUID("0c1a2b3c-4d5e-4f60-8a71-92b3c4d5e6f7")
ROOT_UUID = str(uuid.uuid5(NS, "root"))
STUB = 2.54
CHAR_W = 1.05            # label text width per character at 1.27 mm font (approx.)
GAP = 5.08
PAPERS = [("A2", 594, 420), ("A1", 841, 594), ("A0", 1189, 841)]
TWO_PIN_ROTATE = {"Device:R", "Device:C", "Device:L", "Device:FerriteBead_Small", "Device:Fuse",
                  "Device:C_Polarized", "Jumper:SolderJumper_2_Open", "Device:LED", "Device:D_Schottky",
                  "Diode:1N4148WS", "Device:D_Zener"}

# vendor symbols: icarus name -> (file in vendor_sym, symbol name inside it)
VENDOR_SYMS = {
    "F280049C": ("F280049C.kicad_sym", "F280049CPZS"),
    "TCAN3404": ("TCAN3404.kicad_sym", "PTCAN3404DRQ1"),
    "LMR38020": ("LMR38020.kicad_sym", "PLMR38020FSQDDARQ1"),
    "TPS1H100": ("TPS1H100.kicad_sym", "TPS1H100BQPWPRQ1"),
    "SIT2024B": ("SIT2024B.kicad_sym", "SIT2024BA-S2-XXE-10.000000E"),
}
# custom box symbols: name -> (ref prefix, [(number, name, side, type)])
CUSTOM_SYMS = {
    "TPS3850": ("U", [("1", "VDD", "L", "power_in"), ("10", "SENSE", "L", "input"), ("2", "CWD", "L", "passive"),
                      ("3", "SET0", "L", "input"), ("6", "SET1", "L", "input"), ("4", "CRST", "L", "passive"),
                      ("7", "WDI", "R", "input"), ("8", "WDO", "R", "open_collector"),
                      ("9", "~{RESET}", "R", "open_collector"), ("5", "GND", "B", "power_in"),
                      ("11", "EP", "B", "passive")]),
    "SQJ147ELP": ("Q", [("5", "D", "L", "passive"), ("1", "S", "R", "passive"), ("2", "S", "R", "passive"),
                        ("3", "S", "R", "passive"), ("4", "G", "B", "input")]),
    # TLV733P-Q1 in the DRV (WSON-6) package: SBVS283F Table 5-1. NOT the SOT-23 (DBV) pin order.
    "TLV73312_DRV": ("U", [("6", "IN", "L", "power_in"), ("4", "EN", "L", "input"), ("1", "OUT", "R", "power_out"),
                           ("2", "NC", "R", "no_connect"), ("5", "NC", "R", "no_connect"), ("3", "GND", "B", "power_in"),
                           ("7", "EP", "B", "passive")]),
    # TMS570LS0432 100-pin PZ: every pin from SPNS186C Tables 4-1 .. 4-16 (rev 0.7, CH-B monitor).
    "TMS570LS0432": ("U", [(n, nm, "L", "bidirectional") for n, nm in (
        ("1", "GIOA0/SPI3nCS3"), ("2", "GIOA1/SPI3nCS2"), ("5", "GIOA2/SPI3nCS1"), ("8", "GIOA3/SPI2nCS3"),
        ("9", "GIOA4/SPI2nCS2"), ("10", "GIOA5/EXTCLKIN"), ("12", "GIOA6/SPI2nCS1/N2HET31"), ("18", "GIOA7/N2HET29"),
        ("19", "N2HET0"), ("22", "N2HET2"), ("25", "N2HET4"), ("26", "N2HET6"), ("74", "N2HET8"), ("83", "N2HET10"),
        ("89", "N2HET12"), ("90", "N2HET14"), ("97", "N2HET16"), ("98", "N2HET18"), ("11", "N2HET22"),
        ("64", "N2HET24"), ("58", "ADEVT/N2HET28"), ("27", "MIBSPI1nCS2/N2HET20/N2HET19"),
        ("39", "MIBSPI1nCS3/N2HET26"), ("93", "MIBSPI1nCS1/EQEPS/N2HET17"), ("68", "MIBSPI1nENA/N2HET23/N2HET30"),
        ("73", "MIBSPI1nCS0"), ("65", "MIBSPI1SIMO"), ("66", "MIBSPI1SOMI"), ("67", "MIBSPI1CLK"),
        ("23", "SPI2nCS0"), ("70", "SPI2SIMO"), ("69", "SPI2SOMI"), ("71", "SPI2CLK"), ("35", "SPI3SIMO"),
        ("34", "SPI3SOMI"), ("36", "SPI3CLK/EQEPA"), ("37", "SPI3nENA/EQEPB"), ("38", "SPI3nCS0/EQEPI"),
        ("62", "CAN1TX"), ("63", "CAN1RX"), ("91", "CAN2TX"), ("92", "CAN2RX"), ("95", "LINTX"), ("94", "LINRX"),
        ("84", "ECLK"), ("75", "TMS"), ("77", "TDI"))]
        + [("78", "TDO", "L", "output"), ("79", "TCK", "L", "input"), ("76", "~{TRST}", "L", "input"),
           ("80", "RTCK", "L", "output")]
        + [(n, nm, "R", "input") for n, nm in (
            ("42", "ADIN0"), ("49", "ADIN1"), ("51", "ADIN2"), ("52", "ADIN3"), ("54", "ADIN4"), ("55", "ADIN5"),
            ("56", "ADIN6"), ("43", "ADIN7"), ("57", "ADIN8"), ("48", "ADIN9"), ("50", "ADIN10"), ("53", "ADIN11"),
            ("40", "ADIN16"), ("41", "ADIN17"), ("44", "ADIN20"), ("45", "ADIN21"))]
        + [("46", "VCCAD/ADREFHI", "R", "power_in"), ("47", "VSSAD/ADREFLO", "R", "power_in"),
           ("31", "~{PORRST}", "R", "input"), ("81", "~{RST}", "R", "bidirectional"),
           ("82", "~{ERROR}", "R", "bidirectional"), ("14", "OSCIN", "R", "input"), ("16", "OSCOUT", "R", "output"),
           ("15", "KELVIN_GND", "R", "passive"), ("24", "TEST", "R", "input"), ("3", "FLTP1", "R", "passive"),
           ("4", "FLTP2", "R", "passive")]
        + [(n, "VCC", "B", "power_in") for n in ("13", "21", "30", "32", "61", "88", "99")]
        + [(n, "VCCIO", "B", "power_in") for n in ("6", "28", "60", "85")]
        + [("96", "VCCP", "B", "power_in")]
        + [(n, "VSS", "B", "power_in") for n in ("7", "17", "20", "29", "33", "59", "72", "86", "87", "100")]),
}
assert sorted(int(p[0]) for p in CUSTOM_SYMS["TMS570LS0432"][1]) == list(range(1, 101)), "TMS570: all 100 pins once"


def uid(*key):
    return Str(uuid.uuid5(NS, "/".join(map(str, key))))


def font(size=1.27):
    return ["effects", ["font", ["size", size, size]]]


def snap(v, grid=1.27):
    return round(round(v / grid) * grid, 3)


# --- library symbols ---------------------------------------------------------
_lib_cache = {}


def _load(path):
    if path not in _lib_cache:
        with open(path, encoding="utf-8") as f:
            _lib_cache[path] = parse(f.read())
    return _lib_cache[path]


def _rename(sym, old, new, lib_id):
    sym = copy.deepcopy(sym)
    sym[1] = Str(lib_id)
    for sub in sym:
        if isinstance(sub, list) and sub[0] == "symbol":
            sub[1] = Str(new + str(sub[1])[len(old):])
    return sym


def custom_symbol(name):
    prefix, pins = CUSTOM_SYMS[name]
    sides = {s: [p for p in pins if p[2] == s] for s in "LRB"}
    rows = max(len(sides["L"]), len(sides["R"]))
    half_h = 2.54 * (rows + 1) / 2
    half_w = max(10.16, 2.54 * (len(sides["B"]) + 1) / 2)
    half_h, half_w = snap(half_h, 2.54), snap(half_w, 2.54)
    out = []
    for side, lst in sides.items():
        for i, (num, pname, _s, ptype) in enumerate(lst):
            if side == "L":
                at = [-half_w - 5.08, half_h - 2.54 * (i + 1), 0]
            elif side == "R":
                at = [half_w + 5.08, half_h - 2.54 * (i + 1), 180]
            else:
                at = [-2.54 * (len(lst) - 1) / 2 + 2.54 * i, -half_h - 5.08, 90]
            at = [snap(at[0], 2.54), snap(at[1], 2.54), at[2]]
            out.append(["pin", ptype, "line", ["at", *at], ["length", 5.08],
                        ["name", Str(pname), font()], ["number", Str(num), font()]])
    body = ["symbol", Str(name + "_0_1"),
            ["rectangle", ["start", -half_w, half_h], ["end", half_w, -half_h],
             ["stroke", ["width", 0.254], ["type", "default"]], ["fill", ["type", "background"]]]]
    return ["symbol", Str("icarus:" + name), ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"],
            ["property", Str("Reference"), Str(prefix), ["at", 0, half_h + 1.27, 0], font()],
            ["property", Str("Value"), Str(name), ["at", 0, -half_h - 8.89, 0], font()],
            ["property", Str("Footprint"), Str(""), ["at", 0, 0, 0], ["hide", "yes"], font()],
            ["property", Str("Datasheet"), Str(""), ["at", 0, 0, 0], ["hide", "yes"], font()],
            ["property", Str("Description"), Str(name), ["at", 0, 0, 0], ["hide", "yes"], font()],
            body, ["symbol", Str(name + "_1_1")] + out]


def _pin_type_fix(name, num, pname, ptype):
    """Correct pin electrical types the vendor/stock symbols get wrong (checked against datasheets)."""
    if name == "TPS3702" and num in ("1", "6"):
        return "open_collector"                  # UV/OV are open-drain on TPS3702-Q1
    if name == "TPS1H100" and pname == "OUT":
        return "passive"                         # one output across three pins
    if name in ("LM66100DCK", "MCP73833-xxx-MF", "TPS61092") and ptype == "power_out":
        return "passive"                         # ORed / battery-backed rails: PWR_FLAG marks the source
    if name in VENDOR_SYMS and pname == "GND":
        return "power_in"                        # some vendor symbols mark GND as power_out
    if name == "F280049C" and pname.startswith(("VDD", "VSS")):
        return "power_in"
    if name in VENDOR_SYMS and ptype == "unspecified":
        return "passive"                         # GPIO/analog/strap pins: let the nets decide
    return ptype


def _fix_types(sym, name):
    for sub in sym:
        if isinstance(sub, list) and sub[0] == "symbol":
            for p in sub:
                if isinstance(p, list) and p[0] == "pin":
                    num = str(next(x for x in p if isinstance(x, list) and x[0] == "number")[1])
                    pname = str(next(x for x in p if isinstance(x, list) and x[0] == "name")[1])
                    p[1] = _pin_type_fix(name, num, pname, p[1])
    return sym


def lib_symbol(lib_id):
    return _fix_types(_lib_symbol(lib_id), lib_id.split(":")[1])


def _lib_symbol(lib_id):
    lib, name = lib_id.split(":")
    if lib == "icarus":
        if name in CUSTOM_SYMS:
            return custom_symbol(name)
        fname, orig = VENDOR_SYMS[name]
        return _rename(find_symbol(_load(os.path.join(VENDOR, fname)), orig), orig, name, lib_id)
    tree = _load(os.path.join(KICAD_SYMBOLS, lib + ".kicad_sym"))
    sym = find_symbol(tree, name)
    ext = [x for x in sym if isinstance(x, list) and x[0] == "extends"]
    if not ext:
        return _rename(sym, name, name, lib_id)
    parent_name = str(ext[0][1])
    flat = _rename(find_symbol(tree, parent_name), parent_name, name, lib_id)
    child_props = {str(x[1]): x for x in sym if isinstance(x, list) and x[0] == "property"}
    flat = [x if not (isinstance(x, list) and x[0] == "property" and str(x[1]) in child_props)
            else child_props[str(x[1])] for x in flat]
    return flat


def symbol_pins(sym, unit):
    """[(number, x, y, angle)] for a unit, including pins common to all units (unit 0)."""
    out = []
    for sub in sym:
        if not (isinstance(sub, list) and sub[0] == "symbol"):
            continue
        u = int(str(sub[1]).rsplit("_", 2)[1])
        if u not in (0, unit):
            continue
        for item in sub:
            if isinstance(item, list) and item[0] == "pin":
                at = next(x for x in item if isinstance(x, list) and x[0] == "at")
                num = next(x for x in item if isinstance(x, list) and x[0] == "number")
                out.append((str(num[1]), float(at[1]), float(at[2]), float(at[3])))
    return out


# --- net bookkeeping ---------------------------------------------------------
net_sheets, net_pins = {}, {}
for p in design.PARTS:
    for unit, pins in p["units"].items():
        for num, net in pins.items():
            net_sheets.setdefault(net, set()).add(p["sheet"])
            net_pins.setdefault(net, []).append(f"{p['ref']}.{num}")
single = {n: v for n, v in net_pins.items() if len(v) < 2}
assert not single, f"nets with one pin (typo?): {single}"
GLOBAL = {n for n, s in net_sheets.items() if len(s) > 1 or n in design.POWER_FLAGS}


# --- per-sheet writer --------------------------------------------------------
class Sheet:
    def __init__(self, key, title, notes, page):
        self.key, self.title, self.notes, self.page = key, title, notes, page
        self.uuid = str(uuid.uuid5(NS, "sheet/" + key)) if key != "root" else ROOT_UUID
        self.items, self.libs = [], {}
        self.path = "/" + ROOT_UUID + ("" if key == "root" else "/" + self.uuid)
        self.pwr = 0

    def prop(self, name, value, x, y, hide=False, justify=None, angle=0):
        p = ["property", Str(name), Str(value), ["at", round(x, 3), round(y, 3), angle]]
        if hide:
            p.append(["hide", "yes"])
        eff = font()
        if justify:
            eff.append(["justify", justify])
        p.append(eff)
        return p

    def symbol(self, lib_id, ref, value, x, y, unit=1, rot=0, fp="", extra=None, ref_at=None, val_at=None):
        sym = self.libs.setdefault(lib_id, lib_symbol(lib_id))
        is_power = ref.startswith("#")
        rx, ry = ref_at or (x + 2.54, y - 1.27)
        vx, vy = val_at or (x + 2.54, y + 1.27)
        node = ["symbol", ["lib_id", Str(lib_id)], ["at", x, y, rot], ["unit", unit],
                ["exclude_from_sim", "no"], ["in_bom", "no" if is_power else "yes"],
                ["on_board", "no" if is_power else "yes"], ["dnp", "no"], ["uuid", uid(self.key, "sym", ref, unit)],
                self.prop("Reference", ref, rx, ry, hide=is_power, justify=None if is_power else "left"),
                self.prop("Value", value, vx, vy, justify=None if is_power else "left"),
                self.prop("Footprint", fp, x, y, hide=True), self.prop("Datasheet", "", x, y, hide=True)]
        for k, v in (extra or {}).items():
            node.append(self.prop(k, v, x, y, hide=True))
        for num, *_ in symbol_pins(sym, unit):
            node.append(["pin", Str(num), ["uuid", uid(self.key, "pin", ref, unit, num)]])
        node.append(["instances", ["project", Str(design.PROJECT),
                                   ["path", Str(self.path), ["reference", Str(ref)], ["unit", unit]]]])
        self.items.append(node)
        return sym

    def wire(self, x1, y1, x2, y2):
        self.items.append(["wire", ["pts", ["xy", round(x1, 3), round(y1, 3)], ["xy", round(x2, 3), round(y2, 3)]],
                           ["stroke", ["width", 0], ["type", "default"]], ["uuid", uid(self.key, "w", x1, y1, x2, y2)]])

    def label(self, net, x, y, out):
        angle = {(1, 0): 0, (-1, 0): 0, (0, -1): 90, (0, 1): 270}[out]
        just = "right" if out == (-1, 0) else "left"
        eff = ["effects", ["font", ["size", 1.27, 1.27]], ["justify", just]]
        if net in GLOBAL:
            self.items.append(["global_label", Str(net), ["shape", "passive"], ["at", round(x, 3), round(y, 3),
                               180 if out == (-1, 0) else angle], ["fields_autoplaced", "yes"],
                               ["effects", ["font", ["size", 1.27, 1.27]], ["justify", "right" if out == (-1, 0) else "left"]],
                               ["uuid", uid(self.key, "gl", net, x, y)],
                               ["property", Str("Intersheetrefs"), Str("${INTERSHEET_REFS}"),
                                ["at", round(x, 3), round(y, 3), 0], ["effects", ["font", ["size", 1.27, 1.27]], ["hide", "yes"]]]])
        else:
            eff[2].append("bottom")
            self.items.append(["label", Str(net), ["at", round(x, 3), round(y, 3), angle], eff,
                               ["uuid", uid(self.key, "lbl", net, x, y)]])

    def gnd(self, x, y, out):
        self.pwr += 1
        rot = {(0, 1): 0, (1, 0): 90, (0, -1): 180, (-1, 0): 270}[out]
        self.symbol("power:GND", f"#PWR{self.page:02d}{self.pwr:03d}", "GND", x, y, rot=rot,
                    ref_at=(x, y), val_at=(x + 3.81 * out[0], y + 5.08 * out[1] if out[1] else y - 2.54))

    def no_connect(self, x, y):
        self.items.append(["no_connect", ["at", round(x, 3), round(y, 3)], ["uuid", uid(self.key, "nc", x, y)]])

    def text(self, s, x, y, size=1.27):
        self.items.append(["text", Str(s.replace("\n", "\\n")), ["exclude_from_sim", "no"], ["at", x, y, 0],
                           ["effects", ["font", ["size", size, size]], ["justify", "left", "bottom"]],
                           ["uuid", uid(self.key, "txt", s[:40], x, y)]])


def pin_geometry(sym, unit, rot, nets):
    """Pin endpoints, outward directions, and the bbox including stubs and label text."""
    r = math.radians(rot)
    geo = []
    xs, ys = [0.0], [0.0]
    for num, px, py, ang in symbol_pins(sym, unit):
        qx = px * math.cos(r) - py * math.sin(r)
        qy = px * math.sin(r) + py * math.cos(r)
        ex, ey = round(qx, 3), round(-qy, 3)
        a = math.radians(ang + rot)
        out = (round(-math.cos(a)), round(math.sin(a)))
        net = nets.get(num)
        geo.append((num, ex, ey, out, net))
        xs.append(ex)
        ys.append(ey)
        if net is not None:
            length = STUB + (3.0 if net == "GND" else len(net) * CHAR_W + (3 if net in GLOBAL else 1))
            xs.append(ex + out[0] * length)
            ys.append(ey + out[1] * length)
    return geo, (min(xs) - 1.27, min(ys) - 1.27, max(xs) + 1.27, max(ys) + 1.27)


def layout(sheet_key, width):
    """Return [(part, unit, rot, x, y, bbox)] plus group title positions, packed in rows."""
    placed, titles = [], []
    x0, y = 15.0, 45.0
    row_h, x = 0.0, x0
    current = object()
    for p in [p for p in design.PARTS if p["sheet"] == sheet_key]:
        if p["group"] != current:
            current = p["group"]
            y += row_h + (GAP * 2 if placed else 0)
            titles.append((current, x0, y))
            y += 6.0
            x, row_h = x0, 0.0
        sym = lib_symbol(p["lib"])
        rot = 90 if p["lib"] in TWO_PIN_ROTATE else 0
        for unit, nets in p["units"].items():
            _geo, (bx0, by0, bx1, by1) = pin_geometry(sym, unit, rot, nets)
            w, h = bx1 - bx0, by1 - by0 + 7.0      # room for ref/value text above
            if x + w > width - 15 and x > x0:
                y += row_h + GAP
                x, row_h = x0, 0.0
            sx, sy = snap(x - bx0), snap(y + 7.0 - by0)
            placed.append((p, unit, rot, sx, sy, (bx0, by0, bx1, by1)))
            x += w + GAP
            row_h = max(row_h, h)
    return placed, titles, y + row_h


def build_sheet(key, title, notes, page):
    for paper, w, h in PAPERS:
        placed, titles, bottom = layout(key, w)
        if bottom < h - 20:
            break
    sh = Sheet(key, title, notes, page)
    sh.text(title, 15, 22, 3.0)
    if notes:
        sh.text(notes, 15, 32, 1.5)
    for t, tx, ty in titles:
        sh.text(t, tx, ty + 3.5, 2.0)
    for p, unit, rot, x, y, (bx0, by0, _bx1, _by1) in placed:
        extra = {}
        if p["mpn"]:
            extra["MPN"] = p["mpn"]
        if p["note"]:
            extra["Notes"] = p["note"]
        ref_at = (x + bx0 + 1.27, y + by0 - 4.5)
        val_at = (x + bx0 + 1.27, y + by0 - 1.8)
        sym = sh.symbol(p["lib"], p["ref"], p["value"], x, y, unit=unit, rot=rot, fp=p["fp"], extra=extra,
                        ref_at=ref_at, val_at=val_at)
        geo, _ = pin_geometry(sym, unit, rot, p["units"][unit])
        for num, ex, ey, out, net in geo:
            ex, ey = round(x + ex, 3), round(y + ey, 3)
            if net is None:
                sh.no_connect(ex, ey)
                continue
            sx, sy = ex + STUB * out[0], ey + STUB * out[1]
            sh.wire(ex, ey, sx, sy)
            (sh.gnd if net == "GND" else lambda n_, a, b, o: sh.label(n_, a, b, o))(
                *((sx, sy, out) if net == "GND" else (net, sx, sy, out)))
    if key == "power":
        fx, fy = 15.0, snap(bottom + 15)
        sh.text("Power flags (rails driven by regulators / the protection FET)", fx, fy - 4, 2.0)
        for i, net in enumerate(design.POWER_FLAGS):
            x, yy = snap(fx + 30 * i), fy
            sh.symbol("power:PWR_FLAG", f"#FLG{i + 1:02d}", "PWR_FLAG", x, yy, ref_at=(x, yy), val_at=(x, yy - 3.81))
            sh.wire(x, yy, x, yy + STUB)
            if net == "GND":
                sh.gnd(x, yy + STUB, (0, 1))
            else:
                sh.label(net, x, yy + STUB, (0, 1))
        paper = next(pp for pp in PAPERS if pp[2] > fy + 40)[0]
    return sh, paper


def write_sheet(sh, paper, fname, title):
    tree = ["kicad_sch", ["version", 20260306], ["generator", Str("eeschema")], ["generator_version", Str("10.0")],
            ["uuid", Str(sh.uuid)], ["paper", Str(paper)],
            ["title_block", ["title", Str("Icarus - " + title)], ["date", Str("2026-11-01")], ["rev", Str(design.REV)],
             ["comment", 1, Str("Dual-channel steering torque-overlay unit, Honda Insight EPS")],
             ["comment", 2, Str("Generated by hardware/gen/make_sch.py from gen/design.py - do not hand-edit")]],
            ["lib_symbols"] + list(sh.libs.values())] + sh.items
    if sh.key == "root":
        tree.append(["sheet_instances", ["path", Str("/"), ["page", Str("1")]]])
    tree.append(["embedded_fonts", "no"])
    with open(os.path.join(OUT, fname), "w", encoding="utf-8", newline="\n") as f:
        f.write(dump(tree) + "\n")


root = Sheet("root", "Icarus", "", 1)
root.text("Icarus rev " + design.REV + " - dual-channel, triple-shutdown steering torque-overlay unit", 20, 25, 3.5)
root.text(
    "CH-A (F280049C-Q1) computes the overlay; CH-B (F280049C-Q1) monitors it; CH-C is an analog guard with no software.\n"
    "Overlay reaches the EPS only through relays K1 AND K2; each coil needs CH-A high side + CH-B low side + CH-C guard FET.\n"
    "With no power or any enable missing, NC contacts connect the OEM torque sensor straight to the EPS.\n"
    "Safety concept: ../docs/04_TECHNICAL_SAFETY_CONCEPT.md. Values marked TBD-DATA wait for logger measurements.",
    20, 35, 2.0)
summary = []
for i, (key, title, notes) in enumerate(design.SHEETS):
    sh, paper = build_sheet(key, title, notes, i + 2)
    write_sheet(sh, paper, f"{key}.kicad_sch", title)
    n_parts = len([p for p in design.PARTS if p["sheet"] == key])
    summary.append((key, paper, n_parts))
    col, row = i % 4, i // 4
    x, y = 20 + 95 * col, 70 + 45 * row
    root.items.append(
        ["sheet", ["at", x, y], ["size", 80, 30], ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"],
         ["dnp", "no"], ["fields_autoplaced", "yes"], ["stroke", ["width", 0.1524], ["type", "solid"]],
         ["fill", ["color", 0, 0, 0, 0.0]], ["uuid", Str(sh.uuid)],
         ["property", Str("Sheetname"), Str(key), ["at", x, y - 0.7, 0], ["effects", ["font", ["size", 1.5, 1.5]], ["justify", "left", "bottom"]]],
         ["property", Str("Sheetfile"), Str(f"{key}.kicad_sch"), ["at", x, y + 30.6, 0], ["effects", ["font", ["size", 1.27, 1.27]], ["justify", "left", "top"]]],
         ["instances", ["project", Str(design.PROJECT), ["path", Str("/" + ROOT_UUID), ["page", Str(str(i + 2))]]]]])
    root.text(title, x + 2, y + 8, 1.5)
write_sheet(root, "A3", design.PROJECT + ".kicad_sch", "top level")

# Project library with the vendor + custom symbols, library tables, minimal project file.
lib = ["kicad_symbol_lib", ["version", 20251024], ["generator", Str("make_sch.py")]]
for name in list(VENDOR_SYMS) + list(CUSTOM_SYMS):
    s = lib_symbol("icarus:" + name)
    s[1] = Str(name)
    lib.append(s)
with open(os.path.join(OUT, "icarus.kicad_sym"), "w", encoding="utf-8", newline="\n") as f:
    f.write(dump(lib) + "\n")
with open(os.path.join(OUT, "sym-lib-table"), "w", encoding="utf-8", newline="\n") as f:
    f.write('(sym_lib_table\n\t(version 7)\n\t(lib (name "icarus")(type "KiCad")'
            '(uri "${KIPRJMOD}/icarus.kicad_sym")(options "")(descr "Icarus vendor + custom symbols"))\n)\n')
with open(os.path.join(OUT, "fp-lib-table"), "w", encoding="utf-8", newline="\n") as f:
    f.write('(fp_lib_table\n\t(version 7)\n\t(lib (name "icarus")(type "KiCad")'
            '(uri "${KIPRJMOD}/icarus.pretty")(options "")(descr "Vendor footprints copied from the throttle ECU"))\n)\n')
pro = os.path.join(OUT, design.PROJECT + ".kicad_pro")
if not os.path.exists(pro):
    with open(pro, "w", encoding="utf-8", newline="\n") as f:
        json.dump({"meta": {"filename": design.PROJECT + ".kicad_pro", "version": 3}}, f, indent=2)
for key, paper, n in summary:
    print(f"{key:8s} {paper}  {n:3d} parts")
print("total", len(design.PARTS), "parts,", len(net_pins), "nets,", len(GLOBAL), "global")
