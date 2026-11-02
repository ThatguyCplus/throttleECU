"""Generate bench_stim_wing.kicad_sch from design.py.

Every pin gets a short stub wire ending in a net label, a power symbol or a
no-connect flag, so connectivity is exactly the PARTS table in design.py.
Writes the KiCad 7 file format (KiCad 7-10 all open it); any Python 3.8+:
    python3 make_sch.py
Set KICAD_SYMBOL_DIR if KiCad's symbol libraries aren't in the default place.
"""
import math
import os
import uuid

import design
from sexpr import Str, dump, find_symbol, parse

KICAD_SYMBOLS = os.environ.get("KICAD_SYMBOL_DIR") or next(
    (p for p in ["/usr/share/kicad/symbols", r"C:\Program Files\KiCad\10.0\share\kicad\symbols",
                 r"C:\Program Files\KiCad\9.0\share\kicad\symbols"] if os.path.isdir(p)), "")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", design.PROJECT + ".kicad_sch")
NS = uuid.UUID("2b6a3e7c-5d14-4f0e-8c7a-91d0b3e6f412")
ROOT_UUID = str(uuid.uuid5(NS, "root"))
STUB = 2.54
POWER_NETS = {"GND": "power:GND", "+3V3": "power:+3V3", "+5V": "power:+5V", "+12V": "power:+12V"}
LIB = design.PROJECT


def uid(*key):
    return Str(uuid.uuid5(NS, "/".join(map(str, key))))


def font(size=1.27, hide=False, justify=None):
    e = ["effects", ["font", ["size", size, size]]]
    if justify:
        e.append(["justify"] + (justify if isinstance(justify, list) else [justify]))
    if hide:
        e.append("hide")
    return e


# --- library symbols ---------------------------------------------------------
_lib_cache = {}


def lib_symbol(lib_id):
    lib, name = lib_id.split(":")
    if lib == LIB:
        return feather_symbol()
    if lib not in _lib_cache:
        with open(os.path.join(KICAD_SYMBOLS, lib + ".kicad_sym"), encoding="utf-8") as f:
            _lib_cache[lib] = parse(f.read())
    sym = [x for x in find_symbol(_lib_cache[lib], name)]
    assert not any(isinstance(x, list) and x[0] == "extends" for x in sym), lib_id + " uses extends: pick its base symbol"
    sym[1] = Str(lib_id)
    return sym


def feather_symbol():
    name = "Feather_M4_CAN"
    pins = []
    for n, label in design.FEATHER_PINS.items():
        if n <= 16:
            x, y, rot = -17.78, 19.05 - 2.54 * (n - 1), 0
        else:
            x, y, rot = 17.78, 19.05 - 2.54 * (n - 17), 180
        pins.append(["pin", "passive", "line", ["at", x, y, rot], ["length", 5.08],
                     ["name", Str(label), font()], ["number", Str(str(n)), font()]])
    body = ["symbol", Str(name + "_0_1"),
            ["rectangle", ["start", -12.7, 21.59], ["end", 12.7, -21.59],
             ["stroke", ["width", 0.254], ["type", "default"]], ["fill", ["type", "background"]]]]
    return ["symbol", Str(LIB + ":" + name), ["in_bom", "yes"], ["on_board", "yes"],
            ["property", Str("Reference"), Str("J"), ["at", 0, 24.13, 0], font()],
            ["property", Str("Value"), Str("Feather M4 CAN"), ["at", 0, -24.13, 0], font()],
            ["property", Str("Footprint"), Str(LIB + ":Feather_Wing_Headers"), ["at", 0, 0, 0], font(hide=True)],
            ["property", Str("Datasheet"), Str("https://learn.adafruit.com/adafruit-feather-m4-can-express"),
             ["at", 0, 0, 0], font(hide=True)],
            ["property", Str("ki_description"), Str("Adafruit Feather M4 CAN header pins (stacking headers)"),
             ["at", 0, 0, 0], font(hide=True)],
            body, ["symbol", Str(name + "_1_1")] + pins]


def symbol_pins(sym, unit):
    """[(number, x, y, angle)] for a unit, including pins common to all units."""
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


# --- schematic items ---------------------------------------------------------
items, used_libs = [], {}
counters = {"#PWR": 0, "#FLG": 0}


def prop(name, value, x, y, hide=False, justify=None, angle=0):
    return ["property", Str(name), Str(value), ["at", round(x, 3), round(y, 3), angle],
            font(hide=hide, justify=justify)]


def snap(v, grid=1.27):
    return round(round(v / grid) * grid, 3)


def symbol(lib_id, ref, value, x, y, unit=1, rot=0, fp="", extra=None, ref_at=None, val_at=None, in_bom=True):
    x, y = snap(x), snap(y)
    sym = used_libs.setdefault(lib_id, lib_symbol(lib_id))
    key = (ref, unit)
    rx, ry = ref_at or (x + 2.54, y - 1.27)
    vx, vy = val_at or (x + 2.54, y + 1.27)
    is_power = ref.startswith("#")
    node = ["symbol", ["lib_id", Str(lib_id)], ["at", x, y, rot], ["unit", unit],
            ["in_bom", "no" if is_power or not in_bom else "yes"],
            ["on_board", "no" if is_power else "yes"], ["dnp", "no"], ["uuid", uid("sym", *key)],
            prop("Reference", ref, rx, ry, hide=is_power, justify=None if is_power else "left", angle=(-rot) % 360),
            prop("Value", value, vx, vy, justify=None if is_power else "left", angle=(-rot) % 360),
            prop("Footprint", fp, x, y, hide=True),
            prop("Datasheet", "", x, y, hide=True)]
    for k, v in (extra or {}).items():
        node.append(prop(k, v, x, y, hide=True))
    for num, *_ in symbol_pins(sym, unit):
        node.append(["pin", Str(num), ["uuid", uid("pin", ref, unit, num)]])
    node.append(["instances", ["project", Str(design.PROJECT),
                               ["path", Str("/" + ROOT_UUID), ["reference", Str(ref)], ["unit", unit]]]])
    items.append(node)
    return sym


def wire(x1, y1, x2, y2):
    items.append(["wire", ["pts", ["xy", round(x1, 3), round(y1, 3)], ["xy", round(x2, 3), round(y2, 3)]],
                  ["stroke", ["width", 0], ["type", "default"]], ["uuid", uid("w", x1, y1, x2, y2)]])


def label(net, x, y, outward):
    angle = {(1, 0): 0, (-1, 0): 180, (0, -1): 90, (0, 1): 270}[outward]
    just = "right" if outward in ((-1, 0), (0, 1)) else "left"
    items.append(["label", Str(net), ["at", round(x, 3), round(y, 3), angle],
                  ["effects", ["font", ["size", 1.27, 1.27]], ["justify", just, "bottom"]],
                  ["uuid", uid("lbl", net, x, y)]])


def power(net, x, y, outward):
    counters["#PWR"] += 1
    if net == "GND":
        rot = {(0, 1): 0, (1, 0): 90, (0, -1): 180, (-1, 0): 270}[outward]
    else:
        rot = {(0, -1): 0, (-1, 0): 90, (0, 1): 180, (1, 0): 270}[outward]
    ref = "#PWR%03d" % counters["#PWR"]
    vy = y + 5.08 * outward[1] if outward[1] else y - 2.54
    symbol(POWER_NETS[net], ref, net, x, y, rot=rot, ref_at=(x, y), val_at=(x + 3.81 * outward[0], vy))


def no_connect(x, y):
    items.append(["no_connect", ["at", round(x, 3), round(y, 3)], ["uuid", uid("nc", x, y)]])


def text(s, x, y, size=1.27):
    items.append(["text", Str(s.replace("\n", "\\n")), ["at", x, y, 0],
                  ["effects", ["font", ["size", size, size]], ["justify", "left", "bottom"]],
                  ["uuid", uid("txt", s[:40], x, y)]])


def pin_ends(sym, unit, x, y, rot):
    r = math.radians(rot)
    for num, px, py, ang in symbol_pins(sym, unit):
        # Symbol coordinates are y-up; rotation is counter-clockwise.
        qx = px * math.cos(r) - py * math.sin(r)
        qy = px * math.sin(r) + py * math.cos(r)
        a = math.radians(ang + rot)
        yield num, round(x + qx, 3), round(y - qy, 3), (round(-math.cos(a)), round(math.sin(a)))


def place(part, x, y, unit=1, nets=None, rot=0):
    """Place a symbol; every pin gets a stub + label/power/NC."""
    x, y = snap(x), snap(y)
    extra = {"MPN": part["mpn"]} if part["mpn"] else {}
    if part.get("lcsc"):
        extra["LCSC"] = part["lcsc"]
    if part.get("note"):
        extra["Notes"] = part["note"]
    sym = symbol(part["lib"], part["ref"], part["value"], x, y, unit=unit, rot=rot, fp=part["fp"],
                 extra=extra, in_bom=part.get("in_bom", True))
    nets = nets if nets is not None else part["pins"]
    for num, ex, ey, out in pin_ends(sym, unit, x, y, rot):
        net = nets.get(num)
        if net is None:
            no_connect(ex, ey)
            continue
        sx, sy = ex + STUB * out[0], ey + STUB * out[1]
        wire(ex, ey, sx, sy)
        (power if net in POWER_NETS else label)(net, sx, sy, out)


def extent(part, unit, nets):
    """Width of a placed symbol including its stubs and labels (for flow layout)."""
    sym = used_libs.get(part["lib"]) or lib_symbol(part["lib"])
    xs = []
    for num, ex, ey, out in pin_ends(sym, unit, 0, 0, 0):
        net = nets.get(num, "")
        lab = 0 if net in POWER_NETS or not net else 1.0 * len(net) + 1
        xs.append(ex + out[0] * (STUB + lab))
    return (min(xs + [-5]), max(xs + [5]))


def part(ref):
    return next(p for p in design.PARTS if p["ref"] == ref)


# --- layout: titled blocks, parts flowing left to right ----------------------
BLOCKS = [
    ("Feather M4 CAN (stacking headers)", ["J1"]),
    ("12 V input: fuse, reverse-polarity diode, load-dump TVS", ["J2", "F1", "D1", "D2", "C1", "C2", "R1", "D3"]),
    ("5 V buck (LMR16006, 60 V) and 5 V into the Feather USB pin", ["U1", "C3", "C4", "R2", "C5", "D4", "L1", "R3", "R4",
                                                                  "C6", "C7", "C8", "R5", "D5", "D6", "C9"]),
    ("SAS power (12 V, default off; JP1 = always on)", ["Q1", "R6", "R7", "Q2", "R8", "R9", "JP1", "F2", "R10", "D7"]),
    ("ARM loop (hard kill) and IG1 output (needs IG1_EN and ARM)", ["F3", "R17", "D9", "R18", "R19", "Q3", "R11",
                                                                   "R12", "Q4", "Q5", "R13", "F4", "R16", "D8"]),
    ("Relays: de-energised = car wiring connected", ["K1", "Q6", "R20", "D10", "R21", "D11",
                                                     "K2", "Q7", "R22", "D12", "R23", "D13"]),
    ("NEP (12 V pull-up) and VSP (5 V pull-up) open-drain outputs",
     ["R24", "R25", "Q8", "R26", "R27", "R28", "D14", "R29", "R30", "Q9", "R31", "R32", "R33", "D15"]),
    ("Torque DAC: MCP4728 (2.048 V ref, x1) -> MCP6004 x2.2 on 5 V -> 0-4.5 V",
     ["U2", "R34", "C10", "C11"] + [f"U3.{u}" for u in (1, 2, 3, 4, 5)] +
     [f"R{n}" for n in range(40, 56)] + ["C12", "D16", "D17"]),
    ("I2C expander TCA9534 (0x20): powers up as inputs", ["U4", "C13", "R35", "R36", "D18", "R37"]),
    ("Connectors", ["J3", "J4", "J5"]),
    ("Test pads", [f"TP{i + 1}" for i in range(len(design.TEST_PADS))]),
]
used = {r.split(".")[0] for _, refs in BLOCKS for r in refs}
missing = {p["ref"] for p in design.PARTS} - used
assert not missing, missing

text("Bench stimulus FeatherWing: 12 V power, SAS power + CAN pass-through, EPS NEP/VSP/torque stimulus",
     20, 20, 2.5)
text("Every EPS line goes through a relay whose NC contact is the car's own wire: no power, no ARM loop or\n"
     "no enable bit means the EPS sees the car exactly as without this board. ARM loop on J5 (pins 7-8): wire\n"
     "an NC e-stop or key switch. Faking VT3/VT6 makes the motor steer: wheels off the ground, hands clear.\n"
     "All enables come from U4 (TCA9534), which powers up with every pin an input; 100k pull-downs hold\n"
     "every driver off until firmware sets the bit.", 20, 30)
X0, XMAX, y = 20, 800, 60
for title, refs in BLOCKS:
    text(title, X0, y, 1.8)
    x, row_h, y_row = X0, 0, y + 12
    for r in refs:
        ref, _, unit = r.partition(".")
        p = part(ref)
        unit = int(unit) if unit else 1
        nets = p["units"][unit] if "units" in p else p["pins"]
        sym = used_libs.setdefault(p["lib"], lib_symbol(p["lib"]))
        lo, hi = extent(p, unit, nets)
        ys = [ey for _, _, ey, _ in pin_ends(sym, unit, 0, 0, 0)]
        h = (max(ys) - min(ys)) + 14
        if x - lo + hi > XMAX:
            x, y_row = X0, y_row + row_h
            row_h = 0
        top = -min(ys) + 6
        place(p, x - lo, y_row + top, unit=unit, nets=nets)
        x += hi - lo + 8
        row_h = max(row_h, h)
    y = y_row + row_h + 8

# PWR_FLAGs: the Feather drives +3V3/GND; +12V comes through D1, +5V from L1.
for i, net in enumerate(["+3V3", "GND", "+5V", "+12V"]):
    fx, fy = snap(X0 + 25 * i), snap(y + 10)
    counters["#FLG"] += 1
    symbol("power:PWR_FLAG", "#FLG%02d" % counters["#FLG"], "PWR_FLAG", fx + 7.62, fy,
           ref_at=(fx + 7.62, fy), val_at=(fx + 7.62, fy - 3.81))
    wire(fx, fy, fx + 7.62, fy)
    power(net, fx, fy, (0, 1) if net == "GND" else (0, -1))
text("Power: +3V3/GND from the Feather; +12V via D1; +5V from the buck (L1)", X0, y + 2, 1.27)
PAPER_H = y + 30

# --- write -------------------------------------------------------------------
sch = ["kicad_sch", ["version", 20230121], ["generator", "eeschema"],
       ["uuid", Str(ROOT_UUID)], ["paper", Str("A0")],
       ["title_block", ["title", Str("Bench stimulus FeatherWing")], ["date", Str("2026-11-02")],
        ["rev", Str(design.REV)], ["comment", 1, Str("Honda Insight EPS bench / in-car stimulus + 12 V power")],
        ["comment", 2, Str("Generated by gen/make_sch.py from gen/design.py")]],
       ["lib_symbols"] + list(used_libs.values())] + items + [
       ["sheet_instances", ["path", Str("/"), ["page", Str("1")]]]]
assert PAPER_H < 841, PAPER_H
with open(OUT, "w", encoding="utf-8", newline="\n") as f:
    f.write(dump(sch) + "\n")
print("wrote", os.path.normpath(OUT), len(items), "items")

# Project symbol library holding the Feather header symbol, plus its table.
here = os.path.dirname(OUT)
fs = feather_symbol()
fs[1] = Str("Feather_M4_CAN")
with open(os.path.join(here, design.PROJECT + ".kicad_sym"), "w", encoding="utf-8", newline="\n") as f:
    f.write(dump(["kicad_symbol_lib", ["version", 20220914], ["generator", "make_sch.py"], fs]) + "\n")
with open(os.path.join(here, "sym-lib-table"), "w", encoding="utf-8", newline="\n") as f:
    f.write('(sym_lib_table\n\t(lib (name "%s")(type "KiCad")'
            '(uri "${KIPRJMOD}/%s.kicad_sym")(options "")(descr "Bench stim wing symbols"))\n)\n' % (LIB, LIB))
