"""Generate eps_tap_wing.kicad_sch from design.py.

Every pin gets a short stub wire ending in a net label, a power symbol or a
no-connect flag, so connectivity is exactly the PARTS table in design.py.
Run with KiCad's Python or any Python 3.8+:  python make_sch.py
"""
import math
import os
import uuid

import design
from sexpr import Str, dump, find_symbol, parse

KICAD_SYMBOLS = r"C:\Program Files\KiCad\10.0\share\kicad\symbols"
OUT = os.path.join(os.path.dirname(__file__), "..", design.PROJECT + ".kicad_sch")
NS = uuid.UUID("6b1f4c1e-2f61-4d0a-9a0c-5e7a1d3c2b10")
ROOT_UUID = str(uuid.uuid5(NS, "root"))
STUB = 2.54
POWER_NETS = {"GND": "power:GND", "+3V3": "power:+3V3", "+5V": "power:+5V"}


def uid(*key):
    return Str(uuid.uuid5(NS, "/".join(map(str, key))))


def font(size=1.27):
    return ["effects", ["font", ["size", size, size]]]


# --- library symbols ---------------------------------------------------------
_lib_cache = {}


def lib_symbol(lib_id):
    lib, name = lib_id.split(":")
    if lib == "eps_tap_wing":
        return feather_symbol()
    if lib not in _lib_cache:
        with open(os.path.join(KICAD_SYMBOLS, lib + ".kicad_sym"), encoding="utf-8") as f:
            _lib_cache[lib] = parse(f.read())
    sym = [x for x in find_symbol(_lib_cache[lib], name)]
    assert not any(isinstance(x, list) and x[0] == "extends" for x in sym), lib_id
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
    return ["symbol", Str("eps_tap_wing:" + name),
            ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"],
            ["property", Str("Reference"), Str("J"), ["at", 0, 24.13, 0], font()],
            ["property", Str("Value"), Str("Feather M4 CAN"), ["at", 0, -24.13, 0], font()],
            ["property", Str("Footprint"), Str("eps_tap_wing:Feather_Wing_Headers"),
             ["at", 0, 0, 0], ["hide", "yes"], font()],
            ["property", Str("Datasheet"), Str("https://learn.adafruit.com/adafruit-feather-m4-can-express"),
             ["at", 0, 0, 0], ["hide", "yes"], font()],
            ["property", Str("Description"), Str("Adafruit Feather M4 CAN header pins (stacked below the wing)"),
             ["at", 0, 0, 0], ["hide", "yes"], font()],
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
    p = ["property", Str(name), Str(value), ["at", round(x, 3), round(y, 3), angle]]
    if hide:
        p.append(["hide", "yes"])
    eff = font()
    if justify:
        eff.append(["justify", justify])
    p.append(eff)
    return p


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
            ["exclude_from_sim", "no"], ["in_bom", "no" if is_power or not in_bom else "yes"],
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
    angle = {(1, 0): 0, (-1, 0): 180, (0, -1): 0, (0, 1): 0}[outward]
    just = "right" if outward == (-1, 0) else "left"
    items.append(["label", Str(net), ["at", round(x, 3), round(y, 3), angle],
                  ["effects", ["font", ["size", 1.27, 1.27]], ["justify", just, "bottom"]],
                  ["uuid", uid("lbl", net, x, y)]])


def power(net, x, y, outward):
    counters["#PWR"] += 1
    if net == "GND":
        rot = {(0, 1): 0, (1, 0): 90, (0, -1): 180, (-1, 0): 270}[outward]
    else:
        rot = {(0, -1): 0, (-1, 0): 90, (0, 1): 180, (1, 0): 270}[outward]
    ref = "#PWR%02d" % counters["#PWR"]
    vy = y + 5.08 * outward[1] if outward[1] else y - 2.54
    symbol(POWER_NETS[net], ref, net, x, y, rot=rot, ref_at=(x, y), val_at=(x + 3.81 * outward[0], vy))


def no_connect(x, y):
    items.append(["no_connect", ["at", round(x, 3), round(y, 3)], ["uuid", uid("nc", x, y)]])


def text(s, x, y, size=1.27):
    items.append(["text", Str(s.replace("\n", "\\n")), ["exclude_from_sim", "no"], ["at", x, y, 0],
                  ["effects", ["font", ["size", size, size]], ["justify", "left", "bottom"]],
                  ["uuid", uid("txt", s[:40], x, y)]])


def place(part, x, y, unit=1, nets=None, rot=0, stubs=None, ref_at=None, val_at=None):
    """Place a symbol; every pin gets a stub (length per pin via `stubs`) + label/power/NC."""
    x, y = snap(x), snap(y)
    extra = {"MPN": part["mpn"]} if part["mpn"] else {}
    if part.get("lcsc"):
        extra["LCSC"] = part["lcsc"]
    if part.get("note"):
        extra["Notes"] = part["note"]
    if ref_at:
        ref_at = (x + ref_at[0], y + ref_at[1])
    if val_at:
        val_at = (x + val_at[0], y + val_at[1])
    sym = symbol(part["lib"], part["ref"], part["value"], x, y, unit=unit, rot=rot, fp=part["fp"],
                 extra=extra, ref_at=ref_at, val_at=val_at, in_bom=part.get("in_bom", True))
    nets = nets if nets is not None else part["pins"]
    r = math.radians(rot)
    for num, px, py, ang in symbol_pins(sym, unit):
        # Symbol coordinates are y-up; rotation is counter-clockwise.
        qx = px * math.cos(r) - py * math.sin(r)
        qy = px * math.sin(r) + py * math.cos(r)
        ex, ey = round(x + qx, 3), round(y - qy, 3)
        a = math.radians(ang + rot)
        out = (round(-math.cos(a)), round(math.sin(a)))
        net = nets.get(num)
        if net is None:
            no_connect(ex, ey)
            continue
        stub = (stubs or {}).get(num, STUB)
        sx, sy = ex + stub * out[0], ey + stub * out[1]
        wire(ex, ey, sx, sy)
        (power if net in POWER_NETS else label)(net, sx, sy, out)


def part(ref):
    return next(p for p in design.PARTS if p["ref"] == ref)


def jumper_for(rref):
    """The solder jumper that bypasses series resistor `rref`."""
    n = [r for _pin, r in design.SERIES_R].index(rref) + 1
    return part(f"JP{n}")


# --- layout ------------------------------------------------------------------
text("EPS tap FeatherWing: input front end for the Honda Insight EPS logger", 20, 20, 2.5)
text("Taps connect at the EPS control unit, connector B (14P green) and A2 (+B). Every car-facing\n"
     "line goes through >=100 kOhm (1 MOhm for the torque sensor) before anything else, so a fault on\n"
     "the logger side can't load the EPS. Fit the plug with the ignition OFF. See ../../BOM.md.\n"
     "JP1-JP14 (open as shipped) short the series resistors: bridge them only when the resistors\n"
     "are fitted at the probe ends instead.", 20, 30)

place(part("J1"), 60, 110, stubs={"4": 10.16}, ref_at=(-5.08, -22.86), val_at=(-5.08, 24.13))
text("Feather M4 CAN (headers)", 40, 84, 1.8)
place(part("J2"), 50, 190, stubs={"5": 10.16})
text("J2: EPS taps (JST XA 15-way, 2.5 mm, locking)", 30, 168, 1.8)
for i, (_, short, src) in enumerate(design.J2_PINS):
    text(f"{short} ({src})", 14, 177.6 + 2.54 * i + 0.6)
place(part("J3"), 50, 240, stubs={"2": 10.16})
text("J3: HSTS016L current clamp", 30, 232, 1.8)

OPAMP_TEXT = dict(ref_at=(-2.54, -6.35), val_at=(-2.54, 6.35))
text("Analog inputs, buffered (1 MOhm + divider + 1 nF, MCP6024 follower on 3V3)", 110, 45, 1.8)
for i, (ch, *_rest) in enumerate(design.ANALOG):
    x0 = 110 + 72 * i
    unit = _rest[2]
    text(ch, x0, 52, 1.8)
    place(part(f"R{2 * i + 1}"), x0, 65)
    place(part(f"R{2 * i + 2}"), x0 + 12.7, 65)
    place(part(f"C{i + 1}"), x0 + 25.4, 65)
    u1 = part("U1")
    place(u1, x0 + 50.8, 59.69, unit=unit, nets=u1["units"][unit], **OPAMP_TEXT)
    place(jumper_for(f"R{2 * i + 1}"), x0 + 6.35, 85.09)

text("Digital inputs (100 kOhm + divider + 1 nF, BAT54WS clamp to 3V3)", 110, 105, 1.8)
for i, (ch, *_r) in enumerate(design.DIGITAL):
    x0 = 110 + 57.15 * i
    text(ch, x0, 112, 1.8)
    place(part(f"R{12 + 2 * i}"), x0, 125)
    place(part(f"R{13 + 2 * i}"), x0 + 12.7, 125)
    place(part(f"C{8 + i}"), x0 + 25.4, 125)
    place(part(f"D{1 + i}"), x0 + 38.1, 125, rot=270)
    place(jumper_for(f"R{12 + 2 * i}"), x0 + 6.35, 146.05)

text("Motor current + sensor Vref (HSTS016L, 0-5 V -> 0-3.21 V, 250 Hz)", 110, 165, 1.8)
place(part("R9"), 110, 180)
place(part("R10"), 122.7, 180)
place(part("C6"), 135.4, 180)
place(part("R22"), 152.4, 180)
place(part("R23"), 165.1, 180)
place(part("C13"), 177.8, 180)
place(part("C7"), 195.58, 180)
text("Ground reference to ECU B9 (one point only)", 215, 165, 1.8)
place(part("R11"), 225, 180)
place(jumper_for("R11"), 231.14, 196.85)
text("U1 supply", 300, 165, 1.8)
place(part("U1"), 305, 180, unit=5, nets=part("U1")["units"][5], ref_at=(2.54, -1.27), val_at=(2.54, 1.27))
place(part("C5"), 320, 180)
text("U2 / U3 supply", 345, 165, 1.8)
place(part("U2"), 350, 180, unit=5, nets=part("U2")["units"][5], ref_at=(2.54, -1.27), val_at=(2.54, 1.27))
place(part("C18"), 365, 180)
place(part("C19"), 380, 180)
text("Status LED (D13)", 395, 165, 1.8)
place(part("R32"), 400, 180)
place(part("D6"), 412.7, 180, rot=90)

text("Spare inputs X1-X4 (1 MOhm + 150 kOhm = 0-25 V, MCP6024 buffer -> ADS1115 16-bit, I2C 0x48)", 110, 235, 1.8)
for i, ch in enumerate(design.SPARE):
    x0 = 110 + 72 * i
    text(ch, x0, 242, 1.8)
    place(part(f"R{24 + 2 * i}"), x0, 255)
    place(part(f"R{25 + 2 * i}"), x0 + 12.7, 255)
    place(part(f"C{14 + i}"), x0 + 25.4, 255)
    u2 = part("U2")
    place(u2, x0 + 50.8, 249.69, unit=i + 1, nets=u2["units"][i + 1], **OPAMP_TEXT)
    place(jumper_for(f"R{24 + 2 * i}"), x0 + 6.35, 275.59)
text("U3: ADS1115", 410, 235, 1.8)
place(part("U3"), 430, 260)

# PWR_FLAGs: the Feather drives these nets.
for i, net in enumerate(["+3V3", "GND", "+5V"]):
    x, y = snap(250 + 22 * i), snap(215)
    counters["#FLG"] += 1
    symbol("power:PWR_FLAG", "#FLG%02d" % counters["#FLG"], "PWR_FLAG", x + 7.62, y,
           ref_at=(x + 7.62, y), val_at=(x + 7.62, y - 3.81))
    wire(x, y, x + 7.62, y)
    power(net, x, y, (0, 1) if net == "GND" else (0, -1))
text("Power comes from the Feather: 3V3 regulator, USB pin (power-bank 5 V), GND", 245, 205, 1.27)

# --- write -------------------------------------------------------------------
sch = ["kicad_sch", ["version", 20260306], ["generator", Str("eeschema")], ["generator_version", Str("10.0")],
       ["uuid", Str(ROOT_UUID)], ["paper", Str("A2")],
       ["title_block", ["title", Str("EPS tap FeatherWing")], ["date", Str("2026-10-26")],
        ["rev", Str(design.REV)], ["comment", 1, Str("Honda Insight EPS data logger front end")],
        ["comment", 2, Str("Generated by gen/make_sch.py from gen/design.py")]],
       ["lib_symbols"] + list(used_libs.values())] + items + [
       ["sheet_instances", ["path", Str("/"), ["page", Str("1")]]], ["embedded_fonts", "no"]]
with open(OUT, "w", encoding="utf-8", newline="\n") as f:
    f.write(dump(sch) + "\n")
print("wrote", os.path.normpath(OUT), len(items), "items")

# Project symbol library holding the Feather header symbol, plus its table.
here = os.path.dirname(OUT)
fs = feather_symbol()
fs[1] = Str("Feather_M4_CAN")
with open(os.path.join(here, design.PROJECT + ".kicad_sym"), "w", encoding="utf-8", newline="\n") as f:
    f.write(dump(["kicad_symbol_lib", ["version", 20251024], ["generator", Str("make_sch.py")], fs]) + "\n")
with open(os.path.join(here, "sym-lib-table"), "w", encoding="utf-8", newline="\n") as f:
    f.write('(sym_lib_table\n\t(version 7)\n\t(lib (name "eps_tap_wing")(type "KiCad")'
            '(uri "${KIPRJMOD}/eps_tap_wing.kicad_sym")(options "")(descr "EPS tap wing symbols"))\n)\n')
