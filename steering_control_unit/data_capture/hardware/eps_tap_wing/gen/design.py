"""EPS tap FeatherWing: the single source for parts, nets and placement.

make_sch.py builds the schematic from this file. make_pcb.py builds the board
from the netlist KiCad exports from that schematic, plus PLACE below.
Circuit values follow ../../BOM.md (per-channel circuits) with the probe-end
resistors moved onto the wing. Each on-board series resistor has a solder
jumper across it (open as shipped): bridge it to use probe-end resistors instead.
Mechanics follow the Adafruit Feather specification (0.9" x 2.0", 16-pin strip
centred 1.0" and 12-pin strip 1.2" from the USB edge, 0.8" apart, 0.1" corner holes).

Rev 1.3 adds four spare protected inputs (X1-X4) for future signals, buffered
by a second MCP6024 into an ADS1115 16-bit I2C ADC, on a 15-way locking JST XA.
"""

PROJECT = "eps_tap_wing"
REV = "1.5"

R0402 = "Resistor_SMD:R_0402_1005Metric"
C0402 = "Capacitor_SMD:C_0402_1005Metric"
SOD323 = "Diode_SMD:D_SOD-323"
TSSOP14 = "Package_SO:TSSOP-14_4.4x5mm_P0.65mm"

# Field connector J2: 15-way JST XA (2.5 mm, locking). The torque-sensor lines
# sit at one end, behind GND/SHLD, so a stray strand can't put +B onto them.
# +B sits next to IG1 (also battery level), the digital lines follow the
# Feather's pin order (D12, D11, D9, D6, D5) so their tracks don't cross, and
# the spare inputs X1-X4 sit at the far end.
J2_PINS = [
    ("TAP_VT3", "VT3", "B10"),
    ("TAP_VT6", "VT6", "B8"),
    ("TAP_VCC2", "VCC2", "B2"),
    ("TAP_GND", "GND", "B9"),
    ("GND", "SHLD", "shield"),
    ("TAP_VBAT", "+B", "A2"),
    ("TAP_IG1", "IG1", "B6"),
    ("TAP_RT", "RT+", "B11"),
    ("TAP_WLP", "WLP", "B13"),
    ("TAP_NEP", "NEP", "B14"),
    ("TAP_VSP", "VSP", "B4"),
    ("TAP_X1", "X1", "spare"),
    ("TAP_X2", "X2", "spare"),
    ("TAP_X3", "X3", "spare"),
    ("TAP_X4", "X4", "spare"),
]
J2_PITCH = 2.5
J3_PINS = [("+5V", "+5V"), ("GND", "GND"), ("IMOT_RAW", "OUT"), ("IMOT_VREF", "REF")]

# Feather M4 CAN header pins used (KiCad Module:Adafruit_Feather numbering).
FEATHER_PINS = {
    1: "RST", 2: "3V3", 3: "AREF", 4: "GND", 5: "A0", 6: "A1", 7: "A2", 8: "A3",
    9: "A4", 10: "A5", 11: "SCK", 12: "MOSI", 13: "MISO", 14: "RX", 15: "TX",
    16: "D4", 17: "SDA", 18: "SCL", 19: "D5", 20: "D6", 21: "D9", 22: "D10",
    23: "D11", 24: "D12", 25: "D13", 26: "USB", 27: "EN", 28: "VBAT",
}
FEATHER_NETS = {
    2: "+3V3", 4: "GND", 5: "A0_VCC2", 6: "A1_VBAT", 7: "A2_VT3", 8: "A3_VT6",
    9: "A4_IMOT", 10: "A5_VREF", 17: "SDA", 18: "SCL", 19: "D5_VSP", 20: "D6_NEP",
    21: "D9_WLP", 23: "D11_RT", 24: "D12_IG1", 25: "D13_LED", 26: "+5V",
}

# Part numbers. Values are chosen from JLCPCB's Basic library where possible:
# Basic parts load free, each Extended part type costs a setup fee.
# (value: (MPN, JLCPCB/LCSC part #))
PARTS_LIB = {
    "1M": ("0402WGF1004TCE", "C26083"),
    "100k": ("0402WGF1003TCE", "C25741"),
    "47k": ("0402WGF4702TCE", "C25792"),
    "33k": ("0402WGF3302TCE", "C25779"),
    "10k": ("0402WGF1002TCE", "C25744"),
    "100R": ("0402WGF1000TCE", "C25076"),
    "1nF": ("0402B102K500NT", "C1523"),          # X7R 50 V
    "100nF": ("CL05B104KO5NNNC", "C1525"),       # X7R 16 V
    "1N4148WS": ("1N4148WS", "C2128"),
    "MCP6004": ("MCP6004-I/ST", ""),             # Extended: no Basic rail-to-rail op-amp
    "ADS1115": ("ADS1115IDGSR", "C37593"),       # Extended
    "LED": ("NCD0805R1", "C84256"),              # red 0805
    "1k 0805": ("0805W8F1001T5E", "C17513"),
}
MPN = {k: v[0] for k, v in PARTS_LIB.items()}
LCSC = {k: v[1] for k, v in PARTS_LIB.items()}


def R(ref, value, a, b, note=""):
    return dict(ref=ref, lib="Device:R", value=value, fp=R0402,
                pins={"1": a, "2": b}, mpn=MPN[value], lcsc=LCSC[value], note=note)


def C(ref, value, a, b, note=""):
    return dict(ref=ref, lib="Device:C", value=value, fp=C0402,
                pins={"1": a, "2": b}, mpn=MPN[value], lcsc=LCSC[value], note=note)


def D(ref, anode, cathode):
    # Device:D: pin 1 = K, pin 2 = A (matches D_SOD-323 pad 1 = K). 1N4148WS: low
    # leakage (25 nA), so it can't pull a 33k divider; clamps at about 3.7 V.
    return dict(ref=ref, lib="Device:D", value="1N4148WS", fp=SOD323,
                pins={"1": cathode, "2": anode}, mpn=MPN["1N4148WS"], lcsc=LCSC["1N4148WS"], note="")


def opamp(ref, units, note):
    return dict(ref=ref, lib="Amplifier_Operational:LM2902", value="MCP6004", fp=TSSOP14,
                pins={p: n for u in units.values() for p, n in u.items()},
                units=units, mpn=MPN["MCP6004"], lcsc=LCSC["MCP6004"], note=note)


PARTS = []
# Op-amp section pin sets (+IN, -IN, OUT); E is the power unit.
SECTION_PINS = {1: ("3", "2", "1"), 2: ("5", "6", "7"), 3: ("10", "9", "8"), 4: ("12", "13", "14")}

# Analog, buffered: tap --1M-- node (shunt R + 1 nF) --> op-amp follower --> ADC.
# (channel, tap net, shunt value, U1 section, ADC net). Sections match
# firmware/eps_logger/config.h: U1A VT3, U1B VT6, U1C Vcc2, U1D +B.
ANALOG = [
    ("VCC2", "TAP_VCC2", "1M", 3, "A0_VCC2"),      # 5 V -> 2.50 V
    ("VT3", "TAP_VT3", "1M", 1, "A2_VT3"),
    ("VT6", "TAP_VT6", "1M", 2, "A3_VT6"),
    ("VBAT", "TAP_VBAT", "100k", 4, "A1_VBAT"),    # 36 V full scale
]
U1_UNITS = {}
for i, (ch, tap, rsh, unit, adc) in enumerate(ANALOG):
    node = "IN_" + ch
    PARTS.append(R(f"R{2 * i + 1}", "1M", tap, node, ch + " input, 1 MOhm limits fault current to 16 uA"))
    PARTS.append(R(f"R{2 * i + 2}", rsh, node, "GND", ch + " divider"))
    PARTS.append(C(f"C{i + 1}", "1nF", node, "GND", ch + " filter"))
    p, n, o = SECTION_PINS[unit]
    U1_UNITS[unit] = {p: node, n: adc, o: adc}
U1_UNITS[5] = {"4": "+3V3", "11": "GND"}
PARTS.append(opamp("U1", U1_UNITS, "Rail-to-rail quad, 3V3 supply"))
PARTS.append(C("C5", "100nF", "+3V3", "GND", "U1 decoupling"))

# Motor current: HSTS016L output (1.56-3.44 V over +/-75 A) --10k--+--47k--GND, 100 nF.
PARTS.append(R("R9", "10k", "IMOT_RAW", "A4_IMOT", "IMOT divider"))
PARTS.append(R("R10", "47k", "A4_IMOT", "GND", "IMOT divider, 3.44 V -> 2.84 V"))
PARTS.append(C("C6", "100nF", "A4_IMOT", "GND", "IMOT 250 Hz filter"))
PARTS.append(C("C7", "100nF", "+5V", "GND", "Current sensor supply decoupling"))
# Current sensor Vref (white wire), same divider + filter so Vout - Vref cancels drift.
PARTS.append(R("R22", "10k", "IMOT_VREF", "A5_VREF", "Vref divider"))
PARTS.append(R("R23", "47k", "A5_VREF", "GND", "Vref divider"))
PARTS.append(C("C13", "100nF", "A5_VREF", "GND", "Vref filter, matches C6"))

# Single ground reference to ECU B9 (T/S GND).
PARTS.append(R("R11", "100R", "TAP_GND", "GND", "Ground reference, limits loop current"))

# Digital: tap --100k-- pin, shunt R, 1 nF, 1N4148WS clamp to 3V3.
DIGITAL = [
    ("VSP", "TAP_VSP", "D5_VSP", "100k"),   # 5 V -> 2.50 V
    ("NEP", "TAP_NEP", "D6_NEP", "33k"),    # 14.4 V -> 3.57 V; still high at 9 V cranking
    ("WLP", "TAP_WLP", "D9_WLP", "33k"),
    ("RT", "TAP_RT", "D11_RT", "33k"),
    ("IG1", "TAP_IG1", "D12_IG1", "33k"),
]
for i, (ch, tap, pin, rsh) in enumerate(DIGITAL):
    PARTS.append(R(f"R{12 + 2 * i}", "100k", tap, pin, ch + " input"))
    PARTS.append(R(f"R{13 + 2 * i}", rsh, pin, "GND", ch + " divider"))
    PARTS.append(C(f"C{8 + i}", "1nF", pin, "GND", ch + " filter"))
    PARTS.append(D(f"D{1 + i}", pin, "+3V3"))

# Spare inputs X1-X4: same protection as +B (1 MOhm / 100 kOhm = 0-36 V full
# scale), buffered by U2 into the ADS1115 (U3, I2C 0x48). Change the 100k to
# 1M for a 0-5 V signal.
SPARE = ["X1", "X2", "X3", "X4"]
U2_UNITS = {}
for i, ch in enumerate(SPARE):
    node, ain = "IN_" + ch, "AIN_" + ch
    PARTS.append(R(f"R{24 + 2 * i}", "1M", "TAP_" + ch, node, ch + " input (spare)"))
    PARTS.append(R(f"R{25 + 2 * i}", "100k", node, "GND", ch + " divider, 0-36 V"))
    PARTS.append(C(f"C{14 + i}", "1nF", node, "GND", ch + " filter"))
    p, n, o = SECTION_PINS[i + 1]
    U2_UNITS[i + 1] = {p: node, n: ain, o: ain}
U2_UNITS[5] = {"4": "+3V3", "11": "GND"}
PARTS.append(opamp("U2", U2_UNITS, "Buffers spare inputs X1-X4"))
PARTS.append(C("C18", "100nF", "+3V3", "GND", "U2 decoupling"))
PARTS.append(dict(ref="U3", lib="Analog_ADC:ADS1015IDGS", value="ADS1115",
                  fp="Package_SO:TSSOP-10_3x3mm_P0.5mm",
                  pins={"1": "GND", "3": "GND", "4": "AIN_X1", "5": "AIN_X2", "6": "AIN_X3",
                        "7": "AIN_X4", "8": "+3V3", "9": "SDA", "10": "SCL"},
                  mpn=MPN["ADS1115"], lcsc=LCSC["ADS1115"], note="16-bit I2C ADC, address 0x48 (ADDR to GND); ALERT/RDY unused"))
PARTS.append(C("C19", "100nF", "+3V3", "GND", "U3 decoupling"))

# Status LED on D13: mirrors the firmware's status codes (CFG_PIN_LED = LED_BUILTIN = D13).
# 0805 parts on the back (the wiring face, where you look). Hand-solder them, or order
# bottom-side assembly for these two.
PARTS.append(dict(ref="R32", lib="Device:R", value="1k", pins={"1": "D13_LED", "2": "LED_A"},
                  fp="Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
                  mpn=MPN["1k 0805"], lcsc=LCSC["1k 0805"], note="Status LED, ~1.4 mA"))
PARTS.append(dict(ref="D6", lib="Device:LED", value="RED", fp="LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder",
                  pins={"1": "GND", "2": "LED_A"}, mpn=MPN["LED"], lcsc=LCSC["LED"],
                  note="Red 0805, back side"))

# Bypass jumpers, one per series resistor, open as shipped.
# (J2 pin, series resistor) -> JPn shorts that resistor.
SERIES_R = [(1, "R3"), (2, "R5"), (3, "R1"), (4, "R11"), (6, "R7"), (7, "R20"),
            (8, "R18"), (9, "R16"), (10, "R14"), (11, "R12"),
            (12, "R24"), (13, "R26"), (14, "R28"), (15, "R30")]
for n, (pin, rref) in enumerate(SERIES_R, start=1):
    a, b = next(p for p in PARTS if p["ref"] == rref)["pins"].values()
    PARTS.append(dict(ref=f"JP{n}", lib="Jumper:SolderJumper_2_Open", value="BYPASS",
                      fp="eps_tap_wing:SolderJumper_Bypass",
                      pins={"1": a, "2": b}, mpn="", in_bom=False,
                      note=f"Bridge to bypass {rref} when using a probe-end resistor"))

PARTS.append(dict(ref="J1", lib="eps_tap_wing:Feather_M4_CAN", value="Feather M4 CAN",
                  fp="eps_tap_wing:Feather_Wing_Headers",
                  pins={str(p): n for p, n in FEATHER_NETS.items()},
                  mpn="", note="Female headers on top, hand soldered"))
PARTS.append(dict(ref="J2", lib="Connector_Generic:Conn_01x15", value="EPS taps",
                  fp="Connector_JST:JST_XA_B15B-XASK-1_1x15_P2.50mm_Vertical",
                  pins={str(i + 1): n for i, (n, _, _) in enumerate(J2_PINS)},
                  mpn="JST B15B-XASK-1 + XAP-15V-1 housing, SXA-001T-P0.6 contacts",
                  note="Locking 2.5 mm, hand soldered"))
PARTS.append(dict(ref="J3", lib="Connector:Screw_Terminal_01x04", value="Current clamp",
                  # 2.54 mm fixed screw terminal: a 3.5 mm plug-in 4-way would sit over
                  # the Feather-spec corner mounting holes.
                  fp="TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-4-2.54_1x04_P2.54mm_Horizontal",
                  pins={str(i + 1): n for i, (n, _) in enumerate(J3_PINS)},
                  mpn="Phoenix MPT 0,5/4-2,54 (1725672)",
                  note="Hand soldered"))

# ---------------------------------------------------------------------------
# Board placement, mm in board coordinates (origin = top-left corner, top view).
# side "F" = top: all SMD (one-sided assembly), faces the Feather stacked above.
# side "B" = bottom: terminals + bypass jumpers, the wiring side ("bottom Wing" per the Feather spec).
BOARD_W, BOARD_H = 50.8, 22.86
J2_X0, J2_Y = 4.0, 9.6
PLACE = {
    "J1": (6.35, 21.59, 90, "F"),     # pin 1 (RST); 16-pin row along the bottom edge
    "J2": (J2_X0, J2_Y, 0, "B"),
    "J3": (46.6, 15.24, 270, "B"),    # pin 1 nearest the 16-pin header; pins 2.54 mm apart upwards
    "U1": (17.5, 15.75, 90, "F"),
    "U2": (31.5, 15.75, 90, "F"),
    "U3": (37.0, 5.4, 0, "F"),        # ADS1115, right under SDA/SCL (header pins 17/18)
    "C19": (41.0, 5.4, 90, "F"),
    "R32": (26.0, 4.2, 0, "B"),       # status LED + resistor on the back (wiring face)
    "D6": (30.0, 4.2, 0, "B"),
}
_COL = 1.2                             # 0402 column pitch
_ROW_A = 5.4
_ROWS_B = [12.0, 14.2, 16.4, 18.6]
# Band A (between the 12-pin header and J2): digital clusters, ordered to
# follow the Feather pins D12, D11, D9, D6, D5.
_x = 4.8
for ch in ["IG1", "RT", "WLP", "NEP", "VSP"]:
    i = [d[0] for d in DIGITAL].index(ch)
    for k, ref in enumerate([f"R{12 + 2 * i}", f"R{13 + 2 * i}", f"C{8 + i}"]):
        PLACE[ref] = (_x + _COL * k, _ROW_A, 90, "F")
    PLACE[f"D{1 + i}"] = (_x + 4.0, _ROW_A, 270, "F")   # cathode (3V3) towards the header
    _x += 5.6


def _grid(x0, rows):
    """Place rows of 0402s left to right starting at x0, one list per band-B row."""
    for row_y, refs in zip(_ROWS_B, rows):
        for k, ref in enumerate(refs):
            if ref:
                PLACE[ref] = (x0 + _COL * k, row_y, 90, "F")


# Band B, left of U1: VT3, VT6, VCC2 and the ground resistor.
_grid(5.0, [["R3", "R4", "C2", "R5", "R6", "C3"], ["R1", "R2", "C1", "R11"]])
# Between U1 and U2: U1 decoupling and the +B channel; U2 decoupling.
_grid(21.4, [["C5", "R7", "R8", "C4"], [], [], [None, None, None, None, None, "C18"]])
# Right of U2: spare inputs X1-X4, motor current, Vref.
_grid(35.2, [["R24", "R25", "C14", "R26", "R27", "C15", "R9", "R10"],
             ["R28", "R29", "C16", "R30", "R31", "C17", "C6", "C7"],
             [None, None, None, None, None, "R22", "R23", "C13"]])
# Back side: bypass jumpers in a row under the J2 terminal each one belongs to
# (bare pads, nothing to assemble).
for n, (pin, _r) in enumerate(SERIES_R, start=1):
    PLACE[f"JP{n}"] = (J2_X0 + J2_PITCH * (pin - 1), 15.1, 90, "B")

# Hand-soldered (headers, connectors, back-side LED): not in the assembly BOM.
for _p in PARTS:
    if _p["ref"] in ("J1", "J2", "J3", "D6", "R32"):
        _p["in_bom"] = False

assert set(PLACE) == {p["ref"] for p in PARTS}, set(p["ref"] for p in PARTS) ^ set(PLACE)
