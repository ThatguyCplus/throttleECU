"""Bench stimulus FeatherWing: the single source for parts, nets and placement.

make_sch.py builds the schematic from this file. make_pcb.py builds the board
from the netlist KiCad exports from that schematic, plus PLACE below.

What the board does (see ../README.md and ../HANDOFF.md):
  * takes 12 V (bench supply or car battery) through a fuse, reverse-polarity
    diode and TVS, and makes 5 V with a 40 V (65 V transient) buck;
  * feeds that 5 V to the Feather's USB pin through a Schottky (Adafruit's
    usual way to power a Feather from a Wing), so the logger stack needs no
    power bank;
  * switches 12 V to the Toyota steering-angle sensor (SAS) and passes its CAN
    on to the Feather M4 CAN's terminal block;
  * drives the EPS bench signals NEP (rpm) and VSP (speed) from Feather timer
    pins, and VT3/VT6 (torque) from a 12-bit I2C DAC through 5 V op-amps;
  * switched IG1 output for an EPS that's out of the car.

Safety model (the user asked for it overbuilt):
  * Every EPS line goes through a relay. De-energised, the relay's NC contact
    connects the EPS to the car's own sensor / ECM wire, so power loss, a reset
    or a crash puts the car back exactly as it was.
  * Relay coils (and the IG1 output) only get power through the external ARM
    loop on J5: wire a normally-closed e-stop or a key switch there. Open loop
    = hard kill, whatever the firmware does.
  * Each output also needs its own enable bit from an I2C expander that powers
    up with every pin as an input; 100 k pull-downs hold every driver off.

Mechanics follow the Adafruit Feather specification (0.9" x 2.0", 16-pin strip
centred 1.0" and 12-pin strip 1.2" from the USB edge, 0.8" apart, 0.1" corner
holes). The board stacks between the Adalogger and the OLED FeatherWing on
stacking headers; every cable connector is right-angle and exits a board edge.
"""

PROJECT = "bench_stim_wing"
REV = "1.0"

R0402 = "Resistor_SMD:R_0402_1005Metric"
R0603 = "Resistor_SMD:R_0603_1608Metric"
C0402 = "Capacitor_SMD:C_0402_1005Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"
C1206 = "Capacitor_SMD:C_1206_3216Metric"
LED0603 = "LED_SMD:LED_0603_1608Metric"
SOT23 = "Package_TO_SOT_SMD:SOT-23"
SOD323 = "Diode_SMD:D_SOD-323"
SOD123F = "Diode_SMD:D_SOD-123F"
TSSOP14 = "Package_SO:TSSOP-14_4.4x5mm_P0.65mm"
TSSOP16 = "Package_SO:TSSOP-16_4.4x5mm_P0.65mm"
MSOP10 = "Package_SO:MSOP-10_3x3mm_P0.5mm"
FUSE1206 = "Fuse:Fuse_1206_3216Metric"
GH = "Connector_JST:JST_GH_SM{n:02d}B-GHS-TB_1x{n:02d}-1MP_P1.25mm_Horizontal"

# Part numbers: (MPN, JLCPCB/LCSC part #). Every LCSC number was checked against
# its LCSC/JLCPCB listing (Nov 2026). Stock moves: re-check at upload.
PARTS_LIB = {
    "100R": ("0402WGF1000TCE", "C25076"),
    "470R": ("0402WGF4700TCE", "C25117"),
    "1k": ("0402WGF1001TCE", "C11702"),
    "4.7k 0603": ("0603WAF4701T5E", "C23162"),
    "10k": ("0402WGF1002TCE", "C25744"),
    "12k": ("0402WGF1202TCE", "C25752"),
    "22k": ("0402WGF2202TCE", "C25768"),
    "33k": ("0402WGF3302TCE", "C25779"),
    "56k": ("0402WGF5602TCE", "C25796"),
    "100k": ("0402WGF1003TCE", "C25741"),
    "100nF": ("CL05B104KO5NNNC", "C1525"),          # 0402 X7R 16 V: logic rails only
    "100nF 50V": ("CC0603KRX7R9BB104", "C14663"),   # 0603 X7R 50 V: anything on 12 V
    "10uF 25V": ("CL21A106KAYNNNE", "C15850"),      # 0805 X5R 25 V
    "4.7uF 50V": ("CL31B475KBHNNNE", "C51205"),           # 1206 X7R 50 V, buck input
    "10uF 50V": ("CL31A106KBHNNNE", "C13585"),            # 1206 X5R 50 V, 12 V bulk
    "1N4148WS": ("1N4148WS", "C2128"),
    "SS34": ("SS34", "C8678"),                       # 3 A 40 V Schottky, SMA (MDD; alt. FOSAN C5119706)
    "PMEG6010CEH": ("PMEG6010CEH,115", "C110797"),  # 1 A 60 V Schottky, SOD-123F (alt. -QX C5311567)
    "SMBJ24CA": ("SMBJ24CA (Bourns)", "C1973089"),  # 600 W TVS, 24 V stand-off
    "BAT54S": ("BAT54S", "C171786"),
    "2N7002": ("2N7002", "C8545"),
    "Si2319CDS": ("SI2319CDS-T1-GE3", "C146287"),   # P-FET -40 V, Vgs +/-20 V (Vishay)
    "LMR16006YQ": ("LMR16006YQDDCRQ1", "C544521"),  # 40 V (65 V transient) 0.6 A buck, 0.7 MHz
    "22uH": ("SWPA4030S220MT", "C83472"),           # 4x4x3 mm shielded, Isat 1.3 A
    "MCP4728": ("MCP4728-E/UN", "C108207"),
    "MCP6004": ("MCP6004-I/ST", "C185811"),         # low stock: alt. UMW MCP6004T-I/ST C32713110
    "TCA9534": ("TCA9534PWR", "C783615"),
    "G6K-2F-Y 5V": ("G6K-2F-Y DC5", "C231389"),     # Omron, in stock (C1560486 is the same MPN, unavailable)
    "FUSE 2A": ("0466002.NRHF", "C3105"),           # Littelfuse 466, 1206 very fast 2 A 63 V (halogen-free)
    "PTC 0.1A": ("1206L010/60WR", "C2153714"),      # Littelfuse 1206 PTC, 0.1 A hold, 60 V
    "PTC 0.5A": ("1206L050/30NR (LUTE)", "C7542947"),  # 1206 PTC, 0.5 A hold, 30 V
    "LED RED": ("KT-0603R", "C2286"),
    "LED GRN": ("KT-0603G", "C72043"),
    "LED YEL": ("KT-0603Y", "C72038"),
    "LED BLU": ("KT-0603B", "C72041"),
    # JST GH right-angle SMD headers (extended parts): assembled by JLC
    "GH 5": ("SM05B-GHS-TB(LF)(SN)", "C189896"),
    "GH 6": ("SM06B-GHS-TB(LF)(SN)", "C133065"),
    "GH 8": ("SM08B-GHS-TB(LF)(SN)", "C265111"),
}
MPN = {k: v[0] for k, v in PARTS_LIB.items()}
LCSC = {k: v[1] for k, v in PARTS_LIB.items()}

PARTS = []


def add(ref, lib, value, fp, pins, key=None, note="", **kw):
    key = key or value
    d = dict(ref=ref, lib=lib, value=value, fp=fp, pins=pins,
             mpn=MPN.get(key, ""), lcsc=LCSC.get(key, ""), note=note)
    d.update(kw)
    PARTS.append(d)
    return d


def R(ref, value, a, b, note="", fp=R0402, key=None):
    return add(ref, "Device:R", value.split()[0], fp, {"1": a, "2": b}, key=key or value, note=note)


def C(ref, value, a, b, note="", fp=C0402):
    return add(ref, "Device:C", value.split()[0], fp, {"1": a, "2": b}, key=value, note=note)


def LED(ref, colour, anode, cathode, note):
    # Device:LED pin 1 = K, pin 2 = A; LED_0603 pad 1 = K.
    return add(ref, "Device:LED", colour, LED0603, {"1": cathode, "2": anode}, key="LED " + colour[:3].upper(), note=note)


def NFET(ref, g, s, d, note):
    # BSS138 symbol = 2N7002 pinout (SOT-23: 1 G, 2 S, 3 D).
    return add(ref, "Transistor_FET:BSS138", "2N7002", SOT23, {"1": g, "2": s, "3": d}, note=note)


def PFET(ref, g, s, d, note):
    # TP0610T symbol = Si2319CDS pinout (SOT-23: 1 G, 2 S, 3 D).
    return add(ref, "Transistor_FET:TP0610T", "Si2319CDS", SOT23, {"1": g, "2": s, "3": d}, note=note)


def DIODE(ref, value, anode, cathode, fp, note="", schottky=False):
    return add(ref, "Device:D_Schottky" if schottky else "Device:D", value, fp,
               {"1": cathode, "2": anode}, note=note)


def LEDIND(n_led, n_r, colour, src, label, r="1k", sink="GND"):
    """Indicator: src --R--> LED --> sink."""
    a = "LED_" + label
    R(n_r, r, src, a, label + " LED resistor")
    LED(n_led, colour, a, sink, label + " indicator")


# --- Feather header ----------------------------------------------------------
# KiCad Module:Adafruit_Feather pin numbering, as on the EPS tap wing.
FEATHER_PINS = {
    1: "RST", 2: "3V3", 3: "AREF", 4: "GND", 5: "A0", 6: "A1", 7: "A2", 8: "A3",
    9: "A4", 10: "A5", 11: "SCK", 12: "MOSI", 13: "MISO", 14: "RX", 15: "TX",
    16: "D4", 17: "SDA", 18: "SCL", 19: "D5", 20: "D6", 21: "D9", 22: "D10",
    23: "D11", 24: "D12", 25: "D13", 26: "USB", 27: "EN", 28: "VBAT",
}
# Only these pins connect. Everything else the logger and tap wing already use
# (A0-A5, D5, D6, D9-D13, SPI, RX) passes straight through the stacking headers.
# D4 = SAMD51 PA14 (TCC1/WO2, TCC2/WO0; not TC3, the logger's sample tick); TX = PB16 (TC6/WO0, TCC0/WO4): two
# independent timers. Checked against Adafruit's Feather M4 CAN schematic: D4
# only goes to the header (CAN standby/boost are internal pins PB12/PB13).
FEATHER_NETS = {
    2: "+3V3", 4: "GND", 15: "TX_VSP", 16: "D4_NEP", 17: "SDA", 18: "SCL", 26: "VUSB",
}
add("J1", "bench_stim_wing:Feather_M4_CAN", "Feather M4 CAN", "bench_stim_wing:Feather_Wing_Headers",
    {str(p): n for p, n in FEATHER_NETS.items()}, note="Stacking headers, hand soldered", in_bom=False)

# --- 12 V input ----------------------------------------------------------------
# J2 --F1 2 A--> D1 SS34 (reverse polarity) --> +12V, D2 SMBJ24CA clamps load dump
# (~39 V worst case). Everything on +12V is rated >= 40 V.
add("J2", "Connector_Generic:Conn_01x02", "12V IN",
    "Connector_JST:JST_PH_S2B-PH-K_1x02_P2.00mm_Horizontal",
    {"1": "V12_RAW", "2": "GND"}, note="JST PH 2-way, right angle. Pin 1 = +12 V", in_bom=False)
add("F1", "Device:Fuse", "2A", FUSE1206, {"1": "V12_RAW", "2": "V12_F"}, key="FUSE 2A",
    note="Input fuse, fast, 63 V")
DIODE("D1", "SS34", "V12_F", "+12V", "Diode_SMD:D_SMA", "Reverse-polarity protection", schottky=True)
add("D2", "Device:D_TVS", "SMBJ24CA", "Diode_SMD:D_SMB", {"1": "+12V", "2": "GND"},
    note="Load-dump clamp, 24 V stand-off, ~39 V clamp")
C("C1", "10uF 50V", "+12V", "GND", "12 V bulk", fp=C1206)
C("C2", "100nF 50V", "+12V", "GND", "12 V HF decoupling", fp=C0603)
LEDIND("D3", "R1", "GRN", "+12V", "12V", r="10k")      # ~1 mA at 12 V

# --- 5 V buck ----------------------------------------------------------------
# LMR16006: VFB = 0.765 V, so 56k/10k sets 5.05 V. Asynchronous: D4 is the
# catch diode. 0.7 MHz (Y version) with 22 uH gives ~0.2-0.3 A ripple.
add("U1", "Regulator_Switching:LMR16006YQ", "LMR16006YQ", "Package_TO_SOT_SMD:SOT-23-6",
    {"1": "BUCK_CB", "2": "GND", "3": "BUCK_FB", "4": "BUCK_EN", "5": "+12V", "6": "BUCK_SW"},
    note="40 V (65 V transient) 0.6 A buck, 5 V out")
C("C3", "4.7uF 50V", "+12V", "GND", "Buck input, at VIN", fp=C1206)
C("C4", "100nF 50V", "+12V", "GND", "Buck input HF, at VIN/GND pins", fp=C0603)
R("R2", "100k", "+12V", "BUCK_EN", "Buck always on when 12 V is present")
C("C5", "100nF", "BUCK_CB", "BUCK_SW", "Bootstrap")
DIODE("D4", "PMEG6010CEH", "GND", "BUCK_SW", SOD123F, "Buck catch diode", schottky=True)
add("L1", "Device:L", "22uH", "Inductor_SMD:L_Cenker_CKCS4030", {"1": "BUCK_SW", "2": "+5V"},
    note="Shielded, Isat >= 0.8 A")
R("R3", "56k", "+5V", "BUCK_FB", "FB divider: 0.765 V x (1 + 56/10) = 5.05 V")
R("R4", "10k", "BUCK_FB", "GND", "FB divider")
C("C6", "10uF 25V", "+5V", "GND", "Buck output", fp=C0805)
C("C7", "10uF 25V", "+5V", "GND", "Buck output", fp=C0805)
C("C8", "10uF 25V", "+5V", "GND", "Buck output", fp=C0805)
LEDIND("D5", "R5", "GRN", "+5V", "5V")
# Feed the Feather's USB pin (VBUS) through a Schottky: ~4.7 V there, below a
# PC's 5 V, so with a USB cable also plugged in the PC carries the load and this
# board never back-feeds the PC; the diode stops the PC feeding this board.
DIODE("D6", "PMEG6010CEH", "+5V", "VUSB", SOD123F, "5 V to the Feather USB pin, OR-ing diode", schottky=True)
C("C9", "100nF", "VUSB", "GND", "USB pin decoupling")

# --- SAS power (12 V, ~30 mA) -----------------------------------------------
# Q1 high side; Q2 pulls its gate. 10k/12k divider keeps Vgs <= 18 V at the TVS
# clamp and >= 4 V at 9 V cranking. Off at reset (R8 pull-down, expander high-Z).
PFET("Q1", "SAS_G", "+12V", "SAS_SW", "SAS high-side switch")
R("R6", "10k", "+12V", "SAS_G", "Q1 gate-source")
R("R7", "12k", "SAS_G", "SAS_PD", "Q1 gate drive")
NFET("Q2", "SAS_EN", "GND", "SAS_PD", "Q1 gate pull")
R("R8", "100k", "SAS_EN", "GND", "Default off")
# JP1 (open as shipped): bridge for "SAS always on" with the plain logger firmware.
R("R9", "10k", "+3V3", "SAS_FORCE", "SAS-on jumper feed (limits current if the expander drives low)")
add("JP1", "Jumper:SolderJumper_2_Open", "SAS ON", "bench_stim_wing:SolderJumper_Plain",
    {"1": "SAS_FORCE", "2": "SAS_EN"}, note="Bridge: SAS powered whenever 12 V is present", in_bom=False)
add("F2", "Device:Polyfuse", "0.1A", FUSE1206, {"1": "SAS_SW", "2": "SAS_12V"}, key="PTC 0.1A",
    note="SAS supply fuse")
LEDIND("D7", "R10", "GRN", "SAS_12V", "SAS", r="10k")

# --- ARM loop (hard kill) ----------------------------------------------------
# +5V --F3--> J5 pin 7 ... external NC e-stop / key switch ... J5 pin 8 = ARM.
# ARM powers both relay coils and enables the IG1 output. Open loop: nothing
# can switch away from the car, whatever the firmware does.
add("F3", "Device:Polyfuse", "0.1A", FUSE1206, {"1": "+5V", "2": "ARM_SRC"}, key="PTC 0.1A",
    note="Protects 5 V if the ARM loop wiring shorts")
LEDIND("D9", "R17", "RED", "ARM", "ARMED")              # also pulls ARM low when the loop is open
R("R18", "22k", "ARM", "ARM_SENSE", "ARM sense divider, 5 V -> 3.0 V")
R("R19", "33k", "ARM_SENSE", "GND", "ARM sense divider")

# --- IG1 output (bench EPS only), needs IG1_EN AND ARM ---------------------
PFET("Q3", "IG1_G", "+12V", "IG1_SW", "IG1 high-side switch")
R("R11", "10k", "+12V", "IG1_G", "Q3 gate-source")
R("R12", "12k", "IG1_G", "IG1_PD", "Q3 gate drive")
NFET("Q4", "IG1_EN", "IG1_AND", "IG1_PD", "IG1 enable (expander)")
NFET("Q5", "ARM", "GND", "IG1_AND", "IG1 enable (ARM loop): series with Q4 = AND")
R("R13", "100k", "IG1_EN", "GND", "Default off")
add("F4", "Device:Polyfuse", "0.5A", FUSE1206, {"1": "IG1_SW", "2": "IG1_OUT"}, key="PTC 0.5A",
    note="IG1 output fuse")
LEDIND("D8", "R16", "YEL", "IG1_OUT", "IG1", r="10k")

# --- Relays: de-energised = car wiring connected -----------------------------
# G6K-2 (KiCad symbol): coil 1/8; pole A COM 3, NC 2, NO 4; pole B COM 6, NC 7, NO 5.
RELAYS = [
    # ref, driver, enable net, label, (COM, NC, NO) x2
    ("K1", "Q6", "K1_EN", "TQ", [("VT3_EPS", "VT3_CAR", "VT3_STIM"), ("VT6_EPS", "VT6_CAR", "VT6_STIM")]),
    ("K2", "Q7", "K2_EN", "SIG", [("NEP_EPS", "NEP_CAR", "NEP_STIM"), ("VSP_EPS", "VSP_CAR", "VSP_STIM")]),
]
for i, (ref, q, en, label, poles) in enumerate(RELAYS):
    low = ref + "_L"
    (ca, nca, noa), (cb, ncb, nob) = poles
    add(ref, "Relay:G6K-2", "G6K-2F-Y 5V", "Relay_SMD:Relay_DPDT_Omron_G6K-2F-Y",
        {"1": "ARM", "8": low, "3": ca, "2": nca, "4": noa, "6": cb, "7": ncb, "5": nob},
        note=f"{label} relay: NC = car, NO = stimulus. Verify NC/NO pins against the Omron datasheet")
    NFET(q, en, "GND", low, ref + " coil driver")
    R(f"R{20 + 2 * i}", "100k", en, "GND", ref + " default off")
    DIODE(f"D{10 + 2 * i}", "1N4148WS", low, "ARM", SOD323, ref + " flyback")
    R(f"R{21 + 2 * i}", "1k", "ARM", f"LED_{label}", ref + " LED resistor")
    LED(f"D{11 + 2 * i}", "YEL", f"LED_{label}", low, ref + " energised")

# --- NEP / VSP outputs -------------------------------------------------------
# Open drain like the real sources: NEP pulled to 12 V (ECM), VSP to 5 V (VSS).
# GPIO high = output low. Gate pull-downs: high (idle) at reset.
PULSE = [
    # name, gpio net, pull-up rail, first R, Q, LED
    ("NEP", "D4_NEP", "+12V", 24, "Q8", "D14"),
    ("VSP", "TX_VSP", "+5V", 29, "Q9", "D15"),
]
for name, gpio, rail, n, q, led in PULSE:
    g, drv = name + "_G", name + "_DRV"
    R(f"R{n}", "100R", gpio, g, name + " gate resistor")
    R(f"R{n + 1}", "100k", g, "GND", name + " default off")
    NFET(q, g, "GND", drv, name + " open-drain output")
    R(f"R{n + 2}", "4.7k 0603", rail, drv, f"{name} pull-up to {rail}", fp=R0603)
    R(f"R{n + 3}", "100R", drv, name + "_STIM", name + " output series resistor")
    LEDIND(led, f"R{n + 4}", "BLU", gpio, name)

# --- Torque DAC: MCP4728 (3V3, internal 2.048 V ref, gain 1) -> x2.2 -> 0-4.5 V
add("U2", "Analog_DAC:MCP4728", "MCP4728", MSOP10,
    {"1": "+3V3", "2": "SCL", "3": "SDA", "4": "DAC_LDAC", "6": "DAC_A", "7": "DAC_B",
     "8": "DAC_C", "9": "DAC_D", "10": "GND"},
    note="12-bit quad DAC, I2C 0x60. Use the internal 2.048 V reference, gain x1")
R("R34", "10k", "DAC_LDAC", "GND", "LDAC low: outputs update on write")
C("C10", "100nF", "+3V3", "GND", "U2 decoupling")
# MCP6004 on 5 V, non-inverting gain 1 + 12k/10k = 2.2: 2.048 V -> 4.5 V max,
# so the torque lines can't be driven past the sensor's normal range.
SECTION_PINS = {1: ("3", "2", "1"), 2: ("5", "6", "7"), 3: ("10", "9", "8"), 4: ("12", "13", "14")}
OPAMP = [("A", "VT3_STIM"), ("B", "VT6_STIM")]
U3_UNITS = {}
for i, (ch, out) in enumerate(OPAMP):
    p, nn, o = SECTION_PINS[i + 1]
    inp, neg, oa = f"OP{ch}_P", f"OP{ch}_N", f"OP{ch}_OUT"
    U3_UNITS[i + 1] = {p: inp, nn: neg, o: oa}
    R(f"R{40 + 4 * i}", "10k", "DAC_" + ch, inp, f"DAC {ch} to op-amp, limits current if 5 V is off")
    R(f"R{41 + 4 * i}", "10k", neg, "GND", f"Gain set {ch}")
    R(f"R{42 + 4 * i}", "12k", oa, neg, f"Gain set {ch}: x2.2")
    R(f"R{43 + 4 * i}", "470R", oa, out, f"Output series {ch}")
# Sections C and D are unused: parked as grounded followers (+ to GND, - to out).
# The spare DAC channels C and D go straight to test pads instead (0-2.048 V).
U3_UNITS[3] = {"10": "GND", "9": "OPC_PARK", "8": "OPC_PARK"}
U3_UNITS[4] = {"12": "GND", "13": "OPD_PARK", "14": "OPD_PARK"}
U3_UNITS[5] = {"4": "+5V", "11": "GND"}
add("U3", "Amplifier_Operational:LM2902", "MCP6004", TSSOP14,
    {p: n for u in U3_UNITS.values() for p, n in u.items()}, units=U3_UNITS,
    note="Rail-to-rail quad on 5 V (6 V max)")
C("C12", "100nF", "+5V", "GND", "U3 decoupling")
for i, net in enumerate(["VT3_STIM", "VT6_STIM"]):
    add(f"D{16 + i}", "Diode:BAT54S", "BAT54S", SOT23, {"1": "GND", "2": "+5V", "3": net},
        note=net + " clamp to 0..5 V")

# --- I2C expander: every enable, powers up as inputs ------------------------
EXPANDER = {"4": "SAS_EN", "5": "IG1_EN", "6": "K1_EN", "7": "K2_EN",
            "9": "ARM_SENSE", "10": "V12_SENSE", "11": "EXP_P6", "12": "EXP_P7"}
add("U4", "Interface_Expansion:TCA9534", "TCA9534", TSSOP16,
    dict(EXPANDER, **{"1": "GND", "2": "GND", "3": "GND", "8": "GND", "13": "EXP_INT",
                      "14": "SCL", "15": "SDA", "16": "+3V3"}),
    note="8-bit I2C GPIO, address 0x20. P0 SAS, P1 IG1, P2 K1, P3 K2 (outputs); P4 ARM, P5 12V (inputs)")
C("C13", "100nF", "+3V3", "GND", "U4 decoupling")
R("R35", "100k", "+12V", "V12_SENSE", "12 V sense divider, 14.4 V -> 3.6 V")
R("R36", "33k", "V12_SENSE", "GND", "12 V sense divider")
DIODE("D18", "1N4148WS", "V12_SENSE", "+3V3", SOD323, "12 V sense clamp")
R("R37", "10k", "+3V3", "EXP_INT", "INT pull-up (open drain)")

# --- Connectors --------------------------------------------------------------
# J3, SAS + CAN pass-through (left end). Pins 3/5 and 4/6 are the same nets: one
# cable to the SAS, one pair on to the Feather M4 CAN terminal (X4: 1 L, 3 H).
J3_PINS = [("SAS_12V", "12V"), ("GND", "GND"), ("CANH", "CANH"), ("CANL", "CANL"),
           ("CANH", "H>FTH"), ("CANL", "L>FTH")]
# J4, torque sensor breakout (right end, top): GND between the two channels.
J4_PINS = [("VT3_EPS", "VT3 EPS"), ("VT3_CAR", "VT3 CAR"), ("GND", "GND"),
           ("VT6_EPS", "VT6 EPS"), ("VT6_CAR", "VT6 CAR")]
# J5, ECM signals + IG1 + ARM loop (right end, bottom). 12 V-level lines
# together (IG1, NEP), GND, then the 5 V-level lines (VSP, ARM loop).
J5_PINS = [("IG1_OUT", "IG1"), ("NEP_CAR", "NEP CAR"), ("NEP_EPS", "NEP EPS"), ("GND", "GND"),
           ("VSP_EPS", "VSP EPS"), ("VSP_CAR", "VSP CAR"), ("ARM_SRC", "ARM+"), ("ARM", "ARM")]
for ref, pins, val, note in [
        ("J3", J3_PINS, "SAS / CAN", "JST GH 6-way, right angle, locking"),
        ("J4", J4_PINS, "TORQUE", "JST GH 5-way, right angle, locking"),
        ("J5", J5_PINS, "ECM / IG1 / ARM", "JST GH 8-way, right angle, locking, back side")]:
    n = len(pins)
    nets = {str(i + 1): net for i, (net, _) in enumerate(pins)}
    nets["MP"] = "GND"
    add(ref, f"Connector_Generic_MountingPin:Conn_01x{n:02d}_MountingPin", val, GH.format(n=n), nets,
        key=f"GH {n}", note=note)

# Test pads (back side, bare pads)
TEST_PADS = [("GND", "GND"), ("+12V", "12V"), ("+5V", "5V"), ("+3V3", "3V3"), ("VUSB", "USB"),
             ("NEP_STIM", "NEP"), ("VSP_STIM", "VSP"), ("VT3_STIM", "VT3"), ("VT6_STIM", "VT6"),
             ("DAC_C", "DACC"), ("DAC_D", "DACD"), ("ARM", "ARM"), ("EXP_P6", "P6"), ("EXP_P7", "P7"),
             ("GND", "GND")]
for i, (net, _lbl) in enumerate(TEST_PADS):
    add(f"TP{i + 1}", "Connector:TestPoint", "TP", "TestPoint:TestPoint_Pad_D1.0mm", {"1": net},
        note="Test pad", in_bom=False)

# ---------------------------------------------------------------------------
# Board placement, mm in board coordinates (origin = top-left corner, top view).
# "F" = top (faces the OLED above), "B" = back (faces the Adalogger below).
# Height budget is the ~8.5 mm stacking-header gap on each side: relays (5.2 mm),
# inductor (3 mm) and the cable connectors sit on top; the back holds only parts
# <= 1.25 mm tall, plus J5 at the right end where the Adalogger is bare.
BOARD_W, BOARD_H = 50.8, 22.86
PLACE = {}   # filled in by placement.py
from placement import PLACE  # noqa: E402,F401

HAND_SOLDERED = {p["ref"] for p in PARTS if p.get("in_bom") is False}
