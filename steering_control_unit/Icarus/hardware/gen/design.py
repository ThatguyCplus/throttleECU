"""Icarus rev 0.1 schematic data: every part, pin and net, sheet by sheet.

Architecture and the reasons for each block: ../../docs/04_TECHNICAL_SAFETY_CONCEPT.md.
Net naming: suffix _A = control channel (CH-A, +3V3_A / +5V0_A), _B = monitor channel
(CH-B, +3V3_B / +5V0_B). CH-C (the analog guard) runs on the B-side 5 V rail.
Values marked TBD-DATA wait for logger measurements (docs/02, docs/04 section 7).
"""
PROJECT = "icarus"
REV = "0.1"

SHEETS = []          # (key, title, notes)
PARTS = []           # dicts: ref, lib, value, fp, units {unit: {pin: net}}, sheet, group, mpn, note
_counters = {}
_sheet = None
_group = None

FP = {
    ("R", "0402"): "Resistor_SMD:R_0402_1005Metric",
    ("R", "0603"): "Resistor_SMD:R_0603_1608Metric",
    ("C", "0402"): "Capacitor_SMD:C_0402_1005Metric",
    ("C", "0603"): "Capacitor_SMD:C_0603_1608Metric",
    ("C", "0805"): "Capacitor_SMD:C_0805_2012Metric",
    ("C", "1206"): "Capacitor_SMD:C_1206_3216Metric",
    ("C", "1210"): "Capacitor_SMD:C_1210_3225Metric",
}


def sheet(key, title, notes=""):
    global _sheet, _group
    SHEETS.append((key, title, notes))
    _sheet, _group = key, None


def group(title):
    global _group
    _group = title


def add(prefix, lib, value, fp, pins=None, units=None, mpn="", note=""):
    _counters[prefix] = _counters.get(prefix, 0) + 1
    ref = f"{prefix}{_counters[prefix]}"
    PARTS.append(dict(ref=ref, lib=lib, value=value, fp=fp, units=units or {1: pins},
                      sheet=_sheet, group=_group, mpn=mpn, note=note))
    return ref


def R(value, a, b, size="0402", note="", mpn=""):
    return add("R", "Device:R", value, FP[("R", size)], {"1": a, "2": b}, note=note, mpn=mpn)


def C(value, a, b="GND", size="0402", note=""):
    return add("C", "Device:C", value, FP[("C", size)], {"1": a, "2": b}, note=note)


def divider(src, out, top, bot, cap="1nF", note=""):
    R(top, src, out, note=note)
    R(bot, out, "GND")
    if cap:
        C(cap, out)


def nmos(gate, drain, source="GND", value="BSS138", note=""):
    return add("Q", "Transistor_FET:Q_NMOS_GSD", value, "Package_TO_SOT_SMD:SOT-23",
               {"1": gate, "2": source, "3": drain}, mpn="BSS138Q-7-F", note=note)


def opamp_quad(nets4, vpos, note=""):
    """TLV9064-Q1. nets4: list of 4 (plus, minus, out) tuples, None = spare (follower to GND)."""
    units = {}
    pins = {1: ("3", "2", "1"), 2: ("5", "6", "7"), 3: ("10", "9", "8"), 4: ("12", "13", "14")}
    ref_preview = f"U{_counters.get('U', 0) + 1}"
    for u, n in enumerate(nets4, start=1):
        p, m, o = pins[u]
        if n is None:
            spare = f"SPARE_{ref_preview}_{u}"
            units[u] = {p: "GND", m: spare, o: spare}
        else:
            units[u] = {p: n[0], m: n[1], o: n[2]}
    units[5] = {"4": vpos, "11": "GND"}
    ref = add("U", "Amplifier_Operational:TLV9064", "TLV9064-Q1", "Package_SO:TSSOP-14_4.4x5mm_P0.65mm",
              units=units, mpn="TLV9064QPWRQ1", note=note)
    C("100nF", vpos)
    return ref


def follower(inp, out):
    return (inp, out, out)


# =============================================================================
sheet("power", "Power: input protection and two independent supply chains",
      "Everything after VBAT_P is duplicated per channel (SM-16): buck -> LDO -> supervisors.")

group("Input protection (shared)")
add("F", "Device:Fuse", "5A mini blade", "Fuse:Fuseholder_Blade_Mini_Keystone_3568",
    {"1": "VBAT_IN", "2": "VBAT_F"}, mpn="Keystone 3568 + 5 A ATM fuse")
add("Q", "icarus:SQJ147ELP", "SQJ147ELP-T1_GE3", "icarus:PowerPAK_SO-8L_3_VIS",
    {"1": "VBAT_P", "2": "VBAT_P", "3": "VBAT_P", "4": "QREV_G", "5": "VBAT_F"},
    mpn="SQJ147ELP-T1_GE3", note="Reverse-battery P-FET: drain to battery, source to load (TI SLVA139)")
R("10k", "QREV_G", "GND")
add("D", "Diode:BZX84Cxx", "BZX84C15", "Package_TO_SOT_SMD:SOT-23", {"1": "QREV_G", "3": "VBAT_P"},
    mpn="BZX84C15-Q", note="Gate-source clamp")
add("D", "Device:D_Zener", "SMBJ20A", "Diode_SMD:D_SMB", {"1": "VBAT_P", "2": "GND"},
    mpn="SMBJ20A-Q", note="Load-dump TVS (unidirectional)")
add("C", "Device:C_Polarized", "470uF 50V", "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm",
    {"1": "VBAT_P", "2": "GND"}, note="Hold-up for crank / IMA restart dips")
C("100nF 100V", "VBAT_P", size="0603")

for ch in "AB":
    group(f"Supply chain {ch}: LMR38020-Q1 5.0 V buck, TPS7B81-Q1 3.3 V LDO, TPS3702-Q1 5 V window")
    add("U", "icarus:LMR38020", "LMR38020-Q1", "icarus:DDA0008E-IPC_A",
        {"1": "GND", "2": f"EN5_{ch}", "3": "VBAT_P", "4": f"RT_{ch}", "5": f"FB_{ch}",
         "6": f"PG5_{ch}", "7": f"BOOT_{ch}", "8": f"SW_{ch}", "9": "GND"},
        mpn="LMR38020FSQDDARQ1", note="Same part and RT as the throttle ECU, output set to 5.0 V")
    R("100k", "VBAT_P", f"EN5_{ch}", size="0603")
    R("64.9k", f"RT_{ch}", "GND", note="Switching frequency, as throttle R12")
    R("100k", f"+5V0_{ch}", f"FB_{ch}", note="VOUT = 1.0 V x (1 + 100k/24.9k) = 5.02 V")
    R("24.9k", f"FB_{ch}", "GND")
    R("100k", f"+3V3_{ch}", f"PG5_{ch}")
    C("100nF", f"BOOT_{ch}", f"SW_{ch}")
    add("L", "Device:L", "10uH", "Inductor_SMD:L_Bourns_SRP7028A_7.3x6.6mm", {"1": f"SW_{ch}", "2": f"+5V0_{ch}"},
        mpn="SRP7028A-100M")
    C("10uF 50V", "VBAT_P", size="1210")
    C("10uF 50V", "VBAT_P", size="1210")
    C("100nF 100V", "VBAT_P", size="0603")
    C("22uF 16V", f"+5V0_{ch}", size="1206")
    C("22uF 16V", f"+5V0_{ch}", size="1206")
    add("U", "Regulator_Linear:TPS7B8133DRV", "TPS7B8133-Q1", "Package_SON:WSON-6-1EP_2x2mm_P0.65mm_EP1x1.6mm",
        {"1": f"+5V0_{ch}", "2": f"+5V0_{ch}", "3": "GND", "4": "GND", "6": f"+3V3_{ch}", "7": "GND"},
        mpn="TPS7B8133QDRVRQ1", note="Check 150 mA against F280049C worst-case IDD")
    C("1uF", f"+5V0_{ch}", size="0603")
    C("10uF", f"+3V3_{ch}", size="0805")
    add("U", "Power_Supervisor:TPS3702", "TPS3702-Q1 5V", "Package_TO_SOT_SMD:SOT-23-6",
        {"1": f"SUP_OK_{ch}", "2": "GND", "3": f"+5V0_{ch}", "4": f"+5V0_{ch}", "5": f"+5V0_{ch}",
         "6": f"SUP_OK_{ch}"}, mpn="TPS3702CX50DDCRQ1 (verify variant)",
        note="OV/UV open-drain outputs wired into SUP_OK (SM-03)")
    C("100nF", f"+5V0_{ch}")

# =============================================================================
for ch in "AB":
    other = "B" if ch == "A" else "A"
    role = "control (computes and drives the overlay)" if ch == "A" else "monitor (checks; owns the low-side coil enable)"
    sheet(f"ch_{ch.lower()}", f"CH-{ch}: F280049C-Q1 {role}",
          "Pin mux from SPRS945 Table 5-6. Window watchdog TPS3850-Q1: CWD 10k to VDD, SET0 = SET1 = 1 "
          "-> 1.85 ms open / 11 ms timeout, service every ~5 ms.")
    group(f"MCU {ch}")
    tx, rx = f"FSI_{ch}2{other}", f"FSI_{other}2{ch}"
    gpio = {"1": f"SCI_RX_{ch}", "50": f"{rx}_CLK", "51": f"{rx}_D0", "54": f"VSP_{ch}", "55": f"NEP_{ch}",
            "56": f"ERRSTS_{ch}", "57": f"WLP_{ch}", "58": f"RT_{ch}_IN", "59": f"IG1_{ch}", "78": f"WDI_{ch}",
            "84": f"{tx}_CLK_TX", "97": f"{tx}_D0_TX", "98": f"CAN_RX_{ch}", "99": f"CAN_TX_{ch}",
            "100": f"SCI_TX_{ch}"}
    if ch == "A":
        gpio.update({"79": "EN_A_GPIO", "77": "SUPOK_MON_A"})
    else:
        gpio.update({"79": "PWM_EN_B", "77": "GATE_MON_B", "76": "SUPOK_MON_B"})
    gpio2 = {"53": f"DISNC_{ch}", "61": f"TDO_{ch}", "63": f"TDI_{ch}", "64": f"DISNO_{ch}", "65": f"PG5_{ch}",
             "85": f"CAN_STB_{ch}", "91": f"GTRIP_{ch}", "94": f"LED_{ch}"}
    if ch == "A":
        gpio2.update({"67": "DIAG_EN_A", "92": "ENHW_MON_A"})
    else:
        gpio2.update({"66": "GUARD_TEST", "67": "GUARD_RST", "92": "IND_DRV"})
    ana = {"7": f"EPS6M_{ch}", "9": f"RAW3M_{ch}", "10": f"RAW6M_{ch}", "13": "GND", "14": "GND", "15": "GND",
           "17": f"TSGM_{ch}", "24": f"VREFHI_BC_{ch}", "25": f"VREFHI_A_{ch}", "26": "GND", "27": "GND",
           "31": f"VCC2M_{ch}", "32": "GND", "35": f"VBATM_{ch}", "36": f"V5M_{ch}", "41": f"EPS3M_{ch}", "42": "GND"}
    if ch == "A":
        ana.update({"19": "DELTAM_A", "21": "CSK2_A", "23": "DACA_OUT", "28": "K1ACOM_M", "29": "CSK1_A",
                    "37": "K1BCOM_M"})
    else:
        ana.update({"19": "D3M_B", "21": "LSN2_B", "28": "GNODEM_B", "29": "LSN1_B", "38": "D6M_B", "40": "SM_B"})
    sysp = {"2": f"XRS_{ch}", "60": f"TCK_{ch}", "62": f"TMS_{ch}", "69": "X1_A" if ch == "A" else "GND",
            "73": "GND"}
    pwr = {n: f"+3V3_{ch}" for n in ("3", "47", "70", "88", "80")}
    pwr.update({n: f"VDD12_{ch}" for n in ("4", "46", "71", "87")})
    pwr.update({n: "GND" for n in ("5", "45", "72", "86", "82", "12", "33")})
    pwr.update({"11": f"VDDA_{ch}", "34": f"VDDA_{ch}"})
    add("U", "icarus:F280049C", "F280049CPZQR", "icarus:PZ0100A_M",
        units={1: gpio, 2: gpio2, 3: ana, 4: sysp, 5: pwr}, mpn="F280049CPZQR",
        note="TUV-listed functional-safety part (datasheet Table 3-1); throttle uses FS-QM F280049CPZS")

    group(f"MCU {ch} supplies and reference")
    for _ in range(5):
        C("100nF", f"+3V3_{ch}")
    C("10uF", f"+3V3_{ch}", size="0805")
    for _ in range(4):
        C("100nF", f"VDD12_{ch}")
    C("10uF", f"VDD12_{ch}", size="0805", note="Internal VREG output: check datasheet cap requirement")
    add("FB", "Device:FerriteBead_Small", "60R@100MHz", "Inductor_SMD:L_0603_1608Metric",
        {"1": f"+3V3_{ch}", "2": f"VDDA_{ch}"}, mpn="BLM18AG600SN1D")
    C("2.2uF", f"VDDA_{ch}", size="0603")
    C("2.2uF", f"VDDA_{ch}", size="0603")
    C("100nF", f"VDDA_{ch}")
    C("100nF", f"VDDA_{ch}")
    C("2.2uF", f"VREFHI_A_{ch}", size="0603")
    C("2.2uF", f"VREFHI_BC_{ch}", size="0603")

    group(f"Reset, window watchdog {ch}, hardware enable gate {ch}")
    R("2.2k", f"+3V3_{ch}", f"XRS_{ch}")
    C("10nF", f"XRS_{ch}")
    add("U", "icarus:TPS3850", "TPS3850G33-Q1", "Package_SON:VSON-10-1EP_3x3mm_P0.5mm_EP1.65x2.4mm",
        {"1": f"+3V3_{ch}", "2": f"CWD_{ch}", "3": f"+3V3_{ch}", "5": "GND", "6": f"+3V3_{ch}",
         "7": f"WDI_{ch}", "8": f"SUP_OK_{ch}", "9": f"XRS_{ch}", "10": f"+3V3_{ch}", "11": "GND"},
        mpn="TPS3850G33QDRCRQ1", note="Window watchdog + 3V3 OV/UV on SENSE. CRST open = factory reset delay")
    R("10k", f"+3V3_{ch}", f"CWD_{ch}")
    C("100nF", f"+3V3_{ch}")
    R("10k", f"+3V3_{ch}", f"SUP_OK_{ch}", note="Wired-AND: WDO + TPS3702 OV/UV")
    R("1k", f"SUP_OK_{ch}", f"SUPOK_MON_{ch}")
    en_in = "EN_A_GPIO" if ch == "A" else "PWM_EN_B"
    en_out = "EN_A_HW" if ch == "A" else "PUMP_B"
    add("U", "74xGxx:74LVC1G11", "SN74LVC1G11-Q1", "Package_TO_SOT_SMD:SOT-23-6",
        {"1": en_in, "2": "GND", "3": f"ERRSTS_{ch}", "4": en_out, "5": f"+3V3_{ch}", "6": f"SUP_OK_{ch}"},
        mpn="SN74LVC1G11QDBVRQ1", note="EN = GPIO AND ERRORSTS AND SUP_OK (SM-03)")
    C("100nF", f"+3V3_{ch}")
    R("100k", en_in, "GND", note="Off while the MCU is in reset")
    R("100k", f"ERRSTS_{ch}", "GND", note="GPIO24 = ERRORSTS once configured; low = error")
    if ch == "A":
        R("1k", "EN_A_HW", "ENHW_MON_A")
    else:
        group("Dynamic enable B: charge pump, PWM only (SM-04)")
        C("100nF", "PUMP_B", "PUMPX_B")
        add("D", "Device:D_Schottky", "BAT54WS-Q", "Diode_SMD:D_SOD-323", {"1": "PUMPX_B", "2": "GND"})
        add("D", "Device:D_Schottky", "BAT54WS-Q", "Diode_SMD:D_SOD-323", {"1": "GATE_B", "2": "PUMPX_B"})
        C("100nF", "GATE_B")
        R("100k", "GATE_B", "GND", note="Bleed: gate collapses within ~10 ms without PWM")
        R("10k", "GATE_B", "GATE_MON_B")
        group("Driver indicator output (low side)")
        nmos("IND_DRV", "IND_OUT")
        R("100k", "IND_DRV", "GND")

    group(f"FSI to CH-{other}, CAN, debug, status LED")
    R("470", f"{tx}_CLK_TX", f"{tx}_CLK", note="Limits back-powering an unpowered channel")
    R("470", f"{tx}_D0_TX", f"{tx}_D0")
    add("U", "icarus:TCAN3404", "TCAN3404-Q1", "icarus:SOIC8_D_TEX-M",
        {"1": f"CAN_TX_{ch}", "2": "GND", "3": f"+3V3_{ch}", "4": f"CAN_RX_{ch}", "5": "GND", "6": "CAN_L",
         "7": "CAN_H", "8": f"CAN_STB_{ch}"}, mpn="TCAN3404DRQ1")
    C("100nF", f"+3V3_{ch}")
    R("10k", f"CAN_STB_{ch}", "GND", note="Normal mode by default")
    if ch == "A":
        add("U", "icarus:SIT2024B", "SiT2024B 10MHz", "icarus:SOT_SiT1630-S_SIT",
            {"1": "GND", "3": "+3V3_A", "4": "+3V3_A", "5": "X1_A_S"}, mpn="SIT2024BA-S2-XXE-10.000000E",
            note="CH-B runs on INTOSC2 instead (clock diversity)")
        C("100nF", "+3V3_A")
        R("33", "X1_A_S", "X1_A")
    add("J", "Connector:Conn_ARM_JTAG_SWD_10", f"JTAG {ch}", "Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical",
        {"1": f"+3V3_{ch}", "2": f"TMS_{ch}", "3": "GND", "4": f"TCK_{ch}", "5": "GND", "6": f"TDO_{ch}",
         "8": f"TDI_{ch}", "9": "GND", "10": f"XRS_{ch}"})
    R("2.2k", f"TCK_{ch}", "GND")
    add("J", "Connector_Generic:Conn_01x03", f"UART {ch}", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
        {"1": f"SCI_TX_{ch}", "2": f"SCI_RX_{ch}", "3": "GND"})
    R("1k", f"LED_{ch}", f"LEDA_{ch}")
    add("D", "Device:LED", "LED green", "LED_SMD:LED_0603_1608Metric", {"1": "GND", "2": f"LEDA_{ch}"})

# =============================================================================
sheet("overlay", "Overlay path (CH-A analog, +5V0_A)",
      "EPS gets OVL = RAW + (DELTA - VMID). DELTA = VMID + k(DAC - VMID), k = 10k/92k = 0.109 -> "
      "Delta_max = +/-0.18 V (TBD-DATA, SM-05). One DAC + inverter makes VT6 = -Delta (SM-06). "
      "Summing resistors 0.1 %.")
group("References and Delta generation")
R("10k", "+3V3_A", "VMIDD_A")
R("10k", "VMIDD_A", "GND")
C("100nF", "VMIDD_A")
R("10k", "DACA_OUT", "DACF_A", note="SM-07 slew limit, tau = 10 ms (TBD-DATA)")
C("1uF", "DACF_A", size="0603")
R("82k", "DACF_A", "DPIN_A", note="Authority limit k (SM-05). Changing it changes Delta_max")
R("10k", "DPIN_A", "VMID_A")
R("1M", "VT3_SNS", "RAW3IN_A", note="FSR-14: >= 1 MOhm on every car-side tap")
C("100pF", "RAW3IN_A")
R("1M", "VT6_SNS", "RAW6IN_A")
C("100pF", "RAW6IN_A")
opamp_quad([follower("VMIDD_A", "VMID_A"), follower("DPIN_A", "DELTA_P"),
            ("VMID_A", "INVN_A", "DELTA_N"), follower("RAW3IN_A", "RAW3_A")], "+5V0_A")
R("10k 0.1%", "DELTA_P", "INVN_A")
R("10k 0.1%", "INVN_A", "DELTA_N")

group("Summers: OVL3 = RAW3 + DELTA_P - VMID, OVL6 = RAW6 + DELTA_N - VMID")
opamp_quad([follower("RAW6IN_A", "RAW6_A"), ("SUM3P_A", "SUM3N_A", "OVL3"), ("SUM6P_A", "SUM6N_A", "OVL6"),
            follower("EPS3IN_A", "EPS3_A")], "+5V0_A")
for raw, dlt, sp, sn, out in (("RAW3_A", "DELTA_P", "SUM3P_A", "SUM3N_A", "OVL3"),
                              ("RAW6_A", "DELTA_N", "SUM6P_A", "SUM6N_A", "OVL6")):
    R("10k 0.1%", raw, sp)
    R("10k 0.1%", dlt, sp)
    R("10k 0.1%", "VMID_A", sn)
    R("10k 0.1%", sn, out)
R("100", "OVL3", "OVL3_OUT")
R("100", "OVL6", "OVL6_OUT")

group("EPS-side readback buffers and CH-A ADC scaling (x0.667)")
R("1M", "VT3_EPS", "EPS3IN_A")
C("100pF", "EPS3IN_A")
R("1M", "VT6_EPS", "EPS6IN_A")
C("100pF", "EPS6IN_A")
opamp_quad([follower("EPS6IN_A", "EPS6_A"), None, None, None], "+5V0_A")
for src, dst in (("RAW3_A", "RAW3M_A"), ("RAW6_A", "RAW6M_A"), ("EPS3_A", "EPS3M_A"), ("EPS6_A", "EPS6M_A"),
                 ("DELTA_P", "DELTAM_A")):
    divider(src, dst, "10k", "20k")
divider("K1A_COM", "K1ACOM_M", "1M", "2M", cap="10nF", note="Relay K1 proof-test readback (SM-09)")
divider("K1B_COM", "K1BCOM_M", "1M", "2M", cap="10nF")

# =============================================================================
sheet("guard", "CH-C analog guard (no software, +5V0_B)",
      "Trips when |VT_EPS - VT_SNS| > 0.25 V on either line, or the VT3+VT6 sum moves > 0.10 V "
      "(TBD-DATA). A trip latches until key-off and cuts every coil (SM-15). GUARD_TEST forces a trip "
      "for the key-on self-test (SM-17).")
group("Buffers (1 MOhm from the car side) and VMID_B")
R("10k", "+5V0_B", "VMIDD_B")
R("10k", "VMIDD_B", "GND")
C("100nF", "VMIDD_B")
for src, node in (("VT3_SNS", "RAW3IN_B"), ("VT6_SNS", "RAW6IN_B"), ("VT3_EPS", "EPS3IN_B"), ("VT6_EPS", "EPS6IN_B")):
    R("1M", src, node)
    C("100pF", node)
opamp_quad([follower("VMIDD_B", "VMID_B"), follower("RAW3IN_B", "RAW3_B"), follower("RAW6IN_B", "RAW6_B"),
            follower("EPS3IN_B", "EPS3_B")], "+5V0_B")

group("Difference amps: D3 = EPS3 - RAW3 + VMID, D6 likewise, S = (EPS3+EPS6) - (RAW3+RAW6) + VMID")
opamp_quad([follower("EPS6IN_B", "EPS6_B"), ("D3P", "D3N", "D3"), ("D6P", "D6N", "D6"), ("SP", "SN", "SSUM")],
           "+5V0_B")
for eps, raw, p, n, out in (("EPS3_B", "RAW3_B", "D3P", "D3N", "D3"), ("EPS6_B", "RAW6_B", "D6P", "D6N", "D6")):
    R("10k 0.1%", eps, p)
    R("10k 0.1%", "VMID_B", p)
    R("10k 0.1%", raw, n)
    R("10k 0.1%", n, out)
R("10k 0.1%", "EPS3_B", "SP")
R("10k 0.1%", "EPS6_B", "SP")
R("10k 0.1%", "VMID_B", "SP")
R("10k 0.1%", "RAW3_B", "SN")
R("10k 0.1%", "RAW6_B", "SN")
R("10k 0.1%", "SN", "SSUM")

group("Window thresholds (ladder from +5V0_B) and comparators (TLV1704-Q1, open collector)")
R("45.3k", "+5V0_B", "TH_HD", note="TH_HD 2.75 V, TH_HS 2.60 V, TH_LS 2.40 V, TH_LD 2.25 V (TBD-DATA)")
R("3.01k", "TH_HD", "TH_HS")
R("4.02k", "TH_HS", "TH_LS")
R("3.01k", "TH_LS", "TH_LD")
R("45.3k", "TH_LD", "GND")
for n in ("TH_HD", "TH_HS", "TH_LS", "TH_LD"):
    C("10nF", n)


def comparator_quad(nets4):
    units = {}
    pins = {1: ("5", "4", "2"), 2: ("7", "6", "1"), 3: ("11", "10", "13"), 4: ("9", "8", "14")}
    for u, n in enumerate(nets4, start=1):
        p, m, o = pins[u]
        units[u] = {p: "+5V0_B", m: "GND"} if n is None else {p: n[0], m: n[1], o: "TRIP_N"}
    units[5] = {"3": "+5V0_B", "12": "GND"}
    add("U", "Comparator:LM339", "TLV1704-Q1", "Package_SO:TSSOP-14_4.4x5mm_P0.65mm", units=units,
        mpn="TLV1704AQPWRQ1", note="LM339 symbol: TLV1704-Q1 PW pinout is identical (SLOS890 Pin Functions)")
    C("100nF", "+5V0_B")


comparator_quad([("TH_HD", "D3"), ("D3", "TH_LD"), ("TH_HD", "D6"), ("D6", "TH_LD")])
comparator_quad([("TH_HS", "SSUM"), ("SSUM", "TH_LS"), None, None])
R("10k", "+5V0_B", "TRIP_N")
C("1nF", "TRIP_N")

group("Trip latch, self-test and reset, guard gate, status to both MCUs")
add("U", "74xGxx:74AUC1G74", "SN74LVC1G74-Q1", "Package_SO:VSSOP-8_2.3x2mm_P0.5mm",
    {"1": "GND", "2": "GND", "3": "GUARD_OKQ", "4": "GND", "5": "GTRIP", "6": "GCLR_N", "7": "TRIP_N",
     "8": "+5V0_B"}, mpn="SN74LVC1G74QDCURQ1", note="74x1G74 pinout (DCU): PRE = trip, CLR = power-on/reset")
C("100nF", "+5V0_B")
R("100k", "+5V0_B", "GCLR_N", note="Power-on clear: key-off/on resets the latch")
C("1uF", "GCLR_N", size="0603")
nmos("GUARD_RST", "GCLR_N", note="CH-B may clear the latch (INIT only, by firmware rule)")
R("100k", "GUARD_RST", "GND")
nmos("GUARD_TEST", "TESTD", note="SM-17: pulls TH_HD low -> forced trip")
R("100k", "GUARD_TEST", "GND")
R("1k", "TESTD", "TH_HD")
R("1k", "GUARD_OKQ", "GUARD_GATE")
R("100k", "GUARD_GATE", "GND")
divider("GTRIP", "GTRIP_A", "10k", "20k", cap=None)
divider("GTRIP", "GTRIP_B", "10k", "20k", cap=None)

group("CH-B ADC scaling (x0.667)")
for src, dst in (("RAW3_B", "RAW3M_B"), ("RAW6_B", "RAW6M_B"), ("EPS3_B", "EPS3M_B"), ("EPS6_B", "EPS6M_B"),
                 ("D3", "D3M_B"), ("D6", "D6M_B"), ("SSUM", "SM_B")):
    divider(src, dst, "10k", "20k")

# =============================================================================
sheet("relays", "Signal relays K1/K2 (cascaded) and cross-driven coils",
      "EPS sees the overlay only if K1 AND K2 are energized (SM-14). Each coil: high side from CH-A "
      "(TPS1H100-Q1), low side from CH-B (charge-pumped gate) and the CH-C guard FET in series.")
for k in (1, 2):
    group(f"K{k} coil drive")
    add("U", "icarus:TPS1H100", "TPS1H100-Q1", "icarus:PWP0014F_M",
        units={1: {"2": "GND", "3": "EN_A_HW", "5": f"COIL{k}_HI", "6": f"COIL{k}_HI", "7": f"COIL{k}_HI",
                   "8": "VBAT_P", "9": "VBAT_P", "10": "VBAT_P", "12": "DIAG_EN_A", "13": f"CL{k}",
                   "14": f"CSR{k}", "15": "GND"}},
        mpn="TPS1H100BQPWPRQ1", note="Same part as throttle U2")
    C("100nF 50V", "VBAT_P", size="0603")
    R("4.99k", f"CL{k}", "GND", note="Current limit (TBD vs coil current)")
    R("1k", f"CSR{k}", "GND", note="Current-sense load")
    R("1k", f"CSR{k}", f"CSK{k}_A")
    C("10nF", f"CSK{k}_A")
    add("D", "Diode:BAS21", "BAS21-Q", "Package_TO_SOT_SMD:SOT-323_SC-70", {"1": f"COIL{k}_LO", "3": f"CLMP{k}"})
    add("D", "Diode:BZX84Cxx", "BZX84C18", "Package_TO_SOT_SMD:SOT-23", {"1": f"COIL{k}_HI", "3": f"CLMP{k}"},
        note="SM-12: diode + Zener for fast release")
    nmos("GATE_B", f"COIL{k}_LO", "GNODE", note="CH-B low side")
    divider(f"COIL{k}_LO", f"LSN{k}_B", "100k", "22k", cap="10nF")

group("Relays: K1 picks overlay vs sensor, K2 picks K1 vs sensor")
add("K", "Relay:G6K-2", "G6K-2F-Y 12VDC", "Relay_SMD:Relay_DPDT_Omron_G6K-2F-Y",
    {"1": "COIL1_HI", "8": "COIL1_LO", "3": "K1A_COM", "2": "VT3_SNS", "4": "OVL3_OUT",
     "6": "K1B_COM", "7": "VT6_SNS", "5": "OVL6_OUT"}, mpn="G6K-2F-Y DC12 (automotive equivalent TBD)",
    note="Pins: 1/8 coil, 3/6 COM, 2/7 NC, 4/5 NO (stock symbol). Gold contacts, dry switching")
add("K", "Relay:G6K-2", "G6K-2F-Y 12VDC", "Relay_SMD:Relay_DPDT_Omron_G6K-2F-Y",
    {"1": "COIL2_HI", "8": "COIL2_LO", "3": "VT3_EPS", "2": "VT3_SNS", "4": "K1A_COM",
     "6": "VT6_EPS", "7": "VT6_SNS", "5": "K1B_COM"}, mpn="G6K-2F-Y DC12 (consider a second vendor)")
C("4.7nF", "VT3_EPS", note="SM-11 hold cap (TBD-DATA, A2)")
C("4.7nF", "VT6_EPS")

group("CH-C guard FET (common coil return)")
nmos("GUARD_GATE", "GNODE", note="Off = both coils dead (SM-15)")
divider("GNODE", "GNODEM_B", "100k", "22k", cap="10nF")

# =============================================================================
sheet("io", "Vehicle connector, CAN bus, inputs and monitors",
      "Every car-facing input has its own divider per channel (no shared resistor between CH-A and CH-B). "
      "Torque-sensor and Vcc2 taps are 1 MOhm (FSR-14).")
group("Vehicle connector (Y-harness at EPS connector B; part TBD, TE MCP family as throttle J2)")
J1 = {"1": "VBAT_IN", "2": "GND", "3": "GND", "4": "IG1_IN", "5": "CAN_H", "6": "CAN_L", "7": "VT3_SNS",
      "8": "VT6_SNS", "9": "VT3_EPS", "10": "VT6_EPS", "11": "VCC2_TAP", "12": "TSGND_TAP", "13": "WLP_IN",
      "14": "RT_IN", "15": "VSP_IN", "16": "NEP_IN", "17": "DIS_NO_IN", "18": "DIS_NC_IN", "19": "IND_OUT",
      "20": "GND"}
add("J", "Connector_Generic:Conn_01x20", "Vehicle harness", "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical",
    J1, note="Placeholder footprint. 7/8 = torque sensor side, 9/10 = EPS ECU B10/B8")

group("CAN bus: ESD and split termination (fit JP1 only if Icarus ends the bus)")
add("D", "Power_Protection:NUP2105L", "ESD2CAN24-Q1", "Package_TO_SOT_SMD:SOT-23",
    {"1": "CAN_H", "2": "CAN_L", "3": "GND"}, mpn="ESD2CAN24DBZRQ1")
add("JP", "Jumper:SolderJumper_2_Open", "CAN term", "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm",
    {"1": "CAN_H", "2": "CANH_T"})
R("60.4", "CANH_T", "CAN_SPLIT", size="0603")
R("60.4", "CAN_SPLIT", "CAN_L", size="0603")
C("4.7nF", "CAN_SPLIT")

group("Disengage switch wetting (NO + NC contacts to GND)")
R("4.7k", "VBAT_P", "DIS_NO_IN", size="0603")
R("4.7k", "VBAT_P", "DIS_NC_IN", size="0603")

INPUTS = (("VSP_IN", "VSP", "100k"), ("NEP_IN", "NEP", "33k"), ("WLP_IN", "WLP", "33k"), ("RT_IN", "RT", "33k"),
          ("IG1_IN", "IG1", "33k"), ("DIS_NO_IN", "DISNO", "33k"), ("DIS_NC_IN", "DISNC", "33k"))
for ch in "AB":
    group(f"Digital inputs to CH-{ch} (100k series, divider, 1 nF, clamp to +3V3_{ch})")
    for src, name, bot in INPUTS:
        net = f"{name}_{ch}" if name != "RT" else f"RT_{ch}_IN"
        R("100k", src, net)
        R(bot, net, "GND")
        C("1nF", net)
        add("D", "Diode:1N4148WS", "1N4148WS-Q", "Diode_SMD:D_SOD-323", {"1": f"+3V3_{ch}", "2": net})
    group(f"Analog monitors to CH-{ch}")
    divider("VBAT_P", f"VBATM_{ch}", "100k", "10k", cap="10nF")
    divider(f"+5V0_{ch}", f"V5M_{ch}", "10k", "10k", cap="10nF")
    divider("VCC2_TAP", f"VCC2M_{ch}", "1M", "1M", cap="100nF")
    R("1M", "TSGND_TAP", f"TSGM_{ch}", note="Ground-offset monitor, biased to mid-rail")
    R("1M", f"TSGM_{ch}", f"+3V3_{ch}")
    C("100nF", f"TSGM_{ch}")

# Rails that need a PWR_FLAG (driven by regulators or the protection FET).
POWER_FLAGS = ["VBAT_P", "+5V0_A", "+5V0_B", "VDD12_A", "VDD12_B", "VDDA_A", "VDDA_B",
               "VREFHI_A_A", "VREFHI_BC_A", "VREFHI_A_B", "VREFHI_BC_B", "GND"]
