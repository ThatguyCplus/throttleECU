"""Icarus rev 0.2 schematic data: every part, pin and net, sheet by sheet.

Architecture and the reasons for each block: ../../docs/04_TECHNICAL_SAFETY_CONCEPT.md.
Net naming: suffix _A = control channel (CH-A, +3V3_A / +5V0_A), _B = monitor channel
(CH-B, +3V3_B / +5V0_B). CH-C (the analog guard) runs on the B-side 5 V rail.
Values marked TBD-DATA wait for logger measurements (docs/02, docs/04 section 7).
"""
PROJECT = "icarus"
REV = "0.7"

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


def place_with(title, refs):
    """PCB only: place these parts in another group's cluster; the schematic keeps them where they are."""
    for p in PARTS:
        if p["ref"] in refs:
            p["place_with"] = title


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


def led(src, sink, r, color, name, size="0402", note=""):
    """Indicator: src -> R -> LED -> sink; lit when src is high and sink is low.
    Groups named "Indicators" stay out of the SG1 elements (safety/element_model.py -> E99): every indicator
    hangs on a rail, a spare GPIO, a push-pull output, across a coil, or on an open-drain fault line where a
    shorted LED only adds a weak pull-up the driver still overpowers (rev 0.5, docs/06 F14)."""
    R(r, src, f"LI_{name}", size=size, note=note)
    return add("D", "Device:LED", f"LED {color}", "LED_SMD:LED_0603_1608Metric", {"1": sink, "2": f"LI_{name}"})


# =============================================================================
sheet("power", "Power: input protection and two independent supply chains",
      "Everything after VBAT_P is duplicated per channel (SM-16): buck -> LDO -> supervisors.")

group("Input protection (shared)")
add("F", "Device:Fuse", "5A mini blade", "Fuse:Fuseholder_Blade_Mini_Keystone_3568",
    {"1": "VBAT_IN", "2": "VBAT_F"}, mpn="Keystone 3568 + 5 A ATM fuse")
add("F", "Device:Polyfuse", "1.1A PTC", "Fuse:Fuse_1812_4532Metric", {"1": "VBAT_F", "2": "VBAT_FP"},
    mpn="SMDH1812B110TF/33", note="Resettable fuse behind the blade fuse, as the throttle ECU's F1. "
                                  "Yageo automotive grade, AEC-Q200, 33 V (was 1812L110/33MR, not AEC-Q200)")
add("Q", "icarus:SQJ147ELP", "SQJ147ELP-T1_GE3", "icarus:PowerPAK_SO-8L_3_VIS",
    {"1": "VBAT_P", "2": "VBAT_P", "3": "VBAT_P", "4": "QREV_G", "5": "VBAT_FP"},
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
    out = "+5V0_A_SRC" if ch == "A" else "+5V0_B"      # rail A goes through an ideal diode (backup sheet)
    R("100k", out, f"FB_{ch}", note="VOUT = 1.0 V x (1 + 100k/24.9k) = 5.02 V")
    R("24.9k", f"FB_{ch}", "GND")
    R("100k", f"+3V3_{ch}", f"PG5_{ch}")
    C("100nF", f"BOOT_{ch}", f"SW_{ch}")
    add("L", "Device:L", "10uH", "Inductor_SMD:L_Bourns_SRP7028A_7.3x6.6mm", {"1": f"SW_{ch}", "2": out},
        mpn="SRP7028A-100M")
    C("10uF 50V", "VBAT_P", size="1210")
    C("10uF 50V", "VBAT_P", size="1210")
    C("100nF 100V", "VBAT_P", size="0603")
    C("22uF 16V", out, size="1206")
    C("22uF 16V", out, size="1206")
    if ch == "A":
        add("U", "Power_Management:LM66100DCK", "LM66100-Q1", "Package_TO_SOT_SMD:SOT-363_SC-70-6",
            {"1": "+5V0_A_SRC", "2": "GND", "3": "GND", "6": "+5V0_A"}, mpn="LM66100QDCKRQ1",
            note="Ideal diode: buck A feeds +5V0_A, the LiPo boost ORs in behind it, and the charger hangs off "
                 "+5V0_A_SRC so backup power can't loop back into the charger")
        C("22uF 16V", "+5V0_A", size="1206")
    add("U", "Regulator_Linear:TPS7B8133DRV", "TPS7B8233-Q1", "Package_SON:WSON-6-1EP_2x2mm_P0.65mm_EP1x1.6mm",
        {"1": f"+5V0_{ch}", "2": f"+5V0_{ch}", "3": "GND", "4": "GND", "6": f"+3V3_{ch}", "7": "GND"},
        mpn="TPS7B8233QDRVRQ1",
        note="300 mA. F280049C IDDIO max 113 mA (SPRS945) + TCAN3404 + clock; the 150 mA TPS7B81 was too tight. "
             "Same 2x2 mm DRV pinout (IN 1, EN 2, GND 3-4, DNC 5, OUT 6)")
    C("2.2uF", f"+5V0_{ch}", size="0603")
    C("10uF", f"+3V3_{ch}", size="0805")
    add("U", "Power_Supervisor:TPS3702", "TPS3702CX50", "Package_TO_SOT_SMD:SOT-23-6",
        {"1": f"SUP_OK_{ch}", "2": "GND", "3": f"SNS5_{ch}", "4": f"+5V0_{ch}", "5": f"+5V0_{ch}",
         "6": f"SUP_OK_{ch}"}, mpn="TPS3702CX50QDDCRQ1",
        note="SET = VDD: window 4.80 / 5.20 V (datasheet Table 10-2). OV/UV open-drain into SUP_OK (SM-03). "
             "AEC-Q100 TPS3702-Q1 (SBVS261C), same DDC package and pinout as the commercial part")
    C("100nF", f"+5V0_{ch}")
    R("10k", f"+5V0_{ch}", f"SNS5_{ch}", note="R5 / SM-21: lets SUPTEST pull SENSE to 2.5 V (UV) at key-on")
    R("10k", f"SNS5_{ch}", f"SNS5T_{ch}")
    nmos(f"SUPTEST_{ch}", f"SNS5T_{ch}", note=f"CH-{ch} supervisor self-test; stuck on = channel disabled (safe)")
    R("100k", f"SUPTEST_{ch}", "GND")

group("Supply chain B (rev 0.7): CH-B core 1.2 V, TLV733P-Q1 from +5V0_B, TPS3702-Q1 window into nPORRST")
add("U", "icarus:TLV73312_DRV", "TLV73312P-Q1", "Package_SON:WSON-6-1EP_2x2mm_P0.65mm_EP1x1.6mm",
    {"6": "+5V0_B", "4": "+5V0_B", "3": "GND", "1": "VCC1V2_B", "7": "GND"}, mpn="TLV73312PQDRVRQ1",
    note="DRV pinout (SBVS283F): OUT 1, GND 3, EN 4, IN 6. From +5V0_B, not +3V3_B: the CH-B 3V3 LDO (300 mA) "
         "can't also carry the 145 mA core. 3.8 V x 145 mA = 0.55 W worst case in the WSON thermal pad")
C("1uF", "+5V0_B", size="0603")
C("1uF", "VCC1V2_B", size="0603", note="TLV733P-Q1: 1 uF ceramic recommended at OUT (stable without)")
add("U", "Power_Supervisor:TPS3702", "TPS3702CX12-Q1", "Package_TO_SOT_SMD:SOT-23-6",
    {"1": "PORRST_B", "2": "GND", "3": "VCC1V2_B", "4": "+3V3_B", "5": "+3V3_B", "6": "PORRST_B"},
    mpn="TPS3702CX12QDDCRQ1",
    note="1.2 V window (SET = VDD; check SBVS261C Table 10-2 for CX12: ~1.152 / 1.248 V vs TMS570 1.14-1.32 V). "
         "nPORRST must go low when any supply is out of range (SPNS186C Table 4-9)")
C("100nF", "+3V3_B")

group("Indicators: rail present (green, ~1.4 mA each)")
led("VBAT_P", "GND", "8.2k", "green", "VBAT", size="0603", note="Protected battery rail")
led("+5V0_A_SRC", "GND", "2.2k", "green", "5V0A", note="Buck A, ahead of the ideal diode")
led("+5V0_B", "GND", "2.2k", "green", "5V0B")
led("+3V3_A", "GND", "1k", "green", "3V3A")
led("+3V3_B", "GND", "1k", "green", "3V3B")

# =============================================================================
# rev 0.7: CH-B is a TMS570LS0432 (Arm Cortex-R4F, dual core in lockstep), so the monitor is DIVERSE from
# the F280049C (C28x) control channel: no shared silicon, compiler or toolchain (docs/06 F16).
# Pins from SPNS186C Tables 4-1 .. 4-16; every one of the 100 is checked in make_sch.CUSTOM_SYMS.
TMS570_B = {
    # N2HET: captures, PWMs, fast outputs
    "19": "VSP_B", "22": "NEP_B", "25": "PWM_EN_B", "26": "REFPWM_B", "74": "WDI_B", "83": "LED_B",
    "89": "LEDFLT_B", "90": "LEDCAN_B", "97": "LEDPRM_B", "98": "IND_DRV", "11": "GUARD_TEST", "64": "GUARD_RST",
    # GIOA (interrupt-capable inputs)
    "1": "GTRIP_B", "2": "SPDHI_B", "5": "IG1_B", "8": "DISNO_B", "9": "DISNC_B", "10": "WLP_B", "12": "RT_B_IN",
    "18": "PG5_B",
    # MibSPI1 pins as GPIO
    "93": "LBO_N", "27": "SUPOK_MON_B", "39": "GATE_MON_B", "68": "SUPTEST_B", "67": "GTEST_LO_N", "73": "CAN_STB_B",
    # DCAN1, and the SCI (LIN module in SCI mode) as the link to CH-A
    "62": "CAN_TX_B", "63": "CAN_RX_B", "95": "LINK_B2A_TX", "94": "LINK_A2B",
    # MibADC: 16 inputs, supply = reference = VDDA_B (3.3 V, filtered)
    "42": "EPS3M_B", "49": "EPS6M_B", "51": "RAW3M_B", "52": "RAW6M_B", "54": "D3M_B", "55": "D6M_B", "56": "SM_B",
    "43": "REF3M_B", "57": "LSN1_B", "48": "LSN2_B", "50": "GNODEM_B", "53": "VBATM_B", "40": "V5M_B",
    "41": "VCC2M_B", "44": "TSGM_B", "45": "VLIM_B", "46": "VDDA_B", "47": "GND",
    # system, clock, debug
    "31": "PORRST_B", "81": "NRST_B", "82": "ERRSTS_B", "14": "OSCIN_B", "16": "OSCOUT_B", "15": "KELVIN_B",
    "24": "GND", "75": "TMS_B", "76": "TRST_B", "77": "TDI_B", "78": "TDO_B", "79": "TCK_B", "80": "RTCK_B",
}
TMS570_B.update({n: "VCC1V2_B" for n in ("13", "21", "30", "32", "61", "88", "99")})
TMS570_B.update({n: "+3V3_B" for n in ("6", "28", "60", "85", "96")})
TMS570_B.update({n: "GND" for n in ("7", "17", "20", "29", "33", "59", "72", "86", "87", "100")})
# unconnected: FLTP1/2 (3, 4: must stay unconnected, SPNS186C Table 4-13), ECLK 84, spare SPI pins

for ch in "AB":
    other = "B" if ch == "A" else "A"
    if ch == "A":
        sheet("ch_a", "CH-A: F280049C-Q1 control (computes and drives the overlay)",
              "Pin mux from SPRS945 Table 5-6. Window watchdog TPS3850-Q1: CWD 10k to VDD, SET0 = SET1 = 1 "
              "-> 1.85 ms open / 11 ms timeout, service every ~5 ms. Link to CH-B: SCI-B on GPIO12/13 (rev 0.7).")
    else:
        sheet("ch_b", "CH-B: TMS570LS0432-Q1 monitor (diverse Arm Cortex-R4F lockstep; owns the low-side coil enable)",
              "Pins from SPNS186C. ESM nERROR into the enable gate (SM-03). Core 1.2 V from TLV733P-Q1 (power sheet), "
              "supervised into nPORRST together with the TPS3850. 16 MHz crystal on OSCIN/OSCOUT, load caps to "
              "KELVIN_GND only. Expected-Delta reference from N2HET PWM + RC (no DAC on this part).")
    group(f"MCU {ch}")
    if ch == "A":
        gpio = {"1": "SCI_RX_A", "50": "LINK_B2A", "51": "LINK_A2B_TX", "54": "VSP_A", "55": "NEP_A",
                "56": "ERRSTS_A", "57": "WLP_A", "58": "RT_A_IN", "59": "IG1_A", "78": "WDI_A",
                "98": "CAN_RX_A", "99": "CAN_TX_A", "100": "SCI_TX_A", "89": "LEDFLT_A", "90": "LEDCAN_A",
                "93": "LEDENG_A", "79": "EN_A_GPIO", "77": "SUPOK_MON_A", "75": "SUPTEST_A", "81": "SPDHI_A",
                "83": "CHG_STAT_A"}
        gpio2 = {"53": "DISNC_A", "61": "TDO_A", "63": "TDI_A", "64": "DISNO_A", "65": "PG5_A", "85": "CAN_STB_A",
                 "91": "GTRIP_A", "94": "LED_A", "67": "DIAG_EN_A", "92": "ENHW_MON_A", "66": "GRSTA_TX"}
        ana = {"7": "EPS6M_A", "9": "RAW3M_A", "10": "RAW6M_A", "13": "GND", "14": "GND", "15": "GND",
               "17": "TSGM_A", "24": "VREFHI_BC_A", "25": "VREFHI_A_A", "26": "GND", "27": "GND",
               "31": "VCC2M_A", "32": "GND", "35": "VBATM_A", "36": "V5M_A", "41": "EPS3M_A", "42": "GND",
               "19": "DELTAM_A", "21": "CSK2_A", "23": "DACA_OUT", "28": "K1ACOM_M", "29": "CSK1_A",
               "37": "K1BCOM_M", "44": "VLIM_A", "20": "V5SM_A"}
        sysp = {"2": "XRS_A", "60": "TCK_A", "62": "TMS_A", "69": "X1_A", "73": "GND"}
        pwr = {n: "+3V3_A" for n in ("3", "47", "70", "88", "80")}
        pwr.update({n: "VDD12_A" for n in ("4", "46", "71", "87")})
        pwr.update({n: "GND" for n in ("5", "45", "72", "86", "82", "12", "33")})
        pwr.update({"11": "VDDA_A", "34": "VDDA_A"})
        add("U", "icarus:F280049C", "F280049CPZQR", "icarus:PZ0100A_M",
            units={1: gpio, 2: gpio2, 3: ana, 4: sysp, 5: pwr}, mpn="F280049CPZQR",
            note="TUV-listed functional-safety part (datasheet Table 3-1); throttle uses FS-QM F280049CPZS")
        # supplies and reference: same group as the MCU, so the placer gives each cap its pin (SPRS945)
        for _ in range(5):
            C("100nF", "+3V3_A")
        C("10uF", "+3V3_A", size="0805")
        for _ in range(4):
            C("100nF", "VDD12_A")
        C("10uF", "VDD12_A", size="0805", note="VDD (1.2 V VREG output): SPRS945 pin table asks for ~20 uF total")
        C("10uF", "VDD12_A", size="0805")
        add("FB", "Device:FerriteBead_Small", "60R@100MHz", "Inductor_SMD:L_0603_1608Metric",
            {"1": "+3V3_A", "2": "VDDA_A"}, mpn="BLM18AG600SN1D")
        C("2.2uF", "VDDA_A", size="0603")
        C("2.2uF", "VDDA_A", size="0603")
        C("100nF", "VDDA_A")
        C("100nF", "VDDA_A")
        C("2.2uF", "VREFHI_A_A", size="0603")
        C("2.2uF", "VREFHI_BC_A", size="0603")
        # after the supply caps, so the caps get their pins first; the series R then lands at X1 (pin 69)
        R("33", "X1_A_S", "X1_A", note="Series termination at the oscillator output, next to X1")
        add("U", "icarus:SIT2024B", "SiT2024B 10MHz", "icarus:SOT_SiT1630-S_SIT",
            {"1": "GND", "3": "+3V3_A", "4": "+3V3_A", "5": "X1_A_S"}, mpn="SIT2024BA-S2-XXE-10.000000E",
            note="MEMS oscillator; CH-B uses a quartz crystal instead (clock diversity)")
        C("100nF", "+3V3_A")
    else:
        add("U", "icarus:TMS570LS0432", "TMS570LS0432-Q1", "Package_QFP:LQFP-100_14x14mm_P0.5mm",
            TMS570_B, mpn="TMS5700432BPZQQ1",
            note="Automotive orderable (AEC-Q100). Lockstep Cortex-R4F, ECC flash/RAM, CPU/RAM self-test, ESM")
        # one 100 nF per supply pin (7 x VCC, 4 x VCCIO, VCCP), then bulk; same group so each gets its pin
        for _ in range(7):
            C("100nF", "VCC1V2_B")
        C("10uF", "VCC1V2_B", size="0805")
        for _ in range(5):
            C("100nF", "+3V3_B")
        C("10uF", "+3V3_B", size="0805")
        add("FB", "Device:FerriteBead_Small", "60R@100MHz", "Inductor_SMD:L_0603_1608Metric",
            {"1": "+3V3_B", "2": "VDDA_B"}, mpn="BLM18AG600SN1D", note="VCCAD = ADREFHI: the ADC reference")
        C("2.2uF", "VDDA_B", size="0603")
        C("100nF", "VDDA_B")
        add("Y", "Device:Crystal_GND24", "16MHz", "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
            {"1": "OSCIN_B", "3": "OSCOUT_B", "2": "KELVIN_B", "4": "KELVIN_B"},
            note="16 MHz fundamental, AEC-Q200 grade (part TBD; TMS570 accepts 5-20 MHz). Case to KELVIN_GND")
        C("12pF", "OSCIN_B", "KELVIN_B", note="Load caps: values from the crystal vendor (SPNS186C Fig. 6-4). "
                                              "KELVIN_GND connects to nothing else (Note B)")
        C("12pF", "OSCOUT_B", "KELVIN_B")

    group(f"Reset, window watchdog {ch}, hardware enable gate {ch}")
    rst = "XRS_A" if ch == "A" else "PORRST_B"
    R("2.2k", f"+3V3_{ch}", rst, note="" if ch == "A" else "nPORRST: TPS3850 RESET (3V3) wired-AND TPS3702 (1V2)")
    C("10nF", rst)
    add("U", "icarus:TPS3850", "TPS3850G33-Q1", "Package_SON:VSON-10-1EP_3x3mm_P0.5mm_EP1.65x2.4mm",
        {"1": f"+3V3_{ch}", "2": f"CWD_{ch}", "3": f"+3V3_{ch}", "5": "GND", "6": f"+3V3_{ch}",
         "7": f"WDI_{ch}", "8": f"SUP_OK_{ch}", "9": rst, "10": f"+3V3_{ch}", "11": "GND"},
        mpn="TPS3850G33QDRCRQ1", note="Window watchdog + 3V3 OV/UV on SENSE. CRST open = factory reset delay")
    R("10k", f"+3V3_{ch}", f"CWD_{ch}")
    C("100nF", f"+3V3_{ch}")
    R("10k", f"+3V3_{ch}", f"SUP_OK_{ch}", note="Wired-AND: WDO + TPS3702 OV/UV")
    R("1k", f"SUP_OK_{ch}", f"SUPOK_MON_{ch}")
    en_in = "EN_A_GPIO" if ch == "A" else "PWM_EN_B"
    en_out = "EN_A_HW" if ch == "A" else "PUMP_B"
    add("U", "74xGxx:74LVC1G11", "SN74LVC1G11-Q1", "Package_TO_SOT_SMD:SOT-23-6",
        {"1": en_in, "2": "GND", "3": f"ERRSTS_{ch}", "4": en_out, "5": f"+3V3_{ch}", "6": f"SUP_OK_{ch}"},
        mpn="SN74LVC1G11QDBVRQ1",
        note="EN = GPIO AND ERRORSTS AND SUP_OK (SM-03)" if ch == "A" else "EN = PWM AND ESM nERROR AND SUP_OK (SM-03)")
    C("100nF", f"+3V3_{ch}")
    R("100k", en_in, "GND", note="Off while the MCU is in reset")
    R("100k", f"ERRSTS_{ch}", "GND",
      note="GPIO24 = ERRORSTS once configured; low = error" if ch == "A" else "ESM nERROR: low = error (and in reset)")
    if ch == "A":
        R("1k", "EN_A_HW", "ENHW_MON_A")
    else:
        R("10k", "+3V3_B", "NRST_B", note="nRST: external pull-up recommended (SPNS186C Table 4-9)")
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

    group(f"Link to CH-{other}, CAN, debug")
    if ch == "A":
        R("470", "LINK_A2B_TX", "LINK_A2B", note="UART link to CH-B (SCI-B TX, GPIO12). Limits back-powering an "
                                                 "unpowered channel")
    else:
        R("470", "LINK_B2A_TX", "LINK_B2A", note="UART link to CH-A (SCI TX on LINTX). Limits back-powering")
    add("U", "icarus:TCAN3404", "TCAN3404-Q1", "icarus:SOIC8_D_TEX-M",
        {"1": f"CAN_TX_{ch}", "2": "GND", "3": f"+3V3_{ch}", "4": f"CAN_RX_{ch}", "5": "GND", "6": "CAN_L",
         "7": "CAN_H", "8": f"CAN_STB_{ch}"}, mpn="TCAN3404DRQ1")
    C("100nF", f"+3V3_{ch}")
    R("10k", f"CAN_STB_{ch}", "GND", note="Normal mode by default")
    if ch == "A":
        add("J", "Connector:Conn_ARM_JTAG_SWD_10", "JTAG A", "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical_SMD",
            {"1": "+3V3_A", "2": "TMS_A", "3": "GND", "4": "TCK_A", "5": "GND", "6": "TDO_A",
             "8": "TDI_A", "9": "GND", "10": "XRS_A"})
        R("2.2k", "TCK_A", "GND")
        add("J", "Connector_Generic:Conn_01x03", "UART A", "Connector_PinHeader_1.27mm:PinHeader_1x03_P1.27mm_Vertical_SMD_Pin1Left",
            {"1": "SCI_TX_A", "2": "SCI_RX_A", "3": "GND"})
    else:
        # TI cTI 20-pin (XDS110 native). TMS570 has no EMU pins; nTRST has an internal pull-down, so the TAP
        # stays in reset without a debugger. Pinout to re-check against the XDS110 user guide (debug only).
        add("J", "Connector_Generic:Conn_02x10_Odd_Even", "JTAG B (TI cTI-20)",
            "Connector_PinHeader_1.27mm:PinHeader_2x10_P1.27mm_Vertical_SMD",
            {"1": "TMS_B", "2": "TRST_B", "3": "TDI_B", "4": "GND", "5": "+3V3_B", "7": "TDO_B", "8": "GND",
             "9": "RTCK_B", "10": "GND", "11": "TCK_B", "12": "GND", "15": "NRST_B", "16": "GND", "20": "GND"})
        R("2.2k", "TCK_B", "GND")

    group(f"Indicators {ch}: heartbeat, fault, CAN activity, {'engaged' if ch == 'A' else 'permissive'}, supervisor")
    pin_of = {"A": ("GPIO34", "GPIO5", "GPIO9", "GPIO10"), "B": ("N2HET10", "N2HET12", "N2HET14", "N2HET16")}[ch]
    R("1k", f"LED_{ch}", f"LEDA_{ch}", note=f"{pin_of[0]}: heartbeat")
    add("D", "Device:LED", "LED green", "LED_SMD:LED_0603_1608Metric", {"1": "GND", "2": f"LEDA_{ch}"})
    led(f"LEDFLT_{ch}", "GND", "1k", "red", f"FLT_{ch}", note=f"{pin_of[1]}: latched fault / SAFE state")
    led(f"LEDCAN_{ch}", "GND", "1k", "yellow", f"CAN_{ch}", note=f"{pin_of[2]}: pulsed per CAN frame, stretched to 20 ms")
    if ch == "A":
        led("LEDENG_A", "GND", "1k", "orange", "ENG_A", note=f"{pin_of[3]}: overlay engaged")
    else:
        led("LEDPRM_B", "GND", "1k", "orange", "PRM_B", note=f"{pin_of[3]}: CH-B grants the low-side enable")
    led(f"+3V3_{ch}", f"SUP_OK_{ch}", "2.2k", "red", f"SUP_{ch}",
        note="Hardware: lit while the watchdog or the 5 V window holds SUP_OK low (works with the MCU in reset). "
             "A shorted LED adds 2.2k of pull-up; the open-drain outputs still sink it (<2 mA)")

# =============================================================================
sheet("overlay", "Overlay path (CH-A analog, +5V0_A)",
      "EPS gets OVL = RAW + (DELTA - VMID). DELTA = VMID + k(DAC - VMID), k = 10k/92k = 0.109 -> "
      "Delta_max = +/-0.18 V (TBD-DATA, SM-05). One DAC + inverter makes VT6 = -Delta (SM-06). "
      "Summing resistors 0.1 %.")
group("Reference, Delta command and 1 MOhm sensor-side taps")
R("10k", "+3V3_A", "VMIDD_A")
R("10k", "VMIDD_A", "GND")
C("100nF", "VMIDD_A")
R("10k", "DACA_OUT", "DACF_A", note="SM-07 slew limit, tau = 10 ms (TBD-DATA)")
C("1uF", "DACF_A", size="0603")
R("1M", "VT3_SNS", "RAW3IN_A", note="FSR-14: >= 1 MOhm on every car-side tap")
C("100pF", "RAW3IN_A")
R("1M", "VT6_SNS", "RAW6IN_A")
C("100pF", "RAW6IN_A")
opamp_quad([follower("VMIDD_A", "VMID_A"), follower("DACF_A", "DCMD_A"), follower("RAW3IN_A", "RAW3_A"),
            follower("RAW6IN_A", "RAW6_A")], "+5V0_A")

# SM-23 (rev 0.4): speed-dependent authority in hardware. Above v_switch (TBD-DATA, ~40 mph) the CH-C
# speed stage (guard sheet) pulls SPEED_HI_N low and the analog switch feeds the summers a half-amplitude
# command: DCMD_HALF = VMID + 0.5 (DCMD - VMID). Unpowered, floating or stuck-low control = HALF (safe).
# Firmware (FSR-17) culls Delta further with its own speed table; CH-B's expected Delta uses the same table.
group("Speed-dependent authority (SM-23): half-amplitude command above v_switch")
R("10k 0.1%", "DCMD_A", "DHALFD_A")
R("10k 0.1%", "DHALFD_A", "VMID_A")
opamp_quad([follower("DHALFD_A", "DCMD_HALF"), None, None, None], "+5V0_A")
add("U", "Analog_Switch:TS5A3159DBV", "TS5A3159-Q1", "Package_TO_SOT_SMD:SOT-23-6",
    units={1: {"1": "DCMD_A", "3": "DCMD_HALF", "4": "DCMD_SEL", "6": "SPDSW_IN"}, 2: {"2": "GND", "5": "+5V0_A"}},
    mpn="TS5A3159QDBVRQ1",
    note="DBV pins (SCDS201 Pin Functions): 1 NO, 2 GND, 3 NC, 4 COM, 5 V+, 6 IN. IN low -> COM = NC = HALF "
         "(high speed, floating, unpowered); IN high -> NO = FULL only when the speed stage says slow")
C("100nF", "+5V0_A")
R("1k", "SPEED_HI_N", "SPDSW_IN", note="Cross-channel: 3V3_B logic into a 5V_A switch")
R("100k", "SPDSW_IN", "GND", note="Default = HALF")
R("100k", "DCMD_SEL", "VMID_A", note="Switch open (fault) = zero overlay")
R("1k", "SPEED_HI_N", "SPDHI_A")
R("100k", "SPDHI_A", "GND")

# SM-05 (rev 0.3): the authority limit k lives INSIDE each summer as a resistor ratio, so no op-amp
# upstream of the summer can push more than k x (rail - VMID) = 0.37 V into the EPS line. Before, a
# railed DELTA_P follower could apply ~3.3 V. k = 1.1k / 10k = 0.110 -> Delta_max = +/-0.18 V (TBD-DATA).
# Antisymmetry (SM-06) comes from the two summers using the same four 0.1 % values, mirrored.
group("Resistor-weighted summers: OVL3 = RAW3 + k(DCMD - VMID), OVL6 = RAW6 - k(DCMD - VMID), k = 0.110")
opamp_quad([("SUM3P_A", "SUM3N_A", "OVL3"), ("SUM6P_A", "SUM6N_A", "OVL6"), follower("EPS3IN_A", "EPS3_A"),
            follower("EPS6IN_A", "EPS6_A")], "+5V0_A")
R("10k 0.1%", "RAW3_A", "SUM3P_A")
R("90.9k 0.1%", "DCMD_SEL", "SUM3P_A", note="= 10k / k")
R("10k 0.1%", "VMID_A", "SUM3N_A")
R("1.1k 0.1%", "SUM3N_A", "OVL3", note="= k x 10k")
R("10k 0.1%", "RAW6_A", "SUM6P_A")
R("90.9k 0.1%", "VMID_A", "SUM6P_A")
R("10k 0.1%", "DCMD_SEL", "SUM6N_A")
R("1.1k 0.1%", "SUM6N_A", "OVL6")
R("100", "OVL3", "OVL3_OUT")
R("100", "OVL6", "OVL6_OUT")

group("EPS-side readback taps and CH-A ADC scaling (x0.667)")
R("1M", "VT3_EPS", "EPS3IN_A")
C("100pF", "EPS3IN_A")
R("1M", "VT6_EPS", "EPS6IN_A")
C("100pF", "EPS6IN_A")
for src, dst in (("RAW3_A", "RAW3M_A"), ("RAW6_A", "RAW6M_A"), ("EPS3_A", "EPS3M_A"), ("EPS6_A", "EPS6M_A"),
                 ("DCMD_SEL", "DELTAM_A")):
    divider(src, dst, "10k", "20k")
divider("+5V0_A_SRC", "V5SM_A", "10k", "10k", cap="10nF", note="Buck A output ahead of the ideal diode")
divider("K1A_COM", "K1ACOM_M", "1M", "2M", cap="10nF", note="Relay K1 proof-test readback (SM-09)")
divider("K1B_COM", "K1BCOM_M", "1M", "2M", cap="10nF")

# =============================================================================
sheet("guard", "CH-C analog guard (no software, +5V0_B)",
      "Trips when the overlay actually applied (VT_EPS - VT_SNS) differs from CH-B's own expected Delta by "
      "> 0.25 V on either line (R2 / SM-20), or the VT3+VT6 sum moves > 0.10 V (TBD-DATA). A trip latches "
      "until key-off and cuts every coil (SM-15). GUARD_TEST forces a trip for the key-on self-test (SM-17). "
      "VMID_B and the ladder are ratiometric to +3V3_B, the CH-B DAC reference.")
group("Buffers (1 MOhm from the car side) and VMID_B")
R("10k", "+3V3_B", "VMIDD_B")
R("10k", "VMIDD_B", "GND")
C("100nF", "VMIDD_B")
for src, node in (("VT3_SNS", "RAW3IN_B"), ("VT6_SNS", "RAW6IN_B"), ("VT3_EPS", "EPS3IN_B"), ("VT6_EPS", "EPS6IN_B")):
    R("1M", src, node)
    C("100pF", node)
opamp_quad([follower("VMIDD_B", "VMID_B"), follower("RAW3IN_B", "RAW3_B"), follower("RAW6IN_B", "RAW6_B"),
            follower("EPS3IN_B", "EPS3_B")], "+5V0_B")

group("Expected-Delta reference from CH-B's own DAC (R2 / SM-20): REF3 = VMID - Delta_exp, REF6 = VMID + Delta_exp")
R("10k", "REFPWM_B", "DACF_B", note="Same slew filter as CH-A (SM-07). rev 0.7: CH-B (TMS570) has no DAC: N2HET[6] "
                                     "PWM at ~100 kHz, 3.3 V; this RC leaves < 1 mV ripple, x0.109 after the attenuator")
C("1uF", "DACF_B", size="0603")
R("82k", "DACF_B", "REFIN_B", note="Same attenuator k as CH-A, so |Delta_exp| is also bounded in hardware")
R("10k", "REFIN_B", "VMID_B")
opamp_quad([follower("REFIN_B", "REF3_B"), ("VMID_B", "REFN_B", "REF6_B"), None, None], "+5V0_B")
R("10k 0.1%", "REF3_B", "REFN_B")
R("10k 0.1%", "REFN_B", "REF6_B")
divider("REF3_B", "REF3M_B", "10k", "20k", note="CH-B reads back its reference (latent check of SM-20)")

group("Difference amps: D3 = EPS3 - RAW3 + REF3 (= VMID when the applied Delta matches the expected), D6 likewise, "
      "S = (EPS3+EPS6) - (RAW3+RAW6) + VMID")
opamp_quad([follower("EPS6IN_B", "EPS6_B"), ("D3P", "D3N", "D3"), ("D6P", "D6N", "D6"), ("SP", "SN", "SSUM")],
           "+5V0_B")
for eps, raw, ref, p, n, out in (("EPS3_B", "RAW3_B", "REF3_B", "D3P", "D3N", "D3"),
                                 ("EPS6_B", "RAW6_B", "REF6_B", "D6P", "D6N", "D6")):
    R("10k 0.1%", eps, p)
    R("10k 0.1%", ref, p)
    R("10k 0.1%", raw, n)
    R("10k 0.1%", n, out)
R("10k 0.1%", "EPS3_B", "SP")
R("10k 0.1%", "EPS6_B", "SP")
R("10k 0.1%", "VMID_B", "SP")
R("10k 0.1%", "RAW3_B", "SN")
R("10k 0.1%", "RAW6_B", "SN")
R("10k 0.1%", "SN", "SSUM")

group("Window thresholds (ladder from +3V3_B) and comparators (TLV1704-Q1, open collector)")
R("28k", "+3V3_B", "TH_HD", note="VMID 1.65 V: TH_HD +0.25, TH_HS +0.10, TH_LS -0.10, TH_LD -0.25 V (TBD-DATA)")
R("3.01k", "TH_HD", "TH_HS")
R("4.02k", "TH_HS", "TH_LS")
R("3.01k", "TH_LS", "TH_LD")
R("28k", "TH_LD", "LADB", note="R3: ladder bottom through a normally-on FET; opening it lifts every threshold")
for n in ("TH_HD", "TH_HS", "TH_LS", "TH_LD"):
    C("10nF", n)
nmos("GTEST_LO_N", "LADB", note="R3 / SM-17 low-side test: CH-B drives low -> all thresholds rise -> D3, D6, sum "
                                "low windows trip. Gate pulled up, so a stuck-low GPIO only causes a safe trip")
R("100k", "+3V3_B", "GTEST_LO_N")


def comparator_quad(nets4):
    units = {}
    pins = {1: ("5", "4", "2"), 2: ("7", "6", "1"), 3: ("11", "10", "13"), 4: ("9", "8", "14")}
    for u, n in enumerate(nets4, start=1):
        p, m, o = pins[u]
        units[u] = {p: "+5V0_B", m: "GND"} if n is None else {p: n[0], m: n[1], o: n[2] if len(n) > 2 else "TRIP_N"}
    units[5] = {"3": "+5V0_B", "12": "GND"}
    add("U", "Comparator:LM339", "TLV1704-Q1", "Package_SO:TSSOP-14_4.4x5mm_P0.65mm", units=units,
        mpn="TLV1704AQPWRQ1", note="LM339 symbol: TLV1704-Q1 PW pinout is identical (SLOS890 Pin Functions)")
    C("100nF", "+5V0_B")


comparator_quad([("TH_HD", "D3"), ("D3", "TH_LD"), ("TH_HD", "D6"), ("D6", "TH_LD")])
comparator_quad([("TH_HS", "SSUM"), ("SSUM", "TH_LS"), ("VSP_C", "SQREF", "VSP_SQ"), ("SPDREF", "VSPF", "SPEED_HI_N")])
R("10k", "+5V0_B", "TRIP_N")
C("1nF", "TRIP_N")

group("Speed stage (SM-23, no software): VSP -> squarer -> 2.2 ms one-shot -> average -> comparator")
R("100k", "VSP_IN", "VSP_C", note="Own divider, independent of the CH-A/CH-B VSP inputs")
R("150k", "VSP_C", "GND")
C("1nF", "VSP_C")
add("D", "Diode:1N4148WS", "1N4148WS-Q", "Diode_SMD:D_SOD-323", {"1": "+3V3_B", "2": "VSP_C"})
R("22k", "+3V3_B", "SQREF", note="Squarer threshold 1.0 V")
R("10k", "SQREF", "GND")
R("10k", "+3V3_B", "VSP_SQ", note="Open-collector pull-up: clean 3V3 pulses")
add("U", "74xGxx:74LVC1G123", "SN74LVC1G123", "Package_SO:SSOP-8_2.95x2.8mm_P0.65mm",
    units={1: {"1": "GND", "2": "VSP_SQ", "3": "+3V3_B", "5": "VSP_1S", "6": "OS_C", "7": "OS_RC"},
           2: {"4": "GND", "8": "+3V3_B"}},
    mpn="SN74LVC1G123DCTR", note="Retriggerable one-shot, tW ~ 22k x 100nF = 2.2 ms per VSP pulse. NOT automotive: "
                                 "TI has no -Q1 (the QDCTRQ1 code used before does not exist). Candidates: TI "
                                 "TPUL1G313-Q1 (TI calls it pin-to-pin; pulse maths differ) or Nexperia "
                                 "74LVC1G123DP-Q100H (TSSOP-8). Verify pinout before swapping")
C("100nF", "+3V3_B")
C("100nF", "OS_C", "OS_RC")
R("22k", "+3V3_B", "OS_RC")
R("100k", "VSP_1S", "VSPF", note="Average: V = 3.3 V x 2.2 ms x f_VSP; ~0.36 V at 40 mph (TBD-DATA)")
C("1uF", "VSPF", size="0603")
R("82k", "+3V3_B", "SPDREF", note="v_switch reference 0.36 V (TBD-DATA)")
R("10k", "SPDREF", "GND")
R("1M", "SPEED_HI_N", "SPDREF", note="Hysteresis ~ +/-30 mV (~3 mph)")
R("10k", "+3V3_B", "SPEED_HI_N", note="Low = speed above v_switch -> half authority")
R("1k", "SPEED_HI_N", "SPDHI_B")

group("Trip latch, self-test and reset, guard gate, status to both MCUs")
add("U", "74xGxx:74AUC1G74", "SN74LVC1G74-Q1", "Package_SO:VSSOP-8_2.3x2mm_P0.5mm",
    {"1": "GND", "2": "GND", "3": "GUARD_OKQ", "4": "GND", "5": "GTRIP", "6": "GCLR_N", "7": "TRIP_N",
     "8": "+5V0_B"}, mpn="SN74LVC1G74QDCURQ1", note="74x1G74 pinout (DCU): PRE = trip, CLR = power-on/reset. "
                                                  "Confirm the TI -Q1 orderable. Nexperia 74LVC1G74DC-Q100H is AEC-Q100 "
                                                  "but its datasheet shows RD on pin 3 and Q-bar on pin 6 (TI: Q-bar 3, "
                                                  "CLR 6): NOT a drop-in, it would miswire the trip latch")
C("100nF", "+5V0_B")
R("100k", "+5V0_B", "GCLR_N", note="Power-on clear: key-off/on resets the latch")
C("1uF", "GCLR_N", size="0603")
add("U", "74xGxx:74LVC1G08", "SN74LVC1G08-Q1", "Package_TO_SOT_SMD:SOT-23-5",
    {"1": "GUARD_RST", "2": "GUARD_RST_A", "3": "GND", "4": "GRST_2OO2", "5": "+3V3_B"},
    mpn="SN74LVC1G08QDBVRQ1",
    note="R7: clearing the trip latch needs BOTH MCUs (2oo2), like acting does. INIT only, by firmware rule")
C("100nF", "+3V3_B")
R("1k", "GRSTA_TX", "GUARD_RST_A", note="Cross-channel series R (CH-A GPIO into a CH-B-powered gate)")
R("100k", "GUARD_RST_A", "GND")
nmos("GRST_2OO2", "GCLR_N")
R("100k", "GUARD_RST", "GND")
nmos("GUARD_TEST", "TESTD", note="SM-17 high-side test: collapses the ladder -> D3, D6, sum high windows trip")
R("100k", "GUARD_TEST", "GND")
R("1k", "TESTD", "TH_HD")
R("1k", "GUARD_OKQ", "GUARD_GATE")
R("100k", "GUARD_GATE", "GND")
divider("GTRIP", "GTRIP_A", "10k", "20k", cap=None)
divider("GTRIP", "GTRIP_B", "10k", "20k", cap=None)

group("Indicators: guard tripped")
led("GTRIP", "GND", "2.2k", "red", "GTRIP", note="On the latch Q output; the coil gate uses Q-bar, so the LED can't load it")

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
    R("24.9k", f"CL{k}", "GND", note="I_CL = 1.233 V x 2000 / R_CL = 99 mA (datasheet eq. 2); coil draws ~12 mA")
    R("10k", f"CSR{k}", "GND", note="Current-sense load: V_CS = I_coil / K x 10k (K per datasheet; TBD scale)")
    # ADC filter and clamp: drawn here, placed at the CH-A ADC pin (the cap is the S/H charge reservoir,
    # SPRS945); the long run is then the 10k-loaded CSR node, filtered at the pin
    place_with("MCU A", [
        R("1k", f"CSR{k}", f"CSK{k}_A"),
        C("10nF", f"CSK{k}_A"),
        add("D", "Diode:1N4148WS", "1N4148WS-Q", "Diode_SMD:D_SOD-323", {"1": "+3V3_A", "2": f"CSK{k}_A"},
            note="Clamp: a shorted coil at the 99 mA limit would push V_CS above the ADC range")])
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
R("100k", "VT3_SNS", "VT3_EPS", note="R4: if a contact opens, the EPS input follows the sensor instead of floating "
                                      "(no latching DTC). Loads the 100 R overlay output by 0.1 %. TBD: EPS input impedance")
R("100k", "VT6_SNS", "VT6_EPS")

group("CH-C guard FET (common coil return)")
nmos("GUARD_GATE", "GNODE", note="Off = both coils dead (SM-15)")
divider("GNODE", "GNODEM_B", "100k", "22k", cap="10nF")

group("Indicators: relay energized (across each coil, so it needs high side AND both low sides)")
for k in (1, 2):
    add("D", "Diode:1N4148WS", "1N4148WS-Q", "Diode_SMD:D_SOD-323", {"2": f"COIL{k}_HI", "1": f"LI_K{k}D"},
        note="Blocks the ~19 V release flyback so the LED can't shunt the Zener clamp (release time, SM-12)")
    led(f"LI_K{k}D", f"COIL{k}_LO", "10k", "blue", f"K{k}",
        note="~0.9 mA; adds to the TPS1H100 current-sense reading (coil ~12 mA)")

# =============================================================================
sheet("io", "Vehicle connector, CAN bus, inputs and monitors",
      "Every car-facing input has its own divider per channel (no shared resistor between CH-A and CH-B). "
      "Torque-sensor and Vcc2 taps are 1 MOhm (FSR-14).")
group("Vehicle connector (Y-harness at EPS connector B; part TBD, TE MCP family as throttle J2)")
# R1 (docs/06 F1): torque lines in their own block, GND on both outer sides and between VT3 and VT6,
# so the only adjacent short a torque pin can have is GND, a static tap, or its own SNS/EPS partner.
J1 = {"1": "VBAT_IN", "2": "GND", "3": "IG1_IN", "4": "CAN_H", "5": "CAN_L", "6": "GND", "7": "VT3_EPS",
      "8": "VT3_SNS", "9": "GND", "10": "VT6_SNS", "11": "VT6_EPS", "12": "TSGND_TAP", "13": "VCC2_TAP",
      "14": "WLP_IN", "15": "RT_IN", "16": "VSP_IN", "17": "NEP_IN", "18": "DIS_NO_IN", "19": "DIS_NC_IN",
      "20": "IND_OUT"}
add("J", "Connector_Generic:Conn_01x20", "Vehicle harness", "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical",
    J1, note="Placeholder footprint. 7/11 = EPS ECU B10/B8, 8/10 = torque sensor side. Pin order is safety-relevant (R1)")

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

# =============================================================================
sheet("backup", "1S LiPo backup: charger, boost, ideal-diode ORing into both 5 V rails",
      "Keeps both MCUs, CAN and the analog guard alive through cranking / IMA-restart dips so a dropout is "
      "reported (SG4) and recovery is clean. The relay coils are NOT backed up on purpose: 12 V loss = "
      "pass-through. Charger input is +5V0_A_SRC (ahead of the ideal diode), so backup power cannot loop "
      "back into the charger. Pack must have its own PCM; charge window 0-45 C via the on-board NTC.")
group("Battery connector, PTC, monitors")
add("J", "Connector_Generic:Conn_01x02", "1S LiPo (JST PH)", "Connector_JST:JST_PH_S2B-PH-SM4-TB_1x02-1MP_P2.00mm_Horizontal",
    {"1": "BAT_P_IN", "2": "GND"}, note="Standard 1S LiPo lead. Pack with PCM, 500 mAh or more")
add("F", "Device:Polyfuse", "1.1A PTC", "Fuse:Fuse_1812_4532Metric", {"1": "BAT_P_IN", "2": "VLI"},
    mpn="MINIASMDC110F/16-2", note="Littelfuse automotive, AEC-Q200, 16 V (was 1812L100/12, not AEC-Q200)")
C("10uF", "VLI", size="0805")
divider("VLI", "VLIM_A", "100k", "100k", cap="100nF", note="Cell voltage to both MCUs")
divider("VLI", "VLIM_B", "100k", "100k", cap="100nF")
group("Charger MCP73833 (1S Li-ion, ~210 mA, thermistor window)")
add("U", "Battery_Management:MCP73833-xxx-MF", "MCP73833", "Package_DFN_QFN:DFN-10-1EP_3x3mm_P0.5mm_EP1.58x2.35mm",
    {"1": "+5V0_A_SRC", "2": "+5V0_A_SRC", "3": "CHG_STAT", "4": "CHG_DONE_N", "5": "GND", "6": "CHG_PROG",
     "7": "CHG_PG_N", "8": "CHG_THERM",
     "9": "VLI", "10": "VLI", "11": "GND"}, mpn="MCP73833T-AMI/MF",
    note="Not AEC-Q100; an automotive 1S charger with a TS input is the alternative at review")
C("10uF", "+5V0_A_SRC", size="0805")
C("4.7uF", "VLI", size="0603", note="MCP73833 output cap at VBAT (datasheet: 4.7 uF ceramic)")
R("4.7k", "CHG_PROG", "GND", note="I_reg = 1000 V / R = 213 mA")
add("R", "Device:Thermistor_NTC", "NTC 10k", "Resistor_SMD:R_0603_1608Metric", {"1": "CHG_THERM", "2": "GND"},
    mpn="NCU18XH103F6SRB", note="Near the battery lead; charge inhibited outside 0-50 C. Murata NCU18 is the "
                                "automotive series (NCP18 is consumer): confirm the exact code at order")
R("100k", "+3V3_A", "CHG_STAT")
R("1k", "CHG_STAT", "CHG_STAT_A")
group("Boost TPS61092 (fixed 5.0 V) and low-battery flag")
add("U", "Regulator_Switching:TPS61092", "TPS61092", "Package_DFN_QFN:Texas_RSA_VQFN-16-1EP_4x4mm_P0.65mm_EP2.7x2.7mm_ThermalVias",
    {"1": "VBK5", "3": "SWB", "4": "SWB", "5": "GND", "6": "GND", "7": "GND", "8": "VLI", "9": "LBI", "10": "GND",
     "11": "VLI", "12": "LBO_N", "13": "GND", "14": "VBK5", "15": "VBK5", "16": "VBK5", "17": "GND"}, mpn="TPS61092RSAR",
    note="SYNC = GND: power-save mode, tens of uA at no load. EN = VLI: always armed")
add("L", "Device:L", "6.8uH", "Inductor_SMD:L_Bourns_SRP7028A_7.3x6.6mm", {"1": "VLI", "2": "SWB"}, mpn="SRP7028A-6R8M")
C("10uF", "VLI", size="0805")
C("22uF 16V", "VBK5", size="1206")
C("22uF 16V", "VBK5", size="1206")
R("100k", "VLI", "LBI", note="LBI trips at 0.5 V -> cell 3.0 V")
R("20k", "LBI", "GND")
R("100k", "+3V3_B", "LBO_N", note="Open drain: low = cell below 3.0 V")
group("Boost overvoltage cutoff (SM-24): a boost fault must not reach both 5 V rails")
add("U", "Power_Supervisor:TPS3702", "TPS3702CX50", "Package_TO_SOT_SMD:SOT-23-6",
    {"2": "GND", "3": "VBK5", "4": "VBK5", "5": "VBK5", "6": "BKOV_N"}, mpn="TPS3702CX50QDDCRQ1",
    note="OV at 5.20 V. UV output unused (a low cell is handled by LBO)")
C("100nF", "VBK5")
R("100k", "VBK5", "BKOV_N")
nmos("BKOV_N", "BK_CE_N", note="Inverter: normal -> CE low (enabled); OV, or FET/supervisor open -> CE high (backup off)")
R("100k", "VBK5", "BK_CE_N")

group("Ideal-diode ORing into +5V0_A and +5V0_B (LM66100)")
for rail in ("+5V0_A", "+5V0_B"):
    add("U", "Power_Management:LM66100DCK", "LM66100-Q1", "Package_TO_SOT_SMD:SOT-363_SC-70-6",
        {"1": "VBK5", "2": "GND", "3": "BK_CE_N", "6": rail}, mpn="LM66100QDCKRQ1",
        note="Conducts only when the buck sags below the boost; blocks reverse. CE from the SM-24 cutoff")
    C("100nF", "VBK5")
    C("1uF", rail, size="0603", note="Output cap at the diode (LM66100 application circuit; the rail's bulk is far away)")

group("Indicators: charger and backup cell (no LED on VLI / VBK5, so the cell isn't drained)")
led("+3V3_A", "CHG_STAT", "2.2k", "yellow", "CHG", note="STAT1 low = charging. From +3V3_A so CHG_STAT_A stays in GPIO range")
led("+5V0_A_SRC", "CHG_DONE_N", "2.2k", "green", "CHGD", note="STAT2 low = charge complete")
led("+5V0_A_SRC", "CHG_PG_N", "2.2k", "green", "CHGPG", note="PG low = charger input good")
led("+3V3_B", "LBO_N", "2.2k", "red", "LBO", note="Boost LBO: cell below 3.0 V")

# =============================================================================
sheet("test", "Test points (bench only): incoming, outgoing, measured",
      "1.5 mm SMD pads, no parts fitted. Each pad sits on its net (a probe point, never in series), "
      "in three labelled clusters on the PCB: signals coming in from the car, signals Icarus drives out, "
      "and internal measurement nodes. Each cluster has its own GND pad for the scope ground. "
      "Torque-line pads are for the bench; don't probe VT3/VT6 while driving.")
TEST_POINTS = (
    ("Test points: incoming", ("GND", "VBAT_P", "IG1_A", "VT3_SNS", "VT6_SNS", "VSP_A", "NEP_A", "CAN_H", "CAN_L")),
    ("Test points: outgoing", ("GND", "VT3_EPS", "VT6_EPS", "OVL3_OUT", "OVL6_OUT", "DCMD_SEL", "COIL1_HI",
                               "COIL2_HI", "GNODE", "IND_OUT")),
    ("Test points: measured", ("GND", "+5V0_A", "+5V0_B", "+3V3_A", "+3V3_B", "VMID_A", "VMID_B", "REF3_B", "D3",
                               "D6", "SSUM", "GTRIP", "SPEED_HI_N", "VSPF", "EN_A_HW", "GATE_B", "SUP_OK_A",
                               "SUP_OK_B")),
)
for title, nets in TEST_POINTS:
    group(title)
    for n in nets:
        add("TP", "Connector:TestPoint", n, "TestPoint:TestPoint_Pad_D1.5mm", {"1": n})

# Rails that need a PWR_FLAG (driven by regulators or the protection FET).
POWER_FLAGS = ["VBAT_P", "+5V0_A", "+5V0_B", "+5V0_A_SRC", "VLI", "VBK5", "VDD12_A", "VDDA_A", "VDDA_B",
               "VREFHI_A_A", "VREFHI_BC_A", "GND"]
