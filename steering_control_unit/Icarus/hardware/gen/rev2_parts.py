"""Rev 2.0 new parts: pin maps, footprints and external-component values, from the datasheets.

Every entry cites the document and section it came from (downloaded 2026-11-15, read with PyMuPDF; drawings
read from rendered pages). make_sch.py / design.py rev 2.0 import this instead of re-typing pins, and
check_rev2_parts() guards the maps against the mistakes HANDOFF.md lists (duplicate pins, missing pins).

Sources
  TPS65381A-Q1   TI SLVSDJ1A (Jul 2016, rev May 2017), Pin Functions p.5-6, Electrical Characteristics 4.5,
                 5.3.1 VDD6, 5.3.4 VDD1, 5.3.6 charge pump, 5.3.7 wake-up / POST_RUN, 5.3.8 reset extension,
                 5.4.16 MCU ESM, package DAP0032B (4222438/A) land pattern p.119
  TCAN1044AV-Q1  TI TCAN1044A-Q1 family datasheet, section 5 Pin Configuration and Functions (D, SOIC-8)
  V23079 (P2)    TE "Axicom P2 Relay V23079" datasheet: coil table p.3, terminal assignment + PCB layout p.5
  TX2SA          Panasonic TX series catalog (Rev 2010, DigiKey-hosted): coil table p.3, schematic + SA mounting
                 pad p.6
"""

# --------------------------------------------------------------------------- TPS65381A-Q1 (HTSSOP-32 DAP)
TPS65381A_PINS = {
    "1": ("VBAT_SAFING", "pwr", "battery for VMON + BG2, reverse protected; tie to VBATP"),
    "2": ("VCP", "pwr", "charge-pump output; 100 nF Cstore to VBATP"),
    "3": ("CP1", "pwr", "pump cap high side; 10 nF CP1-CP2"),
    "4": ("CP2", "pwr", "pump cap low side"),
    "5": ("PGND", "gnd", ""),
    "6": ("NRES", "out_od", "cold reset to MCU, active low, internal pull-up"),
    "7": ("DIAG_OUT", "out", "AMUX/DMUX diagnostic output -> MCU ADC"),
    "8": ("NCS", "in", "SPI CS, active low, internal pull-up"),
    "9": ("SDI", "in", "SPI data in, internal pull-down"),
    "10": ("SDO", "out", "SPI data out"),
    "11": ("SCLK", "in", "SPI clock, internal pull-down"),
    "12": ("RSTEXT", "in", "reset extension resistor to GND (22k -> 4.5 ms typ, max 120k)"),
    "13": ("ERROR/WDI", "in", "MCU error input (ESM, TMS570 mode = low-pulse detect); edge triggered"),
    "14": ("CANWU", "in", "CAN wake input, internal pull-down"),
    "15": ("VSFB1", "in", "sensor-supply feedback"),
    "16": ("VSIN", "pwr", "sensor-supply input"),
    "17": ("VSOUT1", "pwr", "sensor-supply output (0.5-10 uF)"),
    "18": ("VTRACK1", "in", "sensor-supply tracking input, internal pull-down"),
    "19": ("GND", "gnd", "analog ground"),
    "20": ("VDD5", "pwr", "5 V LDO, 300 mA, 1-5 uF"),
    "21": ("VDD3/5", "pwr", "3.3 V (SEL open) or 5 V LDO, 300 mA, 1-5 uF"),
    "22": ("VDDIO", "pwr", "I/O supply for MCU-facing pins (tie to VDD3/5 = 3.3 V)"),
    "23": ("GND", "gnd", "analog ground"),
    "24": ("VDD1_SENSE", "in", "VDD1 feedback, Vref 0.8 V (0.792-0.808)"),
    "25": ("PGND", "gnd", ""),
    "26": ("VDD1_G", "out", "gate of external VDD1 NMOS; 100k-1M gate-source"),
    "27": ("VDD6", "pwr", "6 V buck feedback + supply of VDD5, VDD3/5 (and VDD1 drain)"),
    "28": ("SDN6", "pwr", "buck switch node: inductor + external Schottky to PGND"),
    "29": ("VBATP", "pwr", "main supply 5.8-36 V operating, 40 V abs max, must be reverse protected"),
    "30": ("IGN", "in", "ignition wake, 2-3 V threshold, deglitched, internal pull-down; POST_RUN latch"),
    "31": ("SEL_VDD3/5", "in", "open = 3.3 V, GND = 5 V; latched at power-up"),
    "32": ("ENDRV", "out_od", "safing-path enable, internal pull-up, open drain"),
    "33": ("EP", "gnd", "thermal pad: vias to GND plane, tie to GND and PGND"),
}
TPS65381A_FOOTPRINT = dict(
    package="DAP0032B HTSSOP-32 PowerPAD, 11.0 x 6.1 mm body",
    pitch=0.65, pad=(1.5, 0.45), row_span=7.5, thermal_pad=(4.16, 5.72),
    note="Land pattern from SLVSDJ1A p.119 (example board layout). Check the stencil page before release.")

# External parts per PMIC (SLVSDJ1A 4.5 and 5.3). Values are the design choice inside the datasheet range.
TPS65381A_EXTERNALS = {
    "L_VDD6": ("33 uH", "range 22-33 uH; AEC-Q200 shielded, Isat > 1.5 A (VDD6 limit 1.3 A + ripple)"),
    "D_VDD6": ("Schottky 40-60 V, >= 2 A, AEC-Q101", "freewheel diode SDN6 -> PGND (asynchronous buck)"),
    "C_VDD6": ("2x 22 uF 16 V X7R AEC-Q200 + 0.1 ohm ESR resistor", "effective 22-47 uF at 6 V; ESR 100-300 mOhm, "
               "R_ESR ~ L / (15 * C_eff) = 33u / (15 * 22u) = 0.1 ohm"),
    "C_VDD5": ("4.7 uF 10 V X7R", "1-5 uF"),
    "C_VDD35": ("4.7 uF 10 V X7R", "1-5 uF"),
    "C_VDD1": ("10 uF 10 V X7R", "5-40 uF (CH-B only: TMS570 core)"),
    "C_PUMP": ("10 nF 50 V X7R", "CP1-CP2"),
    "C_STORE": ("100 nF 50 V X7R", "VCP-VBATP"),
    "R_RSTEXT": ("22 kOhm", "4.5 ms reset extension"),
    "R_VDD1_TOP": ("10.0 kOhm 0.1 %", "VDD1 = 0.8 * (1 + 10k / 20k) = 1.200 V (TMS570 core 1.14-1.32 V)"),
    "R_VDD1_BOT": ("20.0 kOhm 0.1 %", ""),
    "R_VDD1_GS": ("470 kOhm", "gate-source of the VDD1 NMOS (100k-1M)"),
    "Q_VDD1": ("Logic-level NMOS, AEC-Q101, >= 2 A, DPAK/LFPAK class", "drain from VDD6. Worst case "
               "(6.6 - 1.14) V x 145 mA = 0.8 W (TMS570 LBIST), so it needs copper; no current limit of its own"),
    "C_VBATP": ("100 nF 50 V + 10 uF 50 V X7R", "input decoupling at pin 29"),
    "IGN_NET": ("IG1 -> 10 kOhm series + 100 nF to GND + TVS", "the pin is high-voltage but needs filtering"),
    "R_DIAG_OUT": ("100 kOhm series + 1 nF at the ADC pin", "SLVSDJ1A 5.4.9: at least 100 kOhm series on the "
                   "ADC input filter for the AMUX output"),
    "CANWU": ("tie to GND", "TCAN1044AV-Q1 has no wake output; internal pull-down"),
    "VSOUT1": ("0.47 uF to GND, VSIN to VDD6, VSFB1 to VSOUT1, VTRACK1 to VDD5", "unused: VSOUT1_EN (SENS_CTRL) "
               "stays 0, which is its power-on default. No explicit unused-pin rule found in SLVSDJ1A; confirm "
               "with TI before freezing"),
}
TPS65381A_BEHAVIOUR = {
    "ERROR_mode": "TMS570 mode (ERROR_CFG = 1): detects a low pulse longer than SAFETY_ERR_PWM_L. Both MCUs drive "
                  "an active-low error line (F28004x ERRORSTS: active low, fixed polarity, needs a pull-down; "
                  "TMS570 nERROR: active low), so no inverter",
    "ESM_enable": "MCU ESM is off by default: clear NO_ERROR; only with the watchdog in Q&A mode",
    "ESM_selftest": "In DIAGNOSTIC state: inject an error-pin failure, check ERROR_PIN_FAIL = 1 and ENDRV low, "
                    "then clear (SLVSDJ1A 5.4.16 procedure) -> self-test T11",
    "ENDRV": "Depends on WD_FAIL_CNT, ENABLE_DRV bit and the error monitors; set ENABLE_DRV only with "
             "WD_FAIL_CNT < 5. ENDRV read-back through ENDRV_ERR",
    "post_run": "IGN_PWRL (SAFETY_FUNC_CFG bit 4) is set in DIAGNOSTIC state at boot; IGN low is then ignored "
                "until the MCU clears IGN_PWRL in ACTIVE -> STANDBY. This is the parked post-run window W2: "
                "no external self-hold transistors needed",
    "low_battery": "Full operation 7-36 V; 5.8-7 V functional with VDD6 in dropout; initial power-up needs 5.8 V",
    "BIST": "ABIST at power-up, LBIST on leaving DIAGNOSTIC; either failing -> SAFE state (ENDRV low)",
}

# --------------------------------------------------------------------------- TCAN1044AV-Q1 (SOIC-8, D)
TCAN1044AV_PINS = {
    "1": ("TXD", "in", "integrated pull-up"),
    "2": ("GND", "gnd", ""),
    "3": ("VCC", "pwr", "5 V (4.5-5.5 V) from PMIC VDD5, 100 nF + 4.7 uF"),
    "4": ("RXD", "out", "VIO-level output"),
    "5": ("VIO", "pwr", "1.7-5.5 V logic supply = 3.3 V from PMIC VDD3/5, 100 nF"),
    "6": ("CANL", "bidir", ""),
    "7": ("CANH", "bidir", ""),
    "8": ("STB", "in", "standby, integrated pull-up (standby by default); MCU drives low for normal mode"),
}
TCAN1044AV_FOOTPRINT = "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm"

# --------------------------------------------------------------------------- relays (2 Form C, monostable)
# Same pin numbering and pin positions on both parts: x = 0, 5.08, 7.62, 10.16 mm per row.
RELAY_PINS = {
    "1": ("COIL+", "coil positive"),
    "12": ("COIL-", "coil negative"),
    "4": ("COM1", "pole 1 common"), "3": ("NC1", "pole 1 normally closed"), "5": ("NO1", "pole 1 normally open"),
    "9": ("COM2", "pole 2 common"), "10": ("NC2", "pole 2 normally closed"), "8": ("NO2", "pole 2 normally open"),
}
RELAYS = {
    "K1": dict(mpn="V23079D1003B301", vendor="TE Connectivity (Axicom P2), SMT long terminals",
               coil_ohm=1029, coil_ma=11.7, operate_v=9.0, release_v=1.2, release_ms_max=4.0,
               release_ms_with_diode_max=6.0, contacts="AgNi, gold-covered, bifurcated; min switching 100 uV",
               temp="-40..+85 C", footprint=dict(pitch=2.54, pad=(0.9, 2.85), row_centre_span=8.05,
                                                 note="pads from 5.2 mm inner to 10.9 mm outer (p.5)")),
    "K2": dict(mpn="TX2SA-12V-Z", vendor="Panasonic TX, SA surface-mount, tape-and-reel",
               coil_ohm=1028, coil_ma=11.7, operate_v=9.0, release_v=1.2, release_ms_max=4.0,
               release_ms_with_diode_max=None, contacts="gold-clad twin crossbar",
               temp="-40..+85 C", footprint=dict(pitch=2.54, pad=(1.0, 3.16), row_centre_span=7.24,
                                                 note="suggested mounting pad, top view (p.6)")),
}

# --------------------------------------------------------------------------- checks
def check_rev2_parts():
    assert sorted(map(int, TPS65381A_PINS)) == list(range(1, 34)), "TPS65381A: pins 1-32 + EP, each once"
    assert sorted(map(int, TCAN1044AV_PINS)) == list(range(1, 9))
    assert sorted(map(int, RELAY_PINS)) == [1, 3, 4, 5, 8, 9, 10, 12]
    names = [v[0] for k, v in TPS65381A_PINS.items() if v[1] != "gnd"]
    assert len(names) == len(set(names)), "duplicate TPS65381A signal names"
    for k in ("K1", "K2"):
        r = RELAYS[k]
        assert abs(12.0 / r["coil_ohm"] * 1000 - r["coil_ma"]) < 0.3, k
    return True


if __name__ == "__main__":
    print("rev2 parts OK" if check_rev2_parts() else "FAIL")
