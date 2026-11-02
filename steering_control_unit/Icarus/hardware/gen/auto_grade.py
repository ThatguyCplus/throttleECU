"""Automotive grade of every Icarus BOM line (rev 0.6). Run from hardware/: python gen/auto_grade.py

Status per line:
  Q   AEC-Q100/Q101/Q200 part (the -Q1 / Q100 / automotive orderable code is in the BOM)
  Q?  automotive grade exists and is intended, but the exact orderable code still has to be confirmed
  S   automotive grade has to be SPECIFIED at sourcing (generic R/C, no part number yet)
  N   not automotive: needs a different part (see docs/08_BOM_REVIEW.md)
  -   bench-only or not a component (debug headers, solder jumper)
Sources for each verdict: docs/08_BOM_REVIEW.md.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import design  # noqa: E402

GRADE = [   # (substring of value or mpn, status, short reason); first match wins
    ("SN74LVC1G123", "N", "TI has no -Q1; Nexperia 74LVC1G123DP-Q100H or TI TPUL1G313-Q1 (verify pinout)"),
    ("SN74LVC1G74", "Q?", "TI -Q1 orderable unconfirmed; Nexperia Q100 part is NOT pin-compatible"),
    ("MCP73833", "N", "no automotive grade; TI BQ25171-Q1 is the AEC-Q100 1S charger"),
    ("TPS61092", "N", "no -Q1; TI TPS610333-Q1 (fixed 5 V, 1.8-5.5 V in)"),
    ("G6K-2F-Y", "N", "Omron G6K is not automotive; Panasonic TX2SA is the lead (confirm AEC-Q200)"),
    ("LED", "N", "generic: pick AEC-Q102 (e.g. ROHM CSL0902 series); recheck R values for each Vf"),
    ("JTAG", "-", "bench-only debug header"), ("UART", "-", "bench-only debug header"),
    ("16MHz", "S", "quartz crystal: specify an AEC-Q200 16 MHz 3225 part with the load caps it needs"),
    ("CAN term", "-", "solder jumper"),
    ("Vehicle harness", "N", "placeholder header: automotive sealed connector TBD (open item)"),
    ("JST PH", "N", "JST-PH is not automotive: sealed automotive 2-pin for in-car use"),
    ("mini blade", "Q?", "ATM fuse is automotive; Keystone 3568 holder has no AEC rating (common in vehicles)"),
    ("Q1", "Q", "TI -Q1 (AEC-Q100)"), ("Q100", "Q", "AEC-Q100"),
    ("F280049CPZQR", "Q", "TMS320F280049C-Q1, AEC-Q100"),
    ("SiT2024B", "Q", "SiTime SiT2024 automotive oscillator (AEC-Q100)"),
    ("SQJ147", "Q", "Vishay SQ automotive MOSFET (AEC-Q101)"),
    ("BSS138Q", "Q", "Diodes Q suffix, AEC-Q101"),
    ("-Q", "Q?", "AEC-Q101 grade named; exact orderable code to confirm"),
    ("BZX84C18", "Q?", "specify the AEC-Q101 version (BZX84C18-Q)"),
    ("SRP7028A", "Q", "Bourns SRP7028A, AEC-Q200"),
    ("BLM18AG", "Q", "Murata BLM18AG, AEC-Q200"),
    ("SMDH1812", "Q", "Yageo automotive PTC, AEC-Q200"), ("MINIASMDC", "Q", "Littelfuse automotive PTC, AEC-Q200"),
    ("NCU18", "Q?", "Murata NCU18 automotive NTC: confirm the code"),
    ("470uF", "S", "specify an AEC-Q200 electrolytic (e.g. Panasonic FK/FR automotive)"),
]


def grade(p):
    text = f"{p['value']} {p['mpn']}"
    for key, st, why in GRADE:
        if key in text:
            return st, why
    if p["lib"] in ("Device:R", "Device:C", "Device:Thermistor_NTC"):
        return "S", "generic: specify AEC-Q200 (e.g. Yageo AC / Panasonic ERJ / ERA for 0.1 %, Murata GCM / TDK CGA)"
    return "?", "unclassified"


def main():
    lines = {}
    for p in design.PARTS:
        if p["lib"] == "Connector:TestPoint":
            continue
        k = (p["value"], p["fp"], p["mpn"])
        lines.setdefault(k, []).append(p)
    tally, ptally = {}, {}
    rows = []
    for (value, fp, mpn), ps in lines.items():
        st, why = grade(ps[0])
        tally[st] = tally.get(st, 0) + 1
        ptally[st] = ptally.get(st, 0) + len(ps)
        rows.append((st, len(ps), value, mpn or fp.split(":")[-1], why))
    n, m = sum(tally.values()), sum(ptally.values())
    names = {"Q": "automotive (AEC-Q) part number in BOM", "Q?": "automotive grade, code to confirm",
             "S": "grade to specify at sourcing (generic R/C)", "N": "not automotive: change part",
             "-": "bench-only / not a part", "?": "unclassified"}
    print(f"{n} BOM lines, {m} placements (test pads excluded)")
    for st in ("Q", "Q?", "S", "N", "-", "?"):
        if st in tally:
            print(f"  {st:2s} {tally[st]:3d} lines ({100 * tally[st] / n:3.0f} %), {ptally[st]:3d} placements "
                  f"({100 * ptally[st] / m:3.0f} %)  {names[st]}")
    print()
    for st, q, value, part, why in sorted(rows, key=lambda r: ("N", "Q?", "S", "-", "Q", "?").index(r[0])):
        if st in ("N", "Q?"):
            print(f"  {st:2s} {q:3d}x {value:22s} {part:38s} {why}")


if __name__ == "__main__":
    main()
