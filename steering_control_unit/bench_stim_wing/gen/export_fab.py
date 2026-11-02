"""Export fab outputs to ../fab: Gerbers + drill (zipped), BOM and CPL in JLCPCB's column format.

    python3 export_fab.py
Gerbers are only meaningful once the board is routed (make_pcb.py finish).
The BOM comes straight from design.py (KiCad 7's CLI has no BOM export), so it
carries the same MPN / LCSC numbers as the schematic fields.
"""
import csv
import os
import shutil
import subprocess
import zipfile
from collections import OrderedDict

import design

KICAD_CLI = shutil.which("kicad-cli") or r"C:\Program Files\KiCad\10.0\bin\kicad-cli.exe"
PRJ = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
FAB = os.path.join(PRJ, "fab")
PCB = os.path.join(PRJ, design.PROJECT + ".kicad_pcb")
# JLC's library models of these packages sit rotated relative to KiCad's (same
# correction the EPS tap wing used for TSSOP). Check every part in JLC's preview.
JLC_ROTATION_OFFSET = {"TSSOP-14": -90, "TSSOP-16": -90, "MSOP-10": -90}
LAYERS = "F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"


def cli(*args):
    subprocess.run([KICAD_CLI, *args], check=True, stdout=subprocess.DEVNULL)


def main():
    os.makedirs(FAB, exist_ok=True)
    gerb = os.path.join(FAB, "gerber")
    shutil.rmtree(gerb, ignore_errors=True)
    os.makedirs(gerb)
    cli("pcb", "export", "gerbers", "--layers", LAYERS, "--use-drill-file-origin",
        "--subtract-soldermask", "-o", gerb + os.sep, PCB)
    cli("pcb", "export", "drill", "--drill-origin", "plot", "--excellon-separate-th",
        "--generate-map", "--map-format", "pdf", "-o", gerb + os.sep, PCB)
    with zipfile.ZipFile(os.path.join(FAB, design.PROJECT + "_gerbers.zip"), "w", zipfile.ZIP_DEFLATED) as z:
        for name in sorted(os.listdir(gerb)):
            z.write(os.path.join(gerb, name), name)

    # BOM: SMD parts the assembler fits, grouped by value + footprint + MPN.
    groups = OrderedDict()
    for p in design.PARTS:
        if p.get("in_bom") is False:
            continue
        key = (p["value"], p["fp"].split(":")[1], p["mpn"], p["lcsc"])
        groups.setdefault(key, []).append(p["ref"])
    nat = lambda r: (r.rstrip("0123456789"), int(r[len(r.rstrip("0123456789")):] or 0))
    with open(os.path.join(FAB, design.PROJECT + "_bom_jlc.csv"), "w", newline="", encoding="utf-8") as out:
        w = csv.writer(out)
        w.writerow(["Comment", "Designator", "Footprint", "Manufacturer Part", "JLCPCB Part #", "Quantity"])
        for (value, fp, mpn, lcsc), refs in sorted(groups.items(), key=lambda kv: nat(kv[1][0])):
            refs.sort(key=nat)
            w.writerow([value, ",".join(refs), fp, mpn, lcsc, len(refs)])

    raw_pos = os.path.join(FAB, "pos_kicad.csv")
    cli("pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "both", "--smd-only",
        "--use-drill-file-origin", "-o", raw_pos, PCB)
    with open(raw_pos, newline="", encoding="utf-8") as f, \
            open(os.path.join(FAB, design.PROJECT + "_cpl_jlc.csv"), "w", newline="", encoding="utf-8") as out:
        w = csv.writer(out)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for row in csv.DictReader(f):
            if row["Ref"] in design.HAND_SOLDERED:
                continue
            rot = float(row["Rot"]) + JLC_ROTATION_OFFSET.get(row["Package"].split("_")[0], 0)
            w.writerow([row["Ref"], row["PosX"] + "mm", row["PosY"] + "mm",
                        "Top" if row["Side"] == "top" else "Bottom", "%g" % (rot % 360)])
    os.remove(raw_pos)
    print("fab outputs in", FAB)


if __name__ == "__main__":
    main()
