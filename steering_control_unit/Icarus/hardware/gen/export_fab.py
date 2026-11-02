"""Export fab outputs to ../fab: 4-layer Gerbers + drill (zipped), BOM and CPL in JLCPCB's format.

    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" gen/export_fab.py      (from hardware/)
Only meaningful once the board is routed and DRC-clean (make_pcb.py finish).
"""
import csv
import os
import shutil
import subprocess
import zipfile

import design

KICAD_CLI = r"C:\Program Files\KiCad\10.0\bin\kicad-cli.exe"
PRJ = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
FAB = os.path.join(PRJ, "fab")
PCB = os.path.join(PRJ, design.PROJECT + ".kicad_pcb")
SCH = os.path.join(PRJ, design.PROJECT + ".kicad_sch")
# JLC's library models of these packages sit rotated relative to KiCad's (Fabrication Toolkit
# corrections). Check every IC in JLC's placement preview anyway.
JLC_ROTATION_OFFSET = {"TSSOP-14": -90, "TSSOP-10": -90}   # only offsets verified on the tap wing
LAYERS = "F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"
if '"In4.Cu"' in open(PCB, encoding="utf-8").read(5000):          # 6-layer board
    LAYERS = LAYERS.replace("In2.Cu,", "In2.Cu,In3.Cu,In4.Cu,")
HAND_FITTED = ("Conn_", "PinHeader", "CP_Radial", "Fuseholder", "JST_", "TestPoint_")   # + bare test pads


def cli(*args):
    subprocess.run([KICAD_CLI, *args], check=True, stdout=subprocess.DEVNULL)


def main():
    os.makedirs(FAB, exist_ok=True)
    gerb = os.path.join(FAB, "gerber")
    shutil.rmtree(gerb, ignore_errors=True)
    os.makedirs(gerb)
    cli("pcb", "export", "gerbers", "--layers", LAYERS, "--use-drill-file-origin", "--subtract-soldermask",
        "-o", gerb + os.sep, PCB)
    cli("pcb", "export", "drill", "--drill-origin", "plot", "--excellon-separate-th", "--generate-map",
        "--map-format", "pdf", "-o", gerb + os.sep, PCB)
    with zipfile.ZipFile(os.path.join(FAB, design.PROJECT + "_gerbers.zip"), "w", zipfile.ZIP_DEFLATED) as z:
        for name in sorted(os.listdir(gerb)):
            z.write(os.path.join(gerb, name), name)

    raw_bom = os.path.join(FAB, "bom_kicad.csv")
    cli("sch", "export", "bom", "--fields", "Reference,Value,Footprint,MPN,QUANTITY",
        "--labels", "Refs,Value,Footprint,MPN,Qty", "--group-by", "Value,Footprint,MPN",
        "--ref-range-delimiter", "", "-o", raw_bom, SCH)
    with open(raw_bom, newline="", encoding="utf-8") as f, \
            open(os.path.join(FAB, design.PROJECT + "_bom_jlc.csv"), "w", newline="", encoding="utf-8") as out:
        w = csv.writer(out)
        w.writerow(["Comment", "Designator", "Footprint", "Manufacturer Part", "JLCPCB Part #", "Quantity"])
        for row in csv.DictReader(f):
            fp = row["Footprint"].split(":")[-1]
            if fp.startswith(HAND_FITTED):
                continue                    # connectors, THT caps, fuse holder: fitted by hand
            w.writerow([row["Value"], row["Refs"], fp, row["MPN"], "", row["Qty"]])
    os.remove(raw_bom)

    raw_pos = os.path.join(FAB, "pos_kicad.csv")
    cli("pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "both", "--smd-only",
        "--exclude-dnp", "--use-drill-file-origin", "-o", raw_pos, PCB)
    with open(raw_pos, newline="", encoding="utf-8") as f, \
            open(os.path.join(FAB, design.PROJECT + "_cpl_jlc.csv"), "w", newline="", encoding="utf-8") as out:
        w = csv.writer(out)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for row in csv.DictReader(f):
            if row["Ref"].startswith(("H", "TP")):     # mounting holes, bare test pads
                continue
            pkg = row["Package"]
            off = next((v for k, v in JLC_ROTATION_OFFSET.items() if pkg.startswith(k + "_") or pkg == k), 0)
            w.writerow([row["Ref"], row["PosX"] + "mm", row["PosY"] + "mm",
                        "Top" if row["Side"] == "top" else "Bottom", "%g" % ((float(row["Rot"]) + off) % 360)])
    os.remove(raw_pos)
    print("fab outputs in", FAB)


if __name__ == "__main__":
    main()
