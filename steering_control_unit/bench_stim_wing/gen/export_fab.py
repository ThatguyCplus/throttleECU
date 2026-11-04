"""Export fab outputs to ../fab: Gerbers + drill (zipped), BOM and CPL in JLCPCB's column format.

    python3 export_fab.py
Gerbers are only meaningful once the board is routed (make_pcb.py finish).
The BOM comes straight from design.py (KiCad 7's CLI has no BOM export), so it
carries the same MPN / LCSC numbers as the schematic fields.
"""
import csv
import os
import re
import shutil
import subprocess
import zipfile
from collections import OrderedDict

import design

KICAD_CLI = shutil.which("kicad-cli") or r"C:\Program Files\KiCad\10.0\bin\kicad-cli.exe"
PRJ = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
FAB = os.path.join(PRJ, "fab")
PCB = os.path.join(PRJ, design.PROJECT + ".kicad_pcb")
# JLC's library models of these packages sit rotated relative to KiCad's. Entries
# are the ones that match this board, from the community database that
# kicad-jlcpcb-tools downloads (matthewlai/JLCKicadTools cpl_rotations_db.csv).
# MSOP-10 isn't listed there; it is the same package as VSSOP-10 (270).
# JLC's order preview is still the final word: check every part there.
JLC_ROTATIONS = [
    (r"^SOT-23", -90),                    # SOT-23 and SOT-23-6
    (r"^TSSOP-", 270),
    (r"^MSOP-10_", 270),
    (r"^Relay_DPDT_Omron_G6K-2F-Y", 270),
    (r"^JST_GH_SM", 180),
    (r"^JST_PH_S", 180),
]
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

    write_cpl(os.path.join(FAB, design.PROJECT + "_cpl_jlc.csv"), set(r for refs in groups.values() for r in refs))
    print("fab outputs in", FAB)


def jlc_correction(fp_name):
    """Rotation correction for the longest matching pattern (as kicad-jlcpcb-tools picks)."""
    best = None
    for pat, corr in JLC_ROTATIONS:
        m = re.search(pat, fp_name)
        if m and (best is None or len(m.group(0)) > best[0]):
            best = (len(m.group(0)), corr)
    return best[1] if best else 0


def write_cpl(path, bom_refs):
    """JLC placement file, computed the way Bouni's kicad-jlcpcb-tools does it:
    position = centre of the footprint's copper pads, relative to the drill/aux
    origin (board bottom-left), Y up; bottom-side angles mirrored (180 - a);
    then the JLC library rotation correction for the footprint."""
    import pcbnew
    board = pcbnew.LoadBoard(PCB)
    org = board.GetDesignSettings().GetAuxOrigin()
    rows = []
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        if ref not in bom_refs:
            continue
        pads = [p for p in fp.Pads() if p.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH and p.IsOnCopperLayer()]
        bb = pads[0].GetBoundingBox()
        for p in pads[1:]:
            bb.Merge(p.GetBoundingBox())
        c = bb.GetCenter()
        bottom = fp.GetLayer() != pcbnew.F_Cu
        rot = fp.GetOrientation().AsDegrees()
        if bottom:
            rot = 180 - rot
        name = str(fp.GetFPID().GetLibItemName())
        rot = (rot + jlc_correction(name)) % 360
        rows.append((ref, pcbnew.ToMM(c.x - org.x), -pcbnew.ToMM(c.y - org.y), "Bottom" if bottom else "Top", rot))
    missing = bom_refs - {r[0] for r in rows}
    if missing:
        raise SystemExit("BOM parts missing from the board: %s" % sorted(missing))
    nat = lambda r: (r.rstrip("0123456789"), int(r[len(r.rstrip("0123456789")):] or 0))
    with open(path, "w", newline="", encoding="utf-8") as out:
        w = csv.writer(out)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for ref, x, y, layer, rot in sorted(rows, key=lambda r: nat(r[0])):
            w.writerow([ref, "%.4fmm" % x, "%.4fmm" % y, layer, "%g" % round(rot, 3)])


if __name__ == "__main__":
    main()
