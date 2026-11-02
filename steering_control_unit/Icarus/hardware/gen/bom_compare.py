"""Icarus BOM vs the throttle ECU BOM (rev 0.6). Run from hardware/:
    python gen/bom_compare.py <throttle_bom.csv>

The throttle CSV is a kicad-cli BOM export of a *copy* of the throttle schematic (the throttle
folder is read-only). Matching:
- Active parts and special passives: by orderable part, or by family when only the grade suffix differs.
- R/C/L/FB: by value + package.
Writes docs/08_BOM_REVIEW_DATA.md-ready tables to stdout.
"""
import csv
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import design  # noqa: E402

UNIT = {"p": 1e-12, "n": 1e-9, "u": 1e-6, "m": 1e-3}


def size(fp):
    m = re.search(r"_(0201|0402|0603|0805|1206|1210|1812)_", fp + "_")
    return m.group(1) if m else None


def cap(v):
    m = re.match(r"([\d.]+)\s*([pnum])F", v)
    return round(float(m.group(1)) * UNIT[m.group(2)], 15) if m else None


def res(v):
    v = v.lower().replace("ohm", "").replace("r", "").strip().split()[0]
    m = re.match(r"([\d.]+)([km]?)$", v)
    if not m:
        return None
    return float(m.group(1)) * {"": 1, "k": 1e3, "m": 1e6}[m.group(2)]


def key(kind, value, fp, mpn=""):
    """(kind, normalised value, package) for passives; (kind, family) for the rest."""
    if kind == "C":                               # a voltage rating makes it a different orderable part
        volt = next((t for t in value.split()[1:] if t.endswith("V")), None)
        return ("C", cap(value), size(fp), volt)
    if kind == "R":                               # so does a tolerance (0.1 % summing resistors)
        tol = next((t for t in value.split()[1:] if t.endswith("%")), None)
        return ("R", res(value), size(fp), tol)
    if kind == "FB":
        return ("FB", res(value.split("@")[0]), size(fp))
    if kind == "L":
        return ("L", value.split()[0].lower(), "7.3x6.6" if "7.3x6.6" in fp or "SRP7028" in fp else fp)
    name = (mpn or value).upper()
    fam = re.sub(r"^P(?=[A-Z]{2,}\d)", "", name)            # TI preview prefix (PTCAN..., PLMR...)
    for root in ("F280049C", "TPS1H100", "LMR38020", "SIT2024B", "TCAN3404", "SQJ147", "TPS3702", "SMBJ20A",
                 "BAS21", "3568", "BAV99", "SS24"):
        if root in fam:
            return ("IC", root)
    return ("IC", fam)


def kind_of(lib, ref):
    if lib in ("Device:C", "Device:C_Polarized"):
        return "C"
    if lib in ("Device:R",):
        return "R"
    if lib.startswith("Device:FerriteBead"):
        return "FB"
    if lib == "Device:L":
        return "L"
    return "X"


def throttle_lines(path):
    out = {}
    for row in csv.DictReader(open(path, encoding="utf-8")):
        ref = row["Refs"].split(",")[0]
        if "TestPoint" in row["Footprint"] or ref.startswith(("IO", "T", "A", "B", "SW", "JP")):
            continue
        k = ("C" if re.match(r"C\d", ref) else "R" if re.match(r"R\d|R_", ref) else
             "FB" if ref.startswith("FB") else "L" if re.match(r"L\d", ref) else "X")
        mpn = row["MPN"] or row["Value"]
        out[key(k, row["Value"], row["Footprint"], mpn)] = row["Value"]
    return out


def main(tpath):
    thr = throttle_lines(tpath)
    thr_vals = {(k[0], k[1]) for k in thr if k[0] in ("C", "R", "FB", "L")}
    exact = {"F280049C": False, "TPS3702": False}  # family only: throttle has F280049CPZS / an unspecified TPS3702
    lines = {}
    for p in design.PARTS:
        if p["lib"] == "Connector:TestPoint":
            continue
        k = key(kind_of(p["lib"], p["ref"]), p["value"], p["fp"], p["mpn"])
        e = lines.setdefault(k, dict(value=p["value"], mpn=p["mpn"], lib=p["lib"], fp=p["fp"], refs=[]))
        e["refs"].append(p["ref"])
    same = [k for k in lines if k in thr]
    value_only = [k for k in lines if k not in thr and k[0] in ("C", "R", "FB", "L") and (k[0], k[1]) in thr_vals]
    n_lines, n_parts = len(lines), sum(len(e["refs"]) for e in lines.values())
    s_parts = sum(len(lines[k]["refs"]) for k in same)
    v_parts = sum(len(lines[k]["refs"]) for k in value_only)
    print(f"Icarus: {n_lines} BOM lines, {n_parts} placements (test pads excluded)")
    print(f"identical to a throttle line: {len(same)} lines ({100 * len(same) / n_lines:.0f} %), "
          f"{s_parts} placements ({100 * s_parts / n_parts:.0f} %)")
    print(f"same value, other package:   {len(value_only)} lines, {v_parts} placements ({100 * v_parts / n_parts:.0f} %)")
    fam = [k for k in same if k[0] == "IC" and not exact.get(k[1], True)]
    f_parts = sum(len(lines[k]["refs"]) for k in fam)
    print(f"  of which same family, different grade/variant: {len(fam)} lines, {f_parts} placements")
    print("\nshared lines:")
    for k in sorted(same, key=str):
        e = lines[k]
        tag = "  (family only)" if k in fam else ""
        print(f"  {len(e['refs']):3d}x  {e['value']:22s} {e['mpn'] or ''}{tag}")
    print("\nsame value, other package or rating:")
    for k in sorted(value_only, key=str):
        e = lines[k]
        print(f"  {len(e['refs']):3d}x  {e['value']:22s} {e['fp'].split(':')[-1]}")
    print("\nIcarus-only active / special parts:")
    for k, e in sorted(lines.items(), key=lambda t: str(t[0])):
        if k not in thr and k[0] == "IC":
            print(f"  {len(e['refs']):3d}x  {e['value']:22s} {e['mpn'] or e['fp'].split(':')[-1]}")


if __name__ == "__main__":
    main(sys.argv[1])
