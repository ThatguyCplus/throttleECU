"""Point every 3D model at the project's own copy, so the board renders and exports
STEP the same on any machine and any KiCad version (7 to 10), whatever 3D
libraries are installed there.

    python3 localize_3d.py            # rewrite the board (+ the Feather footprint)
    python3 localize_3d.py --fetch    # also download any missing .step first

Models live in ../bench_stim_wing.3dshapes/ (from KiCad's official
kicad-packages3D, CC-BY-SA 4.0 with the KiCad library exception). Paths become
${KIPRJMOD}/bench_stim_wing.3dshapes/<name>.step: KIPRJMOD is defined by every
KiCad version, unlike KICAD6/7/8/9/10_3DMODEL_DIR.

J1 (the Feather stacking headers) has no library model, so it gets two
PinSocket models (1x16 on pins 1-16, 1x12 on pins 17-28): the female sockets
that sit on top of this wing.
"""
import os
import re
import sys
import urllib.request

PRJ = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
PCB = os.path.join(PRJ, "bench_stim_wing.kicad_pcb")
FEATHER_FP = os.path.join(PRJ, "bench_stim_wing.pretty", "Feather_Wing_Headers.kicad_mod")
SHAPES = "bench_stim_wing.3dshapes"
UPSTREAM = "https://gitlab.com/kicad/libraries/kicad-packages3D/-/raw/master/"
MODEL_RE = re.compile(r'\(model "(?:\$\{[A-Z0-9_]+\}/)?(?:[^"/]+\.3dshapes/)?([^"/]+?)\.(?:wrl|step|stp)"')
# Feather header models: (library folder, model, offset in KiCad's 3D frame: y up)
HEADER_MODELS = [("Connector_PinSocket_2.54mm.3dshapes", "PinSocket_1x16_P2.54mm_Vertical", (0, 0, 0)),
                 ("Connector_PinSocket_2.54mm.3dshapes", "PinSocket_1x12_P2.54mm_Vertical", (20.32, -10.16, 0))]


def local(name):
    return '(model "${KIPRJMOD}/%s/%s.step"' % (SHAPES, name)


def model_block(name, xyz, indent):
    i = indent
    return (f'{i}{local(name)}\n{i}  (offset (xyz {xyz[0]:g} {xyz[1]:g} {xyz[2]:g}))\n'
            f'{i}  (scale (xyz 1 1 1))\n{i}  (rotate (xyz 0 0 0))\n{i})\n')


def block_end(text, start):
    """Index of the ')' closing the s-expression that opens at text[start] == '('."""
    depth, i, in_str = 0, start, False
    while i < len(text):
        c = text[i]
        if in_str:
            if c == "\\":
                i += 1
            elif c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    raise ValueError("unbalanced s-expression")


def add_header_models(text, start_pat, indent):
    """Give the Feather header footprint its two socket models (replacing any old ones)."""
    m = re.search(start_pat, text)
    if not m:
        raise SystemExit("Feather header footprint not found")
    s = m.start()
    e = block_end(text, s)
    body = text[s:e]
    body = re.sub(r'\n[ \t]*\(model "[^"]*PinSocket_[^"]*"(?:.|\n)*?\n[ \t]*\)(?=\n)', "", body)
    body = body.rstrip() + "\n" + "".join(model_block(n, xyz, indent) for _, n, xyz in HEADER_MODELS)
    closing_indent = indent[:-2] if len(indent) >= 2 else ""
    return text[:s] + body + closing_indent + text[e:]


def fetch(names):
    os.makedirs(os.path.join(PRJ, SHAPES), exist_ok=True)
    for folder, name in names:
        dst = os.path.join(PRJ, SHAPES, name + ".step")
        if os.path.exists(dst):
            continue
        print("fetching", name)
        urllib.request.urlretrieve(UPSTREAM + f"{folder}/{name}.step", dst)


def main():
    with open(PCB, encoding="utf-8") as f:
        pcb = f.read()
    wanted = {(m.group(0).split("/")[-2] if "3dshapes/" in m.group(0) else "", m.group(1))
              for m in MODEL_RE.finditer(pcb)}
    if "--fetch" in sys.argv:
        fetch(sorted(wanted | {(f, n) for f, n, _ in HEADER_MODELS}))
    pcb = MODEL_RE.sub(lambda m: local(m.group(1)), pcb)
    pcb = add_header_models(pcb, r'\(footprint "bench_stim_wing:Feather_Wing_Headers"', "    ")
    with open(PCB, "w", encoding="utf-8") as f:
        f.write(pcb)

    with open(FEATHER_FP, encoding="utf-8") as f:
        fp = f.read()
    fp = add_header_models(fp, r'\(footprint "Feather_Wing_Headers"', "\t")
    with open(FEATHER_FP, "w", encoding="utf-8") as f:
        f.write(fp)

    names = set(re.findall(r'\$\{KIPRJMOD\}/' + SHAPES + r'/([^"]+)\.step', pcb))
    missing = [n for n in sorted(names) if not os.path.exists(os.path.join(PRJ, SHAPES, n + ".step"))]
    print(f"{len(names)} models, all under {SHAPES}/" if not missing else f"MISSING: {missing} (run with --fetch)")
    if missing:
        sys.exit(1)


if __name__ == "__main__":
    main()
