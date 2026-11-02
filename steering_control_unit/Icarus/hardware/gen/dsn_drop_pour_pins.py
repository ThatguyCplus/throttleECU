"""Take pour-connected GND pads out of a continuation DSN so Freerouting doesn't chase them.

`make_pcb.py redsn` strips the outer GND pours (Freerouting can't use them), so every GND pad
that only a pour reached looks unrouted, about 500 on rev 0.7. Freerouting spends every pass on
them and never gets to the real gaps. This drops those pads from the GND pin list. They stay in
the DSN as netless pads, so every net still keeps clearance to them. KiCad keeps the board's own
pad nets when it imports the SES, and `finish` re-pours.

A pad counts as wired, and stays in the list, if it is through-hole (it reaches the In1 plane)
or a GND track end or via lands on it.

    python dsn_drop_pour_pins.py <board.kicad_pcb> <in.dsn> <out.dsn>
"""
import re
import sys

import pcbnew


def pour_only_pins(board, net="GND"):
    to = lambda v: (v.x, v.y)
    pts = []
    for t in board.GetTracks():
        if t.GetNetname() != net:
            continue
        if isinstance(t, pcbnew.PCB_VIA):
            pts.append((to(t.GetPosition()), None))
        else:
            pts += [(to(t.GetStart()), t.GetLayer()), (to(t.GetEnd()), t.GetLayer())]
    out = set()
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() != net or p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                continue
            bb = p.GetBoundingBox()
            hit = any(bb.Contains(pcbnew.VECTOR2I(*q)) and (l is None or p.IsOnLayer(l)) for q, l in pts)
            if not hit:
                out.add(f"{fp.GetReference()}-{p.GetNumber()}")
    return out


def drop(text, pins, net="GND"):
    i = text.index(f"    (net {net}\n")
    j = text.index("(pins", i)
    k = text.index(")", j)
    have = text[j + 5:k].split()
    kept = [p for p in have if p not in pins]
    body = "\n        ".join(" ".join(kept[n:n + 12]) for n in range(0, len(kept), 12))
    return text[:j] + "(pins " + body + text[k:], len(have) - len(kept)


if __name__ == "__main__":
    board = pcbnew.LoadBoard(sys.argv[1])
    pins = pour_only_pins(board)
    text, n = drop(open(sys.argv[2], encoding="utf-8").read(), pins)
    open(sys.argv[3], "w", encoding="utf-8").write(text)
    print(f"dropped {n} pour-only GND pins of {len(pins)} found")
