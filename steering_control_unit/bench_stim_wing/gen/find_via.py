"""Find a clear spot for a via + back-side stub that closes one open connection.

    python3 find_via.py REF PAD [radius_mm]

Looks around pad REF.PAD for a via position that keeps CLEAR mm from all
other-net copper on every layer and touches copper of the pad's own net on some
layer (or the GND plane, for GND), and whose straight stub from the pad on the
pad's layer is also clear. Prints the best candidates as MANUAL_ROUTES entries.
"""
import math
import sys

import pcbnew

import make_pcb as M

CLEAR = float(__import__("os").environ.get("FV_CLEAR", "0.15"))
VIA_R = 0.225
STUB_W = 0.25


def seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0 if L == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / L))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def seg_seg(a, b, c, d):
    if (_cross(a, b, c) * _cross(a, b, d) < 0) and (_cross(c, d, a) * _cross(c, d, b) < 0):
        return 0.0                      # the segments cross
    return min(seg_dist(a, c, d), seg_dist(b, c, d), seg_dist(c, a, b), seg_dist(d, a, b))


def mmxy(v):
    return (pcbnew.ToMM(v.x) - M.OX, pcbnew.ToMM(v.y) - M.OY)


def main():
    ref, padnum = sys.argv[1], sys.argv[2]
    radius = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    board = pcbnew.LoadBoard(M.PCB)
    pad = next(p for p in board.FindFootprintByReference(ref).Pads() if p.GetNumber() == padnum)
    net = pad.GetNetname()
    player = pcbnew.B_Cu if pad.IsOnLayer(pcbnew.B_Cu) and not pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.F_Cu
    px, py = mmxy(pad.GetPosition())
    cu = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
    other, own = {l: [] for l in cu}, {l: [] for l in cu}
    for t in board.GetTracks():
        bucket = own if t.GetNetname() == net else other
        w = pcbnew.ToMM(t.GetWidth()) / 2
        if isinstance(t, pcbnew.PCB_VIA):
            for l in cu:
                bucket[l].append(("c", mmxy(t.GetPosition()), w))
        else:
            bucket[t.GetLayer()].append(("s", (mmxy(t.GetStart()), mmxy(t.GetEnd())), w))
    for fp in board.GetFootprints():
        for p in fp.Pads():
            bb = p.GetBoundingBox()
            x0, y0 = mmxy(bb.GetOrigin())
            x1, y1 = x0 + pcbnew.ToMM(bb.GetWidth()), y0 + pcbnew.ToMM(bb.GetHeight())
            bucket = own if (p.GetNetname() == net and net) else other
            for l in cu:
                if p.IsOnLayer(l):
                    bucket[l].append(("r", (x0, y0, x1, y1), 0))

    def dist(item, q, seg=None):
        kind, g, w = item
        if kind == "c":
            d = seg_dist(g, *seg) if seg else math.hypot(q[0] - g[0], q[1] - g[1])
            return d - w
        if kind == "s":
            d = seg_seg(seg[0], seg[1], *g) if seg else seg_dist(q, *g)
            return d - w
        x0, y0, x1, y1 = g
        pts = [seg[0], seg[1]] + [((seg[0][0] + seg[1][0]) / 2, (seg[0][1] + seg[1][1]) / 2)] if seg else [q]
        return min(math.hypot(max(x0 - x, 0, x - x1), max(y0 - y, 0, y - y1)) for x, y in pts)

    found = []
    steps = int(radius / 0.05)
    for i in range(-steps, steps + 1):
        for j in range(-steps, steps + 1):
            q = (px + i * 0.05, py + j * 0.05)
            if not (1.0 < q[0] < 49.8 and 1.0 < q[1] < 21.8):
                continue
            ok = all(dist(it, q) >= VIA_R + CLEAR for l in cu for it in other[l])
            if not ok:
                continue
            touches = net == "GND" or any(dist(it, q) <= VIA_R for l in cu for it in own[l])
            if not touches:
                continue
            stub = (px, py), q
            if any(dist(it, None, stub) < STUB_W / 2 + CLEAR for it in other[player]):
                continue
            found.append((math.hypot(q[0] - px, q[1] - py), q))
    found.sort()
    lname = "B_Cu" if player == pcbnew.B_Cu else "F_Cu"
    for d, q in found[:3]:
        print(f'    ("via", "{net}", {q[0]:.2f}, {q[1]:.2f}),')
        print(f'    ("track", "{net}", "{lname}", {px:.2f}, {py:.2f}, {q[0]:.2f}, {q[1]:.2f}, {STUB_W}),   # {d:.2f} mm')
    if not found:
        print("no clear spot within", radius, "mm")


if __name__ == "__main__":
    main()
