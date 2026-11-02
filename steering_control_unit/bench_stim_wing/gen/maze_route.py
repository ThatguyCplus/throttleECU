"""Close the last few connections Freerouting leaves open, with a small grid router.

    python3 maze_route.py            # reads ../bench_stim_wing.kicad_pcb (after make_pcb.py finish)

For every job in JOBS it runs an A* search on a 0.05 mm grid over all four copper
layers (vias allowed), keeping the board's clearances to every other-net item:
0.127 mm normally, 0.21 mm to plated through-hole pads (JLCPCB pad-to-trace). It
also stitches GND pour islands that aren't tied to the In1 plane. The result is
printed as placement.MANUAL_ROUTES entries; paste them in and re-run finish.
Those entries belong to the committed .ses routing.
"""
import heapq
import math
import re

import numpy as np
import pcbnew

import make_pcb as M

RES = 0.05
LAYERS = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
LNAME = {pcbnew.F_Cu: "F_Cu", pcbnew.In1_Cu: "In1_Cu", pcbnew.In2_Cu: "In2_Cu", pcbnew.B_Cu: "B_Cu"}
CLR = 0.13                 # board rule 0.127 + rounding margin
CLR_PTH = 0.215            # JLCPCB pad-to-trace for plated through-hole pads (0.2) + margin
EDGE = 0.3
VIA_COST, IN1_PENALTY = 25.0, 3.0

# (net, from item, to item). Items: ("pad", ref, num) or ("track", layer, x, y) =
# the same-net track nearest (x, y) on that layer, or ("via", x, y).
JOBS = [
    ("+12V", ("pad", "R2", "1"), ("track", pcbnew.F_Cu, 9.3, 10.6)),
    ("/BUCK_SW", ("pad", "C5", "2"), ("via", 18.5, 9.6)),
    ("/DAC_D", ("pad", "U2", "9"), ("pad", "R52", "1")),
    ("/OPA_OUT", ("pad", "U3", "1"), ("track", pcbnew.B_Cu, 34.0, 13.1)),
]


def mm2(v):
    return pcbnew.ToMM(v.x) - M.OX, pcbnew.ToMM(v.y) - M.OY


class Grid:
    def __init__(self, board):
        self.board = board
        self.nx = int(round(52.0 / RES))
        self.ny = int(round(24.0 / RES))
        xs = (np.arange(self.nx) * RES - 0.6)
        ys = (np.arange(self.ny) * RES - 0.6)
        self.X, self.Y = np.meshgrid(xs, ys, indexing="ij")
        self.x0 = -0.6

    def idx(self, x, y):
        return int(round((x - self.x0) / RES)), int(round((y - self.x0) / RES))

    def xy(self, i, j):
        return i * RES + self.x0, j * RES + self.x0

    def window(self, x0, y0, x1, y1):
        i0, j0 = self.idx(x0, y0)
        i1, j1 = self.idx(x1, y1)
        i0, j0 = max(i0, 0), max(j0, 0)
        i1, j1 = min(i1 + 1, self.nx), min(j1 + 1, self.ny)
        return slice(i0, i1), slice(j0, j1)

    def seg_dist(self, sl, a, b):
        X, Y = self.X[sl], self.Y[sl]
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        t = np.zeros_like(X) if L == 0 else np.clip(((X - ax) * dx + (Y - ay) * dy) / L, 0, 1)
        return np.hypot(X - ax - t * dx, Y - ay - t * dy)

    def rect_dist(self, sl, x0, y0, x1, y1):
        X, Y = self.X[sl], self.Y[sl]
        return np.hypot(np.maximum(np.maximum(x0 - X, 0), X - x1), np.maximum(np.maximum(y0 - Y, 0), Y - y1))


def obstacles(board, g, net, half_w, via_r):
    """blocked[l]: a track centre (half width half_w) can't go here on layer l.
    viablock: a via (radius via_r) can't go here."""
    blocked = {l: np.zeros((g.nx, g.ny), bool) for l in LAYERS}
    viablock = np.zeros((g.nx, g.ny), bool)

    def mark_shape(layers, dist_fn, bbox, extra, clr):
        x0, y0, x1, y1 = bbox
        pad = max(half_w, via_r) + clr + extra + 0.1
        sl = g.window(x0 - pad, y0 - pad, x1 + pad, y1 + pad)
        d = dist_fn(sl) - extra
        for l in layers:
            blocked[l][sl] |= d < half_w + clr
        viablock[sl] |= d < via_r + clr

    for t in board.GetTracks():
        if t.GetNetname() == net:
            continue
        w = pcbnew.ToMM(t.GetWidth()) / 2
        if isinstance(t, pcbnew.PCB_VIA):
            c = mm2(t.GetPosition())
            mark_shape(LAYERS, lambda sl, c=c: g.seg_dist(sl, c, c), (c[0], c[1], c[0], c[1]), w, CLR)
        else:
            a, b = mm2(t.GetStart()), mm2(t.GetEnd())
            mark_shape([t.GetLayer()], lambda sl, a=a, b=b: g.seg_dist(sl, a, b),
                       (min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])), w, CLR)
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() == net and net:
                continue
            bb = p.GetBoundingBox()
            x0, y0 = mm2(bb.GetOrigin())
            x1, y1 = x0 + pcbnew.ToMM(bb.GetWidth()), y0 + pcbnew.ToMM(bb.GetHeight())
            pth = p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)
            layers = [l for l in LAYERS if p.IsOnLayer(l)] or (LAYERS if pth else [])
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                layers = LAYERS
            mark_shape(layers, lambda sl, r=(x0, y0, x1, y1): g.rect_dist(sl, *r), (x0, y0, x1, y1), 0,
                       CLR_PTH if pth else CLR)
    # board edge (rounded corners approximated by the inset rectangle + corner discs)
    X, Y = g.X, g.Y
    W, H, R = 50.8, 22.86, 2.54
    out = (X < EDGE) | (Y < EDGE) | (X > W - EDGE) | (Y > H - EDGE)
    for cx, cy in [(R, R), (W - R, R), (R, H - R), (W - R, H - R)]:
        corner = (np.abs(X - cx) > 0) & (((X < R) if cx == R else (X > W - R)) & ((Y < R) if cy == R else (Y > H - R)))
        out |= corner & (np.hypot(X - cx, Y - cy) > R - EDGE)
    for l in LAYERS:
        blocked[l] |= out
    viablock |= out
    return blocked, viablock


def item_cells(board, g, net, item):
    """(layer, i, j) cells that belong to an item of `net`."""
    cells = set()
    if item[0] == "pad":
        p = next(p for p in board.FindFootprintByReference(item[1]).Pads() if p.GetNumber() == item[2])
        bb = p.GetBoundingBox()
        x0, y0 = mm2(bb.GetOrigin())
        x1, y1 = x0 + pcbnew.ToMM(bb.GetWidth()), y0 + pcbnew.ToMM(bb.GetHeight())
        sl = g.window(x0, y0, x1, y1)
        inside = g.rect_dist(sl, x0 + 0.05, y0 + 0.05, x1 - 0.05, y1 - 0.05) <= 0
        for l in LAYERS:
            if p.IsOnLayer(l):
                ii, jj = np.nonzero(inside)
                cells |= {(l, i + sl[0].start, j + sl[1].start) for i, j in zip(ii, jj)}
        return cells, mm2(p.GetPosition())
    if item[0] == "via":
        i, j = g.idx(item[1], item[2])
        return {(l, i, j) for l in LAYERS}, (item[1], item[2])
    _, layer, x, y = item
    best = min((t for t in board.GetTracks() if t.GetNetname() == net and not isinstance(t, pcbnew.PCB_VIA)
                and t.GetLayer() == layer),
               key=lambda t: F_dist((x, y), mm2(t.GetStart()), mm2(t.GetEnd())))
    a, b = mm2(best.GetStart()), mm2(best.GetEnd())
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / RES))
    for k in range(n + 1):
        i, j = g.idx(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
        cells.add((layer, i, j))
    return cells, (x, y)


def F_dist(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0 if L == 0 else max(0, min(1, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def astar(g, blocked, viablock, src, dst, goal_xy):
    li = {l: k for k, l in enumerate(LAYERS)}
    gi, gj = g.idx(*goal_xy)
    h = lambda i, j: math.hypot(i - gi, j - gj)
    openq, came, cost = [], {}, {}
    for (l, i, j) in src:
        s = (li[l], i, j)
        cost[s] = 0
        heapq.heappush(openq, (h(i, j), 0, s))
    goal = {(li[l], i, j) for (l, i, j) in dst}
    steps = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    src_set = set(cost)
    while openq:
        _, c, s = heapq.heappop(openq)
        if s in goal:
            path = [s]
            while path[-1] in came:
                path.append(came[path[-1]])
            return path[::-1]
        if c > cost.get(s, 1e18):
            continue
        k, i, j = s
        nbrs = []
        for di, dj, w in steps:
            ni, nj = i + di, j + dj
            if 0 <= ni < g.nx and 0 <= nj < g.ny:
                n = (k, ni, nj)
                if n in goal or not blocked[LAYERS[k]][ni, nj]:
                    nbrs.append((n, w * (IN1_PENALTY if LAYERS[k] == pcbnew.In1_Cu else 1)))
        if not viablock[i, j] or s in src_set:
            for k2 in range(len(LAYERS)):
                if k2 != k and (not viablock[i, j]):
                    nbrs.append(((k2, i, j), VIA_COST))
        for n, w in nbrs:
            nc = c + w
            if nc < cost.get(n, 1e18):
                cost[n] = nc
                came[n] = s
                heapq.heappush(openq, (nc + h(n[1], n[2]), nc, n))
    return None


def to_routes(g, net, path, width):
    out = []
    pts = [(LAYERS[k], *g.xy(i, j)) for k, i, j in path]
    seg_start = pts[0]
    prev = pts[0]
    direction = None
    for p in pts[1:]:
        if p[0] != prev[0]:                      # layer change = via
            if (prev[1], prev[2]) != (seg_start[1], seg_start[2]):
                out.append(("track", net, LNAME[prev[0]], seg_start[1], seg_start[2], prev[1], prev[2], width))
            out.append(("via", net, prev[1], prev[2]))
            seg_start, prev, direction = p, p, None
            continue
        d = (round(p[1] - prev[1], 3), round(p[2] - prev[2], 3))
        if direction is not None and d != direction:
            out.append(("track", net, LNAME[prev[0]], seg_start[1], seg_start[2], prev[1], prev[2], width))
            seg_start = prev
        direction = d
        prev = p
    if (prev[1], prev[2]) != (seg_start[1], seg_start[2]):
        out.append(("track", net, LNAME[prev[0]], seg_start[1], seg_start[2], prev[1], prev[2], width))
    return out


def net_width(net):
    short = net.lstrip("/")
    for name, w, members in M.NETCLASSES:
        if short in members:
            return w
    return M.TRACK_SIG


def gnd_islands(board, g):
    """Vias that tie isolated GND pour islands (F, In2, B) to the In1 plane."""
    _, viablock = obstacles(board, g, "GND", 0.0, 0.225)
    out = []
    gvias = [mm2(t.GetPosition()) for t in board.GetTracks()
             if isinstance(t, pcbnew.PCB_VIA) and t.GetNetname() == "GND"]
    gpth = [mm2(p.GetPosition()) for fp in board.GetFootprints() for p in fp.Pads()
            if p.GetNetname() == "GND" and p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
    anchors = gvias + gpth
    for z in board.Zones():
        if z.GetNetname() != "GND" or z.GetLayer() == pcbnew.In1_Cu:
            continue
        polys = z.GetFilledPolysList(z.GetLayer())
        for k in range(polys.OutlineCount()):
            o = polys.Outline(k)
            if any(o.PointInside(pcbnew.VECTOR2I(pcbnew.FromMM(x + M.OX), pcbnew.FromMM(y + M.OY))) for x, y in anchors):
                continue
            bb = o.BBox()
            x0, y0 = mm2(bb.GetOrigin())
            x1, y1 = x0 + pcbnew.ToMM(bb.GetWidth()), y0 + pcbnew.ToMM(bb.GetHeight())
            best = None
            for x in np.arange(x0, x1, RES):
                for y in np.arange(y0, y1, RES):
                    i, j = g.idx(x, y)
                    if viablock[i, j]:
                        continue
                    c = pcbnew.VECTOR2I(pcbnew.FromMM(x + M.OX), pcbnew.FromMM(y + M.OY))
                    r = pcbnew.FromMM(0.25)
                    if all(o.PointInside(pcbnew.VECTOR2I(c.x + int(r * math.cos(a)), c.y + int(r * math.sin(a))))
                           for a in np.linspace(0, 2 * math.pi, 9)):
                        best = (x, y)
                        break
                if best:
                    break
            if best:
                out.append(("via", "GND", round(best[0], 3), round(best[1], 3)))
                anchors.append(best)
            else:
                print("# no via spot in GND island on", LNAME[z.GetLayer()], "near", (round(x0, 1), round(y0, 1)))
    return out


def main():
    board = pcbnew.LoadBoard(M.PCB)
    g = Grid(board)
    routes = []
    for net, a, b in JOBS:
        width = net_width(net)
        via_r = (0.6 if width >= 0.4 else 0.45) / 2
        blocked, viablock = obstacles(board, g, net, width / 2, via_r)
        src, _ = item_cells(board, g, net, a)
        dst, goal_xy = item_cells(board, g, net, b)
        path = astar(g, blocked, viablock, src, dst, goal_xy)
        if path is None:
            print(f"# {net}: no route found")
            continue
        r = to_routes(g, net, path, width)
        print(f"    # {net}: {a[1:]} -> {b[1:]}")
        for e in r:
            print("    " + repr(tuple(round(v, 3) if isinstance(v, float) else v for v in e)) + ",")
        routes += r
    for e in gnd_islands(board, g):
        print("    # GND pour island -> In1 plane")
        print("    " + repr(e) + ",")


if __name__ == "__main__":
    main()
