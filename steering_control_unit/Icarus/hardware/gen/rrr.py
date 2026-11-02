"""Rip-up and reroute for the last open connections in gen/route/drc.rpt.

`make_pcb.py bridge` only routes through free space. On a dense board the leftovers are the
connections Freerouting gave up on, and they need other traces moved out of the way. For each
open connection this:

1. Routes it on a 0.1 mm grid over F.Cu / In2.Cu / B.Cu with vias. It may cross other nets'
   ordinary signal traces at a high cost per step; it may never cross pads, vias, GND, locked
   (fan-out) copper, wide power traces, copper zones other than the GND pours, the inner-layer
   via keep-outs, or the board edge.
2. Takes out the traces it crossed (rip-up).
3. Puts each ripped piece back between the copper it used to join (reroute, no further rip-up).
4. Checks every new segment and via exactly against all remaining copper.

A connection whose ripped pieces can't all be put back is rolled back as a whole, so the board
is never left with a new break. All planning is plain Python geometry; pcbnew is touched only at
the end (add first, then remove, then save: its SWIG containers break after Remove()). Refill
the pours afterwards in a fresh process (`make_pcb.py keepouts`), then run DRC.

    python gen/rrr.py                 # from hardware/, after DRC has written gen/route/drc.rpt
"""
import heapq
import math
import os
import re
import sys

import numpy as np
import pcbnew

import make_pcb as M

CLR = 0.16          # netclass clearance is 0.15
STEP = 0.1
VIA_D, DRILL = 0.5, 0.25
HOLE_CC = 0.75      # via drill to any drill, centre to centre (as bridge())
EDGE = 0.3
RIP_COST = float(os.environ.get("RRR_RIP_COST", "30"))   # per grid step through a rippable trace
DEPTH = int(os.environ.get("RRR_DEPTH", "3"))             # how many nets deep a put-back may rip in turn
TRIES = int(os.environ.get("RRR_TRIES", "8"))             # top-level retries, each banning the last try's rips
VIA_COST = 12.0
ESCAPE = float(os.environ.get("RRR_ESCAPE", "1.0"))   # trace segments ending this close to their own pad are never ripped
BAN = os.environ.get("RRR_BAN", "top")      # retry ban: "top" = first-level rips, "deep" = innermost failing rips
VIAS = os.environ.get("RRR_VIAS", "0") == "1"   # signal vias may be ripped too (with their segments)
FIXED = ("OSCIN", "OSCOUT", "KELVIN")   # crystal loops stay exactly as routed
SQ2 = math.sqrt(2)


class Item:
    __slots__ = ("kind", "g", "layers", "w", "net", "tid", "rip", "removed", "bbox")

    def __init__(self, kind, g, layers, w, net, tid=None, rip=False):
        self.kind, self.g, self.layers, self.w, self.net, self.tid, self.rip = kind, g, layers, w, net, tid, rip
        self.removed = False
        if kind in ("rect", "keep"):
            self.bbox = g
        elif kind == "circ":
            self.bbox = (g[0] - w, g[1] - w, g[0] + w, g[1] + w)
        elif kind == "seg":
            (ax, ay), (bx, by) = g
            self.bbox = (min(ax, bx) - w, min(ay, by) - w, max(ax, bx) + w, max(ay, by) + w)
        else:                                       # poly: list of (x, y)
            xs, ys = [p[0] for p in g], [p[1] for p in g]
            self.bbox = (min(xs), min(ys), max(xs), max(ys))

    # vectorised distance from grid points (X, Y) to this item's copper edge
    def dist(self, X, Y):
        k, g = self.kind, self.g
        if k in ("rect", "keep"):
            dx = np.maximum(np.maximum(g[0] - X, 0), X - g[2])
            dy = np.maximum(np.maximum(g[1] - Y, 0), Y - g[3])
            return np.hypot(dx, dy)
        if k == "circ":
            return np.hypot(X - g[0], Y - g[1]) - self.w
        if k == "seg":
            return _seg_d(X, Y, g[0], g[1]) - self.w
        d = np.full(np.broadcast(X, Y).shape, np.inf)
        inside = np.zeros(d.shape, bool)
        n = len(g)
        for e in range(n):
            p, q = g[e], g[(e + 1) % n]
            d = np.minimum(d, _seg_d(X, Y, p, q))
            cross = ((p[1] > Y) != (q[1] > Y)) & (X < (q[0] - p[0]) * (Y - p[1]) / ((q[1] - p[1]) or 1e-12) + p[0])
            inside ^= cross
        return np.where(inside, 0.0, d)

    # exact distance from segment ab (a == b for a point) to this item's copper edge
    def seg_dist(self, a, b):
        k, g = self.kind, self.g
        if k in ("rect", "keep"):
            return M._rect_seg_dist(g, a, b)
        if k == "circ":
            return M._seg_dist(g, a, b) - self.w
        if k == "seg":
            return M._seg_seg_dist(a, b, g[0], g[1]) - self.w
        n = len(g)
        if _point_in_poly(a, g) or _point_in_poly(b, g):
            return 0.0
        return min(M._seg_seg_dist(a, b, g[e], g[(e + 1) % n]) for e in range(n))


def _key(p):
    return (round(p[0], 3), round(p[1], 3))


def _ends(it):
    """Rounded connection points of a segment (its ends) or a via (its centre)."""
    return {_key(it.g)} if it.kind == "circ" else {_key(it.g[0]), _key(it.g[1])}


def _seg_d(X, Y, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return np.hypot(X - ax, Y - ay)
    t = np.clip(((X - ax) * dx + (Y - ay) * dy) / L2, 0.0, 1.0)
    return np.hypot(X - (ax + t * dx), Y - (ay + t * dy))


def _point_in_poly(p, poly):
    x, y, inside, n = p[0], p[1], False, len(poly)
    for e in range(n):
        (px, py), (qx, qy) = poly[e], poly[(e + 1) % n]
        if (py > y) != (qy > y) and x < (qx - px) * (y - py) / (qy - py) + px:
            inside = not inside
    return inside


class World:
    def __init__(self, board):
        self.board = board
        self.sig = list(M.SIG)
        self.L = len(self.sig)
        to = lambda v: (pcbnew.ToMM(v.x), pcbnew.ToMM(v.y))
        allL = frozenset(range(self.L))
        self.items, self.holes = [], []           # holes: (x, y, net, via item index or None)
        self.tracks = list(board.GetTracks())
        for fp in board.GetFootprints():
            for p in fp.Pads():
                bb = p.GetBoundingBox()
                r = (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()), pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()))
                th = p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)
                ls = allL if th else frozenset(i for i, l in enumerate(self.sig) if p.IsOnLayer(l))
                if ls:
                    self.items.append(Item("rect", r, ls, 0.0, p.GetNetname()))
                if th:
                    self.holes.append((*to(p.GetPosition()), p.GetNetname(), None))
        pad_rects = {}
        for it in self.items:
            pad_rects.setdefault(it.net, []).append(it.g)
        on_pad = lambda p, net: any(r[0] - ESCAPE <= p[0] <= r[2] + ESCAPE and r[1] - ESCAPE <= p[1] <= r[3] + ESCAPE
                                    for r in pad_rects.get(net, ()))
        for tid, t in enumerate(self.tracks):
            net = t.GetNetname()
            if isinstance(t, pcbnew.PCB_VIA):
                c = to(t.GetPosition())
                self.holes.append((*c, net, len(self.items)))
                self.items.append(Item("circ", c, allL, pcbnew.ToMM(t.GetWidth(pcbnew.F_Cu)) / 2, net, tid))
                continue
            if t.GetLayer() not in self.sig:
                continue
            w = pcbnew.ToMM(t.GetWidth())
            a, b = to(t.GetStart()), to(t.GetEnd())
            # pad escape stubs stay: a fine-pitch pin has one way out, and a new trace across it
            # would leave the ripped net no way back to its pin
            rip = net not in ("GND", "") and not any(s in net for s in FIXED) and w <= 0.25 + 1e-6 \
                and not t.IsLocked() and t.Type() == pcbnew.PCB_TRACE_T and not (on_pad(a, net) or on_pad(b, net))
            self.items.append(Item("seg", (a, b), frozenset([self.sig.index(t.GetLayer())]), w / 2, net, tid, rip))
        self.via_segs = {}                         # via item -> the segments touching it
        if VIAS:
            segs_at = {}
            for k, it in enumerate(self.items):
                if it.kind == "seg":
                    for e in it.g:
                        segs_at.setdefault((it.net, _key(e)), []).append(k)
            for k, it in enumerate(self.items):
                if it.kind != "circ" or it.tid is None or it.net in ("GND", "") or any(s in it.net for s in FIXED):
                    continue
                if self.tracks[it.tid].IsLocked() or any(r[0] <= it.g[0] <= r[2] and r[1] <= it.g[1] <= r[3]
                                                          for r in pad_rects.get(it.net, ())):
                    continue                       # a via in a pad is that pad's connection
                touch = segs_at.get((it.net, _key(it.g)), [])
                if all(self.items[j].rip for j in touch):
                    it.rip = True
                    self.via_segs[k] = touch
        for z in board.Zones():
            ls = [i for i, l in enumerate(self.sig) if z.IsOnLayer(l)]
            if not ls:
                continue
            if not z.GetIsRuleArea() and z.GetNetname() == "GND":
                continue                            # the GND pours re-fill around new copper; other zones are real copper
            o = z.Outline()
            for k in range(o.OutlineCount()):
                ch = o.Outline(k)
                pts = [to(ch.CPoint(m)) for m in range(ch.PointCount())]
                if z.GetIsRuleArea():
                    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
                    self.items.append(Item("keep", (min(xs), min(ys), max(xs), max(ys)), frozenset(ls), 0.0, None))
                else:
                    self.items.append(Item("poly", pts, frozenset(ls), 0.0, z.GetNetname()))
        eb = board.GetBoardEdgesBoundingBox()
        self.edge = (pcbnew.ToMM(eb.GetLeft()), pcbnew.ToMM(eb.GetTop()), pcbnew.ToMM(eb.GetRight()), pcbnew.ToMM(eb.GetBottom()))
        self.new = []                              # (kind, data, net, width) committed so far

    # ---------------------------------------------------------------- grid
    def grid(self, net, w, box, rip_ok, margin):
        x0, y0, x1, y1 = box
        nx, ny = int((x1 - x0) / STEP) + 1, int((y1 - y0) / STEP) + 1
        X = x0 + np.arange(nx) * STEP
        Y = y0 + np.arange(ny) * STEP
        hard = np.zeros((self.L, nx, ny), bool)
        rip = np.zeros((self.L, nx, ny), bool)
        vhard = np.zeros((nx, ny), bool)
        vrip = np.zeros((nx, ny), bool)
        need, vneed = w / 2 + CLR + margin, VIA_D / 2 + CLR + margin
        reach = max(need, vneed)
        for it in self.items:
            if it.removed or (it.net == net and it.kind != "keep"):
                continue
            b = it.bbox
            i0, i1 = max(0, math.ceil((b[0] - reach - x0) / STEP)), min(nx - 1, math.floor((b[2] + reach - x0) / STEP))
            j0, j1 = max(0, math.ceil((b[1] - reach - y0) / STEP)), min(ny - 1, math.floor((b[3] + reach - y0) / STEP))
            if i0 > i1 or j0 > j1:
                continue
            d = it.dist(X[i0:i1 + 1, None], Y[None, j0:j1 + 1])
            if it.kind == "keep":
                for li in it.layers:
                    hard[li, i0:i1 + 1, j0:j1 + 1] |= d < w / 2
                vhard[i0:i1 + 1, j0:j1 + 1] |= d < VIA_D / 2
                continue
            soft = it.rip and rip_ok
            tgt = rip if soft else hard
            for li in it.layers:
                tgt[li, i0:i1 + 1, j0:j1 + 1] |= d < need
            (vrip if soft else vhard)[i0:i1 + 1, j0:j1 + 1] |= d < vneed
        for hx, hy, _n, vk in self.holes:
            if vk is not None and self.items[vk].removed:
                continue
            i0, i1 = max(0, math.ceil((hx - HOLE_CC - x0) / STEP)), min(nx - 1, math.floor((hx + HOLE_CC - x0) / STEP))
            j0, j1 = max(0, math.ceil((hy - HOLE_CC - y0) / STEP)), min(ny - 1, math.floor((hy + HOLE_CC - y0) / STEP))
            if i0 <= i1 and j0 <= j1:
                soft = vk is not None and self.items[vk].rip and rip_ok
                (vrip if soft else vhard)[i0:i1 + 1, j0:j1 + 1] |=                     np.hypot(X[i0:i1 + 1, None] - hx, Y[None, j0:j1 + 1] - hy) < HOLE_CC
        ex0, ey0, ex1, ey1 = self.edge
        for m, r in ((None, EDGE + w / 2), ("v", EDGE + VIA_D / 2)):
            out = (X[:, None] < ex0 + r) | (X[:, None] > ex1 - r) | (Y[None, :] < ey0 + r) | (Y[None, :] > ey1 - r)
            if m:
                vhard |= out
            else:
                hard |= out[None]
        rip &= ~hard
        vrip &= ~vhard
        return dict(hard=hard, rip=rip, vhard=vhard, vrip=vrip, x0=x0, y0=y0, nx=nx, ny=ny)

    # ---------------------------------------------------------------- search
    def astar(self, G, a, la, b, lb, rip_cost):
        hard, rip, vhard, vrip = G["hard"], G["rip"], G["vhard"], G["vrip"]
        x0, y0, nx, ny, L = G["x0"], G["y0"], G["nx"], G["ny"], self.L
        cell = lambda p: (min(nx - 1, max(0, int(round((p[0] - x0) / STEP)))), min(ny - 1, max(0, int(round((p[1] - y0) / STEP)))))
        s, g = (la, *cell(a)), (lb, *cell(b))
        for n in (s, g):
            hard[n] = False
            rip[n] = False
        gi, gj = g[1], g[2]
        h = lambda i, j: math.hypot(i - gi, j - gj)
        best = {s: 0.0}
        came = {s: None}
        q = [(h(s[1], s[2]), 0.0, s)]
        moves = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2))
        while q:
            _f, c, n = heapq.heappop(q)
            if n == g:
                break
            if c > best.get(n, 1e18):
                continue
            li, i, j = n
            for di, dj, mc in moves:
                ii, jj = i + di, j + dj
                if not (0 <= ii < nx and 0 <= jj < ny) or hard[li, ii, jj]:
                    continue
                cc = c + mc + (rip_cost if rip[li, ii, jj] else 0.0)
                m = (li, ii, jj)
                if cc < best.get(m, 1e18):
                    best[m] = cc
                    came[m] = n
                    heapq.heappush(q, (cc + h(ii, jj), cc, m))
            if not vhard[i, j] and (rip_cost or not vrip[i, j]):
                vc = c + VIA_COST + (rip_cost if vrip[i, j] else 0.0)
                for lk in range(L):
                    if lk == li or hard[lk, i, j]:
                        continue
                    m = (lk, i, j)
                    if vc < best.get(m, 1e18):
                        best[m] = vc
                        came[m] = n
                        heapq.heappush(q, (vc + h(i, j), vc, m))
        if g not in came:
            return None
        path, n = [], g
        while n:
            path.append(n)
            n = came[n]
        path.reverse()
        pt = lambda n: (x0 + n[1] * STEP, y0 + n[2] * STEP)
        segs, vias, start = [], [], path[0]
        for k in range(1, len(path)):
            prev, cur = path[k - 1], path[k]
            if cur[0] != prev[0]:
                if start != prev:
                    segs.append([pt(start), pt(prev), prev[0]])
                vias.append(pt(prev))
                start = cur
                continue
            nxt = path[k + 1] if k + 1 < len(path) else None
            if nxt is None or nxt[0] != cur[0] or (nxt[1] - cur[1], nxt[2] - cur[2]) != (cur[1] - prev[1], cur[2] - prev[2]):
                segs.append([pt(start), pt(cur), cur[0]])
                start = cur
        if segs and segs[0][0] == pt(path[0]):
            segs[0][0] = a
        if segs and segs[-1][1] == pt(path[-1]):
            segs[-1][1] = b
        if not segs:
            segs = [[a, b, la]] if la == lb else []
        return [(p, q, l) for p, q, l in segs if p != q], vias

    # ---------------------------------------------------------------- exact check
    def check(self, net, w, segs, vias, rip_ok):
        """Exact clearance check. Returns the set of rippable items in the way, or None if any
        fixed copper is in the way."""
        ripped = set()
        for p, q, l in segs:
            for k, it in enumerate(self.items):
                if it.removed or l not in it.layers or (it.net == net and it.kind != "keep"):
                    continue
                lim = w / 2 if it.kind == "keep" else w / 2 + CLR
                b = it.bbox
                if min(p[0], q[0]) - lim > b[2] or max(p[0], q[0]) + lim < b[0] or \
                        min(p[1], q[1]) - lim > b[3] or max(p[1], q[1]) + lim < b[1]:
                    continue
                if it.seg_dist(p, q) < lim - 1e-6:
                    if it.rip and rip_ok:
                        ripped.add(k)
                    else:
                        return None
            ex0, ey0, ex1, ey1 = self.edge
            r = EDGE + w / 2
            if min(p[0], q[0]) < ex0 + r or max(p[0], q[0]) > ex1 - r or min(p[1], q[1]) < ey0 + r or max(p[1], q[1]) > ey1 - r:
                return None
        for v in vias:
            for hx, hy, _n, vk in self.holes:
                if math.hypot(v[0] - hx, v[1] - hy) < HOLE_CC:
                    if vk is not None and self.items[vk].removed:
                        continue
                    if vk is not None and self.items[vk].rip and rip_ok:
                        ripped.add(vk)
                    else:
                        return None
            for k, it in enumerate(self.items):
                if it.removed or (it.net == net and it.kind != "keep"):
                    continue
                if it.kind == "keep" and not (it.layers - {0, self.L - 1}):
                    continue
                lim = VIA_D / 2 if it.kind == "keep" else VIA_D / 2 + CLR
                b = it.bbox
                if v[0] + lim < b[0] or v[0] - lim > b[2] or v[1] + lim < b[1] or v[1] - lim > b[3]:
                    continue
                if it.seg_dist(v, v) < lim - 1e-6:
                    if it.rip and rip_ok:
                        ripped.add(k)
                    else:
                        return None
        for k in [k for k in ripped if k in self.via_segs]:   # a ripped via takes its segments along
            ripped.update(j for j in self.via_segs[k] if not self.items[j].removed)
        return ripped

    def commit(self, net, w, segs, vias):
        for p, q, l in segs:
            self.items.append(Item("seg", (p, q), frozenset([l]), w / 2, net))
            self.new.append(("seg", (p, q, self.sig[l]), net, w))
        for v in vias:
            self.items.append(Item("circ", v, frozenset(range(self.L)), VIA_D / 2, net))
            self.holes.append((*v, net, None))
            self.new.append(("via", v, net, VIA_D))

    def box(self, pts, pad):
        ex0, ey0, ex1, ey1 = self.edge
        return (max(ex0, min(p[0] for p in pts) - pad), max(ey0, min(p[1] for p in pts) - pad),
                min(ex1, max(p[0] for p in pts) + pad), min(ey1, max(p[1] for p in pts) + pad))

    def route(self, net, w, a, la, b, lb, rip_ok, pads=(3.0, 8.0)):
        for pad in pads:
            for margin in (0.02, 0.06):
                G = self.grid(net, w, self.box([a, b], pad), rip_ok, margin)
                r = self.astar(G, a, la, b, lb, RIP_COST if rip_ok else 0.0)
                if r is None:
                    break                        # no path in this window: a bigger margin won't find one
                segs, vias = r
                ripped = self.check(net, w, segs, vias, rip_ok)
                if ripped is not None:
                    return segs, vias, ripped
        return None

    # ---------------------------------------------------------------- reroute
    def terminals(self, net, comp_pts, lay):
        """Ends of a ripped component that touch copper of its net that is still there, on the
        end's own layer. Touching counts anywhere on that copper, so a T-junction onto the middle
        of a segment is a terminal too (an end-point-only test dropped such pieces silently)."""
        out = []
        for p in comp_pts:
            for it in self.items:
                if it.removed or it.net != net or lay[p] not in it.layers:
                    continue
                if it.seg_dist(p, p) <= 1e-3:
                    out.append(p)
                    break
        return out

    def place(self, net, w, a, la, b, lb, depth, pads, journal, indent="  "):
        """Route a to b. With depth > 0 it may rip other traces, and then puts every ripped piece
        back with depth - 1. Ripped items are logged in `journal` so the caller can roll back."""
        r = self.route(net, w, a, la, b, lb, rip_ok=depth > 0, pads=pads)
        if r is None:
            print(f"{indent}{net} near ({a[0]:.2f}, {a[1]:.2f}): no path (rip depth {depth})")
            return False
        segs, vias, ripped = r
        journal_top = bool(journal)                  # False on the first level of a connection
        if not journal:
            self.top_ripped = set(ripped)            # the first level's choice, for connect()'s retries
        for k in ripped:
            self.items[k].removed = True
            journal.append(k)
        self.commit(net, w, segs, vias)
        if ripped:
            print(f"{indent}{net}: {len(segs)} segments, {len(vias)} vias, ripping {len(ripped)} segments "
                  f"of {len({self.items[k].net for k in ripped})} nets")
        # group ripped segments into connected pieces per net, then put each piece back. A piece
        # takes every segment of its net ripped so far in this connection (not just at this level):
        # a nested put-back can rip the neighbour of a piece still waiting here, and the two then
        # go back as one piece. `handled` keeps a piece from being put back twice.
        for rnet in dict.fromkeys(self.items[k].net for k in ripped):
            ks = [k for k in journal if self.items[k].net == rnet and k not in self.handled]
            pieces, left = [], set(ks)
            while left:
                k = left.pop()
                comp, stack = {k}, [k]
                while stack:
                    c = stack.pop()
                    for o in list(left):
                        if _ends(self.items[o]) & _ends(self.items[c]):
                            left.discard(o)
                            comp.add(o)
                            stack.append(o)
                pieces.append(comp)
            for comp in pieces:
                if comp & self.handled:                # a nested level already put this one back
                    continue
                self.handled |= comp
                segc = [c for c in comp if self.items[c].kind == "seg"]
                if not segc:
                    continue                           # a lone via: nothing to put back
                pts = {pt for c in segc for pt in self.items[c].g}
                lay = {pt: next(iter(self.items[c].layers)) for c in segc for pt in self.items[c].g}
                term = self.terminals(rnet, pts, lay)
                if len(term) < 2:
                    # a routed trace piece always joins two things; fewer terminals means it hung on
                    # copper this test can't see, so dropping it could cut the net: roll back
                    print(f"{indent}{rnet}: ripped piece has {len(term)} terminal(s); can't put it back safely")
                    self.fail_net = self.fail_net or rnet
                    self._note_deep(ripped, rnet)
                    return False
                rw = max(self.items[c].w for c in segc) * 2
                for t in term[1:]:
                    if not self.place(rnet, rw, t, lay[t], term[0], lay[term[0]], depth - 1, (3.0, 8.0),
                                      journal, indent + "  "):
                        if not journal_top:
                            self.fail_net = rnet         # the first-level net whose put-back failed
                        self._note_deep(ripped, rnet)
                        return False
        return True

    def _note_deep(self, ripped, rnet):
        """Remember the innermost failing level's rips of `rnet` (copper that existed before this
        connection: anything newer is gone after the rollback)."""
        if self.deep_rips is None:
            ks = {k for k in ripped if self.items[k].net == rnet and k < self.snap}
            if ks:
                self.deep_rips = ks

    def connect(self, net, a, la, b, lb, w, depth=DEPTH):
        """Route one open connection with rip-up; reroute what it ripped. True if all committed.
        When the put-back chain dead-ends, everything is rolled back, the traces the first level
        ripped for the net that couldn't go back become unrippable (RRR_BAN=deep: the innermost
        failing level's rips instead), and the connection is tried again: the next try has to take
        a different way through. Up to TRIES tries."""
        taboo = []
        try:
            for attempt in range(TRIES):
                snap_items, snap_holes, snap_new = len(self.items), len(self.holes), len(self.new)
                journal, self.handled, self.top_ripped, self.fail_net = [], set(), set(), None
                self.snap, self.deep_rips = snap_items, None
                if self.place(net, w, a, la, b, lb, depth, (4.0, 10.0), journal):
                    print(f"  {net}: closed on try {attempt + 1}; {len(journal)} ripped segments put back")
                    return True
                for k in journal:
                    if k < snap_items:
                        self.items[k].removed = False
                del self.items[snap_items:]
                del self.holes[snap_holes:]
                del self.new[snap_new:]
                if not self.top_ripped:
                    break                              # no path even with rip-up: a retry can't differ
                # ban only the first-level rips of the net that couldn't go back; if that's unknown,
                # ban them all
                ban = [k for k in self.top_ripped if self.items[k].net == self.fail_net] or list(self.top_ripped)
                if BAN == "deep" and self.deep_rips:
                    ban = list(self.deep_rips)         # change the deepest choice first
                for k in ban:
                    self.items[k].rip = False
                    taboo.append(k)
                print(f"  {net}: try {attempt + 1} rolled back; {len(taboo)} segments now off limits")
            print(f"  {net}: rolled back")
            return False
        finally:
            for k in taboo:
                self.items[k].rip = True

    # ---------------------------------------------------------------- apply
    def apply(self):
        b = self.board
        for kind, d, net, w in self.new:
            n = b.FindNet(net)
            if kind == "seg":
                p, q, layer = d
                t = pcbnew.PCB_TRACK(b)
                t.SetStart(pcbnew.VECTOR2I(M.mm(p[0]), M.mm(p[1])))
                t.SetEnd(pcbnew.VECTOR2I(M.mm(q[0]), M.mm(q[1])))
                t.SetWidth(M.mm(w))
                t.SetLayer(layer)
                t.SetNet(n)
            else:
                t = pcbnew.PCB_VIA(b)
                t.SetPosition(pcbnew.VECTOR2I(M.mm(d[0]), M.mm(d[1])))
                t.SetWidth(M.mm(VIA_D))
                t.SetDrill(M.mm(DRILL))
                t.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                t.SetNet(n)
            b.Add(t)
        gone = [self.tracks[it.tid] for it in self.items if it.removed and it.tid is not None]
        for t in gone:
            b.Remove(t)
        pcbnew.SaveBoard(M.PCB, b)                 # no more pcbnew calls after Remove()
        return len(self.new), len(gone)


def open_pairs(board):
    rpt = open(os.path.join(M.HERE, "route", "drc.rpt"), encoding="utf-8").read()
    to = lambda v: (pcbnew.ToMM(v.x), pcbnew.ToMM(v.y))
    out = []
    for block in rpt.split("[unconnected_items]")[1:]:
        block = block.split("\n[")[0]
        items = re.findall(r"@\(([\d.]+) mm, ([\d.]+) mm\): (\w+)[^\[]*\[([^\]]+)\][^\n]* on (\S+)", block)
        if len(items) != 2:
            continue
        net = items[0][3]
        ends = []
        for x, y, kind, _n, layer in items:
            p = (float(x), float(y))
            lid = board.GetLayerID(layer.rstrip(","))
            if kind == "Track":
                cands = [to(e) for t in board.GetTracks() if not isinstance(t, pcbnew.PCB_VIA)
                         and t.GetNetname() == net and t.GetLayer() == lid for e in (t.GetStart(), t.GetEnd())]
                p = min(cands, key=lambda q: math.dist(q, p))
            ends.append((p, lid))
        widths = [pcbnew.ToMM(t.GetWidth()) for t in board.GetTracks()
                  if t.GetNetname() == net and not isinstance(t, pcbnew.PCB_VIA)]
        out.append((net, ends, max(set(widths), key=widths.count) if widths else 0.2))
    return out


def main():
    board = pcbnew.LoadBoard(M.PCB)
    pairs = open_pairs(board)
    W = World(board)
    print(f"{len(pairs)} open connections; {sum(it.rip for it in W.items)} rippable trace segments")
    closed = 0
    for net, ((a, la), (b, lb)), w in sorted(pairs, key=lambda x: math.dist(x[1][0][0], x[1][1][0])):
        closed += W.connect(net, a, W.sig.index(la), b, W.sig.index(lb), w)
    if closed:
        added, removed = W.apply()
        print(f"rrr: {closed} of {len(pairs)} closed; {added} items added, {removed} removed")
        print("next: make_pcb.py keepouts (fresh process: refills the pours), then DRC")
    else:
        print(f"rrr: 0 of {len(pairs)} closed; board unchanged")


if __name__ == "__main__":
    main()
