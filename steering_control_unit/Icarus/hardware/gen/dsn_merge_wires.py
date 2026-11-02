"""Merge chained two-point wires in a KiCad-exported Specctra DSN into multi-point paths.

KiCad exports most routed track as separate wires that meet end to end. Freerouting joins them
on load with a recursive PolylineTrace.combine, which overflows the stack on a fully routed
board, so a re-exported board can't be loaded to continue routing. This walks every chain of
wires (same layer, width, net and type) through nodes where exactly two wires meet, and writes
each chain as one path.

A chain never runs through a via or a pad: Freerouting only makes contacts at a trace's end
points, so a pad in the middle of a merged path would look unconnected. `pads` holds the pad
rectangles in DSN units (µm, y up) with their copper layers, as make_pcb.redsn() passes them.

    python dsn_merge_wires.py route/icarus.dsn          # rewrites in place (no pad stops)
"""
import re
import sys
from collections import defaultdict

WIRE = re.compile(r"^(\s*)\(wire \(path (\S+) (\S+)((?:\s+-?[\d.]+\s+-?[\d.]+)+)\)\(net ([^)]+)\)\(type (\w+)\)\)\s*$")
VIA = re.compile(r"^\s*\(via \S+\s+(-?[\d.]+)\s+(-?[\d.]+)")


def _pad_hit(pads, cell=2000.0):
    """Point-in-pad test over a bucket grid; pads are (x0, y0, x1, y1, layers) in DSN units,
    layers None for through-hole pads (every layer)."""
    grid = defaultdict(list)
    for r in pads:
        for i in range(int(r[0] // cell), int(r[2] // cell) + 1):
            for j in range(int(r[1] // cell), int(r[3] // cell) + 1):
                grid[(i, j)].append(r)

    def hit(n, layer):
        x, y = float(n[0]), float(n[1])
        return any(r[0] <= x <= r[2] and r[1] <= y <= r[3] and (r[4] is None or layer in r[4])
                   for r in grid.get((int(x // cell), int(y // cell)), ()))
    return hit


def merge(text, pads=()):
    on_pad = _pad_hit(pads)
    lines = text.split("\n")
    vias = set()
    wires, keep, indent, first = defaultdict(list), [], "    ", None
    for i, ln in enumerate(lines):
        m = WIRE.match(ln)
        if m:
            indent = m.group(1)
            first = i if first is None else first
            nums = m.group(4).split()
            pts = [(nums[k], nums[k + 1]) for k in range(0, len(nums), 2)]
            wires[(m.group(2), m.group(3), m.group(5), m.group(6))].append(pts)
            continue
        v = VIA.match(ln)
        if v:
            vias.add((v.group(1), v.group(2)))
        keep.append((i, ln))
    out, before, after = [], 0, 0
    for (layer, width, net, kind), paths in wires.items():
        before += len(paths)
        ends = defaultdict(list)
        for k, p in enumerate(paths):
            ends[p[0]].append(k)
            ends[p[-1]].append(k)
        joinable = lambda n: len(ends[n]) == 2 and n not in vias and not on_pad(n, layer)
        used = set()

        def walk(k, start):
            """Follow a chain from path k, entered at its `start` end; return the merged points."""
            p = paths[k] if paths[k][0] == start else paths[k][::-1]
            pts = list(p)
            used.add(k)
            while joinable(pts[-1]):
                nxt = [j for j in ends[pts[-1]] if j not in used]
                if not nxt:
                    break
                j = nxt[0]
                q = paths[j] if paths[j][0] == pts[-1] else paths[j][::-1]
                pts.extend(q[1:])
                used.add(j)
            return pts

        # start each chain at an end that isn't joinable, then sweep up anything left (loops)
        for k, p in enumerate(paths):
            if k in used:
                continue
            for e in (p[0], p[-1]):
                if not joinable(e):
                    pts = walk(k, e)
                    break
            else:
                continue
            out.append(pts)
            after += 1
            out[-1] = (layer, width, net, kind, pts)
        for k, p in enumerate(paths):
            if k not in used:
                out.append((layer, width, net, kind, walk(k, p[0])))
                after += 1
    body = [f"{indent}(wire (path {l} {w}  " + "  ".join(f"{x} {y}" for x, y in pts) + f")(net {n})(type {t}))"
            for l, w, n, t, pts in out]
    res = []
    for i, ln in keep:
        if first is not None and i > first and body:
            res.extend(body)
            body = []
        res.append(ln)
    return "\n".join(res), before, after


if __name__ == "__main__":
    path = sys.argv[1]
    text, before, after = merge(open(path, encoding="utf-8").read())
    open(path, "w", encoding="utf-8").write(text)
    print(f"{path}: {before} wires -> {after}")
