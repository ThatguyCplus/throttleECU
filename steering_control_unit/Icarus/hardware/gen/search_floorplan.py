"""Search double-sided floorplans for a board size (rev 0.6). Run with KiCad's Python from hardware/:
    "C:\\Program Files\\KiCad\\10.0\\bin\\python.exe" gen/search_floorplan.py [W H]

Builds the real cluster blocks once (placer.py), then tries every split of the regions into a top
and a bottom set that respects the hard rules below. For each side it also tries every way of cutting
the region order into 2-4 columns. Regions keep the signal-flow order (in -> pass -> out -> MCUs ->
measure). The output lists the fitting floorplans with the most spare width on the tighter side;
paste one into make_pcb.CONFIGS.

Hard rules: the test pads and the LED home region stay on top; the connector-side regions (entry,
pass) stay on top beside the strip, so the pass-through path stays short; coil and backup (the only
SMD parts with through-hole thermal vias) share a side, which is packed first so the other side can
keep clear of the vias.
"""
import itertools
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("ICARUS_CONFIG", "A")
import make_pcb as M  # noqa: E402

ORDER = ["entry", "pass", "inputs", "supply", "coil", "backup", "ch_a", "ch_b", "overlay", "guard", "measure", "tp"]
ALWAYS_TOP = {"entry", "pass", "tp"}


def need(blocks, regions, H):
    """Narrowest total width (mm) for an ordered region list on one side, over column cuts."""
    best = None
    n = len(regions)
    for k in range(1, min(4, n) + 1):
        for cuts in itertools.combinations(range(1, n), k - 1):
            cols = [regions[a:b] for a, b in zip((0,) + cuts, cuts + (n,))]
            total = 0.0
            for i, col in enumerate(cols):
                top, bot = (7.6, H - 7.6) if i == len(cols) - 1 else (2.5, H - 2.5)
                free = bot - top - M.COL_GAP * (len(col) - 1)
                lo = max(min(b[0], b[1]) if b[4] else b[0] for s in col for b in blocks[s])
                fits = lambda w: sum(M._pack_height(blocks[s], w) for s in col) <= free
                if not fits(120.0):
                    total = None
                    break
                hi = 120.0
                if fits(lo):
                    hi = lo
                while hi - lo > 0.25:
                    mid = (lo + hi) / 2
                    lo, hi = (lo, mid) if fits(mid) else (mid, hi)
                total += hi
            if total is not None:
                total += M.COL_GAP * (len(cols) - 1)
                if best is None or total < best[0]:
                    best = (total, cols)
    return best


def spy(blocks):
    W, H = M.W, M.H
    avail = W - M.CONN_X - 1.5
    movable = [r for r in ORDER if r not in ALWAYS_TOP and r in blocks]
    results = []
    for k in range(len(movable) + 1):
        for under in itertools.combinations(movable, k):
            top = [r for r in ORDER if r in blocks and r not in under]
            bot = [r for r in ORDER if r in under]
            if not bot or (("coil" in under) != ("backup" in under)):
                continue                          # parts with through-hole thermal vias (TPS1H100, TPS61092) share a side
            t, b = need(blocks, top, H), need(blocks, bot, H)
            if t and b and t[0] <= avail and b[0] <= avail:
                results.append((min(avail - t[0], avail - b[0]), t, b))
    results.sort(key=lambda r: -r[0])
    print(f"{len(results)} fitting floorplans at {W:.0f} x {H:.0f} mm (usable width {avail:.1f} mm)")
    on_top = lambda r, name: any(name in col for col in r[1][1])
    themes = {   # best floorplans per engineering theme
        "best overall": lambda r: True,
        "MCUs on top (decoupling next to the In1 GND plane)": lambda r: on_top(r, "ch_a") and on_top(r, "ch_b"),
        "supplies on top (switchers next to the In1 GND plane)": lambda r: on_top(r, "supply"),
        "channel split (CH-A on top, CH-B underneath)": lambda r: on_top(r, "ch_a") and not on_top(r, "ch_b"),
        "analog on top (overlay + guard next to the plane)": lambda r: on_top(r, "overlay") and on_top(r, "guard"),
    }
    for name, keep in themes.items():
        rs = [r for r in results if keep(r)]
        print(f"\n{name}: {len(rs)} fit")
        for spare, t, b in rs[:3]:
            print(f"  spare {spare:5.1f} mm | top {t[1]} | bottom {b[1]}")
    raise SystemExit(0)


M.regions = spy
M.place()
