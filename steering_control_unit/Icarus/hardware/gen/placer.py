"""Pin-aware cluster placement for make_pcb.py (rev 0.5).

Each design group becomes a cluster around its anchor (the IC with the most pads). Every other
part goes next to the pad it connects to, in design.py order, which lists each passive right
after the part it serves. That gives the placements the datasheets' layout sections ask for:

- **Decoupling.** A capacitor on a supply net sits at a supply pin of the IC it follows, one per
  pin where the IC has several (F280049C VDDIO / VDD / VDDA / VREFHI per SPRS945), smallest value
  first, so it lands nearest.
- **Timing and feedback.** TPS3850 CWD, LMR38020 RT and FB, 74LVC1G123 Cext / RextCext, TPS61092
  LBI, MCP73833 PROG and TPS1H100 CL / CS sit at their pins.
- **Power stages.** The BOOT cap goes across BOOT/SW, the inductor at SW, and the output caps at the
  inductor (LMR38020, TPS61092 layout guides). The oscillator chains through its series R onto X1.
- **Chains.** A part with no pad on the cluster yet chains onto the part it shares its most specific
  net with (divider bottoms, clamp diodes, LED + resistor pairs).

Groups with no IC (supplies and reference, indicators) dissolve: each part joins the cluster in its
region that holds its most specific net, most specific first, so a rail LED lands at its regulator
and a GPIO LED at its MCU pin. Clusters are then packed as blocks by make_pcb.pack().
"""
import pcbnew

PASSIVE = ("Device:R", "Device:C", "Device:L", "Device:LED", "Device:FerriteBead", "Device:D", "Device:Fuse",
           "Device:Polyfuse", "Device:Thermistor", "Diode:", "Transistor_FET:Q_NMOS", "Connector:TestPoint",
           "Jumper:")
GND = "GND"
STEP = 0.25                      # search grid (mm)
R_MAX = 14.0                     # give up searching beyond this distance from the target pad
GROW = 2.0                       # cost per mm of cluster bounding-box growth (width + height)
ANCHOR_ROT = {}                  # anchor ref -> 0/90/180/270, set by make_pcb from the config's anchor_rot


def is_passive(p):
    return p["lib"].startswith(PASSIVE)


class Geo:
    """Pad offsets and courtyard of one footprint at 0/90/180/270 degrees, relative to its origin."""

    def __init__(self, fp, nets_of, power_pins=()):
        self.rot = {}
        for r in (0, 90, 180, 270):
            fp.SetOrientationDegrees(r)
            fp.SetPosition(pcbnew.VECTOR2I(0, 0))
            fp.BuildCourtyardCaches()
            cy = fp.GetCourtyard(pcbnew.F_CrtYd)
            bb = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False)
            pads, pth = {}, []
            for p in fp.Pads():
                if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                    continue
                pads.setdefault(p.GetNumber(), []).append((pcbnew.ToMM(p.GetPosition().x),
                                                           pcbnew.ToMM(p.GetPosition().y)))
                if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:      # through-hole (thermal vias): reach the other side
                    q = p.GetBoundingBox()
                    pth.append((pcbnew.ToMM(q.GetLeft()), pcbnew.ToMM(q.GetTop()),
                                pcbnew.ToMM(q.GetRight()), pcbnew.ToMM(q.GetBottom())))
            self.rot[r] = dict(pads=pads, pth=pth, cy=(pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                                                       pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())))
        fp.SetOrientationDegrees(0)
        self.nets = nets_of                       # {pad number: net}
        self.power = set(power_pins)              # pads whose pin type is power_in (VDD, VIN, V+ ...)


def _overlap(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


class Cluster:
    """Parts placed in local coordinates around an anchor at (0, 0)."""

    def __init__(self, name, anchor, geo, gap, order, owner=None):
        self.name, self.geo, self.gap, self.order = name, geo, gap, order
        self.owner = owner or {}                  # decoupling cap -> the IC it follows in design.py
        self.pos = {}                             # ref -> (x, y, rot)
        self.rects = {}                           # ref -> inflated courtyard
        self.used = set()                         # (ref, pad) already holding a decoupling cap
        self.put(anchor, 0.0, 0.0, ANCHOR_ROT.get(anchor, 0))

    def rect(self, ref, x, y, rot, margin=None):
        c = self.geo[ref].rot[rot]["cy"]
        m = (self.gap[ref] if margin is None else margin) / 2
        return (x + c[0] - m, y + c[1] - m, x + c[2] + m, y + c[3] + m)

    def put(self, ref, x, y, rot):
        self.pos[ref] = (x, y, rot)
        self.rects[ref] = self.rect(ref, x, y, rot)

    def pads_on(self, net):
        """[(ref, pad, x, y)] of placed pads on `net`."""
        out = []
        for ref, (x, y, rot) in self.pos.items():
            for num, n in self.geo[ref].nets.items():
                if n == net:
                    for dx, dy in self.geo[ref].rot[rot]["pads"].get(num, []):
                        out.append((ref, num, x + dx, y + dy))
        return out

    def bbox(self):
        r = list(self.rects.values())
        return (min(a[0] for a in r), min(a[1] for a in r), max(a[2] for a in r), max(a[3] for a in r))


def is_decap(ref, parts, geo):
    """A capacitor from one supply net to GND."""
    nets = set(geo[ref].nets.values())
    return parts[ref]["lib"] == "Device:C" and GND in nets and len(nets) == 2


def _target(cl, ref, parts, net_size):
    """Best (net, pad-list) to attach `ref` to: IC pads over passive pads; for a decoupling cap, the
    IC's supply-input pins (power_in: VDD, VIN, V+) and among those the ones without a cap yet; then
    the most specific shared net, then the nearest preceding part in design order."""
    geo = cl.geo[ref]
    is_cap = is_decap(ref, parts, cl.geo)
    best = None
    for num, net in geo.nets.items():
        if net == GND:
            continue
        for tref, tpad, x, y in cl.pads_on(net):
            ic = not is_passive(parts[tref])
            supply = tpad in cl.geo[tref].power
            used = (tref, tpad) in cl.used
            back = cl.order[ref] - cl.order[tref]
            own = cl.owner.get(ref)
            key = (0 if ic else 1, (own is not None and tref != own, not supply, used) if is_cap
                   else (False, False, False), net_size[net], back if back > 0 else 10000 - back)
            if best is None or key < best[0]:
                best = (key, num, net, tref, tpad, x, y)
    return best


def attach(cl, ref, parts, net_size):
    """Place `ref` next to its target pad. Returns False if it shares no net with the cluster yet."""
    t = _target(cl, ref, parts, net_size)
    if t is None:
        return False
    _key, num, net, tref, tpad, tx, ty = t
    geo = cl.geo[ref]
    others = []                                   # other pads of ref whose nets are already placed
    for n2, net2 in geo.nets.items():
        if n2 != num and net2 != GND:
            pts = [(x, y) for _r, _p, x, y in cl.pads_on(net2)]
            if pts:
                others.append((n2, pts))
    rects = list(cl.rects.items())
    bx0, by0, bx1, by1 = cl.bbox()
    best = None
    r = 0.0
    while r <= R_MAX:
        ring = _ring(r)
        for dx, dy in ring:
            cx, cy = tx + dx, ty + dy
            for rot in (0, 90, 180, 270):
                ox, oy = geo.rot[rot]["pads"][num][0]
                px, py = cx - ox, cy - oy
                me = cl.rect(ref, px, py, rot)
                if any(_overlap(me, rc) for _rr, rc in rects):
                    continue
                # compactness: growing the cluster's bounding box costs like moving away from the pin
                cost = r + GROW * (max(bx1, me[2]) - min(bx0, me[0]) + max(by1, me[3]) - min(by0, me[1])
                                   - (bx1 - bx0) - (by1 - by0))
                for n2, pts in others:
                    qx, qy = geo.rot[rot]["pads"][n2][0]
                    cost += 0.5 * min(((px + qx - x) ** 2 + (py + qy - y) ** 2) ** 0.5 for x, y in pts)
                if best is None or cost < best[0]:
                    best = (cost, px, py, rot)
        if best is not None and r > best[0]:     # every later ring costs at least r
            break
        r += STEP
    if best is None:
        return False
    cl.put(ref, best[1], best[2], best[3])
    if parts[ref]["lib"] == "Device:C":
        cl.used.add((tref, tpad))
    return True


_RINGS = {}


def _ring(r):
    """Grid points at distance ~r (one STEP wide ring), nearest the axes first."""
    if r not in _RINGS:
        n = int(round(r / STEP))
        pts = []
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                if max(abs(i), abs(j)) == n:
                    pts.append((i * STEP, j * STEP))
        pts.sort(key=lambda p: (abs(p[0]) + abs(p[1]), abs(p[0] * p[1])))
        _RINGS[r] = pts or [(0.0, 0.0)]
    return _RINGS[r]


def free_spot(cl, ref):
    """Nearest free spot to the cluster's bounding box centre (parts with nothing to attach to)."""
    x0, y0, x1, y1 = cl.bbox()
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    rects = list(cl.rects.values())
    r = 0.0
    while True:
        for dx, dy in _ring(r):
            me = cl.rect(ref, cx + dx, cy + dy, 0)
            if not any(_overlap(me, rc) for rc in rects):
                cl.put(ref, cx + dx, cy + dy, 0)
                return
        r += STEP


def build(region_parts, parts, geo, gap, net_size):
    """region_parts: [ref] in design order, all in one region -> [Cluster]."""
    order = {ref: i for i, ref in enumerate(region_parts)}
    # each decoupling cap belongs to the nearest preceding IC of its own group with a pad on its rail
    owner = {}
    for i, ref in enumerate(region_parts):
        if is_decap(ref, parts, geo):
            rail = (set(geo[ref].nets.values()) - {GND}).pop()
            for prev in reversed(region_parts[:i]):
                if parts[prev]["group"] != parts[ref]["group"]:
                    break
                if not is_passive(parts[prev]) and rail in geo[prev].nets.values():
                    owner[ref] = prev
                    break
    groups = {}
    for ref in region_parts:
        groups.setdefault(parts[ref]["group"], []).append(ref)
    clusters, loose = [], []
    for g, refs in groups.items():
        ics = [r for r in refs if not is_passive(parts[r])]
        if not ics:
            loose.append((g, refs))
            continue
        anchor = max(ics, key=lambda r: (not parts[r]["lib"].startswith("Connector"), len(geo[r].nets), -order[r]))
        cl = Cluster(g, anchor, geo, gap, order, owner)
        _fill(cl, [r for r in refs if r != anchor], parts, net_size)
        clusters.append(cl)
    # dissolve IC-less groups into the clusters holding their most specific net
    pending = [r for _g, refs in loose for r in refs]
    while pending:
        best = None
        for ref in pending:
            for cl in clusters:
                t = _target(cl, ref, parts, net_size)
                if not t:
                    continue
                # decoupling first (at the pins, smallest value first), indicators last, then by specificity
                cat = 0 if is_decap(ref, parts, geo) else (2 if (parts[ref]["group"] or "").startswith("Indicators")
                                                           else 1)
                rank = (cat, t[0][0], farads(parts[ref]["value"]) or 0, t[0][2])
                if best is None or rank < best[0]:
                    best = (rank, ref, cl)
        if best is None:
            break
        _size, ref, cl = best
        if not attach(cl, ref, parts, net_size):
            free_spot(cl, ref)
        pending.remove(ref)
    # what still has no home: its own groups become chained clusters (first part as anchor)
    left = {}
    for ref in pending:
        left.setdefault(parts[ref]["group"], []).append(ref)
    for g, refs in left.items():
        cl = Cluster(g, refs[0], geo, gap, order, owner)
        _fill(cl, refs[1:], parts, net_size)
        clusters.append(cl)
    return clusters


def farads(value):
    """'100nF 100V' -> 1e-7 (None if not a capacitance)."""
    v = value.split()[0]
    for suffix, k in (("pF", 1e-12), ("nF", 1e-9), ("uF", 1e-6), ("mF", 1e-3)):
        if v.endswith(suffix):
            try:
                return float(v[:-len(suffix)]) * k
            except ValueError:
                return None
    return None


def _hf_first(refs, parts, geo):
    """Within each run of consecutive capacitors on the same supply net, smallest value first, so the
    high-frequency cap is placed first and ends up nearest the pin (LMR38020 / TPS61092 guides)."""
    out, run, run_net = [], [], None
    def flush():
        out.extend(sorted(run, key=lambda r: farads(parts[r]["value"]) or 0))
    for ref in refs:
        nets = set(geo[ref].nets.values()) - {GND}
        is_cap = parts[ref]["lib"] == "Device:C" and len(nets) == 1
        net = next(iter(nets)) if is_cap else None
        if is_cap and net == run_net:
            run.append(ref)
            continue
        flush()
        run, run_net = ([ref], net) if is_cap else ([], None)
        if not is_cap:
            out.append(ref)
    flush()
    return out


def _dedicated(cl, ref, net_size):
    """Smallest net (<= 3 pads) linking `ref` to a placed IC pin: RT, CWD, PROG, BOOT, CL/CS, FB, LBI,
    Cext/Rext. None if there is none."""
    best = None
    for net in set(cl.geo[ref].nets.values()) - {GND}:
        if net_size[net] <= 3 and any(not is_passive_ref(cl, r) for r, _p, _x, _y in cl.pads_on(net)):
            best = min(best or 99, net_size[net])
    return best


def is_passive_ref(cl, ref):
    return cl.parts_lib[ref].startswith(PASSIVE)


def _fill(cl, refs, parts, net_size):
    """Priority, as the datasheet layout sections rank them: (1) decoupling caps, as soon as the IC
    they follow is placed, smallest value first, so they get the spots at the supply pins; (2) parts
    on a pin's own small net (timing, feedback, BOOT, current limit/sense); (3) the rest in design
    order."""
    cl.parts_lib = {r: parts[r]["lib"] for r in parts}
    todo = _hf_first(refs, parts, cl.geo)
    while todo:
        dec = [r for r in todo if is_decap(r, parts, cl.geo)
               and (cl.owner.get(r) in cl.pos if r in cl.owner else
                    (t := _target(cl, r, parts, net_size)) is not None and t[0][0] == 0)]
        if not dec:
            ded = [(d, cl.order[r], r) for r in todo if (d := _dedicated(cl, r, net_size))]
            dec = [min(ded)[2]] if ded else []
        if dec:
            ref = min(dec, key=lambda r: (farads(parts[r]["value"]) or 0, cl.order[r]))
            if not attach(cl, ref, parts, net_size):
                free_spot(cl, ref)
            todo.remove(ref)
            continue
        progress = False
        for ref in list(todo):
            if attach(cl, ref, parts, net_size):
                todo.remove(ref)
                progress = True
        if not progress:
            free_spot(cl, todo.pop(0))
