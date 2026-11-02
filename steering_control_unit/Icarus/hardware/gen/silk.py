"""Silkscreen for make_pcb.py (rev 0.5): as much as fits on both sides, with no silk on pads or
other silk, so DRC stays clean.

Front: cluster outlines and titles, reference designators where they fit, a function label at
every LED, the net name at every test point, vehicle-connector pin names, header labels.
Back (mirrored, readable with the board flipped): title block, connector pinout, LED legend,
safety and bring-up notes, cluster names, and an "x-ray" map of every reference that fits,
at its part's position. The back has no parts, so most references land there.
"""
import pcbnew

import design

REF_H, LABEL_H, TITLE_H, TABLE_H = 0.8, 0.8, 1.0, 1.0      # text heights (mm); JLC prints >= 0.8 mm
CELL = 4.0
LINE = 1.8                       # line pitch in text heights for back-side blocks

TITLES = {   # region -> (long title, short title)
    "entry": ("IN: POWER ENTRY + CAN", "IN: POWER + CAN"),
    "pass": ("PASS-THROUGH K1/K2 (NC = STOCK)", "PASS-THROUGH"),
    "inputs": ("IN: VEHICLE INPUTS + TEST", "IN: INPUTS"),
    "supply": ("SUPPLIES A | B: 5V0, 3V3", "SUPPLIES A | B"),
    "coil": ("OUT: COIL DRIVE + TEST", "OUT: COILS"),
    "overlay": ("OUT: OVERLAY (CH-A ANALOG)", "OUT: OVERLAY"),
    "backup": ("1S LiPo BACKUP", "BACKUP"),
    "ch_a": ("CH-A CONTROL (F280049C)", "CH-A CONTROL"),
    "ch_b": ("CH-B MONITOR (F280049C)", "CH-B MONITOR"),
    "guard": ("CH-C GUARD (NO SOFTWARE)", "CH-C GUARD"),
    "measure": ("MEASURE: MONITORS + TEST", "MEASURE"),
    "tp": ("TEST PADS: IN | OUT | MEASURED", "TEST PADS"),
}
LED_LABELS = {"LEDA_A": "HB A", "LEDA_B": "HB B", "LI_VBAT": "VBAT", "LI_5V0A": "5V A", "LI_5V0B": "5V B",
              "LI_3V3A": "3V3 A", "LI_3V3B": "3V3 B", "LI_FLT_A": "FLT A", "LI_FLT_B": "FLT B",
              "LI_CAN_A": "CAN A", "LI_CAN_B": "CAN B", "LI_ENG_A": "ENG A", "LI_PRM_B": "PRM B",
              "LI_SUP_A": "SUP A", "LI_SUP_B": "SUP B", "LI_GTRIP": "TRIP", "LI_K1": "K1 ON", "LI_K2": "K2 ON",
              "LI_CHG": "CHG", "LI_CHGD": "DONE", "LI_CHGPG": "PWR", "LI_LBO": "LOW"}
LED_LEGEND = [
    "LEDS (FRONT)",
    "VBAT 5V A/B 3V3 A/B  green: rail present",
    "HB A/B  green: firmware heartbeat",
    "FLT A/B  red: latched fault, SAFE",
    "CAN A/B  yellow: CAN frame",
    "ENG A  orange: overlay engaged",
    "PRM B  orange: CH-B grants its low side",
    "SUP A/B  red: watchdog or 5 V window fault",
    "TRIP  red: CH-C guard tripped (latched)",
    "K1 ON / K2 ON  blue: relay coil energised",
    "CHG yellow / DONE green / PWR green: charger",
    "LOW  red: LiPo below 3.0 V",
]
NOTES = [
    "SAFE STATE: relays off = stock torque path (NC)",
    "Overlay needs K1 AND K2: CH-A high side, CH-B low side, CH-C guard",
    "Relay coils are never on backup power",
    "12 V in: 5 A ATM blade fuse + 1.1 A PTC, SMBJ20A TVS",
    "LiPo: 1S pack WITH its own PCM, JST-PH, pin 1 = +",
    "Stack: F sig | In1 GND plane | In2 sig | B sig",
    "Test pads: bench only. Don't probe VT3/VT6 while driving",
]
TP_LABELS = {"VBAT_P": "VBAT", "SPEED_HI_N": "SPDHI", "DCMD_SEL": "DCMD", "COIL1_HI": "K1HI", "COIL2_HI": "K2HI",
             "OVL3_OUT": "OVL3", "OVL6_OUT": "OVL6", "SUP_OK_A": "SUPA", "SUP_OK_B": "SUPB", "EN_A_HW": "ENHW",
             "+5V0_A": "5V0A", "+5V0_B": "5V0B", "+3V3_A": "3V3A", "+3V3_B": "3V3B"}
J1_SHORT = {   # 3 letters: what fits between J1 and the cluster outline at 0.8 mm (full names on the back)
    "VBAT_IN": "BAT", "IG1_IN": "IG1", "CAN_H": "CNH", "CAN_L": "CNL", "VT3_EPS": "V3E", "VT3_SNS": "V3S",
    "VT6_SNS": "V6S", "VT6_EPS": "V6E", "TSGND_TAP": "TSG", "VCC2_TAP": "VC2", "WLP_IN": "WLP", "RT_IN": "RT",
    "VSP_IN": "VSP", "NEP_IN": "NEP", "DIS_NO_IN": "DNO", "DIS_NC_IN": "DNC", "IND_OUT": "IND", "GND": "GND"}
J1_LONG = {"VBAT_IN": "12 V battery (fused on board)", "GND": "ground", "IG1_IN": "ignition IG1",
           "CAN_H": "CAN high", "CAN_L": "CAN low", "VT3_EPS": "to EPS ECU B10", "VT3_SNS": "from torque sensor VT3",
           "VT6_SNS": "from torque sensor VT6", "VT6_EPS": "to EPS ECU B8", "TSGND_TAP": "sensor ground tap",
           "VCC2_TAP": "sensor 5 V tap", "WLP_IN": "EPS warning lamp", "RT_IN": "RT+ (EPS relay)",
           "VSP_IN": "vehicle speed pulses", "NEP_IN": "engine speed pulses", "DIS_NO_IN": "disengage NO",
           "DIS_NC_IN": "disengage NC", "IND_OUT": "driver indicator (low side)"}


def mm(v):
    return pcbnew.FromMM(v)


def to_mm(v):
    return pcbnew.ToMM(v)


class Index:
    """Grid hash of keep-out rectangles (mm)."""

    def __init__(self):
        self.cells = {}

    def _keys(self, r):
        for i in range(int(r[0] // CELL), int(r[2] // CELL) + 1):
            for j in range(int(r[1] // CELL), int(r[3] // CELL) + 1):
                yield i, j

    def add(self, r):
        for k in self._keys(r):
            self.cells.setdefault(k, []).append(r)

    def hit(self, r):
        for k in self._keys(r):
            for o in self.cells.get(k, ()):
                if r[0] < o[2] and o[0] < r[2] and r[1] < o[3] and o[1] < r[3]:
                    return True
        return False


def _bb(item, grow=0.0):
    b = item.GetBoundingBox()
    return (to_mm(b.GetLeft()) - grow, to_mm(b.GetTop()) - grow, to_mm(b.GetRight()) + grow, to_mm(b.GetBottom()) + grow)


class Silk:
    def __init__(self, board, ox, oy, w, h, regions, side=None):
        self.board, self.ox, self.oy, self.w, self.h, self.regions = board, ox, oy, w, h, regions
        self.side = side or {r: "top" for r in regions}           # region -> "top" / "bottom"
        self.double = "bottom" in self.side.values()
        self.front, self.back = Index(), Index()
        self.pads = {"front": Index(), "back": Index()}   # copper only: outlines break around it
        self.placed = {"front": 0, "back": 0, "skipped": 0}
        for fp in board.GetFootprints():
            cy = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd)
            if cy.OutlineCount():
                b = cy.BBox()
                (self.back if fp.IsFlipped() else self.front).add(
                    (to_mm(b.GetLeft()) - 0.05, to_mm(b.GetTop()) - 0.05,
                     to_mm(b.GetRight()) + 0.05, to_mm(b.GetBottom()) + 0.05))
            for p in fp.Pads():
                r = _bb(p, 0.25)
                if p.IsOnLayer(pcbnew.F_Cu) or p.IsOnLayer(pcbnew.F_Mask):
                    self.front.add(r)
                    self.pads["front"].add(r)
                if p.IsOnLayer(pcbnew.B_Cu) or p.IsOnLayer(pcbnew.B_Mask) or \
                        p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                    self.back.add(r)
                    self.pads["back"].add(r)

    # --- helpers --------------------------------------------------------------------------
    def inside(self, r, margin=0.6):
        return (r[0] > self.ox + margin and r[1] > self.oy + margin and
                r[2] < self.ox + self.w - margin and r[3] < self.oy + self.h - margin)

    def text(self, s, x, y, h, layer, angle=0, thick=None):
        t = pcbnew.PCB_TEXT(self.board)
        t.SetText(s)
        t.SetLayer(layer)
        t.SetTextSize(pcbnew.VECTOR2I(mm(h), mm(h)))
        t.SetTextThickness(mm(thick or max(0.15, h * 0.15)))
        t.SetTextAngleDegrees(angle)
        if layer == pcbnew.B_SilkS:
            t.SetMirrored(True)
        t.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        return t

    def try_place(self, t, spots, side):
        """Move text item t through spots [(x, y, angle)]; keep the first that's free. -> bool"""
        idx = self.front if side == "front" else self.back
        for x, y, a in spots:
            t.SetTextAngleDegrees(a)
            t.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
            r = _bb(t, 0.1)
            if self.inside(r) and not idx.hit(r):
                idx.add(r)
                self.placed[side] += 1
                return True
        self.placed["skipped"] += 1
        return False

    def add_text(self, s, spots, h, side, thick=None):
        layer = pcbnew.F_SilkS if side == "front" else pcbnew.B_SilkS
        t = self.text(s, spots[0][0], spots[0][1], h, layer, spots[0][2], thick)
        if self.try_place(t, spots, side):
            self.board.Add(t)
            return t
        return None

    @staticmethod
    def around(fp_rect, h, extra=0.0):
        """Candidate centres just outside a courtyard: above, below, right, left (rotated), corners."""
        x0, y0, x1, y1 = fp_rect
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        d = h * 0.85 + extra
        spots = [(cx, y0 - d, 0), (cx, y1 + d, 0), (x1 + d, cy, 90), (x0 - d, cy, 90)]
        for k in (1, 2, 3):                      # walk outward a little
            e = k * 0.7
            spots += [(cx, y0 - d - e, 0), (cx, y1 + d + e, 0), (x1 + d + e, cy, 90), (x0 - d - e, cy, 90),
                      (x0, y0 - d - e + 0.7, 0), (x1, y1 + d + e - 0.7, 0)]
        return spots

    # --- front ---------------------------------------------------------------------------
    def outlines(self, title_h):
        """Cluster outlines in the routing gaps, titles in the reserved strip at each region top. A
        bottom-side region gets its outline and title on the back silk, reading from its left edge
        with the board flipped (= its top-view right edge)."""
        for name, (x0, y0, x1, y1) in self.regions.items():
            bottom = self.side.get(name) == "bottom"
            layer, idx = (pcbnew.B_SilkS, self.back) if bottom else (pcbnew.F_SilkS, self.front)
            pts = [(x0 - 0.5, y0 - 0.5), (x1 + 0.5, y0 - 0.5), (x1 + 0.5, y1 + 0.5), (x0 - 0.5, y1 + 0.5)]
            pads = self.pads["back" if bottom else "front"]
            for (ax, ay), (bx, by) in zip(pts, pts[1:] + pts[:1]):
                # 0.4 mm pieces; a piece over copper (e.g. the other side's through-hole via rings) is left out
                n = max(1, int(((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5 / 0.4))
                ok = []
                for k in range(n):
                    x0, y0 = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
                    x1, y1 = ax + (bx - ax) * (k + 1) / n, ay + (by - ay) * (k + 1) / n
                    ok.append(not pads.hit((self.ox + min(x0, x1) - 0.25, self.oy + min(y0, y1) - 0.25,
                                            self.ox + max(x0, x1) + 0.25, self.oy + max(y0, y1) + 0.25)))
                k = 0
                while k < n:
                    if not ok[k]:
                        k += 1
                        continue
                    k0 = k
                    while k < n and ok[k]:
                        k += 1
                    sx, sy = ax + (bx - ax) * k0 / n, ay + (by - ay) * k0 / n
                    ex, ey = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
                    s = pcbnew.PCB_SHAPE(self.board)
                    s.SetShape(pcbnew.SHAPE_T_SEGMENT)
                    s.SetStart(pcbnew.VECTOR2I(mm(self.ox + sx), mm(self.oy + sy)))
                    s.SetEnd(pcbnew.VECTOR2I(mm(self.ox + ex), mm(self.oy + ey)))
                    s.SetLayer(layer)
                    s.SetWidth(mm(0.15))
                    self.board.Add(s)
                    idx.add((self.ox + min(sx, ex) - 0.2, self.oy + min(sy, ey) - 0.2,
                             self.ox + max(sx, ex) + 0.2, self.oy + max(sy, ey) + 0.2))
            for title in TITLES.get(name, (name.upper(),)):
                if not self.tp_titles:            # test pads aren't in the clusters: no "+ TEST" in titles
                    title = title.replace(" + TEST", "")
                ax = x1 - 0.3 if bottom else x0 + 0.3
                t = self.text(title, self.ox + ax, self.oy + y0 + title_h / 2, TITLE_H, layer)
                grow(t, -1 if bottom else +1)
                r = _bb(t, 0.05)
                if r[0] > self.ox + x0 + 0.2 and r[2] < self.ox + x1 - 0.2 and not idx.hit(r):
                    idx.add(r)
                    self.board.Add(t)
                    self.placed["back" if bottom else "front"] += 1
                    break

    def connector_labels(self, j1):
        x1 = _cy(j1)[2]
        for p in j1.Pads():
            net = p.GetNetname().split("/")[-1]
            y = to_mm(p.GetPosition().y)
            t = self.text(f"{J1_SHORT.get(net, net)}", x1 + 0.3, y, 0.8, pcbnew.F_SilkS)
            grow(t, +1)
            if self.try_place(t, [(x1 + 0.3, y, 0), (x1 + 0.2, y, 0), (x1 + 0.5, y, 0)], "front"):
                self.board.Add(t)
            t = self.text(f"{p.GetNumber()} {net}", x1 + 0.3, y, 0.8, pcbnew.B_SilkS)
            grow(t, +1)                          # into the board; from the back it ends at the pin
            if self.try_place(t, [(x1 + 0.3, y, 0), (x1 + 0.6, y, 0)], "back"):
                self.board.Add(t)

    def footprint_labels(self):
        """LED function labels, test-point net names, header names, then reference designators."""
        fps = sorted(self.board.GetFootprints(), key=lambda f: -_area(f))
        for fp in fps:
            ref = fp.GetReference()
            r = _cy(fp)
            lib = fp.GetFPIDAsString()
            label = None
            if lib.startswith("LED_SMD"):
                anode = next((p.GetNetname().split("/")[-1] for p in fp.Pads() if p.GetNumber() == "2"), "")
                label = LED_LABELS.get(anode)
            elif lib.startswith("TestPoint"):
                label = TP_LABELS.get(fp.GetValue(), fp.GetValue().replace("_", "").replace("+", ""))[:6]
            if label:
                spots = self.around(r, LABEL_H)
                if lib.startswith("TestPoint"):          # beside the pad (floater configs reserve it), else below
                    cx, cy = (r[0] + r[2]) / 2, (r[1] + r[3]) / 2
                    spots = [(r[2] + 0.5 + len(label) * 0.34, cy, 0),
                             (cx, r[3] + 0.45 + len(label) * 0.42 + 0.3, 90)] + spots
                self.add_text(label, spots, LABEL_H, "front")
        for fp in fps:
            if fp.GetFPIDAsString().startswith(("TestPoint", "MountingHole")):
                continue
            under = fp.IsFlipped()                # bottom-side part: its reference goes on the back silk
            f = fp.Reference()
            f.SetLayer(pcbnew.B_SilkS if under else pcbnew.F_SilkS)
            f.SetMirrored(under)
            f.SetTextSize(pcbnew.VECTOR2I(mm(REF_H), mm(REF_H)))
            f.SetTextThickness(mm(0.15))
            f.SetKeepUpright(True)
            if self.try_place(f, self.around(_cy(fp), REF_H), "back" if under else "front"):
                f.SetVisible(True)
            else:
                f.SetLayer(pcbnew.B_Fab if under else pcbnew.F_Fab)
                f.SetVisible(True)
                f.SetTextAngleDegrees(0)
                c = fp.GetPosition()
                f.SetPosition(c)

    # --- back ----------------------------------------------------------------------------
    def block(self, lines, x, y, h, x_range, y_range):
        """Place a text block on the back that reads left-aligned with the board flipped: its lines
        share their top-view RIGHT edge at x and grow leftward. Searches left/down in 2 mm steps
        from (x, y) until every line is free. Returns True if placed."""
        for dy in [i * 2.0 for i in range(int(y_range / 2) + 1)]:
            for dx in [-i * 2.0 for i in range(int(x_range / 2) + 1)]:
                items = []
                ok = True
                for k, s in enumerate(lines):
                    t = self.text(s, self.ox + x + dx, self.oy + y + dy + k * h * LINE, h, pcbnew.B_SilkS)
                    grow(t, -1)
                    r = _bb(t, 0.15)
                    if not self.inside(r) or self.back.hit(r):      # lines at LINE pitch can't overlap each other
                        ok = False
                        break
                    items.append((t, r))
                if ok:
                    for t, r in items:
                        self.back.add(r)
                        self.board.Add(t)
                    self.placed["back"] += len(items)
                    return True
        return False

    def back_side(self, j1):
        W, H = self.w, self.h
        self.block([f"ICARUS rev {design.REV}  steering torque overlay",
                    "Honda Insight G1 EPS: VT3/VT6 interceptor",
                    "ISO 26262 development item: SG1 ASIL D = B(D)+B(D)",
                    "Not road-legal until validated"], W - 8.0, H - 14.0, 1.2, 80, 10)
        pins = [(int(p.GetNumber()), p.GetNetname().split("/")[-1]) for p in j1.Pads()]
        lines = ["J1 VEHICLE HARNESS (pin: net, function)"] + \
                [f"{n:2d}  {net}  {J1_LONG.get(net, '')}" for n, net in sorted(pins)]
        self.block(lines, 48.0, 6.0, TABLE_H, 40, 40)
        self.block(LED_LEGEND, W - 4.0, 8.0, TABLE_H, 30, 70)
        self.block(["NOTES"] + NOTES, W - 4.0, 40.0, TABLE_H, 30, 60)
        if not self.double:                       # one-sided board: name every cluster on the back too
            for name, (x0, y0, x1, y1) in self.regions.items():
                self.add_text(TITLES.get(name, (name,))[1], [(self.ox + (x0 + x1) / 2, self.oy + y0 + 1.5 + k, 0)
                                                            for k in range(0, 8)], 1.2, "back")
        # x-ray map: every top-side reference at its part, mirrored, where it fits
        for fp in sorted(self.board.GetFootprints(), key=lambda f: -_area(f)):
            if fp.GetFPIDAsString().startswith("MountingHole") or fp.IsFlipped():
                continue
            r = _cy(fp)
            cx, cy = (r[0] + r[2]) / 2, (r[1] + r[3]) / 2
            spots = [(cx, cy, 0), (cx, cy, 90), (cx, cy - 0.5, 0), (cx, cy + 0.5, 0), (cx - 0.6, cy, 90),
                     (cx + 0.6, cy, 90)]
            self.add_text(fp.GetReference(), spots, 0.8, "back")


def grow(t, direction):
    """Justify text t so it extends from its anchor toward +x (direction +1) or -x (-1) in the
    top view; mirrored back-side text flips the meaning of left/right, so test both."""
    ax = to_mm(t.GetPosition().x)
    for j in (pcbnew.GR_TEXT_H_ALIGN_LEFT, pcbnew.GR_TEXT_H_ALIGN_RIGHT):
        t.SetHorizJustify(j)
        r = _bb(t)
        if (direction > 0 and r[0] >= ax - 0.3) or (direction < 0 and r[2] <= ax + 0.3):
            return
    raise AssertionError("text justification")


def _ov(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def _cy(fp):
    cy = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd)
    b = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False)
    return (to_mm(b.GetLeft()), to_mm(b.GetTop()), to_mm(b.GetRight()), to_mm(b.GetBottom()))


def _area(fp):
    r = _cy(fp)
    return (r[2] - r[0]) * (r[3] - r[1])


def apply(board, ox, oy, w, h, regions, title_h, j1_ref, side=None, tp_titles=True):
    """Regions are in board coordinates (mm, origin top-left of the board); side: region -> top/bottom.
    tp_titles: the test-pad rows sit in their clusters, so titles say "+ TEST"."""
    s = Silk(board, ox, oy, w, h, regions, side)
    s.tp_titles = tp_titles and "tp" not in regions
    j1 = board.FindFootprintByReference(j1_ref)
    s.outlines(title_h)
    s.connector_labels(j1)
    s.footprint_labels()
    s.back_side(j1)
    return s.placed
