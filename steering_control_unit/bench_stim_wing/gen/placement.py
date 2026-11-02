"""Board placement and silkscreen for the bench stim wing.

Coordinates are mm, origin = board top-left corner seen from the top, y down.
(x, y, rotation, side): side "F" = top (faces the OLED above), "B" = back (faces
the Adalogger below). Back-side parts are all <= 1.25 mm tall: the Adalogger's
CR1220 holder (x ~3-17) and microSD socket (x ~27-40) sit under them, ~8.5 mm
away. Tall parts (relays 5.2 mm, inductor 3 mm, connectors) are on top, where
the OLED FeatherWing's underside is bare.

Keep-outs: female header bodies on top at y < 2.6 (x 15.2-45.7) and y > 20.3
(x 5.1-45.7); header pads on the back at y < 2.2 and y > 20.7; corner holes
2.54 mm in from each corner.
"""
import design

F, B = "F", "B"
PLACE = {
    "J1": (6.35, 21.59, 90, F),          # header pads: pin 1 (RST) bottom-left
    # --- top: connectors ----------------------------------------------------
    "J2": (12.7, 6.25, 180, F),          # 12 V IN, opening up through the top edge
    "J3": (2.95, 11.43, 270, F),         # SAS / CAN, opening out of the left end
    "J4": (47.85, 11.43, 90, F),         # TORQUE, opening out of the right end
    # --- top: relays --------------------------------------------------------
    "K2": (29.3, 11.43, 0, F),
    "K1": (39.2, 11.43, 0, F),
    # --- top: 12 V input and 5 V buck (signal flow left to right, top to bottom)
    "F1": (19.0, 4.1, 0, F),
    "D1": (19.0, 7.25, 0, F),
    "D2": (11.5, 10.6, 0, F),
    "C1": (11.5, 14.05, 0, F),
    "C2": (7.1, 14.3, 90, F),
    "U1": (18.0, 11.45, 0, F),
    "C3": (22.4, 11.6, 90, F),
    "C4": (18.0, 13.95, 0, F),
    "D4": (16.9, 16.2, 0, F),
    "L1": (22.0, 16.9, 90, F),
    "D6": (11.6, 16.4, 0, F),
    "C9": (8.4, 16.6, 90, F),
    "C6": (17.0, 18.75, 0, F),
    "C7": (13.4, 18.75, 0, F),
    "C8": (9.8, 18.75, 0, F),
    # top-left corner: 12 V indicator
    "D3": (6.3, 3.6, 0, F),
    "R1": (6.3, 1.5, 0, F),
    # band below the relays (y 16.7-20.3): SAS and IG1 high-side switches
    "F2": (27.1, 18.5, 0, F),
    "Q1": (31.6, 18.5, 0, F),
    "F4": (36.1, 18.5, 0, F),
    "Q3": (40.6, 18.5, 0, F),
}
# Indicator LEDs in one row above the relays (y 2.6-6.2), each with its resistor
# alongside. Order = the legend printed on the back.
LEDS = [("D5", "R5", "5V"), ("D7", "R10", "SAS"), ("D8", "R16", "IG1"), ("D9", "R17", "ARM"),
        ("D11", "R21", "TQ"), ("D13", "R23", "SIG"), ("D14", "R28", "NEP"), ("D15", "R33", "VSP")]
for i, (d, r, _lbl) in enumerate(LEDS):
    x = 23.6 + 2.65 * i
    PLACE[d] = (x, 4.4, 90, F)
    PLACE[r] = (x + 1.25, 4.4, 90, F)

# --- back side -----------------------------------------------------------------
PLACE.update({
    "J5": (47.85, 11.43, 90, B),         # ECM / IG1 / ARM, opening out of the right end
    # ICs along the top of the back, under the LED row
    "U4": (19.8, 5.9, 0, B),             # expander
    "U2": (29.0, 5.0, 0, B),             # DAC
    "U3": (37.2, 5.9, 0, B),             # op-amp, under K1 / J4
    "C13": (19.8, 9.4, 0, B),
    "R37": (15.9, 9.4, 0, B),
    "R35": (24.2, 4.1, 90, B),
    "R36": (24.2, 6.1, 90, B),
    "D18": (14.4, 5.0, 90, B),
    "C10": (29.0, 7.5, 0, B),
    "R34": (26.9, 7.9, 90, B),
    "C12": (32.65, 8.4, 90, B),
    "JP1": (5.8, 6.2, 90, B),
    "F3": (8.0, 2.0, 0, B),
    # buck small parts, near U1 (which is on top at 18.0, 11.45)
    "R4": (11.0, 8.4, 90, B),
    "R2": (12.1, 8.4, 90, B),
    "R3": (13.2, 8.4, 90, B),
    "C5": (14.3, 8.4, 90, B),
})
# Op-amp gain/series resistors: 2 rows of 8 under U3, channel by channel.
for i in range(4):
    for k in range(4):
        n = 40 + 4 * i + k
        PLACE[f"R{n}"] = (34.0 + 2.0 * i, 9.6 + 2.0 * k, 90, B)   # column = channel A-D
PLACE["D16"] = (38.3, 18.4, 0, B)
PLACE["D17"] = (42.3, 18.4, 0, B)

# Drivers: one row of SOT-23s, their resistors underneath.
_ROW_Q, _ROW_R1, _ROW_R2 = 11.7, 14.4, 16.3
_DRIVERS = [  # (x, Q, [row-1 0402s, rot 90], [row-2 parts, rot 0])
    (6.6, "Q2", ["R8", "R9", "R6"], ["R7"]),
    (10.7, "Q4", ["R13", "R12", "R11"], []),
    (14.8, "Q5", ["R18", "R19"], []),
    (18.9, "Q8", ["R24", "R25", "R27"], ["R26"]),
    (23.0, "Q9", ["R29", "R30", "R32"], ["R31"]),
    (27.1, "Q7", ["R22"], ["D12"]),
    (31.2, "Q6", ["R20"], ["D10"]),
]
for x, q, r1, r2 in _DRIVERS:
    PLACE[q] = (x, _ROW_Q, 0, B)
    for k, ref in enumerate(r1):
        PLACE[ref] = (x - 1.1 + 1.1 * k, _ROW_R1, 90, B)
    for k, ref in enumerate(r2):
        PLACE[ref] = (x + 0.4, _ROW_R2, 0, B)
# Test pads: one row along the back, GND at both ends; labels above each pad.
TP_X0, TP_PITCH, TP_Y = 6.5, 2.05, 19.0
for i in range(len(design.TEST_PADS)):
    PLACE[f"TP{i + 1}"] = (TP_X0 + TP_PITCH * i, TP_Y, 0, B)


def silkscreen(board, text, pcbnew):
    FS, BS = pcbnew.F_SilkS, pcbnew.B_SilkS
    # Top: only what fits between parts.
    text(board, "12V", 6.0, 5.05, 0.6, layer=FS, thick=0.1)
    text(board, "TORQUE", 43.95, 11.43, 0.8, layer=FS, angle=90, thick=0.12)
    # Back: 12 V IN polarity at J2's pads, safety note, LED legend, branding,
    # J5 pinout and test pad names. Mirrored, so it reads from below.
    text(board, "+", 12.7, 4.5, 0.8, layer=BS, thick=0.14)
    text(board, "-", 10.7, 4.5, 0.8, layer=BS, thick=0.14)
    text(board, "ARM OPEN = CAR", 4.3, 12.9, 0.75, layer=BS, angle=90, thick=0.12)
    for i, (_net, lbl) in enumerate(design.TEST_PADS):
        text(board, lbl, TP_X0 + TP_PITCH * i, 17.55, 0.5, layer=BS, thick=0.08)
    for i, lbl in enumerate(["IG1", "NEP C", "NEP E", "GND", "VSP E", "VSP C", "ARM+", "ARM"]):
        text(board, lbl, 44.3, 7.05 + 1.25 * i, 0.45, layer=BS, thick=0.08, justify="left")
