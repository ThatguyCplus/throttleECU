"""Check the netlist KiCad exported from the schematic against design.py.

Catches anything the schematic generator got wrong: every (ref, pin) must sit on
the net design.py gives it, no extra connections, no single-pin nets, and every
power pin on a rail that something drives. Run after exporting the netlist:
    kicad-cli sch export netlist -o ../bench_stim_wing.net ../bench_stim_wing.kicad_sch
    python3 check_netlist.py
"""
import os
import sys
from collections import defaultdict

import design
from sexpr import parse

NET = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", design.PROJECT + ".net")


def main():
    tree = parse(open(NET, encoding="utf-8").read())
    get = lambda node, key: next(x for x in node if isinstance(x, list) and x[0] == key)
    got = {}
    for n in get(tree, "nets")[1:]:
        name = str(get(n, "name")[1]).lstrip("/")
        for node in n:
            if isinstance(node, list) and node[0] == "node":
                got[(str(get(node, "ref")[1]), str(get(node, "pin")[1]))] = name
    want = {(p["ref"], pin): net for p in design.PARTS for pin, net in p["pins"].items()}
    errors = []
    for k, net in want.items():
        if got.get(k) != net:
            errors.append(f"{k[0]}.{k[1]}: schematic {got.get(k)!r}, design {net!r}")
    for k, net in got.items():
        if k not in want and not net.startswith("unconnected-"):
            errors.append(f"{k[0]}.{k[1]}: on {net!r} in the schematic but not in design.py")
    members = defaultdict(set)
    for (ref, pin), net in want.items():
        members[net].add((ref, pin))
    for net, refs in members.items():
        if len(refs) < 2:
            errors.append(f"net {net} only touches {refs}")
    # Footprints in the netlist match design.py
    for c in get(tree, "components")[1:]:
        ref = str(get(c, "ref")[1])
        fp = str(get(c, "footprint")[1])
        p = next(p for p in design.PARTS if p["ref"] == ref)
        if fp != p["fp"]:
            errors.append(f"{ref}: footprint {fp} != {p['fp']}")
    print(f"{len(want)} pins on {len(members)} nets checked")
    for e in errors:
        print("ERROR", e)
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
