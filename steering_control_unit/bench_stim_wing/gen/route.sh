#!/bin/sh
# Rebuild the whole board: schematic -> netlist -> parity check -> placement ->
# Freerouting -> import + manual stubs + GND pours + DRC -> fab files + renders.
#
# Needs KiCad 7+ (kicad-cli, and a Python that can import pcbnew) and
# Freerouting 1.9.0 on Java 17+. Freerouting 1.9 needs a display even in batch
# mode: on a headless Linux box run it under xvfb-run (as below).
#
#   JAR=/path/to/freerouting-1.9.0.jar PY=/usr/bin/python3 sh route.sh          # full re-route
#   PY=/usr/bin/python3 sh route.sh --keep-routing                                # reuse bench_stim_wing.ses
#
# A full re-route gives a different result each time, and placement.MANUAL_ROUTES
# belongs to the committed bench_stim_wing.ses: after re-routing, empty
# MANUAL_ROUTES, check DRC, and use find_via.py for anything left open.
set -e
cd "$(dirname "$0")"
PY="${PY:-python3}"
JAR="${JAR:-/opt/freerouting/freerouting-1.9.0.jar}"
"$PY" make_sch.py
kicad-cli sch export netlist -o ../bench_stim_wing.net ../bench_stim_wing.kicad_sch
kicad-cli sch export pdf -o ../doc/bench_stim_wing_schematic.pdf ../bench_stim_wing.kicad_sch
"$PY" check_netlist.py
"$PY" make_pcb.py place
if [ "$1" != "--keep-routing" ]; then
  # -oit 100 stops the post-route optimiser (it otherwise runs for hours)
  xvfb-run -a "${JAVA:-java}" -jar "$JAR" -de route/bench_stim_wing.dsn -do bench_stim_wing.ses -mp 40 -oit 100
fi
"$PY" make_pcb.py finish
"$PY" export_fab.py
sh render_doc.sh
