#!/bin/sh
# Rebuild the whole board: schematic -> netlist -> placement -> Freerouting -> pours -> DRC -> fab.
# Needs KiCad 7+ (kicad-cli, and the pcbnew Python module) and Freerouting 2.x with Java 21+.
# Usage: JAR=/path/to/freerouting.jar PY=/usr/bin/python3 sh route.sh
set -e
cd "$(dirname "$0")"
PY="${PY:-python3}"
JAR="${JAR:-/opt/freerouting/freerouting.jar}"
"$PY" make_sch.py
kicad-cli sch export netlist -o ../bench_stim_wing.net ../bench_stim_wing.kicad_sch
"$PY" check_netlist.py
"$PY" make_pcb.py place
"${JAVA:-java}" -jar "$JAR" -de route/bench_stim_wing.dsn -do route/bench_stim_wing.ses -mp 100 --gui.enabled=false
"$PY" make_pcb.py finish
"$PY" export_fab.py
