#!/bin/sh
# Auto-route the wing with Freerouting (needs Java 25+ for freerouting 2.2.0).
# Usage: JAVA=/path/to/java25/bin/java sh route.sh
set -e
cd "$(dirname "$0")"
PY="/c/Program Files/KiCad/10.0/bin/python.exe"
JAR="${JAR:-$HOME/Documents/KiCad/10.0/3rdparty/plugins/app_freerouting_kicad-plugin/jar/freerouting-2.2.0.jar}"
"$PY" make_pcb.py place
"${JAVA:-java}" -jar "$JAR" -de route/eps_tap_wing.dsn -do route/eps_tap_wing.ses -mp 100 --gui.enabled=false
"$PY" make_pcb.py finish
"/c/Program Files/KiCad/10.0/bin/kicad-cli.exe" pcb drc --schematic-parity --severity-all -o route/drc.rpt ../eps_tap_wing.kicad_pcb
"$PY" export_fab.py
