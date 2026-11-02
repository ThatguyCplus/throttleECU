#!/bin/sh
# Auto-route Icarus with Freerouting 2.2.0 (needs Java 25+).
# Usage (from hardware/): JAVA=/path/to/java25/bin/java sh gen/route.sh [passes]
set -e
cd "$(dirname "$0")"
JAR="${JAR:-$HOME/Documents/KiCad/10.0/3rdparty/plugins/app_freerouting_kicad-plugin/jar/freerouting-2.2.0.jar}"
# analytics and telemetry off on the command line: Freerouting's own settings file
# (%TEMP%/freerouting/freerouting.json) has them on by default, and the board stays on this machine
"${JAVA:-java}" -Xmx8g -jar "$JAR" -de route/icarus.dsn -do route/icarus.ses -mp "${1:-60}" -mt 8 --gui.enabled=false     --usage_and_diagnostic_data.disable_analytics=true --profile.allow_telemetry=false
