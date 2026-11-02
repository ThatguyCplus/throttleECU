#!/bin/sh
# Layer renders for ../doc (KiCad 7's CLI has no 3D render): top and back views
# as SVG, converted to PNG with headless Chromium (Playwright) when it's available.
set -e
cd "$(dirname "$0")"
PCB=../bench_stim_wing.kicad_pcb
kicad-cli pcb export svg --page-size-mode 2 --exclude-drawing-sheet \
  -l "Edge.Cuts,F.Cu,F.Mask,F.Silkscreen,F.Fab" -o ../doc/top.svg $PCB >/dev/null
kicad-cli pcb export svg --page-size-mode 2 --exclude-drawing-sheet --mirror \
  -l "Edge.Cuts,B.Cu,B.Mask,B.Silkscreen,B.Fab" -o ../doc/back.svg $PCB >/dev/null
node svg2png.js ../doc/top.svg ../doc/back.svg 2>/dev/null || echo "no Playwright: SVGs only"
