# 3D models

STEP models from KiCad's official 3D library,
[kicad-packages3D](https://gitlab.com/kicad/libraries/kicad-packages3D), copied here
so the board shows them on any machine without extra setup. The footprints refer to
them as `${KIPRJMOD}/bench_stim_wing.3dshapes/<name>.step`.

Licence: CC-BY-SA 4.0 with the KiCad libraries exception (see
<https://www.kicad.org/libraries/license/>). Unmodified.

`../gen/localize_3d.py --fetch` re-downloads anything missing and rewrites the
board's model paths.
