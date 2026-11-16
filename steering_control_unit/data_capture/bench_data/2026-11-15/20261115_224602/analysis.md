# Lock-to-lock session 20261115_224602

60 s, NEP 900 rpm (assist on). 103 sync edges, 0.44 ms rms. 800 Sa/s per channel.

- Lock to lock: -596 deg to +616 deg (1212 deg, about 3.4 turns). Centre about +10 deg.
- Rest: VT3 2.562 V, VT6 2.580 V.
- Pushing into the stops: VT3 1.21 to 3.80 V, i.e. -1.35 V / +1.24 V from rest. VT6 the same
  (VT6 = 1.003 x VT3 + 0.010 V, correlation 0.9998).
  - Left stop (6.4 s held): median -1.25 V, peak -1.34 V.
  - Right stop (5.9 s held): median +0.79 V, peak +1.23 V.
- Both ends flatten at about 1.2 V and 3.8 V while pushing. That looks like the output (or the
  torsion bar travel) reaching its limit rather than the scope clipping: the scope range here
  covers about -20..+20 V and no CLIP was raised.
- Peak swings are close to symmetric (-1.35 / +1.24 V), so the left/right difference in the
  earlier run was mostly effort, not a sensor offset.
- While turning (not at a stop) the signal sits at about +0.2 to +0.3 V turning right and
  -0.35 to -0.4 V turning left, with almost no dependence on angle (torque_vs_angle panel):
  no centring spring load on this bench rack, just friction and assist.
- NEP: 1 irregular period out of 1319.

Plot: lock_to_lock.png
