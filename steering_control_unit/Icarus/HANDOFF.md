# Icarus: agent handoff

Read this first if you're picking the work up. The facts below are kept current at each chunk boundary.

## What this is

**Icarus** is a dual-channel steering torque-overlay unit for the 1st-gen Honda Insight EPS. It intercepts the torque-sensor lines VT3/VT6 and adds a bounded ±Δ offset. The stock EPS still drives the motor.

- **Architecture:** CH-A (control, F280049C-Q1), CH-B (monitor, F280049C-Q1) and CH-C (an analog guard with no software).
- **Relays:** cascaded relays K1/K2. With no power, or any channel saying stop, the OEM sensor is connected straight through.
- **Safety target:** developed toward ISO 26262, ASIL D for SG1, decomposed B(D)+B(D).
- **Docs:** `docs/00`–`07`; the safety findings are in `docs/06`.

## Rules (from the user)

- **Throttle folder is read-only.** Don't modify anything in `f411testingreliablitymotor/` (the throttle ECU); reading and copying from it is fine.
- **No ordering or publishing.** Downloads need the user's permission.
- **When something is ambiguous or wrong, make the safe move** and say so.
- **Overbuilt for safety is the goal.** When unsure, add redundancy rather than trim it.
- **Commit only when the user asks.** Use the attribution line given in the session.

## How to build

| Step | Command (from `hardware/`) |
|---|---|
| Schematic | `python gen/make_sch.py` (writes all `.kicad_sch` files, `icarus.kicad_sym`, lib tables) |
| ERC | `kicad-cli sch erc --severity-all -o erc.rpt icarus.kicad_sch` (expect 0) |
| Netlist | `kicad-cli sch export netlist --format kicadsexpr -o icarus.net icarus.kicad_sch` |
| Netlist check | compare against `design.py` (snippet in git history / earlier turns; expect 0 mismatches) |
| Safety | `cd ../safety && python analyze.py` → `docs/07_FMEDA_FTA.md` |
| PCB place | `"C:\Program Files\KiCad\10.0\bin\python.exe" gen/make_pcb.py place` → `icarus.kicad_pcb` + `gen/route/icarus.dsn` |
| Route | `JAVA=<jre25>/bin/java sh gen/route.sh` (Freerouting 2.2.0 needs Java 25) |
| PCB finish | `... make_pcb.py finish` (imports SES, GND pours on F/In2/B, fills) |
| DRC | `kicad-cli pcb drc --schematic-parity --severity-all -o gen/route/drc.rpt icarus.kicad_pcb` |

**Tool paths:**
- **Java 25 (Temurin JRE):** the session scratchpad, `.../scratchpad/jre25/jdk-25.0.5+8-jre/bin/java.exe`. It's temporary; re-download with the user's OK if it's gone.
- **Freerouting:** `%USERPROFILE%/Documents/KiCad/10.0/3rdparty/plugins/app_freerouting_kicad-plugin/jar/freerouting-2.2.0.jar`.

## State (2026-11-02)

| Item | State |
|---|---|
| Schematic | **rev 0.7**: 567 parts, CH-B = TMS570LS0432-Q1 (diverse monitor), ERC 0, 292/292 nets match |
| Safety analysis | SPFM 99.99 %, LFM 98.8 %, PMHF 0.086 FIT (placeholder rates); F14-F17 in docs/06 |
| PCB | **rev 0.7, 100 × 100 mm, routed.** Config A (`ICARUS_CONFIG=A`), stack SGSS (F sig · In1 GND · In2 sig · B sig). Parts on both sides; test pads and LEDs float next to their signals, on top. **DRC `--severity-all`: 0 violations, 0 unconnected, 0 parity** (`gen/route/drc_rev07_final.rpt`). Fab outputs and renders in `hardware/fab/` and `hardware/doc/`. Board = `gen/route/A_SGSS_cskpin_routed.kicad_pcb` |
| Earlier boards | Routed rev 0.5 (152 × 115) kept in `gen/route/icarus_rev05_routed.kicad_pcb` + `gen/route/fab_rev05/`. BOM review in docs/08 |
| Last commit | `923742b` (rev 0.2). Rev 0.3 to 0.7 are **uncommitted** |

## Rev 0.4 additions (this chunk)

- **Brought over from the throttle ECU:**
  - TCAN3404-Q1 (already in), TPS3702 window supervisors (already in), SMBJ20A, SQJ147.
  - New: a **1812 PTC** behind the blade fuse, like throttle F1.
- **SM-23, speed-dependent authority in hardware.**
  - CH-C speed stage: VSP → squarer → one-shot → RC average → comparator. Fully analog, no software.
  - Above v_switch (TBD-DATA, ~40 mph), it drives `SPEED_HI_N` low. A TS5A3159 then feeds the summers a half-amplitude command.
  - A floating or failed control line also means half authority; an open switch means zero.
  - Firmware culls Δ further with its own speed table. Both MCUs read `SPDHI` back.
- **1S LiPo backup (`backup` sheet).**
  - JST-PH connector → PTC → MCP73833 charger (fed from `+5V0_A_SRC`, ahead of the ideal diode) → TPS61092 5 V boost → LM66100 ideal diodes into `+5V0_A` and `+5V0_B`.
  - Relay coils are **not** backed up, on purpose: losing 12 V means pass-through.
  - **SM-24:** a TPS3702 on the boost output plus an inverter FET disables both ideal diodes on overvoltage.
- **Errors I caught and fixed:**
  - TPS61091 is the **3.3 V** part; the 5 V one is the **TPS61092**.
  - The boost added a common-cause overvoltage path to both rails; SM-24 closes it.

## PCB flow and lessons (rev 0.4 chunk)

The full board flow, run from `hardware/`:

1. `make_pcb.py place` (board size: `ICARUS_W` / `ICARUS_H`, default 152 × 115 mm)
   - Cleans the vendor footprints in `icarus.pretty` at file level.
   - **`placer.py` (rev 0.5):** builds one cluster per design group around its IC.
     - Decoupling caps go to their IC's supply-input pins first, smallest value nearest.
     - Then the parts on a pin's own small net: timing, feedback, BOOT, CL/CS.
     - Then the rest in design.py order.
     - IC-less groups dissolve into the cluster holding their most specific net.
   - Clusters are skyline-packed, turned 90° where that packs better. They go into signal-flow regions (`COLUMNS` / `RULES`: incoming → pass-through → outgoing → MCUs → measure), each sized by `regions()`.
   - **`silk.py`:** both sides, collision-checked so DRC stays silk-clean.
     - Front: cluster outlines and titles, LED and test-pad labels, connector pin names, references where they fit.
     - Back: title block, J1 pinout, LED legend, notes, an x-ray reference map.
   - Runs the **pre-route GND fan-out**: a locked stub and via on every GND SMD pad.
   - Sets In1 as a **power-plane layer**, then writes the DSN.
2. `gen/route.sh`: Freerouting. It takes about 20–40 minutes.
3. `make_pcb.py finish`: imports the SES, pours GND on F/In2/B, fills, and **fails if any track lands on In1**.
4. `make_pcb.py fanout 6`: post-route GND fan-out for leftover pads, plus one stitching via per isolated F.Cu/B.Cu pour fragment.
5. `make_pcb.py thermals`: solid zone connection on any pads DRC reports as `starved_thermal`.
6. Run DRC again.
7. `gen/check_decoupling.py`: pad-edge gap from every IC supply pin to its nearest cap, and from timing / feedback pins to their parts. The bar is 2 mm; the report lists what's over.

**Lessons:**
- **Keep In1 off-limits to signals.** Left as a signal layer, Freerouting runs signals on In1 and breaks the GND plane into islands; mark it as `LT_POWER` before exporting the DSN.
- **Fan out before routing, not after.** Post-route fan-out can't reach pads that traces have boxed in.
- **Don't edit vendor footprints through pcbnew.** Its `DRAWINGS` container breaks on UltraLibrarian footprints; edit them as S-expressions.
- **Never write Python with backslashes through a bash heredoc.** Use the Edit tool or a file the Write tool creates.
- **After TaskStop, check that the Java child is gone** (`Get-Process java`). In the 2026-11-01 runs it went with the shell, but a stopped run that survives can keep writing the SES file.
- **Confirm unnamed stock-symbol pins against the datasheet.** The TS5A3159 COM/NC swap came from exactly this.
- **Pin tables from the datasheet PDF, not memory:** download it (WebFetch saves the PDF), extract the text with PyMuPDF (`python -P`, keeps user site-packages, no script-dir imports), read the terminal tables. The generator asserts the TMS570's 100 pins are each assigned once.
- **Don't trust a part number from memory.** TPS3702-Q1 exists (docs/05 said it didn't); SN74LVC1G123QDCTRQ1 doesn't (it was in the BOM). Nexperia's 74LVC1G74-Q100 has a different pin 3/6 than TI's DCU part. Verify the orderable code and the pinout before every swap.
- **Floaters** (`place_floaters`): test pads and indicator LEDs are placed last, by the pin that drives their signal, never inside the region packer. Their units are joined by nets only floaters use.
- **Double-sided placement:** mirror whole clusters (`Flip` about the cluster centre line), never single parts, or caps end up on the wrong side of their pins. SMD footprints with through-hole thermal vias (TPS1H100, TPS61092) block the other side: keep their regions on one side and pack it first (`pack(..., keepouts)`).
- **A board copied away from `icarus.kicad_pro` loses its design rules,** so DRC falls back to defaults and reports nonsense. DRC the board in place.
- **design.py order is placement intent.** The placer gives each cap to the nearest preceding IC in its own group. So list an IC's caps right after it, and put parts that compete for the same pins (the oscillator at X1) after the caps.
- **Rotation geometry comes from pcbnew.** `placer.Geo` reads pad and courtyard positions at 0/90/180/270 from pcbnew itself; a cluster turned +90° maps (x, y) → (y, −x), the same as a footprint.

## Next steps (in order)

1. **Commit** rev 0.3 + 0.4 + 0.5 + the PCB, when the user asks.
2. **PCB release items:**
   - creepage rule between VBAT/coil copper and the VT path;
   - real relay and vehicle-connector footprints, which means placing and routing again;
   - JLC part numbers;
   - rotation check in JLC's preview.
3. **Placement items over the 2 mm bar** (`hardware/doc/decoupling_check.txt`): the LMR38020 RT resistors, the MCP73833 PROG resistor, the TPS61092 LBI divider, and a few logic-gate caps.
4. **Finish-step order for a fresh route:** `finish` (includes the VBAT_IN B.Cu corridor), `fanout 6`, DRC, `thermals`, DRC, then `bridge` only for leftovers.
5. **Continuation routing (`redsn` + Freerouting) still doesn't work. Route fresh.** Findings from 2026-11-01:
   - **The crash.** KiCad exports most track as two-point wires meeting end to end. Freerouting rejoins them recursively on load (`PolylineTrace.combine`) and overflows the stack. A bigger `-Xss` (256 MB) doesn't help.
   - **The merge script trades the crash for wrong connectivity.** `gen/dsn_merge_wires.py` (run by `redsn`) merges each chain into one path, and Freerouting then loads. But a merged path that runs through a pad hides that pad: Freerouting only makes contacts at trace end points. It reported 508 unrouted (vs about 6 real) and stayed there.
   - **Stopping merges at pads brings the crash back,** even when only same-layer pads count.
   - **`-inc <netclass>` is parsed but has no effect** in this 2.x build.
   - **The leftover routers have limits.** `bridge` (grid A*) only closes gaps through free space. `gen/rrr.py` (rip-up and reroute) moves other signal traces out of the way. It leaves pad escapes, crystal nets, GND and power alone, and rolls back a connection it can't finish. Neither can open a pin that is walled in on every side: that needs placement room.
   - **`rrr` put-back fixes (2026-11-02).** Two bugs could cut a net while reporting success:
     - **T-junctions.** A ripped piece's terminals were found only at segment end points, so a piece that T-joined the middle of a segment was dropped silently. Now any copper contact on the same layer counts, and a piece with fewer than 2 terminals rolls the whole connection back.
     - **Pieces split across levels.** A nested put-back could rip the neighbour of a piece still waiting at an outer level. A put-back now gathers every still-removed segment of its net from the whole connection's journal.
   - **`rrr` retries.** When the put-back chain dead-ends, the next try bans the first-level rips of the net that failed. `RRR_DEPTH` (default 3) and `RRR_TRIES` (default 8) set the depth and number of tries. Going deeper (6) didn't help: chains ended at fixed copper. Retries closed VCC2M_B.
   - **More `rrr` options,** all off by default:
     - `RRR_BAN=deep` bans the rips of the innermost level that failed, not the first level's.
     - `RRR_ESCAPE=0` lets the router rip pad-escape doglegs too.
     - `RRR_VIAS=1` lets it rip signal vias, which go with every segment they touch.

     None of them closed CSK2_A. With `RRR_ESCAPE=0 RRR_VIAS=1`, rrr closed VMID_B on the final board by moving the VMIDD_B escape that ran under U23 pin 2.
   - **CSK2_A was a placement problem, not a routing one.** Its ADC filter (1 kΩ, 10 nF, clamp diode) sat in the coil-drive cluster, 31 mm from U10 pin 21, boxed in on all three layers. `design.place_with("MCU A", refs)` now places both coils' filters in U10's cluster. It is PCB-only: `make_pcb.placed_as` reads it, and the schematic is unchanged. The route after that (`gen/route/A_SGSS_cskpin.*`) closed it.
   - **When a net stays open over several routes, look at placement first.** Freerouting isn't deterministic with `-mt 8`, but it left the same net open in both runs on the same DSN.
   - **Freerouting telemetry.** Its settings file (`%LOCALAPPDATA%/Temp/freerouting/freerouting.json`) came with `allow_telemetry: true` and `disable_analytics: false`. `gen/route.sh` now passes `--usage_and_diagnostic_data.disable_analytics=true --profile.allow_telemetry=false` on every run. The settings file itself is left as it is.
   - **A fan-out halo round U15 did nothing,** because the parts round it are its own decoupling caps, which have to stay. The probe at pin 95 found every via spot blocked by In2 traces, and the other open nets were overlay nets routed over U15. Config A now puts the overlay (CH-A analog) over CH-A and the supplies over U15. **That fixed it:** the next route left no U15 net open (`gen/route/A_SGSS_ovlswap.*`).
6. **Still open from the analysis:** SM-24 test hook (latent), the relay part, the vehicle connector part, the TPS1H100 K ratio, TPS3702 qualification, and the logger-measured TBD-DATA values.
