# Tasaru V0.0.1 — PCB layout attack plan

> **Purpose:** Ordered checklist for taking this design from **placed** to **routed**, **DRC-clean**, and **fab-ready**.  
> **Companion:** `HANDOFF.md` (architecture, power tree, BOM strategy) and `FIRMWARE_PIN_MAP.md` (net ↔ MCU pin truth).  
> **Stackup intent:** **F.Cu** signals · **In1.Cu** solid **GND** · **In2.Cu** split power (**`/+BATTPROTECTED`**, **`+3V3`**, optional **`GND`** pour) · **B.Cu** test pads / low-speed / optional encoder.

---

## Phase 0 — Before you move another footprint

1. **Freeze schematic inputs for layout**  
   Export or diff schematic so you know which netlist this PCB matches. Regenerate netlist after any schematic edit.

2. **Confirm design rules** (`File` → `Board Setup` → **Design Rules**)  
   - Minimum trace / clearance vs fab (e.g. 4/4 mil or 5/5 mil).  
   - **Annular ring** for vias and through-holes — current `DRC.rpt` flags annular violations; fix **board minimums** or **pad/via sizes** so both agree.  
   - **Copper-to-edge** clearance for your manufacturer.

3. **BOM sanity**  
   In KiCAD: **Tools → Generate Bill of Materials** (or your Fabrication Toolkit preset).  
   Project currently points BOM export to `v1fbomthecu.csv` under Downloads (see `HANDOFF.md` §10). Consider also committing a dated CSV under the repo when you tag a release.

4. **Decide mechanical truth**  
   - Connector exit direction and standoff keep-out.  
   - **Encoder magnet** side (F vs **B.Cu**) and shaft axis — fixes whether **U8** stays on top or moves under the board.

---

## Phase 1 — Floorplan (no routing yet)

5. **Draw the “noise map” on paper or User Comments layer**  
   Mark: battery entry, fuse/TVS, DC-DC (**U5**), motor driver (**U3**), motor connector, CAN (**U7**), high-side (**U2**) if it switches hot loads.

6. **Place / adjust blocks (targets)**  
   - **Input protection + bulk hold-up** (`F1`, `D1`, `D2`, `C24`, EMI `Lf` / `Rd` / `Cb` / `Cf`) tight along the **battery edge**.  
   - **U5 TPSM33620** immediately after the input filter, with **Cin** and **Cout** on the **same side** as U5 and **short** loops into **GND vias** to **In1.Cu**.  
   - **U3 DRV8873** near the **motor connector**; wide path preview on F.Cu for **motor current**.  
   - **U4 MCU** on the **quiet** side with room for **cap clusters** and **U6** clock close to clock pins.  
   - **U8 AS5147U** on-axis to the magnet, **≥ ~20–25 mm** from **U3** and motor leads if possible (magnetic gradient from PWM matters more than “under MCU”).  
   - **U7 CAN** at CAN harness approach; keep stub to connector short.

7. **Courtyard and mounting**  
   Run **DRC** for courtyard overlaps; fix before you invest in routing.

---

## Phase 2 — Power first (irreversible geometry)

8. **TPSM33620 power loops** (**U5**)  
   - **Input loop:** `Cin` → `VIN` → `PGND`/thermal pad → `Cin` return — **smallest possible** loop, top layer if you can, vias to **In1** at cap GND ends.  
   - **Output loop:** `Cout` hugging `VOUT` and local **GND**.  
   - **EP / thermal pad:** **9+ vias** (0.2–0.3 mm drill typical) to **In1.Cu** in a tight grid; follow TI layout note for the footprint.

9. **In2.Cu power zones**  
   - Keep **`/+BATTPROTECTED`** under the **noisy** side.  
   - Keep **`+3V3`** under **MCU + encoder + CAN** side.  
   - Avoid routing **fast F.Cu traces** across the **zone split** without a nearby **return path** (decap bridges return current at HF).

10. **High current** (**U3**, battery distribution, **U2** if applicable)  
    - **Motor phase** copper: width per IPC or fab calculator; **multiple vias** per layer transition.  
    - Under **U3**, prefer a **compact** switching loop with GND on **In1** directly under the part.

---

## Phase 3 — Sensitive analog and clocks

11. **U6 SIT2024 → U4 clock pins**  
    - Short, matched-ish pair if differential clock routing applies to your chosen pin mode; otherwise single-ended **short** route.  
    - **No unrelated signals** under the MEMS or crystal area; **GND pour** on **F.Cu** with vias to **In1**.

12. **VDDA / VDDIO_SW islands** (ferrites **L2**, **L3** per `HANDOFF.md`)  
    - Place **L2/L3** at the boundary between digital and analog regions.  
    - **VDDA** decaps hug **MCU VDDA** pins on the **same layer** when possible.

13. **ADC runs** (brake sense, current sense, etc.)  
    - Route on **F.Cu** with **In1** underneath; keep away from **motor PWM** and **switching** nodes.  
    - RC filters sit **at the MCU pin** side if that is your topology.

---

## Phase 4 — Digital control and buses

14. **SPI to U8 (encoder)**  
    - Route **CLK / MOSI / MISO** as a **group**; **CS** adjacent.  
    - If **U8** moves to **B.Cu**, use a **tight via bundle** next to **U4** to minimize stub length.

15. **CAN (U7)**  
    - Controlled impedance if your fab stack supports it; otherwise keep **short** and balanced to the connector.  
    - Place **ESD / CM choke / split termination** per your schematic **before** you lock silk.

16. **JTAG / UART**  
    - Low priority once power and analog are stable; keep away from motor switching if possible.

---

## Phase 5 — Ground stitching and pours

17. **In1.Cu GND zone**  
    - Outline should track **board edge** with consistent margin (avoid oversized polygons far outside `Edge.Cuts`).  
    - **Stitching vias:** denser under **U5**, **U3**, connectors — typical **3–5 mm** grid elsewhere unless EMI plan says tighter.

18. **F.Cu / B.Cu GND pours** (optional but common)  
    - Same net as **In1**; do **not** let pours create **isolated islands** — delete slivers, add vias.

19. **Review `In2.Cu` GND pour**  
    If it starves **`+3V3`** copper fingers, **shrink** it after motor and buck paths are satisfied.

---

## Phase 6 — Verification and release

20. **Fill and DRC loop**  
    - **B** — fill zones.  
    - **DRC** until **zero errors** (warnings triaged).  
    - Re-read `DRC.rpt` only as history; always trust live DRC.

21. **Netlist vs PCB**  
    Tools → **Update PCB from Schematic** (forward) when appropriate; resolve **unconnected** and **NC** mismatches.

22. **Outputs for fab**  
    - Gerbers + drill + IPC-356 netlist if fab asks.  
    - **Interactive Html BOM** (PCM plugin) for assembly.  
    - **3D STEP** export for mechanical check against enclosure.

23. **Documentation pass**  
    Update `HANDOFF.md` §10 snapshot (board revision, DRC count, BOM filename) when you order prototypes.

---

## Quick reference — designators on this PCB

| Ref | IC |
|-----|-----|
| U4 | F280049 MCU |
| U5 | TPSM33620 power module |
| U3 | DRV8873 motor driver |
| U8 | AS5147U encoder |
| U7 | PTCAN3404 CAN |
| U2 | TPS1H100 high-side switch |
| U6 | SIT2024 MEMS 10 MHz |
| U9 | TPS3702 supervisor |
| U1 | SQJ147ELP MOSFET |

---

## Helpful KiCAD habits (novice-friendly)

- **Plugin and Content Manager:** Interactive Html BOM; fabrication helpers compatible with KiCAD 10.  
- **Official docs:** [Copper zones](https://docs.kicad.org/master/en/pcbnew/pcbnew_copper_zones.html) in PCB Editor manual.  
- **Community:** [KiCad.info](https://forum.kicad.info/) — post **one** clear question, screenshots of **DRC** and **relevant layers**.

---

## When you are “done”

You are ready to order when: **DRC = 0 errors**, **netlist = 0 unconnected** (except approved NC), **zones filled**, **fab stackup** matches impedance needs (if any), and **BOM** matches the **exact** schematic revision you exported to gerbers.
