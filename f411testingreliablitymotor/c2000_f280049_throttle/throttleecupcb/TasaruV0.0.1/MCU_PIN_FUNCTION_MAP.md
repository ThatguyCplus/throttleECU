# U4 (F280049CPZS) — Pin map by **function** (for double-check)

> **Source of truth for nets:** `TasaruV0.0.1.kicad_pcb` footprint **U4** (package pin numbers **1–100**, TQFP-100).  
> **Last extracted:** 2026-04-11 (matches current board netlist after last schematic → PCB update).  
> **Related:** `FIRMWARE_PIN_MAP.md` (ADC limits, safety notes), `HANDOFF.md` (block diagram), **`MCU_PIN_GROUPING_OPTIONS.md`** (TQFP **physical** clusters + locked pins for remap).

---

## Why GPIO looks “all over the place” (not a safety layout trick)

1. **The chip is already bonded.** TI decides which **physical pad** on the die goes to which **package pin**. You cannot move GPIO28 next to GPIO29 on the **silicon** or the **TQFP** pattern.

2. **Peripherals are muxed to many pads.** On F28004x, each GPIO can offer **different** mux options (ePWM, SPI, CAN, I2C, …). You pick **legal** pins from the **Technical Reference Manual** pinmux table, not “whatever is geometrically cute” on the package.

3. **Electrical regions still matter.** TI groups **power**, **ADC**, **JTAG**, etc. roughly by bank, but **your** chosen mix of CAN + motor PWM + two SPIs + ADC will **never** sit in one contiguous corner** — that is normal.

4. **What *is* a deliberate design choice** is splitting **motor / brake / solenoid** sensing across **ADC-A / B / C** (see `FIRMWARE_PIN_MAP.md`) so one ADC fault does not remove every analog monitor. That is **system safety**, not “pins scattered for fun.”

**Bottom line:** routing cost is paid in **fanout** and **via planning**, not by expecting GPIO numbers to be consecutive on the package edge.

---

## What you are seeing on **U4A / U4B** (schematic vs package)

Your screenshot is correct: **GPIO index** (software view) and **package pin** (layout view) are **two different orderings**. The symbol is drawn so **GPIO0–31** read nicely on paper; the **TQFP** pin order follows the **datasheet pinout**, not GPIO order.

### U4A — “obvious” peripherals (from your schematic)

| Package pin | GPIO (label on sch) | Net / function |
|-------------|---------------------|----------------|
| 79 | GPIO00 | `MOT_PWM` |
| 77 | GPIO02 | `MOT_PH` |
| 89 | GPIO05 | `MOT_nSLEEP` |
| 97 | GPIO06 | `MOT_DISABLE` |
| 75 | GPIO04 | `SENSOR_PWM` |
| 84 | GPIO07 | `SOL_GATE` |
| 51 | GPIO12 | `nFault` (DRV8873 fault) |
| 57 | GPIO25 | `BRK_SW` |
| 74 | GPIO08 | `SPI_MOSI` |
| 90 | GPIO09 | `SPI_CLK` |
| 93 | GPIO10 | `SPI_MISO` |
| 52 | GPIO11 | `SPI_CS` |
| 98 | GPIO30 | `CAN_RX` |
| 99 | GPIO31 | `CAN_TX` |

So **SPI-A is exactly GPIO8–11** — which *is* a contiguous **logical** group — but those signals exit on **pins 52, 74, 90, 93** on the **package**. That is why the board looks “weird” until you overlay the **datasheet pin diagram**.

**Also broken out as generic IO (test / future use):** pins **1, 50, 95, 96, 54, 55, 56, 58, 59, 68, 81, 83, 100** (GPIO28, 13, 15, 16, 17, 24, 26, 27, 18/X2, 23/SW, 22/SW, 29 per your sheet — always match your symbol text).

### U4B — JTAG + second SPI + more IO

Matches your **U4B** sheet and the KiCad library symbol `F280049CPZS` (unit B): **TCK/TMS** use dedicated pin names on the symbol; **TDO/TDI** also show GPIO37 / GPIO35.

| Package pin | Symbol pin name | Net on PCB |
|-------------|-----------------|------------|
| 60 | `TCK` | `/TCK` |
| 61 | `GPIO37_TDO` | `/TDO` |
| 62 | `TMS` | `/TMS` |
| 63 | `GPIO35_TDI` | `/TDI` |
| 65 | `GPIO56` | `/B_SPI_SIMO` → **T10** |
| 66 | `GPIO57` | `/B_SPI_SOMI` → **T11** |
| 67 | `GPIO58` | `/B_SPI_CLK` → **T12** |
| 92 | `GPIO59` | `/B_SPI_CS` → **T13** |
| 64 | `GPIO32` | `/GPIO32` |
| 53 | `GPIO33` | `Net-(U4B-GPIO33)` |
| 94 | `GPIO34` | `Net-(U4B-GPIO34)` |
| 91 | `GPIO39` | `Net-(U4B-GPIO39)` |
| 85 | `GPIO40` | `Net-(U4B-GPIO40)` |

The library also marks **pin 69** as **`X1`** (dedicated clock input). Your **PCB** still showed **no net** on pad 69 at last extract — if the MEMS only uses **X2** (**GPIO18** / pin **68**), **X1** can be NC; otherwise tie **X1** in schematic and re-export the PCB.

### PCB vs schematic — two pins to keep straight

On **`kicad_pcb`**, **pin 76** = `Net-(U4A-GPIO03)` and **pin 78** = `Net-(U4A-GPIO01)` (still **GPIO3** / **GPIO1** balls, nets not renamed to `MOT_*` in the netlist). On your **U4A** drawing, **GPIO3** is often **sleep / enable** in motor apps — confirm in the schematic whether **GPIO1** and **GPIO3** are intentionally separate from **`MOT_PH` / `MOT_nSLEEP`** or should be tied to the same nets (so the **PCB** and **sch** never diverge).

---

## Quick checklist table (by **job**)

| Job | Package pins | Net name(s) on PCB | Notes |
|-----|----------------|---------------------|--------|
| **Reset** | 2 | `/XRSn` | External reset / supervisor |
| **Core + I/O power** | 3, 47, 70, 80, 88 | `VDDIO` | 3.3 V I/O |
| **Core power** | 4, 46, 71, 87 | `1V2_Core` | Digital core rail (per your PDN) |
| **Analog supply** | 11, 34 | `VDDA` | 3.3 V analog |
| **Ground** | 5, 12–15, 26, 27, 32, 33, 42, 45, 72, 82, 86 | `GND` | |
| **VREG enable strap** | 73 | `/VREGENZ` | Check strap vs TI requirement |
| **ADC ref high** | 24, 25 | `Net-(U4C-VREFHIA)` | Both pins tied to same ref net in this design |
| **Brake analog** | 19 | `Net-(JP9-B)` → jumper **JP9** value **BRK_SENSE** | To divider / `BRK_SENSE` chain |
| **Motor current A** | 23 | `Net-(JP11-B)` → **JP11** **MOT_IPROPI1** | From **U3** IPROPI |
| **Solenoid current sense** | 29 | `Net-(JP10-B)` → **JP10** **SOL_CS_CURRENT** | From **U2** CS path |
| **Motor current B** | 41 | `Net-(JP12-B)` → **JP12** **MOT_IPROPI2** | From **U3** 2nd IPROPI |
| **Spare / analog pads** | 8, 9, 10 | `/B3`, `/A2`, `/A3` | ADC-capable nets; verify use in firmware |
| **External timebase** | 68 | `Net-(U4A-GPIO18_X2)` | Routed to **U6** MEMS clock path (name carries **X2**) |
| **Clock pin (check!)** | 69 | **`<NO NET>`** in PCB | Footprint shows `X1_69` — **must** match schematic clock wiring; **fix if truly open** |
| **JTAG** | 60–63 | `/TCK`, `/TDO`, `/TMS`, `/TDI` | Debug |
| **CAN to U7** | 98, 99 | `/CAN_RX`, `/CAN_TX` | To **U7** PTCAN3404 |
| **Motor bridge control** | 77, 78, 79, 89, 97 | `/MOT_PH`, `Net-(U4A-GPIO01)` **→ often `MOT_EN`/`GPIO1` in docs**, `/MOT_PWM`, `/MOT_nSLEEP`, `/MOT_DISABLE` | Cross-check **GPIO1** net name vs your firmware mux |
| **Driver fault** | 51 | `/nFault` | From **U3** |
| **SPI-A (labels)** | 52, 74, 90, 93 | `/SPI_CS`, `/SPI_MOSI`, `/SPI_CLK`, `/SPI_MISO` | Typical **master** to **U8** encoder (verify mux = SPIA or SPIB in TRM) |
| **SPI-B (labels)** | 65–67, 92 | `/B_SPI_SIMO`, `/B_SPI_SOMI`, `/B_SPI_CLK`, `/B_SPI_CS` | Second SPI instance / bus — verify target (**U8** vs flash vs other) |
| **High-side / solenoid** | 84 | `/SOL_GATE` | To **U2** gate drive chain |
| **Brake switch (digital)** | 57 | `/BRK_SW` | Digital input |
| **Encoder PWM / misc** | 75 | `/SENSOR_PWM` | Name suggests AS5147 PWM mode or test — confirm in sch |
| **Spare / test GPIO** | 83, 91 | `Net-(U4A-GPIO22_SW)`, `Net-(U4B-GPIO39)` | Silk test points **IO22_SW1** / similar — **not** the same string as `/PGOOD` on PCB; confirm if one should tie to **`/PGOOD`** |

---

## Full pin list (physical pin → net)

Use this to tick against the **TQFP pinout** drawing in TI `TMS320F280049` datasheet (pin **1** index).

| Pin | Net |
|-----|-----|
| 1 | `Net-(U4A-GPIO28)` |
| 2 | `/XRSn` |
| 3 | `VDDIO` |
| 4 | `1V2_Core` |
| 5 | `GND` |
| 6 | `unconnected-(U4C-A6_PGA5_OF-Pad6)` |
| 7 | `unconnected-(U4C-B2_C6_PGA3_OF-Pad7)` |
| 8 | `/B3` |
| 9 | `/A2` |
| 10 | `/A3` |
| 11 | `VDDA` |
| 12 | `GND` |
| 13 | `GND` |
| 14 | `GND` |
| 15 | `GND` |
| 16 | `unconnected-(U4C-PGA5_IN-Pad16)` |
| 17 | `unconnected-(U4C-C4-Pad17)` |
| 18 | `unconnected-(U4C-PGA1_IN-Pad18)` |
| 19 | `Net-(JP9-B)` → **BRK_SENSE** |
| 20 | `unconnected-(U4C-PGA3_IN-Pad20)` |
| 21 | `unconnected-(U4C-C2-Pad21)` |
| 22 | `unconnected-(U4C-A1_DACB_OUT-Pad22)` |
| 23 | `Net-(JP11-B)` → **MOT_IPROPI1** |
| 24 | `Net-(U4C-VREFHIA)` |
| 25 | `Net-(U4C-VREFHIA)` |
| 26 | `GND` |
| 27 | `GND` |
| 28 | `unconnected-(U4C-C5_PGA6_IN-Pad28)` |
| 29 | `Net-(JP10-B)` → **SOL_CS_CURRENT** |
| 30 | `unconnected-(U4C-PGA2_IN-Pad30)` |
| 31 | `unconnected-(U4C-C3_PGA4_IN-Pad31)` |
| 32 | `GND` |
| 33 | `GND` |
| 34 | `VDDA` |
| 35 | `unconnected-(U4C-A5-Pad35)` |
| 36 | `unconnected-(U4C-A4_B8_PGA2_OF-Pad36)` |
| 37 | `unconnected-(U4C-A8_PGA6_OF-Pad37)` |
| 38 | `unconnected-(U4C-A9-Pad38)` |
| 39 | `unconnected-(U4C-B4_C8_PGA4_OF-Pad39)` |
| 40 | `unconnected-(U4C-B1_A10_C10_PGA7_OF-Pad40)` |
| 41 | `Net-(JP12-B)` → **MOT_IPROPI2** |
| 42 | `GND` |
| 43 | `unconnected-(U4C-PGA7_IN-Pad43)` |
| 44 | `unconnected-(U4C-C14-Pad44)` |
| 45 | `GND` |
| 46 | `1V2_Core` |
| 47 | `VDDIO` |
| 48 | `unconnected-(U4D-FLT2-Pad48)` |
| 49 | `unconnected-(U4D-FLT1-Pad49)` |
| 50 | `Net-(U4A-GPIO13)` |
| 51 | `/nFault` |
| 52 | `/SPI_CS` |
| 53 | `Net-(U4B-GPIO33)` |
| 54 | `Net-(U4A-GPIO16)` |
| 55 | `Net-(U4A-GPIO17)` |
| 56 | `/GPIO24` |
| 57 | `/BRK_SW` |
| 58 | `Net-(U4A-GPIO26)` |
| 59 | `Net-(U4A-GPIO27)` |
| 60 | `/TCK` |
| 61 | `/TDO` |
| 62 | `/TMS` |
| 63 | `/TDI` |
| 64 | `/GPIO32` |
| 65 | `/B_SPI_SIMO` |
| 66 | `/B_SPI_SOMI` |
| 67 | `/B_SPI_CLK` |
| 68 | `Net-(U4A-GPIO18_X2)` |
| 69 | **`<NO NET>`** |
| 70 | `VDDIO` |
| 71 | `1V2_Core` |
| 72 | `GND` |
| 73 | `/VREGENZ` |
| 74 | `/SPI_MOSI` |
| 75 | `/SENSOR_PWM` |
| 76 | `Net-(U4A-GPIO03)` |
| 77 | `/MOT_PH` |
| 78 | `Net-(U4A-GPIO01)` |
| 79 | `/MOT_PWM` |
| 80 | `VDDIO` |
| 81 | `Net-(U4A-GPIO23_SW)` |
| 82 | `GND` |
| 83 | `Net-(U4A-GPIO22_SW)` |
| 84 | `/SOL_GATE` |
| 85 | `Net-(U4B-GPIO40)` |
| 86 | `GND` |
| 87 | `1V2_Core` |
| 88 | `VDDIO` |
| 89 | `/MOT_nSLEEP` |
| 90 | `/SPI_CLK` |
| 91 | `Net-(U4B-GPIO39)` |
| 92 | `/B_SPI_CS` |
| 93 | `/SPI_MISO` |
| 94 | `Net-(U4B-GPIO34)` |
| 95 | `Net-(U4A-GPIO15)` |
| 96 | `Net-(U4A-GPIO14)` |
| 97 | `/MOT_DISABLE` |
| 98 | `/CAN_RX` |
| 99 | `/CAN_TX` |
| 100 | `Net-(U4A-GPIO29)` |

---

## How to keep this file honest

1. After any schematic change: **Update PCB from Schematic**, then re-export or re-run your EDA **pin report** / DRC.  
2. If pin **69** is supposed to be **X1** (single-ended clock) or tied **NC** for pure **crystal-less** MEMS feed on **X2 only**, resolve that in **both** sch and PCB so the netlist is not lying.  
3. For firmware, always confirm each GPIO against **Table 4-3 / mux** in **`sprui33h`** (F28004x TRM) for the exact package.

---

## Suggested improvement to `FIRMWARE_PIN_MAP.md`

Add at the top a **one-line link**: “Package pin ↔ function: see `MCU_PIN_FUNCTION_MAP.md`.” That doc can stay focused on **ADC rules** and **software traps**; this one is for **hardware bring-up**.
