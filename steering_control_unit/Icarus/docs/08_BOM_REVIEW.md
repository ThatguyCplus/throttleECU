# 08: BOM review: reuse from the throttle ECU and automotive grade

| | |
|---|---|
| Revision | 0.6, 2026-11-01 |
| Based on | `hardware/gen/design.py` rev 0.6; throttle ECU schematic `TasaruV0.0.1` (read-only, BOM exported from a copy) |
| Scripts | `hardware/gen/bom_compare.py` (reuse), `hardware/gen/auto_grade.py` (automotive grade) |

Test pads are excluded throughout (bare copper, nothing to buy).

## 1. Shared with the throttle ECU

The throttle BOM has no part numbers on its passives, so matching is conservative:
- **Active and special parts** match by orderable part.
- **Resistors, capacitors, inductors and ferrites** match by value + package, and only when Icarus doesn't ask for a voltage rating or a tolerance the throttle line doesn't state.

| | BOM lines | Placements |
|---|---|---|
| Icarus total | 89 | 526 |
| **Identical to a throttle line** | **23 (26 %)** | **199 (38 %)** |
| of which the same family, different grade | 2 | 5 |
| Same value, other package or rating (easy to unify) | 8 | 117 (22 %) |
| Icarus only | 58 | 210 |

The two family-only matches:
- **F280049C:** the throttle uses F280049CPZS; Icarus uses the F280049CPZQR automotive orderable.
- **TPS3702:** the throttle doesn't specify the variant.

**Shared parts:**
- **Active and special:** F280049C, LMR38020-Q1, TPS1H100-Q1, TCAN3404-Q1, SiT2024B 10 MHz, SQJ147ELP, TPS3702CX50, SMBJ20A, BAS21, the 60 Ω ferrite and the 10 µH SRP7028A.
- **Passives:** the common values (1k, 2.2k, 4.7k, 10k, 22k, 64.9k, 100k; 1, 2.2, 4.7 and 10 µF).

**Near misses that could be unified:**
- **100 nF decoupling** (59 placements): Icarus uses 0402 to keep caps at the pins; the throttle uses 0603.
- **10 nF:** 0402 vs 0603.
- **Rated parts:** the 0.1 % summing resistors and the 100 V / 50 V input caps are specific to Icarus.

## 2. Automotive grade

| Status | BOM lines | Placements |
|---|---|---|
| AEC-Q part number already in the BOM | 22 (25 %) | 50 (10 %) |
| Automotive grade intended, exact orderable code to confirm | 9 (10 %) | 30 (6 %) |
| Generic R/C: AEC-Q200 grade to specify at sourcing | 41 (46 %) | 412 (78 %) |
| **Not automotive: needs a different part** | **12 (13 %)** | **29 (6 %)** |
| Bench-only / not a part (debug headers, solder jumper) | 5 (6 %) | 5 (1 %) |

### Changed in rev 0.6 (verified, same footprint and pinout)

| Part | Was | Now | Note |
|---|---|---|---|
| 5 V window supervisor ×3 | TPS3702CX50DDCR | **TPS3702CX50QDDCRQ1** | An AEC-Q100 TPS3702-Q1 exists (TI SBVS261C). **docs/05 previously said it didn't; that was wrong** and is corrected |
| Input PTC | Littelfuse 1812L110/33MR | **Yageo SMDH1812B110TF/33** | AEC-Q200, 33 V, 1.1 A hold |
| LiPo PTC | Littelfuse 1812L100/12 | **Littelfuse MINIASMDC110F/16-2** | AEC-Q200, 16 V, 1.1 A hold |
| Charger NTC | Murata NCP18XH103F03RB | **Murata NCU18XH103F6SRB** | NCU18 is Murata's automotive NTC series; confirm the exact code at order |
| One-shot | SN74LVC1G123**QDCTRQ1** | SN74LVC1G123DCTR (commercial, honest) | **The QDCTRQ1 code I used before doesn't exist**: TI has no -Q1 of this part. See below |

### Not automotive: needs a different part (no drop-in, design change)

| Part | Issue | Candidate | Change needed |
|---|---|---|---|
| SN74LVC1G123 one-shot (speed stage, SM-23) | No TI -Q1 | Nexperia **74LVC1G123DP-Q100H** (AEC-Q100, TSSOP-8), or TI **TPUL1G313-Q1** (TI calls it a pin-to-pin replacement, but the pulse-width maths differ) | Footprint swap; **verify the pinout from the datasheet first** |
| MCP73833 1S charger | No automotive grade | TI **BQ25171-Q1** (AEC-Q100, 1S Li-ion linear charger) | New symbol, footprint, PROG/TS network |
| TPS61092 5 V boost | No -Q1 | TI **TPS610333-Q1** (fixed 5 V, 1.8–5.5 V in, 1.85 A switch). A 1S cell is 3.0–4.2 V, so the input range fits | New footprint and pinout; SM-24 cutoff unchanged |
| G6K-2F-Y relays ×2 | Omron G6K is not automotive | Panasonic **TX2SA** (12 V DPDT SMD, listed for automotive use); **AEC-Q200 not confirmed**. Two vendors for K1/K2 is still the plan | New footprint; check the coil current against the TPS1H100 limit |
| Indicator LEDs ×22 | Generic | AEC-Q102 0603, e.g. ROHM **CSL0902** series. Red CSL0902VT1C and yellow CSL0902YT1C run at about 2.1 V. The 527 nm green CSL0902ET1C runs at about 3 V, which is too high for the 3.3 V rails with today's resistors. Pick a 560–570 nm green or recalculate | Resistor values per V_F |
| Vehicle connector J1 | 1×20 header placeholder | Sealed automotive connector (TE MCP or similar). Choosing it is already an open item | Real footprint. A 2-row part would also shorten the edge strip, which is now the limit below 100 mm |
| LiPo connector | JST-PH isn't automotive | Sealed automotive 2-pin for in-car use | Footprint |

### Intended automotive, orderable code to confirm

- **Diodes with the "-Q" (AEC-Q101) grade named:** 1N4148WS (19), BAT54WS, BAS21, BZX84C15, SMBJ20A. Also BZX84C18, which needs the -Q grade written in.
- **SN74LVC1G74-Q1 (the CH-C trip latch):** confirm TI's -Q1 orderable.
  - **Don't substitute** Nexperia 74LVC1G74DC-Q100H as a drop-in.
  - Its datasheet appears to put RD on pin 3 and Q̄ on pin 6, where TI's DCU part has Q̄ on 3 and CLR on 6.
  - A blind swap would miswire the guard's trip latch.
- **Blade-fuse holder:** the Keystone 3568 has no AEC rating; the ATM fuse itself is automotive.
- **SMBJ20A:** 600 W is small for an unsuppressed ISO 7637-2 load dump. Fine if the vehicle has centralized load-dump suppression; otherwise move to an SM8S-class TVS.

### Generic resistors and capacitors (the 78 % of placements)

There's no part number yet, so the grade is set at sourcing:
- Thick-film resistors: an AEC-Q200 series such as Yageo AC or Panasonic ERJ.
- The 0.1 % summing resistors: AEC-Q200 thin-film, e.g. Panasonic ERA.
- MLCCs: Murata GCM, TDK CGA or Samsung automotive.
- The 470 µF hold-up electrolytic: an AEC-Q200 series, e.g. Panasonic FK/FR automotive.

At JLC this means choosing extended parts over basic ones; the BOM's LCSC column is still empty.

## 3. Sources

- [TI TPS3702-Q1](https://www.ti.com/product/TPS3702-Q1) and [datasheet SBVS261C](https://ti.com/document-viewer/TPS3702-Q1/datasheet/GUID-716A002F-B25C-4AF5-B285-4E68DA67BBCE)
- [F280049CPZQR at DigiKey (AEC-Q100)](https://www.digikey.com.br/en/products/detail/texas-instruments/F280049CPZQR/9745479), [TI F28004x-Q1 package addendum](https://www.tij.co.jp/jp/ods/sysadd/oa/symlink/tms320f280040-q1_oa.pdf)
- [TI TS5A3159QDBVRQ1](https://ti.com/product/TS5A3159-Q1/part-details/TS5A3159QDBVRQ1), [TI TPS7B8233QDRVRQ1](https://ti.com/product/TPS7B82-Q1/part-details/TPS7B8233QDRVRQ1), [TI SN74LVC1G08-Q1](https://www.ti.com/product/SN74LVC1G08-Q1)
- [TI SN74LV123A-Q1 / SN74LVC1G123 (no -Q1)](https://ti.com/product/de-de/SN74LV123A-Q1), [TI SN74LVC1G123 page naming TPUL1G313-Q1](https://www.ti.com.cn/product/de-de/SN74LVC1G123), [Nexperia 74LVC1G123DP-Q100H](https://www.digikey.com.mx/en/products/detail/nexperia-usa-inc/74LVC1G123DP-Q100H/4288992)
- [Nexperia 74LVC1G74-Q100](https://www.nexperia.com/group/74lvc1g74-q100)
- [TI TPS610333-Q1](https://www.ti.com/product/TPS610333-Q1), [TI BQ25171-Q1](https://www.ti.com/product/es-mx/BQ25171-Q1)
- [Bourns SRP series for automotive](https://www.digikey.de/en/product-highlight/b/bourns/srp-series-for-automotive-applications), [Murata BLM18AG listing](https://www.omo-ic.com/tags/BLM18AG6.html)
- [Yageo SMDH1812B110TF/33](https://www.digikey.sg/en/products/detail/yageo/SMDH1812B110TF-33/26284026), [Littelfuse MINIASMDC110F/16-2](https://www.digikey.ie/en/products/detail/littelfuse-inc/MINIASMDC110F-16-2/3885066)
- [ROHM CSL0902UT1C (AEC-Q102)](https://www.digikey.com/en/products/detail/rohm-semiconductor/CSL0902UT1C/16285860)
- [Omron G6K-2F-DC12 (not automotive)](https://www.arrow.com/en/products/G6K2FDC12/omron.html), [Panasonic TX2SA listing](https://uk.rs-online.com/web/p/signal-relays/6996348)
- [MCP73833 listings (no automotive variant found)](https://www.lcsc.com/product-image/C635264.html)
