// ═══════════════════════════════════════════════════════════════════════
// EPS Data Logger — configuration
// Target: Adafruit Feather M4 CAN Express (ATSAME51J19) + Adalogger FeatherWing
//
//  ┌───────────────────────────────────────────────────────┐
//  │  Every value that depends on the hardware build is    │
//  │  here. The front end is drawn in LOGGER_WIRING.svg    │
//  │  and listed in BOM.md — if you change a resistor on   │
//  │  the Proto Wing, change the matching value below.     │
//  └───────────────────────────────────────────────────────┘
// ═══════════════════════════════════════════════════════════════════════
#pragma once

#define CFG_FIRMWARE_NAME  "eps_logger 1.0"

// ═══════════════════════════════════
// ANALOG PINS (Proto Wing outputs, never wired straight to the car)
// ═══════════════════════════════════
#define CFG_PIN_VCC2   A0    // U1C output  — PA02, ADC0 AIN0
#define CFG_PIN_VBAT   A1    // U1D output  — PA05, ADC0 AIN5
#define CFG_PIN_VT3    A2    // U1A output  — PB08, ADC1 AIN0 (or ADC0 AIN2)
#define CFG_PIN_VT6    A3    // U1B output  — PB09, ADC0 AIN3
#define CFG_PIN_IMOT   A4    // HSTS016L via 10k/18k — PA04, ADC0 AIN4

// SAMD51 ADC input numbers for the pins above (datasheet I/O multiplexing;
// PB08 = ADC1 AIN0 is the same mapping Adafruit's Grand Central M4 variant uses)
#define CFG_AIN0_VCC2   0
#define CFG_AIN0_VBAT   5
#define CFG_AIN0_VT6    3
#define CFG_AIN0_IMOT   4
#define CFG_AIN0_VT3    2    // VT3 on ADC0 — used for the cross-check and the fallback
#define CFG_AIN1_VT3    0    // VT3 on ADC1 — converts at the same instant as VT6 on ADC0

// ═══════════════════════════════════
// DIGITAL PINS — INPUT only, no pull-ups (the Proto Wing dividers set the levels)
// ═══════════════════════════════════
#define CFG_PIN_VSP    5     // vehicle speed pulses   (EXTINT 0)
#define CFG_PIN_NEP    6     // engine speed pulses    (EXTINT 2)
#define CFG_PIN_WLP    9     // EPS lamp: LOW = lamp ON
#define CFG_PIN_RT     11    // motor relay drive: HIGH = relay ON
#define CFG_PIN_IG1    12    // ignition: HIGH = key ON
#define CFG_HAVE_RT    1     // 0 if the optional RT+ lead isn't fitted

#define CFG_PIN_SD_CS  10    // Adalogger FeatherWing default
#define CFG_PIN_LED    LED_BUILTIN   // D13
// Leave D4 (CAN 5 V boost enable) and A6 (battery monitor divider) unconnected.

// ═══════════════════════════════════
// FRONT END (must match the board you built)
// ═══════════════════════════════════
// 1 = EPS tap wing rev 1.5+ (JLC Basic-part values); 0 = hand-built Proto Wing (PROTO_BUILD.md)
#define CFG_FRONT_END_TAP_WING 1

#define CFG_ADC_VREF       3.3f      // reference = VDDANA (Feather 3.3 V regulator)
#define CFG_R_TOP_VT       1.0e6f    // series: VT3, VT6, Vcc2
#define CFG_R_TOP_VBAT     1.0e6f    // series: +B
#define CFG_R_TOP_IMOT     10.0e3f   // current sensor output divider
#if CFG_FRONT_END_TAP_WING
#define CFG_R_BOT_VT       1.0e6f    // op-amp input to GND (5 V -> 2.50 V)
#define CFG_R_BOT_VBAT     100.0e3f  // 36 V full scale
#define CFG_R_BOT_IMOT     47.0e3f   // 3.44 V (75 A) -> 2.84 V
#else
#define CFG_R_BOT_VT       1.8e6f    // op-amp input to GND (5 V -> 3.21 V)
#define CFG_R_BOT_VBAT     150.0e3f  // 25 V full scale
#define CFG_R_BOT_IMOT     18.0e3f
#endif
#define CFG_IMOT_V_PER_A   0.0125f   // HSTS016L ±50 A: 0.625 V at 50 A
#define CFG_IMOT_ZERO_V    2.5f      // nominal output at 0 A (re-measured at every key-on)

// ═══════════════════════════════════
// PULSE SCALING (display only; the log keeps raw edge counts)
// ═══════════════════════════════════
// NEP: 3 pulses per 2 engine revs (3-cylinder), as MuddersMIMA engine_signals.h uses
#define CFG_NEP_PULSES_PER_REV   1.5f
// VSP: UNVERIFIED estimate — 5 pulses per tyre rev (EcoModder/Insight Central),
// 165/65R14 ≈ 1.81 m around → ~4,450 pulses/mile. Calibrate against GPS or a measured roll.
#define CFG_VSP_PULSES_PER_MILE  4450.0f

// ═══════════════════════════════════
// SAMPLING & LOGGING
// ═══════════════════════════════════
#define CFG_SAMPLE_HZ        1000    // one record per tick
#define CFG_ADC_AVG_LOG2     2       // hardware averaging: 2^n conversions per result (0..4)
#define CFG_RING_RECORDS     2048    // 64 KB = 2 s of card stalls absorbed at 1 kHz
#define CFG_WRITE_MIN_RECORDS 128    // 4 KB per card write (multi-block, fewer card stalls)
#define CFG_SYNC_MS          1000    // flush the file at least this often
#define CFG_IG1_DEBOUNCE_MS  200     // IG1 must be stable this long to count
#define CFG_KEYOFF_CLOSE_MS  3000    // IG1 OFF this long → close the log file
#define CFG_EDGE_MIN_US      150     // ignore VSP/NEP edges closer than this (noise)
#define CFG_ZERO_WINDOW_MS   1000    // relay-open time averaged for the current-sensor zero
