// ═══════════════════════════════════════════════════════════════════════
// EPS Data Logger — on-card file format
//
// One file per ignition cycle: EPSnnnnn.BIN
//   [LogHeader, 512 bytes][LogRecord, 32 bytes] × N
// Little-endian, every field naturally aligned (no compiler padding).
// tools/decode_log.py reads this — change both together and bump
// LOG_FORMAT_VERSION.
// ═══════════════════════════════════════════════════════════════════════
#pragma once
#include <stdint.h>
#include <stddef.h>

#define LOG_MAGIC           "EPSLOG1"
#define LOG_FORMAT_VERSION  1

struct LogRecord {           // one per sample tick
  uint32_t tUs;              // micros() at the tick (wraps every ~71 min; decoder unwraps)
  uint32_t vspEdgeUs;        // micros() of the latest VSP rising edge
  uint32_t nepEdgeUs;        // micros() of the latest NEP rising edge
  uint16_t vt3;              // ADC counts, 12-bit, after hardware averaging
  uint16_t vt6;
  uint16_t vcc2;
  uint16_t vbat;
  uint16_t imot;
  uint16_t vspCount;         // running count of VSP rising edges (wraps)
  uint16_t nepCount;         // running count of NEP rising edges (wraps)
  uint16_t diag;             // lead diagnostic flags (lead_diag.h)
  uint16_t dropped;          // records lost to a full buffer since the file opened (saturates)
  uint8_t  dio;              // DIO_* bits
  uint8_t  seq;              // tick counter, low byte — gaps show lost records
};

enum : uint8_t {
  DIO_IG1    = 0x01,         // key ON
  DIO_WLP_ON = 0x02,         // EPS lamp ON (WLP pulled low by the EPS unit)
  DIO_RT_ON  = 0x04,         // motor relay drive ON
};

struct LogHeader {           // first 512-byte block of every file
  char     magic[8];         // LOG_MAGIC
  uint16_t formatVersion;
  uint16_t headerSize;       // 512
  uint16_t recordSize;       // 32
  uint16_t sampleHz;
  uint32_t startUnix;        // RTC time at file open, 0 if the RTC isn't running
  uint32_t startUs;          // micros() at file open
  float    vPerCountVt;      // car-side volts per count: VT3, VT6, Vcc2
  float    vPerCountVbat;    // car-side volts per count: +B
  float    vPerCountImot;    // current-sensor-output volts per count
  float    imotVPerA;        // current sensor sensitivity, V/A
  float    imotZeroV;        // sensor output at 0 A (measured if imotZeroMeasured)
  uint8_t  adcDual;          // 1 = VT3/VT6 converted together on ADC1+ADC0
  uint8_t  haveRt;           // 1 = RT+ lead fitted
  uint8_t  imotZeroMeasured; // 1 = imotZeroV was measured with the relay open
  uint8_t  reserved;
  char     firmware[16];
  uint8_t  pad[448];
};

static_assert(sizeof(LogRecord) == 32, "LogRecord must stay 32 bytes");
static_assert(offsetof(LogRecord, vt3) == 12 && offsetof(LogRecord, diag) == 26 &&
              offsetof(LogRecord, dio) == 30, "LogRecord layout changed — update decode_log.py");
static_assert(sizeof(LogHeader) == 512, "LogHeader must be one 512-byte block");
static_assert(offsetof(LogHeader, vPerCountVt) == 24 && offsetof(LogHeader, adcDual) == 44 &&
              offsetof(LogHeader, firmware) == 48, "LogHeader layout changed — update decode_log.py");
