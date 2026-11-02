// ═══════════════════════════════════════════════════════════════════════
// EPS Data Logger — Honda Insight G1 electric power steering
// Target: Adafruit Feather M4 CAN Express + Adalogger FeatherWing
//         + Proto Wing front end (BOM.md, LOGGER_WIRING.svg)
//
// Listen-only. Every pin wired to the car is an INPUT behind a 1 MΩ or
// 100 kΩ resistor at the probe; nothing in this sketch can drive the EPS.
//
// What it does
//   • 1 kHz: VT3 + VT6 (same instant), Vcc2, +B, motor current, VSP/NEP
//     edge counts and times, IG1 / EPS lamp / relay states → one 32-byte
//     record (log_format.h)
//   • key ON opens EPSnnnnn.BIN on the SD card, key OFF for 3 s closes it,
//     so each drive is one file. tools/decode_log.py turns it into CSV.
//   • lead diagnostics check every tap against the Honda manual's values
//     (lead_diag.h); the result is stamped into every record
//
//  ┌───────────────────────────────────────────────────────┐
//  │  ALL BUILD-SPECIFIC VALUES ARE IN config.h            │
//  └───────────────────────────────────────────────────────┘
//
// USB serial, 115200, one command per line:
//   c   install check: every tap vs the manual → PASS / WARN / FAIL
//   l   live values at 10 Hz (toggle)
//   s   status: card, file, records, drops, ADC mode, RTC
//   r   record without IG1 (bench test toggle)
//   t<unix seconds>   set the RTC, e.g. t1793000000
// ═══════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <SdFat.h>      // "SdFat - Adafruit Fork"
#include <RTClib.h>     // Adafruit RTClib (PCF8523 on the Adalogger)
#include "config.h"
#include "log_format.h"
#include "lead_diag.h"
#include "fast_adc.h"

// ═══════════════════════════════════
// DERIVED CONSTANTS (from config.h)
// ═══════════════════════════════════
const float V_PER_COUNT_ADC  = CFG_ADC_VREF / 4096.0f;
const float V_PER_COUNT_VT   = V_PER_COUNT_ADC * (CFG_R_TOP_VT + CFG_R_BOT_VT) / CFG_R_BOT_VT;
const float V_PER_COUNT_VBAT = V_PER_COUNT_ADC * (CFG_R_TOP_VBAT + CFG_R_BOT_VBAT) / CFG_R_BOT_VBAT;
const float V_PER_COUNT_IMOT = V_PER_COUNT_ADC * (CFG_R_TOP_IMOT + CFG_R_BOT_IMOT) / CFG_R_BOT_IMOT;

const uint32_t RING_MASK         = CFG_RING_RECORDS - 1;
const uint32_t RECORDS_PER_BLOCK = 512 / sizeof(LogRecord);    // 16
static_assert((CFG_RING_RECORDS & (CFG_RING_RECORDS - 1)) == 0, "CFG_RING_RECORDS must be a power of two");
static_assert(CFG_RING_RECORDS % RECORDS_PER_BLOCK == 0, "ring must hold whole 512-byte blocks");
static_assert(CFG_ADC_AVG_LOG2 <= 4, "SAMD51 averages at most 16 conversions");

// ═══════════════════════════════════
// STATE SHARED WITH INTERRUPTS
// ═══════════════════════════════════
static LogRecord ring[CFG_RING_RECORDS];      // 64 KB
static volatile uint32_t ringHead = 0;        // advanced by the sample interrupt
static volatile uint32_t ringTail = 0;        // advanced by loop()
static volatile bool     ringOn   = false;    // interrupt queues records only while a file is open
static volatile uint16_t droppedRecs = 0;

static LogRecord latest;                      // newest record; loop() copies it with interrupts off
static volatile uint16_t diagShared = 0;      // lead flags stamped into every record

static volatile uint16_t vspCount = 0, nepCount = 0;
static volatile uint32_t vspEdgeUs = 0, nepEdgeUs = 0;

static volatile bool    adcDual = true;       // VT3 on ADC1, converted with VT6 on ADC0
static volatile uint8_t adcMismatch = 0;      // consecutive failed ADC1 cross-checks

// ═══════════════════════════════════
// LOOP STATE
// ═══════════════════════════════════
static SdFs   sd;
static FsFile logFile;
static RTC_PCF8523 rtc;
static LeadDiag leadDiag;

static bool sdOk = false, sdFault = false, rtcOk = false, rtcFromBuild = false;
static bool fileOpen = false, manualRecord = false, liveView = false;
static char fileName[16] = "";
static uint32_t nextFileIndex = 1, recordsWritten = 0;
static uint32_t lastSyncMs = 0, keyOffSinceMs = 0, lastSdRetryMs = 0;

static bool ig1 = false, ig1Raw = false;      // debounced / raw
static uint32_t ig1ChangeMs = 0, keyOnMs = 0;

static bool nepSeen = false, nepActive = false;
static uint16_t lastNepCount = 0;
static uint32_t lastNepMs = 0;
static float vspHz = 0, nepHz = 0;
static uint16_t freqVsp = 0, freqNep = 0;
static uint32_t freqMs = 0;

static float imotZeroV = CFG_IMOT_ZERO_V;     // current sensor output at 0 A
static bool zeroDone = false;                 // measured during this key cycle
static float zeroSum = 0;
static uint32_t zeroN = 0, zeroStartMs = 0;

struct Derived {                              // one record in engineering units
  float vt3, vt6, vcc2, vbat, imotSensorV, imotA;
  bool  ig1, wlpOn, rtOn;
};

// ═══════════════════════════════════
// INTERRUPTS
// ═══════════════════════════════════
void onVspEdge() {
  const uint32_t t = micros();
  if (t - vspEdgeUs >= CFG_EDGE_MIN_US) { vspEdgeUs = t; vspCount++; }
}

void onNepEdge() {
  const uint32_t t = micros();
  if (t - nepEdgeUs >= CFG_EDGE_MIN_US) { nepEdgeUs = t; nepCount++; }
}

static void sampleTick() {
  static uint16_t tick = 0;
  static uint8_t seq = 0;
  LogRecord r;
  r.tUs = micros();

  if (adcDual) {
    fastadc::convertPair(CFG_AIN0_VT6, CFG_AIN1_VT3, r.vt6, r.vt3);
  } else {
    r.vt3 = fastadc::convert(ADC0, CFG_AIN0_VT3);
    r.vt6 = fastadc::convert(ADC0, CFG_AIN0_VT6);
  }
  r.vcc2 = fastadc::convert(ADC0, CFG_AIN0_VCC2);
  r.vbat = fastadc::convert(ADC0, CFG_AIN0_VBAT);
  r.imot = fastadc::convert(ADC0, CFG_AIN0_IMOT);

  // Once a second, also read VT3 through ADC0. If ADC1 disagrees three times
  // running while VT3 carries a real signal, fall back to sequential sampling.
  if (++tick >= CFG_SAMPLE_HZ) {
    tick = 0;
    if (adcDual && r.vt3 > 300) {
      const int diff = (int)fastadc::convert(ADC0, CFG_AIN0_VT3) - (int)r.vt3;
      adcMismatch = (diff > 60 || diff < -60) ? (uint8_t)(adcMismatch + 1) : 0;
      if (adcMismatch >= 3) adcDual = false;
    }
  }

  // Edge interrupts outrank this one; copy each count with its timestamp as a pair
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  r.vspCount = vspCount; r.vspEdgeUs = vspEdgeUs;
  r.nepCount = nepCount; r.nepEdgeUs = nepEdgeUs;
  __set_PRIMASK(primask);

  uint8_t dio = 0;
  if (digitalRead(CFG_PIN_IG1) == HIGH) dio |= DIO_IG1;
  if (digitalRead(CFG_PIN_WLP) == LOW)  dio |= DIO_WLP_ON;          // EPS unit sinks WLP to light the lamp
  if (CFG_HAVE_RT && digitalRead(CFG_PIN_RT) == HIGH) dio |= DIO_RT_ON;
  r.dio = dio;
  r.seq = seq++;
  r.diag = diagShared | (adcDual ? 0 : DIAG_ADC_SEQ);
  r.dropped = droppedRecs;
  latest = r;

  if (ringOn) {
    const uint32_t head = ringHead;
    const uint32_t next = (head + 1) & RING_MASK;
    if (next == ringTail) {
      if (droppedRecs < 0xFFFF) droppedRecs++;
    } else {
      ring[head] = r;
      __DMB();                                                       // record lands before the new head
      ringHead = next;
    }
  }
}

extern "C" void TC3_Handler(void) {
  TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
  sampleTick();
}

// TC3 is unused by the core (tone() uses TC0 on SAMD51)
static void startSampleTimer(uint32_t hz) {
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC3;
  GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;   // 48 MHz
  while (!(GCLK->PCHCTRL[TC3_GCLK_ID].reg & GCLK_PCHCTRL_CHEN)) {}

  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT16.SYNCBUSY.bit.ENABLE) {}
  TC3->COUNT16.CTRLA.bit.SWRST = 1;
  while (TC3->COUNT16.SYNCBUSY.bit.SWRST) {}
  TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV16;    // 3 MHz
  TC3->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;                                 // top = CC0
  TC3->COUNT16.CC[0].reg = (uint16_t)(48000000ul / 16 / hz - 1);
  while (TC3->COUNT16.SYNCBUSY.bit.CC0) {}
  TC3->COUNT16.INTENSET.reg = TC_INTENSET_OVF;

  NVIC_SetPriority(TC3_IRQn, 2);                                                // below the edge interrupts
  NVIC_EnableIRQ(TC3_IRQn);
  TC3->COUNT16.CTRLA.bit.ENABLE = 1;
  while (TC3->COUNT16.SYNCBUSY.bit.ENABLE) {}
}

// ═══════════════════════════════════
// HELPERS
// ═══════════════════════════════════
static Derived derive(const LogRecord& r) {
  Derived d;
  d.vt3  = r.vt3  * V_PER_COUNT_VT;
  d.vt6  = r.vt6  * V_PER_COUNT_VT;
  d.vcc2 = r.vcc2 * V_PER_COUNT_VT;
  d.vbat = r.vbat * V_PER_COUNT_VBAT;
  d.imotSensorV = r.imot * V_PER_COUNT_IMOT;
  d.imotA = (d.imotSensorV - imotZeroV) / CFG_IMOT_V_PER_A;
  d.ig1   = r.dio & DIO_IG1;
  d.wlpOn = r.dio & DIO_WLP_ON;
  d.rtOn  = r.dio & DIO_RT_ON;
  return d;
}

static LogRecord snapshot() {
  noInterrupts();
  const LogRecord r = latest;
  interrupts();
  return r;
}

// Fixed-point float formatting (newlib-nano printf has no %f)
static const char* fmtFixed(char* buf, size_t n, float v, uint8_t digits) {
  int32_t scale = 1;
  for (uint8_t i = 0; i < digits; i++) scale *= 10;
  int32_t x = (int32_t)lroundf(v * scale);
  const char* sign = x < 0 ? "-" : "";
  if (x < 0) x = -x;
  if (digits) snprintf(buf, n, "%s%ld.%0*ld", sign, (long)(x / scale), (int)digits, (long)(x % scale));
  else        snprintf(buf, n, "%s%ld", sign, (long)x);
  return buf;
}

static void col(const char* s, uint8_t width) {
  Serial.print(s);
  for (size_t n = strlen(s); n < width; n++) Serial.print(' ');
}

// ═══════════════════════════════════
// SD CARD
// ═══════════════════════════════════
static uint32_t findNextIndex() {
  FsFile dir, f;
  uint32_t maxIdx = 0;
  char nm[16];
  if (!dir.open("/")) return 1;
  while (f.openNext(&dir, O_RDONLY)) {
    if (!f.isDir() && f.getName(nm, sizeof(nm)) && strncmp(nm, "EPS", 3) == 0) {
      const uint32_t idx = strtoul(nm + 3, nullptr, 10);
      if (idx > maxIdx) maxIdx = idx;
    }
    f.close();
  }
  dir.close();
  return maxIdx + 1;
}

static void fillHeader(LogHeader& h) {
  memset(&h, 0, sizeof(h));
  memcpy(h.magic, LOG_MAGIC, sizeof(h.magic));
  h.formatVersion = LOG_FORMAT_VERSION;
  h.headerSize = sizeof(LogHeader);
  h.recordSize = sizeof(LogRecord);
  h.sampleHz = CFG_SAMPLE_HZ;
  h.startUnix = rtcOk ? rtc.now().unixtime() : 0;
  h.startUs = micros();
  h.vPerCountVt = V_PER_COUNT_VT;
  h.vPerCountVbat = V_PER_COUNT_VBAT;
  h.vPerCountImot = V_PER_COUNT_IMOT;
  h.imotVPerA = CFG_IMOT_V_PER_A;
  h.imotZeroV = imotZeroV;
  h.imotZeroMeasured = zeroDone;
  h.adcDual = adcDual;
  h.haveRt = CFG_HAVE_RT;
  strncpy(h.firmware, CFG_FIRMWARE_NAME, sizeof(h.firmware));
}

static LogHeader fileHeader;   // kept so it can be rewritten with the measured zero at close

static bool openLogFile() {
  while (nextFileIndex <= 99999) {
    snprintf(fileName, sizeof(fileName), "EPS%05lu.BIN", (unsigned long)nextFileIndex);
    if (!sd.exists(fileName)) break;
    nextFileIndex++;
  }
  if (nextFileIndex > 99999 || !logFile.open(fileName, O_RDWR | O_CREAT | O_EXCL)) {
    sdFault = true;
    return false;
  }
  fillHeader(fileHeader);
  if (logFile.write(&fileHeader, sizeof(fileHeader)) != sizeof(fileHeader)) {
    logFile.close();
    sdFault = true;
    return false;
  }
  noInterrupts();
  ringHead = ringTail = 0;          // restart at a block boundary
  droppedRecs = 0;
  ringOn = true;
  interrupts();
  nextFileIndex++;
  recordsWritten = 0;
  fileOpen = true;
  lastSyncMs = keyOffSinceMs = millis();
  if (Serial) { Serial.print(F("log: opened ")); Serial.println(fileName); }
  return true;
}

// Writes queued records. Normally whole 512-byte blocks in chunks of at least
// CFG_WRITE_MIN_RECORDS; flushAll writes everything (used when closing).
static bool writeRing(bool flushAll) {
  const uint32_t head = ringHead;
  __DMB();
  uint32_t tail = ringTail;
  uint32_t avail = (head - tail) & RING_MASK;
  if (!flushAll && avail < CFG_WRITE_MIN_RECORDS) return true;
  while (avail) {
    uint32_t n = (tail + avail <= CFG_RING_RECORDS) ? avail : CFG_RING_RECORDS - tail;   // contiguous run
    if (!flushAll && tail + n < CFG_RING_RECORDS) n -= n % RECORDS_PER_BLOCK;
    if (n == 0) break;
    const size_t bytes = n * sizeof(LogRecord);
    if (logFile.write(&ring[tail], bytes) != bytes) { sdFault = true; return false; }
    tail = (tail + n) & RING_MASK;
    ringTail = tail;
    avail -= n;
    recordsWritten += n;
  }
  return true;
}

static void closeLogFile() {
  ringOn = false;
  writeRing(true);
  // Rewrite the header now that this key cycle's current-sensor zero is known
  fileHeader.imotZeroV = imotZeroV;
  fileHeader.imotZeroMeasured = zeroDone;
  fileHeader.adcDual = adcDual;
  if (logFile.seekSet(0)) logFile.write(&fileHeader, sizeof(fileHeader));
  logFile.close();
  fileOpen = false;
  if (Serial) {
    Serial.print(F("log: closed ")); Serial.print(fileName);
    Serial.print(F(", ")); Serial.print(recordsWritten); Serial.print(F(" records, "));
    Serial.print(droppedRecs); Serial.println(F(" dropped"));
  }
}

static void serviceLogFile(uint32_t nowMs) {
  if (!sdOk || sdFault) {
    if (fileOpen) { logFile.close(); fileOpen = false; ringOn = false; }
    // Retry the card every 5 s while there's something to record
    if ((ig1 || manualRecord) && nowMs - lastSdRetryMs >= 5000) {
      lastSdRetryMs = nowMs;
      sd.end();
      sdOk = sd.begin(SdSpiConfig(CFG_PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(12)));
      if (sdOk) { sdFault = false; nextFileIndex = findNextIndex(); }
    }
    return;
  }
  if (!fileOpen) {
    if (ig1 || manualRecord) openLogFile();
    return;
  }
  if (!writeRing(false)) return;
  if (ig1 || manualRecord) keyOffSinceMs = nowMs;
  if (nowMs - keyOffSinceMs >= CFG_KEYOFF_CLOSE_MS) {
    closeLogFile();
  } else if (nowMs - lastSyncMs >= CFG_SYNC_MS) {
    if (!logFile.sync()) sdFault = true;
    lastSyncMs = nowMs;
  }
}

// ═══════════════════════════════════
// SIGNAL BOOKKEEPING (runs every pass of loop)
// ═══════════════════════════════════
static void trackSignals(const LogRecord& s, const Derived& d, uint32_t nowMs) {
  // IG1 debounce; a fresh key-on re-measures the current-sensor zero
  if (d.ig1 != ig1Raw) { ig1Raw = d.ig1; ig1ChangeMs = nowMs; }
  if (nowMs - ig1ChangeMs >= CFG_IG1_DEBOUNCE_MS && ig1 != ig1Raw) {
    ig1 = ig1Raw;
    if (ig1) { keyOnMs = nowMs; zeroDone = false; zeroN = 0; }
  }

  if (s.nepCount != lastNepCount) { lastNepCount = s.nepCount; lastNepMs = nowMs; nepSeen = true; }
  nepActive = nepSeen && nowMs - lastNepMs < 500;

  if (nowMs - freqMs >= 1000) {
    const float dt = (nowMs - freqMs) / 1000.0f;
    vspHz = (uint16_t)(s.vspCount - freqVsp) / dt;
    nepHz = (uint16_t)(s.nepCount - freqNep) / dt;
    freqVsp = s.vspCount; freqNep = s.nepCount; freqMs = nowMs;
  }

  // Zero the current sensor while the motor relay is open (no current can flow)
  const bool relayOpen = CFG_HAVE_RT ? !d.rtOn : !nepActive;
  const bool sensorOk = d.imotSensorV > 0.5f && d.imotSensorV < 4.5f;
  if (!zeroDone && ig1 && nowMs - keyOnMs > 500 && relayOpen && sensorOk) {
    if (zeroN == 0) { zeroSum = 0; zeroStartMs = nowMs; }
    zeroSum += d.imotSensorV;
    zeroN++;
    if (nowMs - zeroStartMs >= CFG_ZERO_WINDOW_MS) { imotZeroV = zeroSum / zeroN; zeroDone = true; }
  } else if (!zeroDone) {
    zeroN = 0;
  }
}

static void reportDiagChanges(uint16_t flags) {
  static uint16_t prev = 0;
  if (flags == prev) return;
  const uint16_t set = flags & ~prev, cleared = prev & ~flags;
  prev = flags;
  if (!Serial) return;
  for (uint8_t b = 0; b < 16; b++) {
    if (set & (1u << b))     { Serial.print(F("diag + ")); Serial.println(LeadDiag::name(b)); }
    if (cleared & (1u << b)) { Serial.print(F("diag - ")); Serial.println(LeadDiag::name(b)); }
  }
}

static void updateLed(uint32_t nowMs) {
  bool on;
  if (!sdOk || sdFault)                    on = (nowMs / 100) % 2;          // fast: card problem
  else if (fileOpen && (diagShared & DIAG_LEAD_MASK)) on = (nowMs / 250) % 2; // 2 Hz: lead warning
  else if (fileOpen)                       on = true;                       // solid: logging
  else                                     on = (nowMs % 2000) < 60;        // blip: waiting for key
  digitalWrite(CFG_PIN_LED, on ? HIGH : LOW);
}

// ═══════════════════════════════════
// SERIAL REPORTS
// ═══════════════════════════════════
static float nepRpm() { return nepHz * 60.0f / CFG_NEP_PULSES_PER_REV; }
static float vspMph() { return vspHz * 3600.0f / CFG_VSP_PULSES_PER_MILE; }

static void checkRow(const char* sig, const char* pin, const char* reads, const char* expect, const char* result) {
  col(sig, 8); col(pin, 13); col(reads, 20); col(expect, 38); Serial.println(result);
}

static void printInstallCheck() {
  const LogRecord s = snapshot();
  const Derived d = derive(s);
  const uint16_t flags = diagShared;
  const bool running = nepActive || d.rtOn;
  const bool vcc2Good = d.vcc2 >= 4.5f && d.vcc2 <= 5.5f;
  char v[24], b[16], b2[16];

  Serial.println(F("\n---- EPS logger install check ----"));
  Serial.println(F("Key ON. Run it with the engine OFF, then again with the engine running."));
  checkRow("signal", "pin", "reads", "expected (Honda manual, Fig. 9)", "result");

  snprintf(v, sizeof(v), "%s V", fmtFixed(b, sizeof(b), d.vbat, 2));
  checkRow("+B", "A2 WHT", v, "battery voltage, always",
           d.vbat < 1.0f ? "FAIL: +B lead or ground lead open" :
           (d.vbat < 10.5f || d.vbat > 15.5f) ? "WARN: outside 10.5-15.5 V" : "PASS");

  checkRow("IG1", "B6 YEL", d.ig1 ? "ON" : "OFF", "ON with the key ON",
           (flags & DIAG_IG1_OPEN) ? "FAIL: car is on but IG1 reads OFF" : d.ig1 ? "PASS" : "WARN: key OFF?");

  snprintf(v, sizeof(v), "%s V", fmtFixed(b, sizeof(b), d.vcc2, 2));
  checkRow("Vcc2", "B2 YEL", v, "about 5 V (engine running)",
           vcc2Good ? "PASS" :
           d.vcc2 < 0.5f ? (running ? "FAIL: Vcc2 lead open" : "INFO: 0 V, start the engine and re-check")
                         : "WARN: outside 4.5-5.5 V");

  const char* needVcc2 = "INFO: needs Vcc2 first";
  snprintf(v, sizeof(v), "%s V", fmtFixed(b, sizeof(b), d.vt3, 3));
  checkRow("VT3", "B10 BLU", v, "about 1.5-3.5 V while steering",
           !vcc2Good ? needVcc2 : d.vt3 < 0.3f ? "FAIL: VT3 lead open" :
           (d.vt3 < 1.0f || d.vt3 > 4.0f) ? "WARN: outside 1.0-4.0 V" : "PASS");
  snprintf(v, sizeof(v), "%s V", fmtFixed(b, sizeof(b), d.vt6, 3));
  checkRow("VT6", "B8 RED", v, "about 1.5-3.5 V while steering",
           !vcc2Good ? needVcc2 : d.vt6 < 0.3f ? "FAIL: VT6 lead open" :
           (d.vt6 < 1.0f || d.vt6 > 4.0f) ? "WARN: outside 1.0-4.0 V" : "PASS");
  snprintf(v, sizeof(v), "%s V", fmtFixed(b, sizeof(b), (d.vt3 + d.vt6) / 2, 3));
  checkRow("", "VT3+VT6 avg", v, "ECU checks this average (DTC 16)", "INFO");

  snprintf(v, sizeof(v), "%s A", fmtFixed(b, sizeof(b), d.imotA, 1));
  const bool sensorBad = d.imotSensorV < 0.5f || d.imotSensorV > 4.5f;
  checkRow("Imot", "GRN clamp", v, "0 A until the engine starts",
           sensorBad ? "FAIL: current sensor unpowered or unplugged" :
           (!running && (d.imotA > 2.0f || d.imotA < -2.0f)) ? "WARN: not 0 A with the relay open" : "PASS");

  checkRow("WLP", "B13 YEL/RED", d.wlpOn ? "lamp ON" : "lamp OFF", "ON at key-on, OFF once running",
           (running && d.wlpOn) ? "WARN: lamp ON while running (EPS fault, or lead open)" : "INFO");
  if (CFG_HAVE_RT)
    checkRow("RT+", "B11 PNK/BLU", d.rtOn ? "ON" : "OFF", "ON once the engine runs",
             (flags & DIAG_RT_OPEN) ? "WARN: OFF while running (lead open?)" : "INFO");
  snprintf(v, sizeof(v), "%s Hz = %u rpm", fmtFixed(b, sizeof(b), nepHz, 1), (unsigned)nepRpm());
  checkRow("NEP", "B14 BLU", v, "pulses once the engine runs",
           (flags & DIAG_NEP_MISSING) ? "WARN: relay ON, no pulses (lead open?)" : nepActive ? "PASS" : "INFO");
  snprintf(v, sizeof(v), "%s Hz = %s mph", fmtFixed(b, sizeof(b), vspHz, 1), fmtFixed(b2, sizeof(b2), vspMph(), 1));
  checkRow("VSP", "B4 BLU/WHT", v, "pulses when the car rolls", vspHz > 0 ? "PASS" : "INFO: roll the car to check");

  Serial.print(F("Active diag flags: "));
  bool any = false;
  for (uint8_t i = 0; i < 16; i++) {
    if ((flags | (adcDual ? 0 : DIAG_ADC_SEQ)) & (1u << i)) { Serial.print(LeadDiag::name(i)); Serial.print(' '); any = true; }
  }
  Serial.println(any ? "" : "none");
}

static void printLive(const LogRecord& s, const Derived& d) {
  char b[16];
  Serial.print(F("t="));     Serial.print(fmtFixed(b, sizeof(b), s.tUs / 1e6f, 3));
  Serial.print(F(" VT3="));  Serial.print(fmtFixed(b, sizeof(b), d.vt3, 3));
  Serial.print(F(" VT6="));  Serial.print(fmtFixed(b, sizeof(b), d.vt6, 3));
  Serial.print(F(" Vcc2=")); Serial.print(fmtFixed(b, sizeof(b), d.vcc2, 2));
  Serial.print(F(" +B="));   Serial.print(fmtFixed(b, sizeof(b), d.vbat, 2));
  Serial.print(F(" I="));    Serial.print(fmtFixed(b, sizeof(b), d.imotA, 1));
  Serial.print(F("A NEP=")); Serial.print(fmtFixed(b, sizeof(b), nepHz, 1));
  Serial.print(F("Hz("));    Serial.print((unsigned)nepRpm());
  Serial.print(F("rpm) VSP="));Serial.print(fmtFixed(b, sizeof(b), vspHz, 1));
  Serial.print(F("Hz("));    Serial.print(fmtFixed(b, sizeof(b), vspMph(), 1));
  Serial.print(F("mph)"));
  Serial.print(d.ig1 ? F(" IG1") : F(" ---"));
  Serial.print(d.wlpOn ? F(" LAMP") : F(" ----"));
  Serial.print(d.rtOn ? F(" RLY") : F(" ---"));
  Serial.print(F(" diag=0x")); Serial.println(s.diag, HEX);
}

static void printStatus() {
  Serial.print(F("firmware: ")); Serial.println(F(CFG_FIRMWARE_NAME));
  Serial.print(F("card: ")); Serial.println(!sdOk ? "not found" : sdFault ? "FAULT" : "ok");
  Serial.print(F("file: ")); Serial.println(fileOpen ? fileName : "(none open)");
  Serial.print(F("records written: ")); Serial.println(recordsWritten);
  Serial.print(F("dropped: ")); Serial.println(droppedRecs);
  Serial.print(F("adc: ")); Serial.println(adcDual ? "VT3/VT6 simultaneous (ADC1 + ADC0)" : "sequential (ADC1 cross-check failed)");
  Serial.print(F("current zero: ")); Serial.print(imotZeroV, 4);
  Serial.println(zeroDone ? F(" V (measured this key cycle)") : F(" V (not yet measured)"));
  Serial.print(F("manual record: ")); Serial.println(manualRecord ? "ON" : "off");
  Serial.print(F("rtc: "));
  if (!rtcOk) Serial.println(F("not found (files get startUnix = 0)"));
  else {
    Serial.print(rtc.now().timestamp());
    Serial.println(rtcFromBuild ? F("  (set from build time, use t<unix> to correct)") : F(""));
  }
}

static void runCommand(const char* line) {
  switch (line[0]) {
    case 'c': printInstallCheck(); break;
    case 'l': liveView = !liveView; break;
    case 's': printStatus(); break;
    case 'r':
      manualRecord = !manualRecord;
      Serial.println(manualRecord ? F("manual record ON") : F("manual record off"));
      break;
    case 't': {
      const uint32_t unixTime = strtoul(line + 1, nullptr, 10);
      if (rtcOk && unixTime > 1600000000ul) {
        rtc.adjust(DateTime(unixTime));
        rtcFromBuild = false;
        Serial.print(F("rtc set: ")); Serial.println(rtc.now().timestamp());
      } else {
        Serial.println(F("usage: t<unix seconds>, e.g. t1793000000"));
      }
      break;
    }
    default:
      Serial.println(F("c install check | l live toggle | s status | r record toggle | t<unix> set clock"));
  }
}

static void handleSerial() {
  static char line[24];
  static uint8_t len = 0;
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (len) { line[len] = 0; runCommand(line); len = 0; }
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}

// ═══════════════════════════════════
// SETUP / LOOP
// ═══════════════════════════════════
void setup() {
  // Car-facing pins first: plain inputs, no pull-ups, never outputs
  pinMode(CFG_PIN_VSP, INPUT);
  pinMode(CFG_PIN_NEP, INPUT);
  pinMode(CFG_PIN_WLP, INPUT);
  pinMode(CFG_PIN_IG1, INPUT);
  if (CFG_HAVE_RT) pinMode(CFG_PIN_RT, INPUT);
  pinMode(CFG_PIN_LED, OUTPUT);

  Serial.begin(115200);                         // never waits for a host

  const uint8_t analogPins[] = {CFG_PIN_VCC2, CFG_PIN_VBAT, CFG_PIN_VT3, CFG_PIN_VT6, CFG_PIN_IMOT};
  fastadc::begin(analogPins, sizeof(analogPins), CFG_ADC_AVG_LOG2);

  rtcOk = rtc.begin();
  if (rtcOk) {
    if (!rtc.initialized() || rtc.lostPower()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      rtcFromBuild = true;
    }
    rtc.start();
  }

  sdOk = sd.begin(SdSpiConfig(CFG_PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(12)));
  if (sdOk) nextFileIndex = findNextIndex();

  attachInterrupt(digitalPinToInterrupt(CFG_PIN_VSP), onVspEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(CFG_PIN_NEP), onNepEdge, RISING);
  startSampleTimer(CFG_SAMPLE_HZ);
  freqMs = millis();
}

void loop() {
  static uint32_t lastDiagMs = 0, lastLiveMs = 0;
  const uint32_t nowMs = millis();

  handleSerial();
  const LogRecord s = snapshot();
  const Derived d = derive(s);
  trackSignals(s, d, nowMs);

  if (nowMs - lastDiagMs >= 50) {               // lead diagnostics at 20 Hz
    lastDiagMs = nowMs;
    DiagInputs in;
    in.vt3 = d.vt3; in.vt6 = d.vt6; in.vcc2 = d.vcc2; in.vbat = d.vbat;
    in.imotSensorV = d.imotSensorV; in.imotA = d.imotA;
    in.ig1 = ig1; in.wlpOn = d.wlpOn; in.rtOn = d.rtOn;
    in.haveRt = CFG_HAVE_RT; in.nepActive = nepActive;
    const uint16_t lead = leadDiag.update(in, nowMs);
    diagShared = lead | ((sdFault || droppedRecs) ? DIAG_SD_FAULT : 0);
    reportDiagChanges(diagShared | (adcDual ? 0 : DIAG_ADC_SEQ));
  }

  serviceLogFile(nowMs);
  updateLed(nowMs);

  if (liveView && Serial && nowMs - lastLiveMs >= 100) {
    lastLiveMs = nowMs;
    printLive(s, d);
  }
}
