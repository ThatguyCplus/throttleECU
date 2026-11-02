// ═══════════════════════════════════════════════════════════════════════
// EPS Data Logger — lead diagnostics
//
// Checks every tap against what the Honda service manual says it should
// read (electrical_power_steering.pdf, Fig. 9), so a loose back-probe or a
// broken lead shows up on the bench or in the first minute of a drive, not
// after an hour of useless data.
//
// Every tap has a pull-down on the Proto Wing, so an open lead reads ~0 V
// (digital: LOW). "Reads zero when the car says it can't be zero" is the
// main open-lead test.
//
// Plain C++ with no Arduino calls, so it compiles on any toolchain.
// ═══════════════════════════════════════════════════════════════════════
#pragma once
#include <stdint.h>

enum DiagFlag : uint16_t {
  DIAG_VBAT_MISSING = 1u << 0,   // +B < 1 V. A2 is always hot → +B lead or the ground lead is open
  DIAG_VBAT_RANGE   = 1u << 1,   // +B outside 10.5–15.5 V with the key ON
  DIAG_VCC2_MISSING = 1u << 2,   // Vcc2 ~0 V with the engine running → Vcc2 lead open
  DIAG_VCC2_RANGE   = 1u << 3,   // Vcc2 outside 4.5–5.5 V (manual: about 5 V)
  DIAG_VT3_OPEN     = 1u << 4,   // VT3 ~0 V while Vcc2 is good → VT3 lead open
  DIAG_VT3_RANGE    = 1u << 5,   // VT3 outside 1.0–4.0 V (manual: about 1.5–3.5 V)
  DIAG_VT6_OPEN     = 1u << 6,
  DIAG_VT6_RANGE    = 1u << 7,
  DIAG_IMOT_SENSOR  = 1u << 8,   // current sensor output < 0.5 V or > 4.5 V → unpowered or unplugged
  DIAG_IMOT_ZERO    = 1u << 9,   // |I| > 2 A while the relay is open (must be 0 A)
  DIAG_IG1_OPEN     = 1u << 10,  // IG1 reads OFF but Vcc2 or NEP shows the car is on
  DIAG_WLP_ON_RUN   = 1u << 11,  // EPS lamp ON 30 s with the engine running → EPS fault, or WLP lead open
  DIAG_RT_OPEN      = 1u << 12,  // relay drive OFF while running with the lamp OFF → RT+ lead open
  DIAG_NEP_MISSING  = 1u << 13,  // relay ON but no engine pulses for 3 s → NEP lead open (or auto-stop)
  DIAG_ADC_SEQ      = 1u << 14,  // ADC1 cross-check failed: VT3/VT6 now sampled one after the other
  DIAG_SD_FAULT     = 1u << 15,  // card write failed or records were dropped
};

// Lead faults the persistence logic owns; the last two are set by the logger itself.
static const uint16_t DIAG_LEAD_MASK = 0x3FFF;

struct DiagInputs {
  float vt3, vt6, vcc2, vbat;    // car-side volts
  float imotSensorV;             // current sensor output, volts (before the 10k/18k divider)
  float imotA;                   // amps, zero-corrected
  bool  ig1, wlpOn, rtOn;        // debounced digital states
  bool  haveRt;                  // RT+ lead fitted
  bool  nepActive;               // NEP edges seen in the last 500 ms
};

class LeadDiag {
 public:
  // Call at 10–50 Hz. Returns the lead flags currently active.
  uint16_t update(const DiagInputs& in, uint32_t nowMs) {
    const bool running = in.nepActive || (in.haveRt && in.rtOn);
    if (in.ig1 != ig1Prev_) { ig1Prev_ = in.ig1; ig1SinceMs_ = nowMs; }
    if (running != runPrev_) { runPrev_ = running; runSinceMs_ = nowMs; }
    const uint32_t keyOnMs = in.ig1 ? nowMs - ig1SinceMs_ : 0;
    const uint32_t runMs   = running ? nowMs - runSinceMs_ : 0;
    const bool vcc2Good    = in.vcc2 >= 4.5f && in.vcc2 <= 5.5f;
    const bool relayOpen   = in.haveRt ? !in.rtOn : !running;
    const bool sensorBad   = in.imotSensorV < 0.5f || in.imotSensorV > 4.5f;

    check(0,  in.vbat < 1.0f, 300, nowMs);
    check(1,  in.ig1 && keyOnMs > 2000 && in.vbat >= 1.0f && (in.vbat < 10.5f || in.vbat > 15.5f), 1000, nowMs);
    check(2,  runMs > 2000 && in.vcc2 < 0.5f, 300, nowMs);
    check(3,  in.ig1 && keyOnMs > 1000 && in.vcc2 >= 0.5f && !vcc2Good, 300, nowMs);
    check(4,  vcc2Good && in.vt3 < 0.3f, 300, nowMs);
    check(5,  vcc2Good && in.vt3 >= 0.3f && (in.vt3 < 1.0f || in.vt3 > 4.0f), 300, nowMs);
    check(6,  vcc2Good && in.vt6 < 0.3f, 300, nowMs);
    check(7,  vcc2Good && in.vt6 >= 0.3f && (in.vt6 < 1.0f || in.vt6 > 4.0f), 300, nowMs);
    check(8,  sensorBad, 300, nowMs);
    check(9,  in.ig1 && keyOnMs > 1500 && relayOpen && !sensorBad && (in.imotA > 2.0f || in.imotA < -2.0f), 500, nowMs);
    check(10, !in.ig1 && (vcc2Good || in.nepActive), 500, nowMs);
    check(11, running && in.wlpOn, 30000, nowMs);
    check(12, in.haveRt && in.nepActive && runMs > 10000 && !in.wlpOn && !in.rtOn, 2000, nowMs);
    check(13, in.haveRt && in.rtOn && !in.nepActive, 3000, nowMs);
    return flags_;
  }

  uint16_t flags() const { return flags_; }

  static const char* name(uint8_t bit) {
    static const char* const kNames[16] = {
      "VBAT_MISSING", "VBAT_RANGE", "VCC2_MISSING", "VCC2_RANGE", "VT3_OPEN", "VT3_RANGE",
      "VT6_OPEN", "VT6_RANGE", "IMOT_SENSOR", "IMOT_ZERO", "IG1_OPEN", "WLP_ON_RUN",
      "RT_OPEN", "NEP_MISSING", "ADC_SEQ", "SD_FAULT"};
    return bit < 16 ? kNames[bit] : "?";
  }

 private:
  // A flag sets after its condition holds for setMs and clears after it has
  // been false for 1 s, so one noisy sample can't flicker it.
  void check(uint8_t bit, bool cond, uint32_t setMs, uint32_t nowMs) {
    const uint16_t mask = (uint16_t)(1u << bit);
    if (cond) {
      if (!active_[bit]) { active_[bit] = true; sinceMs_[bit] = nowMs; }
      if (nowMs - sinceMs_[bit] >= setMs) flags_ |= mask;
    } else {
      if (active_[bit]) { active_[bit] = false; sinceMs_[bit] = nowMs; }
      if ((flags_ & mask) && nowMs - sinceMs_[bit] >= 1000) flags_ &= (uint16_t)~mask;
    }
  }

  uint16_t flags_ = 0;
  bool     active_[14] = {};
  uint32_t sinceMs_[14] = {};
  bool     ig1Prev_ = false, runPrev_ = false;
  uint32_t ig1SinceMs_ = 0, runSinceMs_ = 0;
};
