// eps_bench_steer: steer the Insight EPS on the bench by replacing the torque sensor's
// VT3 / VT6 outputs with the Feather's two DACs, while keeping everything eps_bench_nep does
// (fake engine speed NEP, SAS over CAN, full CAN dump, D13 sync pulse for the Hantek).
//
// Use this sketch ONLY with VT3 and VT6 cut from the sensor and fed by the DACs (README).
// With the sensor still connected, use eps_bench_nep instead.
//
// DISCLAIMER: for research and bench testing only. Experimental, not safety-validated, not for
// use on a road vehicle. Use at your own risk.
//
// VT3 and VT6 move IN PHASE with the same gain (2026-11-15 captures: VT6 = 1.003 x VT3 + 10..20 mV),
// so both outputs are driven as rest + D:
//   VT3 = rest3 + D        VT6 = rest6 + D
// D is slew-limited, clamped to +-D_MAX, and each output to OUT_MIN..OUT_MAX.
//
// Sensor power is emulated (2026-11-21 logging with the real sensor): both lines are 0 V until the
// engine runs, VT3 steps to rest 0.72 s after NEP starts and VT6 0.14 s later, and `k` (key off)
// drops them to 0 V again. All voltages are AT THE EPS PIN: the DAC is corrected for the ECU's
// input loading through the 1 kOhm (`cal`). A0/A1 float while the Feather resets or is being
// flashed: key off before flashing or resetting.
//
// Outputs on USB serial (115200), one event per line:
//   # ...                                     comment / status
//   N,<us>,<rpm>                              NEP command (on change and once a second)
//   A,<us>,<deg>,<dps>,<zeroed_deg>           SAS frame (CAN 0x25)
//   C,<us>,<id hex>,<ext>,<rtr>,<len>,<hex>   every CAN frame (c0 turns off)
//   S,<us>,<level>,<n>                        D13 sync edge
//   V,<us>,<vt3_cmd>,<vt6_cmd>,<D>            commanded DAC outputs, volts (on change, max 50/s,
//                                             and once a second)
//
// Serial commands (Enter after each):
//   d0.05   set D target to +0.05 V (slews there)     x / d0   back to rest
//           D is only applied while NEP runs; r0 (or any NEP off) ramps D back to rest
//   s0.5    slew rate in V/s (0.05..5)                 r900     NEP rpm (0 = off, 500..3000)
//   rest3 2.560 / rest6 2.580   rest voltages          vref 3.300  DAC reference (calibration)
//   z  zero SAS     ?  status     c1 / c0  CAN frame dump on / off
//   clr / clr0  DTC clear mode on / off (NEP off only): L hold left, R release, cd-0.80 left level
//               (README "Clearing a latched DTC"). A left hold releases itself after 15 s or if
//               the wheel moves 20 deg (that would mean the EPS is assisting)
// OLED buttons: A  D back to rest if steering, else NEP on/off (clear mode: hold left / release)
//               B  next rpm preset     C  zero SAS
//
// NEP is 1.5 pulses per engine rev: 900 rpm = 22.5 Hz. Never send speed without rpm (DTC 22;
// VSP is not driven here) and don't hold >= 2000 rpm at 0 mph for 6 min (dropped to 900 at 5 min).
//
// Build: arduino-cli compile --fqbn adafruit:samd:adafruit_feather_m4_can eps_bench_steer

#include <CANSAME5x.h>       // "Adafruit CAN"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// ---------------------------------------------------------------- DAC steering (VT3 / VT6)
static const float D_MAX   = 0.30f;          // V, clamp on manual D (`d`)
static const float OUT_MIN = 1.90f;          // V, clamp on each output (normal / angle mode)
static const float OUT_MAX = 3.20f;
// High-voltage permission mode (`hv1` / `hv0`): lifts the D limit for manual D and angle mode to
// `hvmax` (default 0.60, at most 1.00 V: the real sensor read about +-1.3 V at the end stops). The
// Icarus data put full motor current at 0.6-0.7 V from centre, so this is where the motor can push
// hard. It switches itself off after 2 min, and on NEP off, key off, or `hv0` (Bench window: 0 key).
static const float HV_CAP = 1.00f, HV_DEFAULT = 0.60f;
static const float HV_OUT_MIN = 1.40f, HV_OUT_MAX = 3.70f;
static const uint32_t HV_TIMEOUT_MS = 120000;
static bool hvOn = false;
static float hvMax = HV_DEFAULT;
static uint32_t hvSinceMs = 0;
static const float SLEW_DEFAULT = 0.5f;      // V/s
static const float SLEW_MIN = 0.05f, SLEW_MAX = 5.0f;
// DAC reference = VDDANA (core: REFSEL VREFPU), nominally 3.300 V. 3.210 is calibrated against
// the bench Hantek 1008B (2026-11-16): with 3.300 the scope read 2.7 % low at every level (pure
// gain, no offset). The rest values come from the real sensor recorded on that same scope, so
// matching the DAC to the scope reproduces the sensor's true voltages whichever instrument is off.
// Residual after calibration: -3..+8 mV over D = -0.30..+0.30 V.
static const float VREF_DEFAULT = 3.210f;
static const uint32_t DAC_PERIOD_US = 1000;  // output update rate 1 kHz
static const uint32_t V_LOG_MIN_MS = 20;     // V lines at most 50/s while moving

// DTC clear mode (`clr`): the service manual's erase sequence needs "45 deg left and hold" with the
// engine stopped, read by the ECU through VT3/VT6. With the sensor cut, the DACs supply that left
// torque. Only with NEP off; NEP can't be turned on until clear mode is left.
static const float CLEAR_D_DEFAULT = -0.80f;  // V, left torque for the hold steps
// The real sensor reached -1.35 V (VT3 about 1.2 V) pushing into the left end stop (2026-11-15),
// and holding 45 deg with the tyres on the ground in a car is about that much torque. -0.80 did
// not turn the lamp off on the first try (2026-11-21), so allow down to the measured stop.
static const float CLEAR_D_MIN = -1.40f;      // V, most left torque allowed (VT3 about 1.16 V)
static const float CLEAR_OUT_MIN = 1.10f;     // V, just below the 1.2 V the sensor showed at the stop
static const float CLEAR_SLEW = 3.0f;         // V/s: the manual's windows are 4 s
static const uint32_t CLEAR_HOLD_MAX_MS = 15000;  // dead-man: a left hold releases itself
static const float CLEAR_MOVE_ABORT_DEG = 20.0f;  // wheel moves while holding -> release
static bool clearMode = false, clearLeft = false;
static float clearD = CLEAR_D_DEFAULT, clearStartDeg = 0;
static uint32_t clearSinceMs = 0;               // time of the last left / release change

// Angle mode (`a<deg>`): a PID on the SAS angle (CAN 0x25, ~84 Hz) sets D to steer to a target
// angle. Same interlocks as `d` (NEP on, sensor powered, D within +-D_MAX), plus: target within
// +-90 deg of the SAS zero, and the loop drops out (D back to 0) if the angle passes +-135 deg, the
// rate passes 300 deg/s, or the SAS goes stale. Sign: + angle = right, + D = right torque.
static const float ANG_TARGET_MAX = 90.0f, ANG_ABORT_DEG = 135.0f, ANG_ABORT_DPS = 300.0f;
static const float PID_SLEW = 3.0f;              // V/s while the PID drives D
static const float PID_I_MAX = 0.25f;            // V, cap on the integral term
static const float PID_MAX_CAP = 0.55f;          // V, most `pmax` allows without HV mode
static bool angMode = false;
static float angTarget = 0, angI = 0, angErr = 0;
static float pidKp = 0.004f;    // V per deg      (30 deg error -> 0.12 V)
static float pidKi = 0.002f;    // V per deg*s
static float pidKd = 0.0012f;   // V per deg/s    (150 deg/s -> 0.18 V of braking)
// Dead-band compensation, added in the error's direction outside the hold window. The bench rack
// broke away at +0.21..+0.27 V (right) and -0.27..-0.30 V (left), and never moved below 0.20 V
// (session 20261122_000423). Left: -0.27 V stalled and -0.30 V crept (session 20261122_013517).
static float pidDzR = 0.20f, pidDzL = 0.30f;
// Above ~0.35 V the unloaded rack runs at 225..660 deg/s (session 20261122_013517: median rate
// 3 deg/s at 0.30..0.35 V, 225 at 0.35..0.40, 400 at 0.40..0.45). Stay under that cliff.
static float pidMax = 0.33f;    // V, D limit in angle mode (manual `d` keeps D_MAX)
// Speed brake: heading for the target faster than this, D = 0 (coast on friction).
static float pidBrake = 150.0f; // deg/s
static float pidTol = 2.0f;     // deg, hold window: inside it D = 0 and the integral resets
static uint32_t angLastMs = 0;

// ---------------------------------------------------------------- sensor power emulation
// The ECU only powers the real torque sensor once the engine runs (2026-11-21 logging, real sensor,
// session 20261121_230808): VT3 = VT6 = 0 V at key off, IG1 and IG2 with the engine stopped. 0.72 s
// after the first NEP pulse VT3 steps to rest, VT6 follows 0.14 s later (steps, no ramp). They stay
// at rest when NEP stops with the key still on, and drop to 0 V when the key goes back to IG1 (VT6
// about 40 ms before VT3). The Feather can't see the key, so key-off is the `k` command: send it
// AFTER the key is down. `k` with the key still ON = 0 V on a powered sensor = DTC 16 (2026-11-21).
// Driving rest with the engine never started (what older builds did) is a state a real sensor
// never shows: the likely cause of the latched DTC 16.
static const uint32_t SENS_ON3_MS = 720, SENS_ON6_MS = 857;   // after NEP starts
static const uint32_t SENS_OFF3_MS = 44;                       // VT6 drops first, VT3 44 ms later
enum SensPhase : uint8_t { SENS_OFF, SENS_STARTING, SENS_ON, SENS_STOPPING };
static uint8_t sensPhase = SENS_OFF;
static uint32_t sensT0 = 0;
static bool on3 = false, on6 = false;             // is each line "powered" right now

// EPS input loading: with the ECU powered, B10/B8 read higher than the DAC drives through the
// 1 kOhm (2026-11-21 Hantek: rest 2.560 -> 2.648, 1.160 -> 1.398). Fit EPS = a * DAC + b, so the
// DAC is driven at (target - b) / a to put the target ON THE PIN. `cal a b3 b6`, `cal off`.
// Offsets trimmed with the key ON and the sensor "on" (2026-11-21: 0.362 / 0.383 gave pin
// 2.524 / 2.550 for 2.540 / 2.557, so b down by 16 / 7 mV). With the key OFF the ECU doesn't load
// the line and the pin reads ~0.1 V lower, which doesn't matter: nothing reads it then.
static float calA = 0.893f, calB3 = 0.346f, calB6 = 0.376f;

static const char *sensName() {
  switch (sensPhase) {
    case SENS_STARTING: return "starting";
    case SENS_ON: return "on";
    case SENS_STOPPING: return "stopping";
    default: return "off";
  }
}

// Rest values are the real sensor's, measured at the EPS pins with the engine running (2026-11-21).
static float rest3 = 2.540f, rest6 = 2.557f;
static float dTarget = 0, slew = SLEW_DEFAULT, vref = VREF_DEFAULT;
static float dOut = 0;                            // slewed in-phase offset
static float out3 = 0, out6 = 0;                  // target voltage AT THE EPS PIN right now
static uint16_t code3 = 0xFFFF, code6 = 0xFFFF;   // last written DAC codes
static uint16_t rpm = 0, lastOnRpm = 900;         // NEP command (see setRpm)

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static uint16_t toCode(float volts) {
  // 12-bit DAC, full scale = vref. NEVER call analogRead() on A0/A1: in this core it
  // switches the DAC channel off.
  const float c = volts / vref * 4095.0f + 0.5f;
  return (uint16_t)clampf(c, 0, 4095);
}

static float outMin() { return clearMode ? CLEAR_OUT_MIN : (hvOn ? HV_OUT_MIN : OUT_MIN); }
static float outMax() { return hvOn ? HV_OUT_MAX : OUT_MAX; }
static float manualMax() { return hvOn ? hvMax : D_MAX; }
static float angleMax() { return hvOn ? hvMax : (pidMax < PID_MAX_CAP ? pidMax : PID_MAX_CAP); }

static float dacVolts(float pin, float b) {        // DAC voltage that puts `pin` volts on the EPS pin
  if (pin <= 0.0f) return 0.0f;                    // "unpowered sensor": DAC at 0
  return (pin - b) / calA;
}

static void writeDacs(bool force) {
  const uint16_t c3 = toCode(dacVolts(out3, calB3));
  const uint16_t c6 = toCode(dacVolts(out6, calB6));
  if (force || c3 != code3) { analogWrite(A0, c3); code3 = c3; }
  if (force || c6 != code6) { analogWrite(A1, c6); code6 = c6; }
}

static void dacsToZeroNow() {          // first thing at boot: an unpowered sensor reads 0 V
  analogWriteResolution(12);
  out3 = out6 = 0;
  writeDacs(true);
  // Write both again. The core's first analogWrite to A1 disables and re-enables the whole DAC to
  // switch channel 1 on, and channel 0 is only right after it is written again (2026-11-16).
  writeDacs(true);
}

static float dNow() { return dOut; }

static void logV(bool force) {
  static uint32_t lastMs = 0;
  static uint16_t lc3 = 0xFFFF, lc6 = 0xFFFF;
  const uint32_t now = millis();
  const bool changed = code3 != lc3 || code6 != lc6;
  if (force || (changed && now - lastMs >= V_LOG_MIN_MS) || now - lastMs >= 1000) {
    Serial.printf("V,%lu,%.4f,%.4f,%.4f\n", micros(), out3, out6, dNow());
    lastMs = now; lc3 = code3; lc6 = code6;
  }
}

static void runSensor() {
  const uint32_t el = millis() - sensT0;
  if (sensPhase == SENS_STARTING) {
    if (!on3 && el >= SENS_ON3_MS) { on3 = true; Serial.println("# sensor: VT3 powered (rest)"); }
    if (!on6 && el >= SENS_ON6_MS) { on6 = true; sensPhase = SENS_ON; Serial.println("# sensor: ON (VT3 + VT6 at rest)"); }
  } else if (sensPhase == SENS_STOPPING) {
    if (on6) { on6 = false; }
    if (el >= SENS_OFF3_MS) { on3 = false; sensPhase = SENS_OFF; Serial.println("# sensor: OFF (0 V)"); }
  }
}

static void sensorStart() {            // NEP just started
  if (sensPhase != SENS_OFF) return;
  sensPhase = SENS_STARTING;
  sensT0 = millis();
  Serial.println("# sensor: engine started, powering in 0.72 s");
}

static void runDacs() {
  static uint32_t lastUs = 0;
  const uint32_t nowUs = micros();
  const uint32_t el = nowUs - lastUs;
  if (el < DAC_PERIOD_US) return;
  lastUs = nowUs;
  runSensor();
  const float step = (clearMode ? CLEAR_SLEW : (angMode ? PID_SLEW : slew)) * (el > 50000 ? 50000 : el) * 1e-6f;
  // NEP interlock: no torque offset unless NEP is running and the sensor is powered.
  // Clear mode is the one exception: left only (D <= 0), NEP off, see setClear().
  const bool powered = on3 && on6;
  const float d = clearMode ? (rpm ? 0.0f : clampf(dTarget, CLEAR_D_MIN, 0.0f))
                            : ((rpm && powered) ? clampf(dTarget, -(angMode ? angleMax() : manualMax()),
                                                                     angMode ? angleMax() : manualMax()) : 0.0f);
  dOut += clampf(d - dOut, -step, step);
  // Each line steps to rest when "powered" (like the real sensor) and to 0 V when not.
  out3 = on3 ? clampf(rest3 + dOut, outMin(), outMax()) : 0.0f;
  out6 = on6 ? clampf(rest6 + dOut, outMin(), outMax()) : 0.0f;
  writeDacs(false);
  logV(false);
}

static void hvSet(bool on, const char *why) {
  if (on == hvOn) { if (on) hvSinceMs = millis(); return; }   // hv1 again restarts the 2 min
  hvOn = on;
  hvSinceMs = millis();
  if (!on) {   // back inside the normal limits straight away (the slew brings D down)
    dTarget = clampf(dTarget, -(angMode ? angleMax() : D_MAX), angMode ? angleMax() : D_MAX);
  }
  Serial.printf("# HV mode %s %s: D limit %.2f V\n", on ? "ON" : "OFF", why, on ? hvMax : D_MAX);
}

static void angOff(const char *why) {
  if (!angMode) return;
  angMode = false;
  angI = 0;
  dTarget = 0;
  // Drop D at once, no slew: ramping 0.45 V down at the manual 0.5 V/s kept assist on for
  // ~0.9 s after a rate abort, and the wheel ran past the target to +206 / +228 deg
  // (session 20261122_013517).
  dOut = 0;
  Serial.printf("# angle mode OFF %s: D back to rest\n", why);
}

static void setD(float d, const char *why) {
  angOff("(manual D)");             // any manual D / x takes over from the angle loop
  if (clearMode) {
    if (d != 0) { Serial.println("# clear mode: use L (hold left) / R (release), clr0 to leave"); return; }
    dTarget = 0;
    if (clearLeft) { clearLeft = false; clearSinceMs = millis(); }
    Serial.printf("# D target +0.000 V %s\n", why);
    logV(true);
    return;
  }
  if (d != 0 && rpm == 0) {
    Serial.println("# NEP is off: D stays at rest. Send r900 first.");
    dTarget = 0;
    return;
  }
  if (d != 0 && sensPhase != SENS_ON) {
    Serial.println("# sensor not powered yet (starts 0.72 s after NEP): D stays 0.");
    dTarget = 0;
    return;
  }
  dTarget = clampf(d, -manualMax(), manualMax());
  Serial.printf("# D target %+.3f V %s\n", dTarget, why);
  logV(true);
}

// ---------------------------------------------------------------- NEP / SAS / CAN / sync
static const uint8_t  NEP_PIN  = 4;      // PA14 -> XY-MOS TRIG
static const uint8_t  SYNC_PIN = 13;     // red LED too
static const uint8_t  BTN_A = 9, BTN_B = 6, BTN_C = 5;

static const float    NEP_PULSES_PER_REV = 1.5f;
static const uint16_t RPM_MIN = 500, RPM_MAX = 3000;
static const uint16_t RPM_PRESETS[] = {900, 1200, 1500, 2000};
static const uint16_t RPM_HIGH = 2000;
static const uint32_t RPM_HIGH_MAX_MS = 5UL * 60UL * 1000UL;   // Honda's limit is 6 min

static const uint32_t CAN_BITRATE = 500000;
static const uint16_t SAS_CAN_ID  = 0x25;
static const uint32_t SAS_STALE_MS = 250;

static CANSAME5x CAN;
static Adafruit_SH1107 oled(64, 128, &Wire);

struct CanFrame { uint32_t us; uint32_t id; uint8_t d[8]; uint8_t len; uint8_t ext; uint8_t rtr; };
static const uint16_t RING = 256;               // power of two; uint8_t indices wrap with it
static volatile CanFrame ring[RING];
static volatile uint8_t ringHead = 0, ringTail = 0;
static volatile uint32_t ringDrops = 0, busFrames = 0;
static bool canDump = true;

static void onCan(int size) {                   // CAN interrupt
  const uint32_t us = micros();
  busFrames++;
  const uint8_t next = (uint8_t)(ringHead + 1);
  if (next == ringTail) { ringDrops++; while (CAN.available()) CAN.read(); return; }
  volatile CanFrame &f = ring[ringHead];
  f.us = us;
  f.id = CAN.packetId();
  f.ext = CAN.packetExtended();
  f.rtr = CAN.packetRtr();
  f.len = 0;
  if (f.rtr) f.len = size > 8 ? 8 : (size < 0 ? 0 : size);
  else while (CAN.available() && f.len < 8) f.d[f.len++] = CAN.read();
  while (CAN.available()) CAN.read();
  ringHead = next;
}

static int16_t signExtend(uint16_t v, uint8_t bits) {
  const uint16_t sign = 1u << (bits - 1);
  return (int16_t)((v ^ sign) - sign);
}

static float sasDeg = 0, zeroDeg = 0;
static int16_t sasRate = 0;
static uint32_t sasLastMs = 0, sasCount = 0;

static void pidOnSas();   // angle loop, run on every SAS frame (below)

static void drainCan() {
  uint8_t budget = 32;                           // keep the DAC slew loop on time
  while (ringTail != ringHead && budget--) {
    CanFrame f;
    noInterrupts();
    f.us = ring[ringTail].us;
    f.id = ring[ringTail].id;
    f.ext = ring[ringTail].ext;
    f.rtr = ring[ringTail].rtr;
    f.len = ring[ringTail].len;
    for (uint8_t i = 0; i < 8; i++) f.d[i] = ring[ringTail].d[i];
    ringTail = (uint8_t)(ringTail + 1);
    interrupts();
    if (canDump) {
      char hex[17];
      static const char H[] = "0123456789ABCDEF";
      const uint8_t nd = f.rtr ? 0 : f.len;
      for (uint8_t i = 0; i < nd; i++) { hex[2 * i] = H[f.d[i] >> 4]; hex[2 * i + 1] = H[f.d[i] & 15]; }
      hex[2 * nd] = 0;
      Serial.printf("C,%lu,%lX,%u,%u,%u,%s\n", f.us, (unsigned long)f.id, f.ext, f.rtr, f.len, hex);
    }
    if (f.ext || f.rtr || f.id != SAS_CAN_ID || f.len < 6) continue;
    const float angle = signExtend(((f.d[0] & 0x0F) << 8) | f.d[1], 12) * 1.5f;
    const float frac  = signExtend((f.d[4] & 0xF0) >> 4, 4) * 0.1f;
    sasRate = signExtend(((f.d[4] & 0x0F) << 8) | f.d[5], 12);
    sasDeg = angle + frac;
    sasLastMs = millis();
    sasCount++;
    Serial.printf("A,%lu,%.1f,%d,%.1f\n", f.us, sasDeg, sasRate, sasDeg - zeroDeg);
    pidOnSas();
  }
}

static uint32_t highSinceMs = 0;

static void setRpm(uint16_t r, const char *why) {
  if (r != 0) r = constrain(r, RPM_MIN, RPM_MAX);
  if (r == rpm) return;
  if (r && clearMode) {             // the erase sequence needs the engine stopped, and NEP on
    Serial.println("# clear mode: NEP stays off. Send clr0 first.");   // would add assist
    return;
  }
  rpm = r;
  if (rpm == 0) hvSet(false, "(NEP off)");
  if (rpm == 0 && dTarget != 0) {   // NEP interlock: steering offset only while NEP runs
    dTarget = 0;
    Serial.println("# NEP off: D ramps back to rest");
  }
  if (rpm) {
    lastOnRpm = rpm;
    tone(NEP_PIN, rpm * NEP_PULSES_PER_REV / 60.0f);
    sensorStart();                  // the ECU powers the sensor 0.72 s after the engine starts
  } else {
    noTone(NEP_PIN);
    digitalWrite(NEP_PIN, LOW);
    if (sensPhase == SENS_STARTING) {   // stopped before the ECU would have powered it
      sensPhase = SENS_OFF;
      on3 = on6 = false;
      Serial.println("# sensor: start cancelled (NEP stopped within 0.86 s)");
    }
  }
  highSinceMs = rpm >= RPM_HIGH ? millis() : 0;
  Serial.printf("N,%lu,%u\n", micros(), rpm);
  Serial.printf("# rpm %u (%.2f Hz) %s\n", rpm, rpm * NEP_PULSES_PER_REV / 60.0f, why);
}

static void guardRpm() {
  if (highSinceMs && millis() - highSinceMs > RPM_HIGH_MAX_MS) {
    Serial.println("# >= 2000 rpm for 5 min at 0 mph: dropping to 900");
    setRpm(900, "(time guard)");
  }
}

static uint16_t lfsr = 0xACE1;
static uint32_t syncNextMs = 0, syncCount = 0;
static bool syncLevel = false;

static void runSync() {
  const uint32_t now = millis();
  if ((int32_t)(now - syncNextMs) < 0) return;
  syncLevel = !syncLevel;
  digitalWrite(SYNC_PIN, syncLevel);
  const uint32_t us = micros();
  syncCount++;
  Serial.printf("S,%lu,%d,%lu\n", us, syncLevel, syncCount);
  lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u);
  syncNextMs = now + 200 + (lfsr % 801);
}

static void keyOff(const char *why) {   // key back to IG1 / off: the ECU unpowers the sensor
  setRpm(0, why);
  dTarget = 0;
  dOut = 0;
  if (sensPhase == SENS_OFF) { Serial.println("# sensor already off (0 V)"); return; }
  sensPhase = SENS_STOPPING;
  sensT0 = millis();
  Serial.printf("# key off %s: sensor powering down\n", why);
}

// ---------------------------------------------------------------- DTC clear mode
static bool sasFresh() { return millis() - sasLastMs < SAS_STALE_MS; }

static void setClear(bool on) {
  if (on == clearMode) { Serial.printf("# clear mode already %s\n", on ? "on" : "off"); return; }
  if (on && rpm) { Serial.println("# clear mode needs NEP off (engine stopped). Send r0 first."); return; }
  dTarget = 0;
  clearLeft = false;
  clearSinceMs = millis();
  clearMode = on;
  // In SCS mode the ECU must read the torque with the engine stopped, so it presumably powers the
  // sensor then. Not measured yet: clear mode powers both lines at rest, clr0 unpowers them.
  dOut = 0;
  sensPhase = on ? SENS_ON : SENS_OFF;
  on3 = on6 = on;
  if (on) {
    Serial.printf("# CLEAR MODE on: L = hold left (D %+.2f V), R = release, clr0 = leave\n", clearD);
    Serial.println("# SCS to ground, then EPS 12 V on. Watch the lamp; button A toggles L / R.");
  } else {
    Serial.println("# CLEAR MODE off: sensor off (0 V)");
  }
}

static void setClearLeft(bool left, const char *why) {
  if (!clearMode) { Serial.println("# not in clear mode (send clr)"); return; }
  clearLeft = left;
  dTarget = left ? clearD : 0;
  clearSinceMs = millis();
  clearStartDeg = sasDeg;
  Serial.printf("# CLR %s (D %+.2f V) %s\n", left ? "LEFT HOLD" : "RELEASE", dTarget, why);
  logV(true);
}

static void guardClear() {          // dead-man and "the wheel moved" checks while holding left
  if (!clearMode || !clearLeft) return;
  if (millis() - clearSinceMs > CLEAR_HOLD_MAX_MS) {
    setClearLeft(false, "(15 s hold limit)");
  } else if (sasFresh() && fabsf(sasDeg - clearStartDeg) > CLEAR_MOVE_ABORT_DEG) {
    Serial.println("# WHEEL MOVED during the hold: the EPS is assisting. Released. Cut the EPS 12 V.");
    setClearLeft(false, "(wheel moved)");
  }
}

// ---------------------------------------------------------------- angle mode (PID on the SAS)
static void angOn(float target) {
  if (clearMode) { Serial.println("# angle mode not in clear mode"); return; }
  if (!rpm) { Serial.println("# NEP is off: angle mode refused. Send r900 first."); return; }
  if (sensPhase != SENS_ON) { Serial.println("# sensor not powered yet: angle mode refused."); return; }
  if (!sasFresh()) { Serial.println("# SAS stale: angle mode refused."); return; }
  angTarget = clampf(target, -ANG_TARGET_MAX, ANG_TARGET_MAX);
  if (!angMode) { angI = 0; angLastMs = millis(); }
  angMode = true;
  Serial.printf("# angle mode ON: target %+.1f deg (now %+.1f)\n", angTarget, sasDeg - zeroDeg);
}

// P,<us>,<target>,<angle>,<err>,<D target>  on every SAS frame while the loop runs
static void pidOnSas() {
  if (!angMode) return;
  if (!rpm || sensPhase != SENS_ON || clearMode) { angOff("(NEP or sensor off)"); return; }
  const float ang = sasDeg - zeroDeg;
  if (fabsf(ang) > ANG_ABORT_DEG) { angOff("(angle past 135 deg)"); return; }
  if (abs(sasRate) > ANG_ABORT_DPS) { angOff("(rate past 300 deg/s)"); return; }
  const uint32_t now = millis();
  const float dt = clampf((now - angLastMs) * 1e-3f, 0.002f, 0.05f);
  angLastMs = now;
  angErr = angTarget - ang;
  const float lim = angleMax();
  float u = 0;
  if (fabsf(angErr) <= pidTol) {
    angI = 0;                                     // in the hold window: let friction hold it
  } else {
    const float p = pidKp * angErr;
    const float dterm = -pidKd * sasRate;         // rate damping (derivative on measurement)
    const float dz = angErr > 0 ? pidDzR : -pidDzL;   // jump past the assist dead band
    u = dz + p + angI + dterm;
    // Speed brake: already heading for the target fast, so no push (it overshoots otherwise).
    if (sasRate * angErr > 0 && abs(sasRate) > pidBrake) u = 0;
    // Integrate only while the output isn't saturated in the same direction (anti-windup).
    if (!(fabsf(u) >= lim && (u > 0) == (angErr > 0))) {
      angI = clampf(angI + pidKi * angErr * dt, -PID_I_MAX, PID_I_MAX);
    }
  }
  dTarget = clampf(u, -lim, lim);
  Serial.printf("P,%lu,%.1f,%.1f,%.1f,%.4f\n", micros(), angTarget, ang, angErr, dTarget);
}

static void printPid() {   // two short lines: this core's printf truncates long output
  Serial.printf("# pid kp %.4f ki %.4f kd %.5f\n", pidKp, pidKi, pidKd);
  Serial.printf("# pid2 max %.2f dzr %.2f dzl %.2f tol %.1f\n", pidMax, pidDzR, pidDzL, pidTol);
  Serial.printf("# brake above %.0f deg/s toward target\n", pidBrake);
}

static void guardAngle() {
  if (angMode && !sasFresh()) angOff("(SAS stale)");
  if (hvOn && millis() - hvSinceMs > HV_TIMEOUT_MS) hvSet(false, "(2 min limit)");
}

// ---------------------------------------------------------------- commands / UI
static void status() {   // short lines: this core's printf truncates long output
  Serial.printf("# rpm %u, SAS %s %.1f deg (zeroed %.1f)\n",
                rpm, millis() - sasLastMs < SAS_STALE_MS ? "ok" : "STALE", sasDeg, sasDeg - zeroDeg);
  Serial.printf("# D %+.3f V (target %+.3f), slew %.2f V/s\n", dNow(), dTarget, slew);
  Serial.printf("# VT3 %.3f V (rest %.3f), VT6 %.3f V (rest %.3f)\n", out3, rest3, out6, rest6);
  Serial.printf("# vref %.3f, codes %u / %u\n", vref, code3, code6);
  Serial.printf("# frames: sas %lu, bus %lu, drops %lu\n", sasCount, busFrames, ringDrops);
  Serial.printf("# sync %lu, can dump %s\n", syncCount, canDump ? "on" : "off");
  Serial.printf("# clear mode %s%s, clear D %+.2f V\n", clearMode ? "ON" : "off",
                clearMode ? (clearLeft ? " LEFT" : " released") : "", clearD);
  Serial.printf("# sensor %s (VT3 %s, VT6 %s)\n", sensName(), on3 ? "on" : "0V", on6 ? "on" : "0V");
  Serial.printf("# cal pin = %.3f x dac + %.3f / %.3f\n", calA, calB3, calB6);
  Serial.printf("# angle mode %s, target %+.1f deg\n", angMode ? "ON" : "off", angTarget);
  printPid();
  Serial.printf("# HV mode %s, hvmax %.2f V\n", hvOn ? "ON" : "off", hvMax);
}

static void handleCommand(char *b) {
  if (!strcmp(b, "hv1")) {
    if (!rpm || sensPhase != SENS_ON) Serial.println("# HV mode needs NEP on and the sensor ON.");
    else hvSet(true, "(serial)");
  } else if (!strcmp(b, "hv0")) {
    hvSet(false, "(serial)");
  } else if (!strncmp(b, "hvmax", 5)) {
    hvMax = clampf(atof(b + 5), D_MAX, HV_CAP);
    Serial.printf("# hvmax %.2f V\n", hvMax);
  } else if (!strcmp(b, "ax")) {
    angOff("(serial)");
  } else if (b[0] == 'a') {
    angOn(atof(b + 1));
  } else if (b[0] == 'p' && strchr("kidmtb", b[1])) {
    float *dst = nullptr;
    float lo = 0, hi = 0;
    const char *num = b + 2;
    if (!strncmp(b, "pbrk", 4)) { dst = &pidBrake; lo = 30; hi = 300; num = b + 4; }
    else if (!strncmp(b, "pmax", 4)) { dst = &pidMax; lo = 0.05f; hi = PID_MAX_CAP; num = b + 4; }
    else if (!strncmp(b, "ptol", 4)) { dst = &pidTol; lo = 0; hi = 15; num = b + 4; }
    else if (!strncmp(b, "pdzr", 4)) { dst = &pidDzR; lo = 0; hi = 0.40f; num = b + 4; }
    else if (!strncmp(b, "pdzl", 4)) { dst = &pidDzL; lo = 0; hi = 0.40f; num = b + 4; }
    else if (!strncmp(b, "pdz", 3)) {  // both directions
      pidDzR = pidDzL = clampf(atof(b + 3), 0, 0.40f);
    }
    else if (b[1] == 'k') { dst = &pidKp; hi = 0.05f; }
    else if (b[1] == 'i') { dst = &pidKi; hi = 0.05f; }
    else if (b[1] == 'd') { dst = &pidKd; hi = 0.01f; }
    if (dst) *dst = clampf(atof(num), lo, hi);
    angI = 0;
    printPid();
  } else if (!strcmp(b, "k") || !strcmp(b, "key0")) {
    keyOff("(serial)");
  } else if (!strncmp(b, "cal", 3)) {
    if (strstr(b, "off")) { calA = 1.0f; calB3 = calB6 = 0.0f; }
    else {
      float a = 0, b3 = 0, b6 = 0;
      if (sscanf(b + 3, "%f %f %f", &a, &b3, &b6) == 3 && a > 0.7f && a < 1.1f &&
          fabsf(b3) < 0.6f && fabsf(b6) < 0.6f) { calA = a; calB3 = b3; calB6 = b6; }
      else Serial.println("# cal <a 0.7..1.1> <b3> <b6> (|b| < 0.6), or cal off");
    }
    writeDacs(true);
    Serial.printf("# cal pin = %.3f x dac + %.3f / %.3f\n", calA, calB3, calB6);
  } else if (!strcmp(b, "clr") || !strcmp(b, "clr1")) {
    setClear(true);
  } else if (!strcmp(b, "clr0")) {
    setClear(false);
  } else if (!strncmp(b, "cd", 2)) {
    const float v = atof(b + 2);
    if (v <= -0.30f && v >= CLEAR_D_MIN) { clearD = v; Serial.printf("# clear D %+.2f V\n", clearD); }
    else Serial.println("# clear D must be -0.30..-1.40 (left)");
    if (clearLeft) dTarget = clearD;
  } else if (b[0] == 'L' || b[0] == 'l') {
    setClearLeft(true, "(serial)");
  } else if (b[0] == 'R') {
    setClearLeft(false, "(serial)");
  } else if (!strncmp(b, "rest3", 5) || !strncmp(b, "rest6", 5)) {
    const float v = clampf(atof(b + 5), OUT_MIN, OUT_MAX);
    (b[4] == '3' ? rest3 : rest6) = v;
    Serial.printf("# rest%c %.3f V (outputs slew there)\n", b[4], v);
  } else if (!strncmp(b, "vref", 4)) {
    const float v = atof(b + 4);
    if (v > 3.0f && v < 3.6f) { vref = v; writeDacs(true); Serial.printf("# vref %.3f\n", vref); }
    else Serial.println("# vref must be 3.0..3.6");
  } else if (b[0] == 'd') {
    setD(atof(b + 1), "(serial)");
  } else if (b[0] == 'x') {
    setD(0, "(stop: back to rest)");
  } else if (b[0] == 's') {
    slew = clampf(atof(b + 1), SLEW_MIN, SLEW_MAX);
    Serial.printf("# slew %.2f V/s\n", slew);
  } else if (b[0] == 'r') {
    setRpm(atoi(b + 1), "(serial)");
  } else if (b[0] == 'z') {
    zeroDeg = sasDeg; Serial.printf("# SAS zero %.1f\n", zeroDeg);
  } else if (b[0] == 'c') {
    canDump = b[1] != '0'; Serial.printf("# CAN frame dump %s\n", canDump ? "on" : "off");
  } else if (b[0] == '?') {
    status();
  } else {
    Serial.println("# d<V> x s<V/s> rest3/rest6 <V> vref <V> r<rpm> z ? c1/c0");
    Serial.println("# k = key off (sensor 0 V), cal <a> <b3> <b6> / cal off");
    Serial.println("# a<deg> angle hold, ax off; pk pi pd pdzr pdzl pmax ptol pbrk <v>");
    Serial.println("# hv1 / hv0 high-voltage mode (2 min), hvmax <V>");
    Serial.println("# DTC clear: clr / clr0, L hold left, R release, cd-0.80");
  }
}

static void handleSerial() {
  static char buf[24];
  static uint8_t n = 0;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c != '\n' && c != '\r') { if (n < sizeof buf - 1) buf[n++] = c; continue; }
    buf[n] = 0;
    if (n) handleCommand(buf);
    n = 0;
  }
}

static void handleButtons() {
  static bool pa = true, pb = true, pc = true;
  static uint8_t preset = 0;
  const bool a = digitalRead(BTN_A), b = digitalRead(BTN_B), c = digitalRead(BTN_C);
  if (pa && !a) {
    if (angMode) angOff("(button A)");
    else if (clearMode) setClearLeft(!clearLeft, "(button A)");
    else if (dTarget != 0 || fabsf(dNow()) > 0.001f) setD(0, "(button A: back to rest)");
    else setRpm(rpm ? 0 : lastOnRpm, "(button A)");
  }
  if (pb && !b) {
    preset = (preset + 1) % (sizeof RPM_PRESETS / sizeof RPM_PRESETS[0]);
    if (rpm) setRpm(RPM_PRESETS[preset], "(button B)");
    else lastOnRpm = RPM_PRESETS[preset];
  }
  if (pc && !c) { zeroDeg = sasDeg; Serial.printf("# SAS zero %.1f\n", zeroDeg); }
  pa = a; pb = b; pc = c;
}

// 128 x 64 layout:
//   y 0   NEP state / rpm                       right: LOCK | STEER | REST
//   y 11  steering angle, large                 right: "deg" + rate
//   y 30  D bar: centre tick, fills left/right to +-D_MAX, hollow mark at the target
//   y 40  D value (and target while ramping)
//   y 47  commanded VT3 / VT6
//   y 56  button hints for the current state (inverted)
// Clear mode: big LEFT / REL with seconds since the last change, to count the manual's 4 s windows.
static void drawClear() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SH110X_WHITE);
  oled.fillRect(0, 0, 128, 9, SH110X_WHITE);
  oled.setTextColor(SH110X_BLACK);
  oled.setCursor(1, 1);
  oled.print("DTC CLEAR  NEP off");
  oled.setTextColor(SH110X_WHITE);
  oled.setTextSize(2);
  oled.setCursor(0, 13);
  oled.print(clearLeft ? "LEFT" : "REL");
  oled.setCursor(64, 13);
  oled.printf("%4.1fs", (millis() - clearSinceMs) / 1000.0f);
  oled.setTextSize(1);
  oled.setCursor(0, 33);
  oled.printf("D %+.3f V (L %+.2f)", dNow(), clearD);
  oled.setCursor(0, 43);
  oled.printf("VT3 %.3f VT6 %.3f", out3, out6);
  oled.fillRect(0, 56, 128, 8, SH110X_WHITE);
  oled.setTextColor(SH110X_BLACK);
  oled.setCursor(1, 57);
  oled.print(clearLeft ? "A: RELEASE" : "A: HOLD LEFT");
  oled.setTextColor(SH110X_WHITE);
  oled.display();
}

static void drawOled() {
  if (clearMode) { drawClear(); return; }
  const float d = dNow();
  const bool steering = fabsf(d) > 0.0005f || dTarget != 0;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SH110X_WHITE);

  oled.setCursor(0, 0);
  if (rpm) oled.printf("NEP %u", rpm);
  else oled.printf("NEP off (%u)", lastOnRpm);
  const char *mode = sensPhase == SENS_OFF ? "S-OFF" : sensPhase == SENS_STARTING ? "S-WAIT"
                   : !rpm ? "LOCK" : hvOn ? "HV!" : (steering ? "STEER" : "REST");
  const int16_t mw = strlen(mode) * 6 + 4;
  if (steering && rpm) {                 // inverted badge while the motor is being commanded
    oled.fillRect(127 - mw, 0, mw + 1, 9, SH110X_WHITE);
    oled.setTextColor(SH110X_BLACK);
  }
  oled.setCursor(129 - mw, 1);
  oled.print(mode);
  oled.setTextColor(SH110X_WHITE);

  oled.setTextSize(2);
  oled.setCursor(0, 11);
  const bool sasOk = millis() - sasLastMs < SAS_STALE_MS;
  if (sasOk) oled.printf("%+6.1f", sasDeg - zeroDeg);
  else oled.print("SAS --");
  oled.setTextSize(1);
  oled.setCursor(92, 11);
  oled.print("deg");
  if (sasOk) { oled.setCursor(86, 20); oled.printf("%+4d/s", sasRate); }

  // D bar
  const int16_t cx = 64, half = 62, by = 30, bh = 7;
  oled.drawRect(cx - half - 1, by, 2 * half + 2, bh, SH110X_WHITE);
  oled.drawFastVLine(cx, by - 2, bh + 4, SH110X_WHITE);
  const int16_t px = (int16_t)(clampf(d / D_MAX, -1, 1) * half);
  if (px > 0) oled.fillRect(cx, by + 1, px, bh - 2, SH110X_WHITE);
  else if (px < 0) oled.fillRect(cx + px, by + 1, -px, bh - 2, SH110X_WHITE);
  const int16_t tx = cx + (int16_t)(clampf(dTarget / D_MAX, -1, 1) * half);
  oled.drawFastVLine(tx, by - 2, bh + 4, SH110X_WHITE);   // target tick
  oled.setCursor(0, 39);
  if (fabsf(d - dTarget) > 0.0005f) oled.printf("D %+.3f > %+.3f V", d, dTarget);
  else oled.printf("D %+.3f V", d);
  if (angMode) { oled.setCursor(80, 39); oled.printf(">%+4.0f", angTarget); }

  oled.setCursor(0, 48);
  oled.printf("VT3 %.3f VT6 %.3f", out3, out6);

  oled.fillRect(0, 56, 128, 8, SH110X_WHITE);
  oled.setTextColor(SH110X_BLACK);
  oled.setCursor(1, 57);
  oled.print(steering ? "A:rest  B:rpm  C:zero" : (rpm ? "A:NEP off B:rpm C:0" : "A:NEP on  B:rpm C:0"));
  oled.setTextColor(SH110X_WHITE);
  oled.display();
}

void setup() {
  dacsToZeroNow();                 // FIRST: like the real sensor, 0 V until the engine runs

  Serial.begin(115200);
  pinMode(NEP_PIN, OUTPUT);
  digitalWrite(NEP_PIN, LOW);
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_C, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(400000);
  delay(250);
  oled.begin(0x3C, true);
  oled.setRotation(1);
  oled.setTextColor(SH110X_WHITE);

  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false);
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true);
  const bool canOk = CAN.begin(CAN_BITRATE);
  if (canOk) CAN.onReceive(onCan);

  Serial.println("# eps_bench_steer: DAC A0->VT3 A1->VT6 in phase, NEP D4, sync D13, CAN 500k");
  Serial.println("# N,us,rpm | A,us,deg,dps,zd | C,us,id,ext,rtr,len,hex | S,us,lvl,n");
  Serial.println("# V,us,vt3_cmd,vt6_cmd,D");
  if (!canOk) Serial.println("# CAN start failed");
  Serial.println("# sensor: OFF (0 V)");
  Serial.printf("N,%lu,0\n", micros());
  logV(true);
}

void loop() {
  static uint32_t lastDraw = 0, lastRpmMs = 0;
  runDacs();
  drainCan();
  runDacs();
  if (millis() - lastRpmMs >= 1000) {
    lastRpmMs = millis();
    Serial.printf("N,%lu,%u\n", micros(), rpm);
  }
  runSync();
  handleSerial();
  handleButtons();
  guardRpm();
  guardClear();
  guardAngle();
  if (millis() - lastDraw >= 200) {     // the I2C redraw blocks ~25 ms; the slew catches up after
    lastDraw = millis();
    drawOled();
  }
}
