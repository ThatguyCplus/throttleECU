// ═══════════════════════════════════════════════════════════════════════
// Throttle ECU — PID Position Control (NO CAN)
// Target: STM32F411CE (WeAct BlackPill)
// Safety: Encoder loss, overcurrent, watchdog, power monitoring
//
//  ┌───────────────────────────────────────────────────────┐
//  │  ALL TUNABLE PARAMETERS ARE IN config.h               │
//  │  Open that file to change pins, limits, gains, etc.   │
//  │  Do NOT edit values in this file.                     │
//  └───────────────────────────────────────────────────────┘
//
// ═══════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include "config.h"
#include "initialsafe.h"

// ═══════════════════════════════════
// SERIAL PORT SELECTION (STM32)
// ═══════════════════════════════════
#if defined(SERIAL_PORT_MONITOR)
  #define MON_PORT SERIAL_PORT_MONITOR
#else
  #define MON_PORT Serial
#endif

// ═══════════════════════════════════
// DERIVED CONSTANTS (from config.h)
// ═══════════════════════════════════
const uint8_t  PIN_RPWM  = CFG_PIN_RPWM;
const uint8_t  PIN_LPWM  = CFG_PIN_LPWM;
const uint8_t  PIN_REN   = CFG_PIN_REN;
const uint8_t  PIN_LEN   = CFG_PIN_LEN;
const uint8_t  PIN_RIS   = CFG_PIN_RIS;
const uint8_t  PIN_LIS   = CFG_PIN_LIS;
const uint8_t  PIN_RELAY = CFG_PIN_RELAY;
const uint8_t  PIN_ENC   = CFG_PIN_ENC;

const uint32_t PWM_FREQ_HZ = CFG_PWM_FREQ_HZ;
const uint8_t  PWM_BITS    = CFG_PWM_BITS;
const uint16_t PWM_MAX     = (1u << CFG_PWM_BITS) - 1;

const uint16_t ENC_OFFSET      = CFG_ENC_OFFSET;
const uint16_t ENC_DATA        = CFG_ENC_DATA;
const uint16_t ENC_PERIOD_CLKS = CFG_ENC_PERIOD_CLKS;

// Throttle range — set by auto-cal on boot, or fallback from config.h
int32_t  ANGLE_MIN = CFG_ANGLE_MIN;
int32_t  ANGLE_MAX = CFG_ANGLE_MAX;

const int32_t  PID_DEADBAND    = CFG_PID_DEADBAND;
const uint16_t MIN_DUTY_THRESH = CFG_MIN_DUTY_THRESH;
const uint32_t SETTLE_TIME_MS  = CFG_SETTLE_TIME_MS;
const int32_t  SETTLE_WINDOW   = CFG_SETTLE_WINDOW;

// PID gains (live-tunable via serial, defaults from config.h)
float Kp = CFG_KP_DEFAULT;
float Ki = CFG_KI_DEFAULT;
float Kd = CFG_KD_DEFAULT;

// ═══════════════════════════════════
// STATE
// ═══════════════════════════════════
enum Mode { MODE_MANUAL = 0, MODE_PID = 1, MODE_SAFE = 2 };
volatile Mode mode = MODE_MANUAL;
int8_t   dir         = +1;
uint16_t duty        = 0;
int32_t  targetAngle = ANGLE_MIN;
int16_t  throttlePct = 0;

// PID state
float    pidIntegral = 0.0f;
int32_t  pidPrevErr  = 0;
uint32_t pidLastUs   = 0;

// Settle state — tracks when motor has reached target and can coast
bool     settled     = false;
uint32_t settleStart = 0;

// Encoder
volatile uint32_t encRise   = 0;
volatile uint32_t encHigh   = 0;
volatile uint32_t encPeriod = 0;
volatile bool     encValid  = false;

// Serial buffers
static char    usbBuf[32],  uartBuf[32];
static uint8_t usbLen = 0,  uartLen = 0;

// Telemetry
uint32_t lastPrint    = 0;
uint32_t lastSafeTick = 0;

// ═══════════════════════════════════
// ENCODER ISR
// ═══════════════════════════════════
void encISR() {
  uint32_t now = micros();
  if (digitalRead(PIN_ENC)) {
    if (encRise != 0) encPeriod = now - encRise;
    encRise = now;
  } else {
    encHigh  = now - encRise;
    encValid = (encPeriod > 0);
  }
}

int32_t getAngle() {
  noInterrupts();
  uint32_t h = encHigh;
  uint32_t p = encPeriod;
  bool     v = encValid;
  interrupts();

  if (!v || p == 0) return -1;

  int32_t raw = (int32_t)((uint64_t)h * ENC_PERIOD_CLKS / p) - ENC_OFFSET;
  if (raw < 0)    raw = 0;
  if (raw > 4094) raw = 4094;

  return (int32_t)((uint32_t)raw * 36000UL / ENC_DATA);
}

// ═══════════════════════════════════
// MOTOR CONTROL
// ═══════════════════════════════════
void setMotor(int32_t cmd) {
  if (mode == MODE_SAFE) {
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, 0);
    digitalWrite(PIN_REN, LOW);
    digitalWrite(PIN_LEN, LOW);
    return;
  }

  cmd = constrain(cmd, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);
  if (cmd > 0) {
    digitalWrite(PIN_REN, HIGH);
    digitalWrite(PIN_LEN, HIGH);
    analogWrite(PIN_LPWM, 0);
    analogWrite(PIN_RPWM, (uint16_t)cmd);
  } else if (cmd < 0) {
    digitalWrite(PIN_REN, HIGH);
    digitalWrite(PIN_LEN, HIGH);
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, (uint16_t)(-cmd));
  } else {
    // Coast mode — disable H-bridge enables so motor isn't actively braking
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, 0);
    digitalWrite(PIN_REN, LOW);
    digitalWrite(PIN_LEN, LOW);
  }
}

uint16_t adcAvg(uint8_t pin) {
  uint32_t s = 0;
  for (uint8_t i = 0; i < CFG_ADC_OVERSAMPLE; i++) s += analogRead(pin);
  return s / CFG_ADC_OVERSAMPLE;
}

void printBoth(const char* msg) {
  MON_PORT.println(msg);
  Serial1.println(msg);
}

// ═══════════════════════════════════
// PID CONTROLLER
// ═══════════════════════════════════
int32_t runPID(int32_t current, int32_t target) {
  uint32_t now = micros();
  float dt = (pidLastUs == 0) ? 0.005f : (now - pidLastUs) * 1e-6f;
  pidLastUs = now;

  if (dt > 0.05f) dt = 0.05f;
  if (dt <= 0.0f) dt = 0.001f;

  int32_t err = target - current;

  // Wide deadband — prevents constant micro-corrections that cause audible noise
  if (abs(err) < PID_DEADBAND) {
    pidIntegral = 0;
    pidPrevErr  = 0;
    return 0;
  }

  float pTerm = Kp * err;
  pidIntegral += Ki * err * dt;
  pidIntegral  = constrain(pidIntegral, -CFG_PID_INTEGRAL_LIMIT, CFG_PID_INTEGRAL_LIMIT);
  float dTerm  = (dt > 0) ? Kd * (err - pidPrevErr) / dt : 0;
  pidPrevErr   = err;

  float output = pTerm + pidIntegral + dTerm;
  int32_t cmd = constrain((int32_t)output, -(int32_t)PWM_MAX, (int32_t)PWM_MAX);

  // Minimum duty cutoff — below this threshold the motor buzzes but can't move
  if (abs(cmd) < (int32_t)MIN_DUTY_THRESH) cmd = 0;

  return cmd;
}

void resetPID() {
  pidIntegral = 0;
  pidPrevErr  = 0;
  pidLastUs   = 0;
}

// ═══════════════════════════════════
// PASSIVE END-STOP LEARNING
// ═══════════════════════════════════
struct EndStopLearner {
  int32_t  lastAngle;
  uint32_t stallStart;
  bool     minLearned;
  bool     maxLearned;
} endstop = {-1, 0, false, false};

void endstopCheck(int32_t currentAngle, uint16_t currentDuty, int32_t pidError) {
  if (currentAngle < 0) return;

  uint16_t dutyThresh = (uint16_t)((uint32_t)PWM_MAX * CFG_ENDSTOP_DUTY_THRESH / 100);

  if (currentDuty < dutyThresh) {
    endstop.stallStart = 0;
    endstop.lastAngle = currentAngle;
    return;
  }

  if (endstop.lastAngle >= 0) {
    int32_t delta = abs(currentAngle - endstop.lastAngle);

    if (delta < CFG_ENDSTOP_STALL_THRESH) {
      if (endstop.stallStart == 0) {
        endstop.stallStart = millis();
      } else if ((millis() - endstop.stallStart) >= CFG_ENDSTOP_STALL_MS) {
        char buf[80];

        if (pidError > 0) {
          int32_t newMax = currentAngle - CFG_ENDSTOP_MARGIN;
          if (abs(newMax - CFG_ANGLE_MAX) <= CFG_ENDSTOP_SANITY) {
            if (newMax < ANGLE_MAX) {
              ANGLE_MAX = newMax;
              endstop.maxLearned = true;
              snprintf(buf, sizeof(buf), "[LEARN] Open stop: %ld.%02ld°",
                       (long)(ANGLE_MAX / 100), (long)(ANGLE_MAX % 100));
              printBoth(buf);
            }
          }
        } else if (pidError < 0) {
          int32_t newMin = currentAngle + CFG_ENDSTOP_MARGIN;
          if (abs(newMin - CFG_ANGLE_MIN) <= CFG_ENDSTOP_SANITY) {
            if (newMin > ANGLE_MIN) {
              ANGLE_MIN = newMin;
              endstop.minLearned = true;
              snprintf(buf, sizeof(buf), "[LEARN] Closed stop: %ld.%02ld°",
                       (long)(ANGLE_MIN / 100), (long)(ANGLE_MIN % 100));
              printBoth(buf);
            }
          }
        }

        endstop.stallStart = 0;
      }
    } else {
      endstop.stallStart = 0;
    }
  }

  endstop.lastAngle = currentAngle;
}

// ═══════════════════════════════════
// SAFE STATE
// ═══════════════════════════════════
void enterSafeState(const char* reason) {
  mode = MODE_SAFE;
  analogWrite(PIN_RPWM, 0);
  analogWrite(PIN_LPWM, 0);
  digitalWrite(PIN_REN, LOW);
  digitalWrite(PIN_LEN, LOW);
  digitalWrite(PIN_RELAY, LOW);
  pidIntegral = 0;
  duty        = 0;
  throttlePct = 0;
  targetAngle = ANGLE_MIN;
  settled     = false;
  settleStart = 0;

  char msg[64];
  snprintf(msg, sizeof(msg), "[SAFE] %s", reason);
  printBoth(msg);
}

// ═══════════════════════════════════
// SERIAL I/O
// ═══════════════════════════════════
bool readLine(Stream& port, char* buf, uint8_t& len, uint8_t maxLen) {
  while (port.available()) {
    char c = port.read();
    if (c == '\n' || c == '\r') {
      if (len == 0) continue;
      buf[len] = '\0';
      len = 0;
      return true;
    }
    if (len < maxLen - 1) buf[len++] = c;
  }
  return false;
}

void processCmd(const char* s) {
  if (strcasecmp(s, "f") == 0) {
    dir = +1;
  }
  else if (strcasecmp(s, "r") == 0) {
    dir = -1;
  }
  else if (strcasecmp(s, "s") == 0) {
    duty = 0;
    mode = MODE_MANUAL;
    resetPID();
  }
  else if (strcasecmp(s, "reset") == 0) {
    if (mode == MODE_SAFE) {
      safe_clear_faults();
      mode = MODE_MANUAL;
      duty = 0;
      digitalWrite(PIN_REN, HIGH);
      digitalWrite(PIN_LEN, HIGH);
      printBoth("Safe state cleared — mode=MAN");
    }
  }
  else if (strcasecmp(s, "on") == 0) {
    digitalWrite(PIN_RELAY, HIGH);
    printBoth("Relay ON");
  }
  else if (strcasecmp(s, "off") == 0) {
    digitalWrite(PIN_RELAY, LOW);
    printBoth("Relay OFF");
  }
  else if (strcasecmp(s, "diag") == 0) {
    safe_print_status(printBoth);
  }
  else if (strcasecmp(s, "clearfaults") == 0) {
    safe_clear_faults();
    printBoth("Faults cleared");
  }
  else if (strcasecmp(s, "config") == 0) {
    char buf[80];
    printBoth("═══════════════════════════════════");
    printBoth("  Active Configuration (config.h)");
    printBoth("═══════════════════════════════════");
    snprintf(buf, sizeof(buf), "PWM:       %luHz  %u-bit  (max=%u)", (unsigned long)CFG_PWM_FREQ_HZ, CFG_PWM_BITS, PWM_MAX);        printBoth(buf);
    snprintf(buf, sizeof(buf), "Throttle:  %.2f° – %.2f°  min:%s max:%s",
             ANGLE_MIN/100.0, ANGLE_MAX/100.0,
             endstop.minLearned ? "learned" : "initial",
             endstop.maxLearned ? "learned" : "initial");  printBoth(buf);
    snprintf(buf, sizeof(buf), "PID:       Kp=%.2f  Ki=%.2f  Kd=%.2f", (double)Kp, (double)Ki, (double)Kd);                         printBoth(buf);
    snprintf(buf, sizeof(buf), "Deadband:  %.2f°   MinDuty=%u", CFG_PID_DEADBAND/100.0, CFG_MIN_DUTY_THRESH);                       printBoth(buf);
    snprintf(buf, sizeof(buf), "Settle:    %lums window  %ldms timer", (unsigned long)CFG_SETTLE_TIME_MS, (long)CFG_SETTLE_WINDOW);  printBoth(buf);
    snprintf(buf, sizeof(buf), "OC Thresh: %u ADC counts  (debounce=%u)", CFG_OVERCURRENT_THRESH, CFG_OVERCURRENT_DEBOUNCE);        printBoth(buf);
    snprintf(buf, sizeof(buf), "Enc Tout:  %ums  (debounce=%u)", CFG_ENCODER_TIMEOUT_MS, CFG_ENCODER_DEBOUNCE);                     printBoth(buf);
    snprintf(buf, sizeof(buf), "Watchdog:  %lums", (unsigned long)(CFG_WATCHDOG_TIMEOUT_US/1000));                                   printBoth(buf);
    snprintf(buf, sizeof(buf), "Telemetry: %ums   ADC avg=%u samples", CFG_TELEMETRY_RATE_MS, CFG_ADC_OVERSAMPLE);                  printBoth(buf);
    snprintf(buf, sizeof(buf), "Serial:    %lu baud", (unsigned long)CFG_SERIAL_BAUD);                                              printBoth(buf);
    printBoth("═══════════════════════════════════");
  }
  else if (strcasecmp(s, "learn") == 0) {
    char buf[80];
    printBoth("───────────────────────────────────");
    printBoth("  End-Stop Learning Status");
    printBoth("───────────────────────────────────");
    snprintf(buf, sizeof(buf), "  Initial:  %ld.%02ld° – %ld.%02ld°",
             (long)(CFG_ANGLE_MIN / 100), (long)(CFG_ANGLE_MIN % 100),
             (long)(CFG_ANGLE_MAX / 100), (long)(CFG_ANGLE_MAX % 100));
    printBoth(buf);
    snprintf(buf, sizeof(buf), "  Current:  %ld.%02ld° – %ld.%02ld°",
             (long)(ANGLE_MIN / 100), (long)(ANGLE_MIN % 100),
             (long)(ANGLE_MAX / 100), (long)(ANGLE_MAX % 100));
    printBoth(buf);
    snprintf(buf, sizeof(buf), "  Min learned: %s    Max learned: %s",
             endstop.minLearned ? "YES" : "no",
             endstop.maxLearned ? "YES" : "no");
    printBoth(buf);
    snprintf(buf, sizeof(buf), "  Usable range: %ld.%02ld°",
             (long)((ANGLE_MAX - ANGLE_MIN) / 100),
             (long)((ANGLE_MAX - ANGLE_MIN) % 100));
    printBoth(buf);
    printBoth("───────────────────────────────────");
  }
  else if (strcasecmp(s, "relearn") == 0) {
    // Reset learned values back to initial estimates
    ANGLE_MIN = CFG_ANGLE_MIN;
    ANGLE_MAX = CFG_ANGLE_MAX;
    endstop.minLearned = false;
    endstop.maxLearned = false;
    endstop.stallStart = 0;
    endstop.lastAngle = -1;
    targetAngle = ANGLE_MIN;
    printBoth("End-stop learning reset to initial values");
  }
  else if (s[0] == 't' || s[0] == 'T') {
    if (mode == MODE_SAFE) { printBoth("[SAFE] Cmd rejected"); return; }
    long pct = atol(s + 1);
    pct = constrain(pct, 0L, 100L);
    throttlePct = (int16_t)pct;
    targetAngle = ANGLE_MIN + (int32_t)((int64_t)(ANGLE_MAX - ANGLE_MIN) * pct / 100);
    mode        = MODE_PID;
    settled     = false;
    settleStart = 0;
    resetPID();
  }
  else if (s[0] == 'd' || s[0] == 'D') {
    if (mode == MODE_SAFE) { printBoth("[SAFE] Cmd rejected"); return; }
    long v = atol(s + 1);
    duty = (uint16_t)constrain(v, 0L, (long)PWM_MAX);
    mode = MODE_MANUAL;
    resetPID();
  }
  else if (s[0] == 'p' || s[0] == 'P') {
    Kp = atof(s + 1);
    char buf[32];
    snprintf(buf, sizeof(buf), "Kp=%.2f", (double)Kp);
    printBoth(buf);
  }
  else if (s[0] == 'i' || s[0] == 'I') {
    Ki = atof(s + 1);
    char buf[32];
    snprintf(buf, sizeof(buf), "Ki=%.2f", (double)Ki);
    printBoth(buf);
  }
  else if (s[0] == 'k' || s[0] == 'K') {
    Kd = atof(s + 1);
    char buf[32];
    snprintf(buf, sizeof(buf), "Kd=%.2f", (double)Kd);
    printBoth(buf);
  }
}

// ═══════════════════════════════════
// SETUP
// ═══════════════════════════════════
void setup() {
  MON_PORT.begin(CFG_SERIAL_BAUD);
  Serial1.begin(CFG_SERIAL_BAUD);

  uint32_t t0 = millis();
  while ((millis() - t0) < CFG_USB_SETTLE_MS) { /* USB CDC enumerate */ }

  pinMode(PIN_REN, OUTPUT);
  pinMode(PIN_LEN, OUTPUT);
  digitalWrite(PIN_REN, HIGH);
  digitalWrite(PIN_LEN, HIGH);

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  pinMode(PIN_ENC, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC), encISR, CHANGE);

  analogWriteResolution(PWM_BITS);
  analogWriteFrequency(PWM_FREQ_HZ);
  analogReadResolution(12);
  setMotor(0);

  // Initialize safety module (also checks for watchdog resets)
  safe_init();

  printBoth("═══════════════════════════════════");
  printBoth("  Throttle ECU — PID Control");
  printBoth("═══════════════════════════════════");

  // Set initial target to closed position (will be refined by end-stop learning)
  targetAngle = ANGLE_MIN;

  if (safe_was_reset_by_watchdog()) {
    printBoth("[WARN] Recovered from watchdog reset!");
  }

  printBoth("Commands: t0-t100, d0-d4095, f, r, s");
  printBoth("         p## i## k##, on, off, reset");
  printBoth("         diag, clearfaults, config");
  printBoth("         learn, relearn");
}

// ═══════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════
void loop() {
  if (readLine(MON_PORT, usbBuf,  usbLen,  sizeof(usbBuf)))  processCmd(usbBuf);
  if (readLine(Serial1,  uartBuf, uartLen, sizeof(uartBuf))) processCmd(uartBuf);

  int32_t  angle = getAngle();
  uint32_t now   = millis();

  // Read currents (needed for safety checks and telemetry)
  uint16_t ris = adcAvg(PIN_RIS);
  uint16_t lis = adcAvg(PIN_LIS);

  // ── Safety checks ──────────────────
  safe_check_encoder(angle >= 0, now);
  safe_check_current(ris, lis, now);

  // If safety module has tripped, enter safe state with the actual reason
  if (g_safety.safe_state_active && mode != MODE_SAFE) {
    enterSafeState(g_safety.last_reason);
  }

  // Periodic safety tick (watchdog kick + recovery polling)
  if ((now - lastSafeTick) >= 100) {
    lastSafeTick = now;
    safe_tick(now);
  }

  // ── Motor control ──────────────────
  switch (mode) {
    case MODE_PID:
      if (angle >= 0) {
        // Clamp target to current learned range
        int32_t clampedTarget = constrain(targetAngle, ANGLE_MIN, ANGLE_MAX);
        int32_t posErr = abs(clampedTarget - angle);
        int32_t pidErr = clampedTarget - angle;  // signed error for end-stop learning

        if (posErr < SETTLE_WINDOW) {
          if (!settled) {
            if (settleStart == 0) {
              settleStart = now;
            } else if ((now - settleStart) >= SETTLE_TIME_MS) {
              settled = true;
            }
          }
        } else {
          settled     = false;
          settleStart = 0;
        }

        int32_t pidCmd = settled ? 0 : runPID(angle, clampedTarget);
        setMotor(pidCmd);

        // Passive end-stop learning: check if stalled at a mechanical limit
        endstopCheck(angle, (uint16_t)abs(pidCmd), pidErr);
      } else {
        setMotor(0);
      }
      break;

    case MODE_MANUAL:
      setMotor(duty == 0 ? 0 : (dir > 0 ? (int32_t)duty : -(int32_t)duty));
      break;

    case MODE_SAFE:
      setMotor(0);
      break;
  }

  // ── Telemetry (200ms) ──────────────
  if ((now - lastPrint) >= CFG_TELEMETRY_RATE_MS) {
    lastPrint = now;

    int16_t actPct = -1;
    if (angle >= 0) {
      actPct = (int16_t)constrain(
        (int32_t)((int64_t)(angle - ANGLE_MIN) * 100 / (ANGLE_MAX - ANGLE_MIN)),
        0L, 100L
      );
    }

    uint8_t errFlags = safe_get_fault_flags();
    static const char* modeStr[] = {"MAN", "PID", "SAFE"};
    char line[128];

    if (angle >= 0) {
      snprintf(line, sizeof(line),
        "mode=%s dir=%s duty=%u RIS=%u LIS=%u pos=%ld.%02ld thr=%d%% tgt=%d%% err=0x%02X",
        modeStr[mode], dir > 0 ? "F" : "R", duty, ris, lis,
        (long)(angle / 100), (long)(angle % 100),
        (int)actPct, (int)throttlePct, errFlags);
    } else {
      snprintf(line, sizeof(line),
        "mode=%s dir=%s duty=%u RIS=%u LIS=%u pos=--- thr=---%% tgt=%d%% err=0x%02X",
        modeStr[mode], dir > 0 ? "F" : "R", duty, ris, lis,
        (int)throttlePct, errFlags);
    }
    printBoth(line);
  }
}
