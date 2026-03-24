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

// Throttle range — hardcoded from config.h
const int32_t  ANGLE_MIN = CFG_ANGLE_MIN;
const int32_t  ANGLE_MAX = CFG_ANGLE_MAX;

// Usable range (5-95%) — avoids noisy encoder edges near mechanical stops
const int32_t  ANGLE_RANGE  = CFG_ANGLE_MAX - CFG_ANGLE_MIN;
const int32_t  USABLE_MIN   = CFG_ANGLE_MIN + (ANGLE_RANGE * 5 / 100);   // 5% from closed
const int32_t  USABLE_MAX   = CFG_ANGLE_MIN + (ANGLE_RANGE * 95 / 100);  // 95% from closed

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

// Encoder rolling average (kills random spikes)
#define ENC_AVG_SIZE  8
int32_t encBuf[ENC_AVG_SIZE];
uint8_t encBufIdx = 0;
uint8_t encBufCnt = 0;  // How many valid samples we have

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

// Get raw angle from encoder (no averaging)
int32_t getAngleRaw() {
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

// Get averaged angle — rolling average of last ENC_AVG_SIZE readings
// Rejects spikes: if a new reading is more than 5° (500 centideg) from
// the current average, it's ignored (likely noise)
int32_t getAngle() {
  int32_t raw = getAngleRaw();
  if (raw < 0) return -1;

  // Spike rejection: if we have enough samples, reject outliers
  if (encBufCnt >= 3) {
    int32_t sum = 0;
    for (uint8_t i = 0; i < encBufCnt; i++) sum += encBuf[i];
    int32_t avg = sum / encBufCnt;

    if (abs(raw - avg) > CFG_ENC_SPIKE_THRESH) {
      return avg;  // Reject spike, return current average instead
    }
  }

  // Add to rolling buffer
  encBuf[encBufIdx] = raw;
  encBufIdx = (encBufIdx + 1) % ENC_AVG_SIZE;
  if (encBufCnt < ENC_AVG_SIZE) encBufCnt++;

  // Compute average
  int32_t sum = 0;
  for (uint8_t i = 0; i < encBufCnt; i++) sum += encBuf[i];
  return sum / encBufCnt;
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
// THROTTLE MAPPING (simplified)
// ═══════════════════════════════════
// Uses 5-95% of the physical range to avoid noisy encoder edges.
// 0% throttle = USABLE_MIN, 100% throttle = USABLE_MAX.
// PID targets are clamped to this range automatically.
int32_t throttleToAngle(int16_t pct) {
  pct = constrain(pct, (int16_t)0, (int16_t)100);
  return USABLE_MIN + (int32_t)((int64_t)(USABLE_MAX - USABLE_MIN) * pct / 100);
}

int16_t angleToThrottle(int32_t angle) {
  if (angle < USABLE_MIN) return 0;
  if (angle > USABLE_MAX) return 100;
  return (int16_t)((int64_t)(angle - USABLE_MIN) * 100 / (USABLE_MAX - USABLE_MIN));
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
    // Config dump — prints to serial log (visible in GUI serial log panel)
    char buf[80];
    printBoth("═══════════════════════════════════");
    printBoth("  Active Configuration (config.h)");
    printBoth("═══════════════════════════════════");
    snprintf(buf, sizeof(buf), "PWM:       %luHz  %u-bit  (max=%u)",
             (unsigned long)CFG_PWM_FREQ_HZ, CFG_PWM_BITS, PWM_MAX);
    printBoth(buf);
    snprintf(buf, sizeof(buf), "Full range:  %.2f° – %.2f°",
             ANGLE_MIN/100.0, ANGLE_MAX/100.0);
    printBoth(buf);
    snprintf(buf, sizeof(buf), "Usable (5-95%%): %.2f° – %.2f°",
             USABLE_MIN/100.0, USABLE_MAX/100.0);
    printBoth(buf);
    snprintf(buf, sizeof(buf), "PID:       Kp=%.2f  Ki=%.2f  Kd=%.2f",
             (double)Kp, (double)Ki, (double)Kd);
    printBoth(buf);
    snprintf(buf, sizeof(buf), "Deadband:  %.2f°   MinDuty=%u",
             CFG_PID_DEADBAND/100.0, CFG_MIN_DUTY_THRESH);
    printBoth(buf);
    snprintf(buf, sizeof(buf), "Settle:    %lums window  %ldms timer",
             (unsigned long)CFG_SETTLE_TIME_MS, (long)CFG_SETTLE_WINDOW);
    printBoth(buf);
    snprintf(buf, sizeof(buf), "OC Thresh: %u ADC counts  (debounce=%u)",
             CFG_OVERCURRENT_THRESH, CFG_OVERCURRENT_DEBOUNCE);
    printBoth(buf);
    snprintf(buf, sizeof(buf), "Enc Tout:  %ums  (debounce=%u)  AvgSize=%u  SpikeThresh=%u",
             CFG_ENCODER_TIMEOUT_MS, CFG_ENCODER_DEBOUNCE, ENC_AVG_SIZE, CFG_ENC_SPIKE_THRESH);
    printBoth(buf);
    snprintf(buf, sizeof(buf), "Watchdog:  %lums",
             (unsigned long)(CFG_WATCHDOG_TIMEOUT_US/1000));
    printBoth(buf);
    snprintf(buf, sizeof(buf), "Telemetry: %ums   ADC avg=%u samples",
             CFG_TELEMETRY_RATE_MS, CFG_ADC_OVERSAMPLE);
    printBoth(buf);
    snprintf(buf, sizeof(buf), "Serial:    %lu baud",
             (unsigned long)CFG_SERIAL_BAUD);
    printBoth(buf);
    printBoth("═══════════════════════════════════");
  }
  else if (s[0] == 't' || s[0] == 'T') {
    if (mode == MODE_SAFE) { printBoth("[SAFE] Cmd rejected"); return; }
    long pct = atol(s + 1);
    pct = constrain(pct, 0L, 100L);
    throttlePct = (int16_t)pct;
    targetAngle = throttleToAngle(throttlePct);
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

  // Set initial target to closed position (usable 5% point)
  targetAngle = USABLE_MIN;

  char bootBuf[80];
  snprintf(bootBuf, sizeof(bootBuf), "Usable range: %.2f° – %.2f° (5-95%%)",
           USABLE_MIN / 100.0, USABLE_MAX / 100.0);
  printBoth(bootBuf);

  if (safe_was_reset_by_watchdog()) {
    printBoth("[WARN] Recovered from watchdog reset!");
  }

  printBoth("Commands: t0-t100, d0-d4095, f, r, s");
  printBoth("         p## i## k##, on, off, reset");
  printBoth("         diag, clearfaults, config");
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
        // Clamp target to usable range (5-95%)
        int32_t clampedTarget = constrain(targetAngle, USABLE_MIN, USABLE_MAX);
        int32_t posErr = abs(clampedTarget - angle);

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
      actPct = angleToThrottle(angle);
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
