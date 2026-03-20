// ═══════════════════════════════════════════════════════════════════════
// Throttle ECU — PID Position Control (NO CAN)
// Target: STM32F411CE (WeAct BlackPill)
// ═══════════════════════════════════════════════════════════════════════

#include <Arduino.h>

// ═══════════════════════════════════
// SERIAL PORT SELECTION (STM32)
// - On STM32duino, the "Serial Monitor" port might be USB CDC or a UART,
//   depending on Tools settings.
// - Prefer the core's monitor/USB macros when available.
// ═══════════════════════════════════
#if defined(SERIAL_PORT_MONITOR)
  #define MON_PORT SERIAL_PORT_MONITOR
#else
  #define MON_PORT Serial
#endif

#if defined(SERIAL_PORT_USBVIRTUAL)
  #define USB_PORT SERIAL_PORT_USBVIRTUAL
#else
  #define USB_PORT MON_PORT
#endif

// ═══════════════════════════════════
// PIN ASSIGNMENTS
// ═══════════════════════════════════
const uint8_t PIN_RPWM  = PA9;
const uint8_t PIN_LPWM  = PA8;
const uint8_t PIN_REN   = PB0;
const uint8_t PIN_LEN   = PB1;
const uint8_t PIN_RIS   = PA1;
const uint8_t PIN_LIS   = PA0;
const uint8_t PIN_RELAY = PB10;
const uint8_t PIN_ENC   = PB4;   // AS5048A PWM

// ═══════════════════════════════════
// MOTOR PWM CONFIG
// ═══════════════════════════════════
const uint32_t PWM_FREQ_HZ = 20000;
const uint8_t  PWM_BITS    = 12;
const uint16_t PWM_MAX     = (1u << PWM_BITS) - 1;  // 4095

// ═══════════════════════════════════
// AS5048A ENCODER
// ═══════════════════════════════════
const uint16_t ENC_OFFSET = 16;
const uint16_t ENC_DATA   = 4095;
const uint16_t ENC_PERIOD_CLKS = 4119;

// ═══════════════════════════════════
// THROTTLE CALIBRATION (0.01° units)
// ═══════════════════════════════════
const int32_t ANGLE_MIN = 7032;   // 70.32° = closed (0%)
const int32_t ANGLE_MAX = 17940;  // 179.40° = wide open (100%)

// ═══════════════════════════════════
// PID GAINS
// ═══════════════════════════════════
float Kp = 12.0f;
float Ki = 0.3f;
float Kd = 1.5f;

// ═══════════════════════════════════
// NOISE REDUCTION CONFIG
// ═══════════════════════════════════
const int32_t PID_DEADBAND    = 50;    // 0.50° — wider deadband stops hunting noise
const uint16_t MIN_DUTY_THRESH = 50;   // Below this duty, motor vibrates but won't move
const uint32_t SETTLE_TIME_MS  = 500;  // After reaching target, disable motor after this delay
const int32_t  SETTLE_WINDOW   = 100;  // 1.00° — position must stay within this to settle

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
float   pidIntegral = 0.0f;
int32_t pidPrevErr  = 0;
uint32_t pidLastUs  = 0;

// Settle state — tracks when motor has reached target and can coast
bool     settled      = false;
uint32_t settleStart  = 0;

// Encoder
volatile uint32_t encRise   = 0;
volatile uint32_t encHigh   = 0;
volatile uint32_t encPeriod = 0;
volatile bool     encValid  = false;

// Serial buffers
static char    usbBuf[32],  uartBuf[32];
static uint8_t usbLen = 0,  uartLen = 0;

// Telemetry
uint32_t lastPrint = 0;

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
  for (uint8_t i = 0; i < 16; i++) s += analogRead(pin);
  return s >> 4;
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
  pidIntegral  = constrain(pidIntegral, -2000.0f, 2000.0f);
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
// SAFE STATE
// ═══════════════════════════════════
void enterSafeState() {
  mode = MODE_SAFE;
  analogWrite(PIN_RPWM, 0);
  analogWrite(PIN_LPWM, 0);
  digitalWrite(PIN_REN, LOW);
  digitalWrite(PIN_LEN, LOW);
  digitalWrite(PIN_RELAY, LOW);
  pidIntegral = 0;
  duty = 0;
  throttlePct = 0;
  targetAngle = ANGLE_MIN;
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
      mode = MODE_MANUAL;
      digitalWrite(PIN_REN, HIGH);
      digitalWrite(PIN_LEN, HIGH);
      printBoth("Safe state cleared");
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
  else if (s[0] == 't' || s[0] == 'T') {
    long pct = atol(s + 1);
    pct = constrain(pct, 0L, 100L);
    throttlePct = (int16_t)pct;
    targetAngle = ANGLE_MIN + (int32_t)((int64_t)(ANGLE_MAX - ANGLE_MIN) * pct / 100);
    mode = MODE_PID;
    settled = false;
    settleStart = 0;
    resetPID();
  }
  else if (s[0] == 'd' || s[0] == 'D') {
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
  // Start both ports:
  // - MON_PORT: whatever the core uses for Serial Monitor (often USB CDC if enabled)
  // - Serial1 : TTL UART (pins depend on variant; commonly PA9/PA10 or PB6/PB7)
  MON_PORT.begin(115200);
  Serial1.begin(115200);

  // Don't block forever waiting for a host connection.
  // Some STM32 core/USB configurations never make `while(!Serial)` true.
  uint32_t t0 = millis();
  while ((millis() - t0) < 250) { /* brief settle */ }

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

  printBoth("═══════════════════════════════════");
  printBoth("  Throttle ECU — PID Control");
  printBoth("═══════════════════════════════════");
  printBoth("Commands: t0-t100, d0-d4095, f, r, s");
  printBoth("         p## i## k##, on, off, reset");
}

// ═══════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════
void loop() {
  if (readLine(MON_PORT,  usbBuf,  usbLen,  sizeof(usbBuf)))  processCmd(usbBuf);
  if (readLine(Serial1, uartBuf, uartLen, sizeof(uartBuf))) processCmd(uartBuf);

  int32_t angle = getAngle();

  switch (mode) {
    case MODE_PID:
      if (angle >= 0) {
        int32_t err = abs(targetAngle - angle);

        // Settle detection: if within window, start/continue settle timer
        if (err < SETTLE_WINDOW) {
          if (!settled) {
            if (settleStart == 0) {
              settleStart = millis();
            } else if ((millis() - settleStart) >= SETTLE_TIME_MS) {
              settled = true;  // Position held long enough — coast
            }
          }
        } else {
          // Position drifted out — re-engage PID
          settled = false;
          settleStart = 0;
        }

        if (settled) {
          setMotor(0);  // Coast — motor off, no buzzing
        } else {
          int32_t cmd = runPID(angle, targetAngle);
          setMotor(cmd);
        }
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

  uint32_t now = millis();
  if (now - lastPrint >= 200) {
    lastPrint = now;
    uint16_t ris = adcAvg(PIN_RIS);
    uint16_t lis = adcAvg(PIN_LIS);

    int16_t actPct = -1;
    if (angle >= 0) {
      actPct = (int16_t)constrain(
        (int32_t)((int64_t)(angle - ANGLE_MIN) * 100 / (ANGLE_MAX - ANGLE_MIN)),
        0L, 100L
      );
    }

    static const char* modeStr[] = {"MAN", "PID", "SAFE"};
    char line[120];
    if (angle >= 0) {
      snprintf(line, sizeof(line),
        "mode=%s dir=%s duty=%u RIS=%u LIS=%u pos=%ld.%02ld thr=%d%% tgt=%d%%",
        modeStr[mode], dir > 0 ? "F" : "R", duty, ris, lis,
        (long)(angle / 100), (long)(angle % 100),
        (int)actPct, (int)throttlePct);
    } else {
      snprintf(line, sizeof(line),
        "mode=%s dir=%s duty=%u RIS=%u LIS=%u pos=--- thr=---%% tgt=%d%%",
        modeStr[mode], dir > 0 ? "F" : "R", duty, ris, lis,
        (int)throttlePct);
    }
    printBoth(line);
  }
}
