// eps_bench_nep: wake the Insight EPS on the bench with a fake engine-speed
// signal (NEP), read the Toyota steering angle sensor over CAN, and put out a
// sync pulse the Hantek 1008B records, so the PC can line both logs up.
//
// Outputs on USB serial (115200), one event per line:
//   # ...                      comment / status
//   N,<us>,<rpm>               NEP command changed (0 = off)
//   A,<us>,<deg>,<dps>,<zeroed_deg>   SAS frame (CAN 0x25), timestamped in the CAN interrupt
//   S,<us>,<level>,<n>         sync edge on D13, timestamped the instant the pin changed
// <us> is micros(): 32-bit, wraps every 71.6 min. tools/feather_log.py unwraps it.
//
// Serial commands:  r900  set rpm (0 = off, else 500..3000)   z  zero SAS   ?  status
// OLED buttons:     A (D9) NEP on/off   B (D6) next rpm preset   C (D5) zero SAS
//
// Wiring (breadboard): see README.md next to this file.
//   D4  -> 10k -> NPN base (2N3904 / 2N2222; emitter to GND). Collector = NEP node.
//          NEP node -> 10k -> +12 V (the EPS supply), NEP node -> EPS B14.
//   D13 -> Hantek channel (sync). GND -> T/S GND and the Hantek probe grounds.
//   CAN-H / CAN-L -> SAS, 120 ohm at each end of the bus.
//
// NEP is 1.5 pulses per engine rev (EPS_PINOUT.md): 900 rpm = 22.5 Hz.
// Honda DTC 22 / lamp rules (EPS_PINOUT.md): never send speed without rpm (VSP is
// not driven here, so the EPS sees 0 mph), and don't hold >= 2000 rpm at 0 mph for
// 6 min. This sketch drops back to 900 rpm after 5 min at >= 2000.
//
// Build: arduino-cli compile --fqbn adafruit:samd:adafruit_feather_m4_can eps_bench_nep

#include <CANSAME5x.h>       // "Adafruit CAN"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

static const uint8_t  NEP_PIN  = 4;      // PA14, header only (not the CAN boost enable)
static const uint8_t  SYNC_PIN = 13;     // red LED too, so you can see it blink
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

// ---------------------------------------------------------------- CAN ring buffer
struct CanFrame { uint32_t us; uint8_t d[8]; uint8_t len; };
static const uint8_t RING = 64;                 // power of two
static volatile CanFrame ring[RING];
static volatile uint8_t ringHead = 0, ringTail = 0;
static volatile uint32_t ringDrops = 0, busFrames = 0;

static void onCan(int size) {                   // runs in the CAN interrupt
  const uint32_t us = micros();
  busFrames++;
  if (CAN.packetExtended() || CAN.packetRtr() || CAN.packetId() != SAS_CAN_ID || size < 6) {
    while (CAN.available()) CAN.read();
    return;
  }
  const uint8_t next = (ringHead + 1) & (RING - 1);
  if (next == ringTail) { ringDrops++; while (CAN.available()) CAN.read(); return; }
  volatile CanFrame &f = ring[ringHead];
  f.us = us;
  f.len = 0;
  while (CAN.available() && f.len < 8) f.d[f.len++] = CAN.read();
  while (CAN.available()) CAN.read();
  ringHead = next;
}

// ---------------------------------------------------------------- SAS decode (same as feather_sas_oled)
static int16_t signExtend(uint16_t v, uint8_t bits) {
  const uint16_t sign = 1u << (bits - 1);
  return (int16_t)((v ^ sign) - sign);
}

static float sasDeg = 0, zeroDeg = 0;
static int16_t sasRate = 0;
static uint32_t sasLastMs = 0, sasCount = 0;

static void drainCan() {
  while (ringTail != ringHead) {
    CanFrame f;
    noInterrupts();
    f.us = ring[ringTail].us;
    f.len = ring[ringTail].len;
    for (uint8_t i = 0; i < 8; i++) f.d[i] = ring[ringTail].d[i];
    ringTail = (ringTail + 1) & (RING - 1);
    interrupts();
    const float angle = signExtend(((f.d[0] & 0x0F) << 8) | f.d[1], 12) * 1.5f;
    const float frac  = signExtend((f.d[4] & 0xF0) >> 4, 4) * 0.1f;
    sasRate = signExtend(((f.d[4] & 0x0F) << 8) | f.d[5], 12);
    sasDeg = angle + frac;
    sasLastMs = millis();
    sasCount++;
    Serial.printf("A,%lu,%.1f,%d,%.1f\n", f.us, sasDeg, sasRate, sasDeg - zeroDeg);
  }
}

// ---------------------------------------------------------------- NEP
static uint16_t rpm = 0, lastOnRpm = 900;
static uint32_t highSinceMs = 0;

static void setRpm(uint16_t r, const char *why) {
  if (r != 0) r = constrain(r, RPM_MIN, RPM_MAX);
  if (r == rpm) return;
  rpm = r;
  if (rpm) {
    lastOnRpm = rpm;
    tone(NEP_PIN, rpm * NEP_PULSES_PER_REV / 60.0f);   // 50 % square wave, timer interrupt
  } else {
    noTone(NEP_PIN);
    digitalWrite(NEP_PIN, LOW);                         // transistor off: line idles high
  }
  highSinceMs = rpm >= RPM_HIGH ? millis() : 0;
  Serial.printf("N,%lu,%u\n", micros(), rpm);
  Serial.printf("# rpm %u (%.2f Hz) %s\n", rpm, rpm * NEP_PULSES_PER_REV / 60.0f, why);
}

static void guardRpm() {
  if (highSinceMs && millis() - highSinceMs > RPM_HIGH_MAX_MS) {
    Serial.println("# >= 2000 rpm for 5 min at 0 mph: dropping to 900 to avoid the EPS lamp");
    setRpm(900, "(time guard)");
  }
}

// ---------------------------------------------------------------- sync pulse
// Pseudo-random 200..1000 ms intervals, so the merge script can't lock on the wrong edge.
static uint16_t lfsr = 0xACE1;
static uint32_t syncNextMs = 0, syncCount = 0;
static bool syncLevel = false;

static void runSync() {
  const uint32_t now = millis();
  if ((int32_t)(now - syncNextMs) < 0) return;
  syncLevel = !syncLevel;
  digitalWrite(SYNC_PIN, syncLevel);
  const uint32_t us = micros();                 // taken right after the edge
  syncCount++;
  Serial.printf("S,%lu,%d,%lu\n", us, syncLevel, syncCount);
  lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u);
  syncNextMs = now + 200 + (lfsr % 801);
}

// ---------------------------------------------------------------- UI
static void handleSerial() {
  static char buf[16];
  static uint8_t n = 0;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c != '\n' && c != '\r') { if (n < sizeof buf - 1) buf[n++] = c; continue; }
    buf[n] = 0;
    if (n == 0) continue;
    if (buf[0] == 'r') setRpm(atoi(buf + 1), "(serial)");
    else if (buf[0] == 'z') { zeroDeg = sasDeg; Serial.printf("# SAS zero %.1f\n", zeroDeg); }
    else if (buf[0] == '?') {
      Serial.printf("# rpm %u, SAS %s %.1f deg (zeroed %.1f), sas frames %lu, bus frames %lu, drops %lu, sync %lu\n",
                    rpm, millis() - sasLastMs < SAS_STALE_MS ? "ok" : "STALE", sasDeg, sasDeg - zeroDeg,
                    sasCount, busFrames, ringDrops, syncCount);
    } else Serial.println("# commands: r<rpm>  z  ?");
    n = 0;
  }
}

static void handleButtons() {
  static bool pa = true, pb = true, pc = true;
  static uint8_t preset = 0;
  const bool a = digitalRead(BTN_A), b = digitalRead(BTN_B), c = digitalRead(BTN_C);
  if (pa && !a) setRpm(rpm ? 0 : lastOnRpm, "(button A)");
  if (pb && !b) {
    preset = (preset + 1) % (sizeof RPM_PRESETS / sizeof RPM_PRESETS[0]);
    if (rpm) setRpm(RPM_PRESETS[preset], "(button B)");
    else lastOnRpm = RPM_PRESETS[preset];
  }
  if (pc && !c) { zeroDeg = sasDeg; Serial.printf("# SAS zero %.1f\n", zeroDeg); }
  pa = a; pb = b; pc = c;
}

static void drawOled() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.printf("NEP %s", rpm ? "ON" : "off");
  oled.setCursor(70, 0);
  oled.printf("sync %lu", syncCount % 1000);
  oled.setTextSize(2);
  oled.setCursor(0, 12);
  oled.printf("%4u rpm", rpm ? rpm : lastOnRpm);
  oled.setCursor(0, 32);
  if (millis() - sasLastMs < SAS_STALE_MS) oled.printf("%+6.1f", sasDeg - zeroDeg);
  else oled.print(" SAS --");
  oled.setTextSize(1);
  oled.setCursor(0, 54);
  oled.print("A on/off B rpm C zero");
  oled.display();
}

void setup() {
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

  Serial.println("# eps_bench_nep: NEP on D4, sync on D13, SAS 0x25 @ 500k");
  Serial.println("# N,us,rpm | A,us,deg,dps,zeroed_deg | S,us,level,n");
  if (!canOk) Serial.println("# CAN start failed");
  Serial.printf("N,%lu,0\n", micros());
}

void loop() {
  static uint32_t lastDraw = 0;
  drainCan();
  runSync();
  handleSerial();
  handleButtons();
  guardRpm();
  if (millis() - lastDraw >= 200) {     // 5 fps: the I2C transfer blocks ~25 ms
    lastDraw = millis();
    drawOled();
  }
}
