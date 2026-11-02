// oled_test: bench check for the logger stack.
// Feather M4 CAN + Adalogger FeatherWing (PCF8523 RTC, microSD on D10)
// + 128x64 OLED FeatherWing (SH1107, I2C 0x3C).
//
// The OLED shows: RTC time, SD card status, I2C devices found, and A0-A5
// in volts, refreshed 5x a second. The same report goes to USB serial.
//
// Build: arduino-cli compile --fqbn adafruit:samd:adafruit_feather_m4_can firmware/oled_test

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <RTClib.h>
#include <SdFat.h>      // "SdFat - Adafruit Fork"

static const uint8_t OLED_ADDR = 0x3C;
static const uint8_t RTC_ADDR  = 0x68;
static const uint8_t SD_CS     = 10;

static Adafruit_SH1107 oled(64, 128, &Wire);
static RTC_PCF8523 rtc;
static SdFs sd;

static bool haveOled = false;
static bool haveRtc  = false;
static bool haveSd   = false;
static uint8_t i2cFound[8];
static uint8_t i2cCount = 0;

static void scanI2c() {
  i2cCount = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0 && i2cCount < sizeof(i2cFound)) i2cFound[i2cCount++] = a;
  }
}

static bool i2cHas(uint8_t addr) {
  for (uint8_t i = 0; i < i2cCount; i++) if (i2cFound[i] == addr) return true;
  return false;
}

static float readVolts(uint8_t pin) {
  return analogRead(pin) * (3.3f / 4095.0f);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  Wire.begin();
  Wire.setClock(400000);
  delay(250);                          // let the OLED power up

  scanI2c();
  if (i2cHas(OLED_ADDR) && oled.begin(OLED_ADDR, true)) {
    haveOled = true;
    oled.setRotation(1);               // landscape, 128 wide x 64 tall
    oled.setTextSize(1);
    oled.setTextColor(SH110X_WHITE);
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("EPS logger");
    oled.println("bench test");
    oled.display();
  }
  if (i2cHas(RTC_ADDR) && rtc.begin()) {
    haveRtc = true;
    if (!rtc.initialized() || rtc.lostPower()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));   // set to compile time
    }
    rtc.start();
  }
  haveSd = sd.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(12)));
  delay(800);
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < 200) return;
  last = millis();

  char timeStr[20] = "no RTC";
  if (haveRtc) {
    DateTime t = rtc.now();
    snprintf(timeStr, sizeof(timeStr), "%02u:%02u:%02u %02u/%02u",
             t.hour(), t.minute(), t.second(), t.day(), t.month());
  }
  char sdStr[22] = "SD: none";
  if (haveSd) {
    uint32_t gb10 = (uint32_t)(sd.card()->sectorCount() / (2097152UL / 10));   // 512 B sectors
    snprintf(sdStr, sizeof(sdStr), "SD: %s %lu.%luGB",
             sd.fatType() == FAT_TYPE_EXFAT ? "exFAT" : "FAT", gb10 / 10, gb10 % 10);
  }
  char i2cStr[22] = "I2C:";
  for (uint8_t i = 0; i < i2cCount && strlen(i2cStr) < 18; i++) {
    char b[5];
    snprintf(b, sizeof(b), " %02X", i2cFound[i]);
    strcat(i2cStr, b);
  }
  const uint8_t pins[6] = {A0, A1, A2, A3, A4, A5};
  float v[6];
  for (uint8_t i = 0; i < 6; i++) v[i] = readVolts(pins[i]);

  if (haveOled) {
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println(timeStr);
    oled.println(sdStr);
    oled.println(i2cStr);
    for (uint8_t i = 0; i < 6; i += 2) {
      oled.printf("A%u %.2f  A%u %.2f\n", i, v[i], i + 1, v[i + 1]);
    }
    oled.printf("up %lus", millis() / 1000);
    oled.display();
  }

  Serial.printf("%s | %s | %s | OLED %s |", timeStr, sdStr, i2cStr, haveOled ? "ok" : "missing");
  for (uint8_t i = 0; i < 6; i++) Serial.printf(" A%u=%.3f", i, v[i]);
  Serial.println();
}
