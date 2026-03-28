/*
 * esp32_can_test.ino — ESP32 TWAI (CAN) TX/RX test
 *
 * Target hardware
 * ---------------
 * • ESP32 dev board, USB-C, CH340C USB–UART. Use the CH340 COM port at 115200 baud for
 *   Serial Monitor (this is unrelated to CAN bitrate).
 * • WeAct ISOCANFD Module V1 — CA-IS2062A, 2.5 kV isolated CAN/CAN-FD transceiver,
 *   logic I/O 2.5–5.5 V (3.3 V ESP32 is OK). The transceiver can do CAN FD; ESP32 TWAI
 *   here is classic CAN only (e.g. 500 kbps), which is fine for your throttle ECU bus.
 *
 * Wiring (follow WeAct silkscreen; names below are typical)
 * --------------------------------------------------------
 * Logic side → ESP32:
 *   3V3 / VCC → 3.3V
 *   GND       → GND
 *   Serial to/from ESP32 TWAI (Espressif-style pin choice):
 *     GPIO21 = TWAI TX (MCU out) → transceiver pin that accepts MCU transmit
 *     GPIO22 = TWAI RX (MCU in)  ← transceiver pin that drives MCU receive
 *   Some boards label these TXD/RXD from the transceiver’s point of view, which is the
 *   opposite of the MCU — if you get no RX/TX, swap GPIO21 and GPIO22 once.
 *
 * Isolated / bus side (per module datasheet):
 *   VBUS → 5 V (bus-side supply input where required)
 *   GNDB → isolated ground reference for CANH/CANL (not the same net as ESP32 GND)
 *   CANH / CANL → bus.
 *   IMPORTANT: With only this module on the bus, put ~120 Ω across CANH–CANL at the
 *   module. Floating lines cause bit errors → error passive → bus-off → TX returns 0x103
 *   (ESP_ERR_INVALID_STATE) until recovery finishes.
 *
 * TWAI mode vs F280049 on the same bus:
 *   NO_ACK helps solo bench TX but (on ESP-IDF) the node does not ACK other
 *   transmitters — the LAUNCHXL then gets ACK errors, bus-off, and RX/sniffer
 *   look dead. Use CAN_USE_NO_ACK_MODE 0 (NORMAL) with the C2000. Solo bench
 *   only: set CAN_USE_NO_ACK_MODE 1 and add ~120 Ω across CANH–CANL.
 *
 * 500 kbps; sniffer/TI/ESP32 must share one CANH/CANL pair + ground reference.
 */

#include <Arduino.h>
#include <string.h>
#include "driver/twai.h"
#include "esp_err.h"

static const char *twai_state_name(twai_state_t s)
{
    switch (s) {
    case TWAI_STATE_STOPPED:
        return "STOPPED";
    case TWAI_STATE_RUNNING:
        return "RUNNING";
    case TWAI_STATE_BUS_OFF:
        return "BUS_OFF";
    case TWAI_STATE_RECOVERING:
        return "RECOVERING";
    default:
        return "?";
    }
}

#define CAN_TX_PIN GPIO_NUM_21
#define CAN_RX_PIN GPIO_NUM_22

/* Must match all other devices on the bus (C2000 project uses 500000). */
#define CAN_BITRATE_KBPS 500

/* 0 = NORMAL — required with F280049 (or any peer) so you ACK their frames.
 * 1 = NO_ACK — solo bench only (see file header). */
#define CAN_USE_NO_ACK_MODE 0

#define TX_PERIOD_MS 500U

#define TX_CAN_ID 0x200U

static uint32_t txCount  = 0;
static uint32_t rxCount  = 0;
static uint16_t txCtr    = 0;
static uint32_t lastTxMs     = 0;
static uint32_t lastStateLog = 0;
static bool     driverOk     = false;

static bool installTwai(void)
{
    twai_mode_t mode = CAN_USE_NO_ACK_MODE ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL;

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, mode);

#if CAN_BITRATE_KBPS == 500
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
#elif CAN_BITRATE_KBPS == 250
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
#elif CAN_BITRATE_KBPS == 1000
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
#else
#error Set CAN_BITRATE_KBPS to 250, 500, or 1000
#endif

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[CAN] ERROR: twai_driver_install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[CAN] ERROR: twai_start failed");
        twai_driver_uninstall();
        return false;
    }

    /* Older Arduino-ESP32 cores lack TWAI_ALERT_RECOVERY_COMPLETE; RUNNING is polled before TX. */
    uint32_t alerts = TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS |
                      TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_DATA |
                      TWAI_ALERT_RX_QUEUE_FULL;
    if (twai_reconfigure_alerts(alerts, NULL) != ESP_OK) {
        Serial.println("[CAN] WARN: twai_reconfigure_alerts failed (continuing)");
    }

    return true;
}

static void handleAlerts(void)
{
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) != ESP_OK) {
        return;
    }

    if (alerts & TWAI_ALERT_BUS_OFF) {
        Serial.println("[CAN] Bus-off — twai_initiate_recovery() (no TX until RUNNING)");
        twai_initiate_recovery();
    }
    if (alerts & TWAI_ALERT_ERR_PASS) {
        Serial.println("[CAN] Alert: error passive (check 120R on CANH-CANL, wiring)");
    }
    if (alerts & TWAI_ALERT_BUS_ERROR) {
        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK) {
            Serial.printf("[CAN] Bus error (count=%lu)\n",
                          (unsigned long)st.bus_error_count);
        }
    }
    if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
        Serial.println("[CAN] RX queue full — draining");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(300);

    Serial.printf("[CAN] TWAI init: %u kbps, mode=%s\n",
                  (unsigned)CAN_BITRATE_KBPS,
                  CAN_USE_NO_ACK_MODE ? "NO_ACK (no peer ACK required)" : "NORMAL");

    driverOk = installTwai();
    if (!driverOk) {
        return;
    }

    lastTxMs = millis();
    Serial.printf("[CAN] Ready — TX id=0x%03X every %u ms, RX any\n",
                  (unsigned)TX_CAN_ID, (unsigned)TX_PERIOD_MS);
    Serial.println("[CAN] Tip: single-node bench needs ~120 Ohm between CANH and CANL.");
}

void loop()
{
    if (!driverOk) {
        delay(1000);
        return;
    }

    handleAlerts();

    twai_message_t rx;
    while (twai_receive(&rx, 0) == ESP_OK) {
        rxCount++;
        Serial.printf("[RX] #%lu  id=0x%03X  dlc=%u  %02X %02X %02X %02X %02X %02X %02X %02X\n",
                      (unsigned long)rxCount,
                      (unsigned)rx.identifier,
                      (unsigned)rx.data_length_code,
                      rx.data[0], rx.data[1], rx.data[2], rx.data[3],
                      rx.data[4], rx.data[5], rx.data[6], rx.data[7]);
    }

    uint32_t now = millis();
    if ((uint32_t)(now - lastTxMs) < TX_PERIOD_MS) {
        delay(1);
        return;
    }

    twai_status_info_t bus;
    if (twai_get_status_info(&bus) != ESP_OK) {
        delay(1);
        return;
    }

    /* twai_transmit returns ESP_ERR_INVALID_STATE (0x103) if not RUNNING (e.g. bus-off / recovering). */
    if (bus.state != TWAI_STATE_RUNNING) {
        if ((uint32_t)(now - lastStateLog) >= 2000U) {
            lastStateLog = now;
            Serial.printf("[CAN] TX held: state=%s — need RUNNING to transmit\n",
                          twai_state_name(bus.state));
            Serial.println("      Fix: ~120 Ohm between CANH & CANL (bench), bitrate, GPIO21/22 swap.");
            if (bus.state == TWAI_STATE_BUS_OFF) {
                twai_initiate_recovery();
            }
        }
        lastTxMs = now;
        delay(1);
        return;
    }

    lastTxMs = now;

    twai_message_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.identifier       = TX_CAN_ID;
    tx.extd             = 0;
    tx.data_length_code = 4;
    tx.data[0]          = 0xBE;
    tx.data[1]          = 0xEF;
    tx.data[2]          = (uint8_t)(txCtr & 0xFF);
    tx.data[3]          = (uint8_t)((txCtr >> 8) & 0xFF);
    txCtr++;

    esp_err_t txErr = twai_transmit(&tx, pdMS_TO_TICKS(100));
    if (txErr == ESP_OK) {
        txCount++;
        Serial.printf("[TX] #%lu  id=0x%03X  %02X %02X %02X %02X\n",
                      (unsigned long)txCount,
                      (unsigned)tx.identifier,
                      tx.data[0], tx.data[1], tx.data[2], tx.data[3]);
    } else {
        Serial.printf("[TX] FAILED err=0x%X", (unsigned)txErr);
        if (txErr == ESP_ERR_INVALID_STATE) {
            Serial.print(" (INVALID_STATE: not RUNNING — bus-off/recovery/stopped)");
        }
        Serial.println();
        if (twai_get_status_info(&bus) == ESP_OK) {
            Serial.printf("      state=%s tx_fail=%lu bus_err=%lu\n",
                          twai_state_name(bus.state),
                          (unsigned long)bus.tx_failed_count,
                          (unsigned long)bus.bus_error_count);
        }
    }
}
