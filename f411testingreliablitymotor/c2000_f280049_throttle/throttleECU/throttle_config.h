/*
 * throttle_config.h — F280049C LaunchXL throttle ECU (C2000)
 * Same role as f411testingreliablitymotor_CLEAN/config.h (numeric + pin mux).
 */
#ifndef THROTTLE_CONFIG_H
#define THROTTLE_CONFIG_H

#include <stdint.h>
#include "device.h"
#include "driverlib.h"

#define CFG_SCIA_RX_CONFIG   GPIO_28_SCIA_RX
#define CFG_SCIA_TX_CONFIG   GPIO_29_SCIA_TX

#define CFG_EPWM_L_BASE      EPWM1_BASE
#define CFG_LPWM_PIN_CONFIG  GPIO_0_EPWM1_A   /* EN/IN1  — speed PWM */

#define CFG_MOT_DIR_PIN_CONFIG  GPIO_2_GPIO2  /* PH/IN2  — direction */
#define CFG_MOT_DIR_PIN         2U

#define CFG_MOT_NSLEEP_PIN_CONFIG  GPIO_3_GPIO3
#define CFG_MOT_NSLEEP_PIN         3U
#define CFG_MOT_DISABLE_PIN_CONFIG GPIO_4_GPIO4
#define CFG_MOT_DISABLE_PIN        4U

#define CFG_RELAY_PIN_CONFIG GPIO_7_GPIO7

/* Encoder: AS5147U via bit-bang SPI
 *   MOSI  GPIO56  pkg-pin 65  (MCU → sensor, commands/NOP)
 *   MISO  GPIO57  pkg-pin 66  (sensor → MCU, angle data)
 *   CLK   GPIO58  pkg-pin 67  (SPI clock, idle LOW)
 *   CS    GPIO59  pkg-pin 92  (chip-select, active LOW)
 *
 *   Note: schematic incorrectly listed GPIO40/56/57 — corrected to actual
 *   package pin → GPIO mapping from C2000Ware LAUNCHXL_F280049C.syscfg.json.
 *   W/PWM output (SENSOR_PWM net, GPIO5) is not used — SPI is primary.
 */
#define CFG_ENC_SPI_MOSI_PIN     56U
#define CFG_ENC_SPI_MOSI_CONFIG  GPIO_56_GPIO56
#define CFG_ENC_SPI_MISO_PIN     57U
#define CFG_ENC_SPI_MISO_CONFIG  GPIO_57_GPIO57
#define CFG_ENC_SPI_CLK_PIN      58U
#define CFG_ENC_SPI_CLK_CONFIG   GPIO_58_GPIO58
#define CFG_ENC_SPI_CS_PIN       59U
#define CFG_ENC_SPI_CS_CONFIG    GPIO_59_GPIO59

#define CFG_ADC_RIS_SOC      ADC_SOC_NUMBER0
#define CFG_ADC_RIS_CH       ADC_CH_ADCIN1
#define CFG_ADC_LIS_SOC      ADC_SOC_NUMBER1
#define CFG_ADC_LIS_CH       ADC_CH_ADCIN2

#define CFG_ENC_SPIKE_THRESH   500   /* 0.01° spike rejection threshold */
#define CFG_ENC_SPIKE_CONSEC   3U    /* consecutive spikes → real movement, flush filter */

/* Throttle travel limits in 0.01° units (0–35999 = 0°–360°).
 * MUST be recalibrated after mounting the AS5147U:
 *   1. Physically move throttle to fully-closed, read "pos" in telemetry → set CFG_ANGLE_MIN
 *   2. Physically move throttle to fully-open,   read "pos" in telemetry → set CFG_ANGLE_MAX */
/* Physical travel measured 2026-09-06 (raw CAN angle, post pin-fix):
 *   open  (100%, smaller angle): 22.11° = 2211 in 0.01° units
 *   closed  (0%, larger angle): 132.62° = 13262 in 0.01° units
 * 2° end-stop buffer applied to each side → MIN=2411, MAX=13062.
 * Firmware adds its own 5%/95% usable guard on top of these.
 * Regenerate with the GUI Calibration panel if remounted. */
#define CFG_ANGLE_MIN         2411    /* physical open  + 2° buffer */
#define CFG_ANGLE_MAX         13062   /* physical closed - 2° buffer */

#define CFG_KP_DEFAULT         12.0f
#define CFG_KI_DEFAULT         0.3f
#define CFG_KD_DEFAULT         1.5f
#define CFG_PID_INTEGRAL_LIMIT 2000.0f

/* Spring return feed-forward
 * Applies a holding force toward open proportional to how open the throttle is,
 * compensating for the throttle body return spring.
 * At 100% open → FF = -CFG_FF_SPRING_GAIN (toward open)
 * At   0% closed → FF = 0 (no force needed, spring is not loaded)
 * Tune this up from 0 until hunting at 100% stops. 500 ≈ 12% duty at full open.
 * Set to 0 to disable. */
#define CFG_FF_SPRING_GAIN     600

#define CFG_PID_DEADBAND       100
#define CFG_MIN_DUTY_THRESH    50
#define CFG_SETTLE_TIME_MS     500U
#define CFG_SETTLE_WINDOW      100

/* Setpoint slew rate limiter
 * Ramps the active target angle toward the commanded target at a fixed rate,
 * preventing instantaneous large steps that cause current spikes and VM sag.
 *
 * CFG_SLEW_RATE_MS : interval (ms) between each slew step
 * CFG_SLEW_STEP    : max change in 0.01° units per interval
 *
 * Effective slew: (CFG_SLEW_STEP / CFG_SLEW_RATE_MS) × 1000 units/s
 * Default: 200 / 10 × 1000 = 20 000 units/s ≈ 200°/s → full travel in ~0.5 s
 * Set CFG_SLEW_STEP to 0 to disable (instantaneous, legacy behaviour). */
#define CFG_SLEW_RATE_MS  10U
#define CFG_SLEW_STEP     200

#define CFG_OVERCURRENT_THRESH    4095U
#define CFG_OVERCURRENT_DEBOUNCE  10U
#define CFG_ENCODER_TIMEOUT_MS    30000U
#define CFG_ENCODER_DEBOUNCE      2U

#define CFG_POWER_LOW_MV       9000U
#define CFG_POWER_DEBOUNCE     4U

#define CFG_RECOVERY_DELAY_MS  1000U

#define CFG_SERIAL_BAUD        115200UL
#define CFG_TELEMETRY_RATE_MS  200U
#define CFG_ADC_OVERSAMPLE     16U

#define CFG_PWM_FREQ_HZ        20000U
#define CFG_PWM_MAX            4095U

/* ── CAN bus (CANA via WeAct CANFDSIO / IS2062A transceiver) ─────────────── */
/* GPIO31/30: avoids onboard SN65 on 32/33 when using external transceiver; free vs 4/5 enc/LEN */
#define CFG_CAN_TX_PIN         31U
#define CFG_CAN_TX_CONFIG      GPIO_31_CANA_TX
#define CFG_CAN_RX_PIN         30U
#define CFG_CAN_RX_CONFIG      GPIO_30_CANA_RX

/* 500 kbps — ISO 11898, standard automotive CAN */
#define CFG_CAN_BITRATE        500000UL

/* Mailbox IDs (1-32 for C2000 CANA) */
#define CFG_CAN_RX_MAILBOX     1U
#define CFG_CAN_TX_MAILBOX     2U

/* Pin mux for CANA: 0=GPIO32/33 1=GPIO4/5 2=GPIO30/31 (recommended with external transceiver) */
#define CFG_CAN_PINMUX         2U

/* Application-level 11-bit CAN IDs */
#define CFG_CAN_RX_ID          0x100U   /* commands from external controller → ECU */
#define CFG_CAN_TX_ID          0x101U   /* telemetry from ECU → external controller */

#define CFG_CAN_RX_DLC         4U
#define CFG_CAN_TX_DLC         8U

/* Timing */
#define CFG_CAN_RX_TIMEOUT_MS  200U     /* >200 ms without RX → CAN_TIMEOUT fault  */
#define CFG_CAN_TX_RATE_MS     20U      /* 50 Hz telemetry transmit rate            */

/* After first valid 0x100 frame: require heartbeats within CFG_CAN_RX_TIMEOUT_MS */
#define CFG_CAN_HEARTBEAT_EN   1U

/* RX command byte flags (byte 0 of 4-byte RX frame) */
#define CFG_CAN_FLAG_RELAY     0x01U    /* 1 = relay on,  0 = relay off */
#define CFG_CAN_FLAG_PID       0x02U    /* 1 = PID mode,  0 = manual stop */
#define CFG_CAN_FLAG_ESTOP     0x04U    /* 1 = trigger safe state immediately */
#define CFG_CAN_FLAG_RESET     0x08U    /* 1 = clear faults and exit safe state */

#endif
