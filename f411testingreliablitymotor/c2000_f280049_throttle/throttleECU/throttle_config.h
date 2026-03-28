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

#define CFG_EPWM_R_BASE      EPWM2_BASE
#define CFG_EPWM_L_BASE      EPWM1_BASE
#define CFG_RPWM_PIN_CONFIG  GPIO_2_EPWM2_A
#define CFG_LPWM_PIN_CONFIG  GPIO_0_EPWM1_A

#define CFG_REN_PIN_CONFIG   GPIO_6_GPIO6
#define CFG_LEN_PIN_CONFIG   GPIO_5_GPIO5
#define CFG_RELAY_PIN_CONFIG GPIO_7_GPIO7

#define CFG_ENC_PIN_CONFIG   GPIO_4_GPIO4
#define CFG_ENC_GPIO_PIN     4U

#define CFG_ADC_RIS_SOC      ADC_SOC_NUMBER0
#define CFG_ADC_RIS_CH       ADC_CH_ADCIN1
#define CFG_ADC_LIS_SOC      ADC_SOC_NUMBER1
#define CFG_ADC_LIS_CH       ADC_CH_ADCIN2

#define CFG_ENC_OFFSET        16U
#define CFG_ENC_DATA          4095U
#define CFG_ENC_PERIOD_CLKS   4119U
#define CFG_ENC_SPIKE_THRESH  500

#define CFG_ANGLE_MIN         7032
#define CFG_ANGLE_MAX         17940

#define CFG_KP_DEFAULT         12.0f
#define CFG_KI_DEFAULT         0.3f
#define CFG_KD_DEFAULT         1.5f
#define CFG_PID_INTEGRAL_LIMIT 2000.0f

#define CFG_PID_DEADBAND       100
#define CFG_MIN_DUTY_THRESH    50
#define CFG_SETTLE_TIME_MS     500U
#define CFG_SETTLE_WINDOW      100

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

#endif
