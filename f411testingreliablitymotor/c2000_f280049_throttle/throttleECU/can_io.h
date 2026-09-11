#ifndef CAN_IO_H
#define CAN_IO_H

#include <stdint.h>
/* C28x: <stdint.h> has no uint8_t; DriverLib typedefs it in hw_types.h */
#include "device.h"

void CanIo_init(void);

/* RX + watchdog + bus-off (call early each loop, before motor control). */
void CanIo_serviceRx(uint32_t now_ms);

/* Telemetry on 0x101 at CFG_CAN_TX_RATE_MS (call after motor/PID update).
 *
 * Frame layout (8 bytes):
 *   [0] mode(bits1:0) | relay(bit4) | brake(bit5)
 *   [1] act_pct_spi  (0-100 or 0xFF = no encoder)
 *   [2] tgt_pct
 *   [3] fault_flags
 *   [4] motor_cmd low byte
 *   [5] motor_cmd high byte
 *   [6] raw_angle_hundredths low byte  (0.01 deg units, 0-35999)
 *   [7] raw_angle_hundredths high byte
 */
void CanIo_serviceTx(uint32_t now_ms,
                     uint8_t mode_u8,
                     uint8_t act_pct_spi,
                     uint8_t tgt_pct,
                     uint8_t fault_flags,
                     int16_t motor_cmd,
                     uint8_t relay_on,
                     uint16_t raw_angle_hundredths,
                     uint8_t brake_on);

/* Current sense telemetry on 0x102 at CFG_CAN_TX2_RATE_MS (call after ADC read).
 *
 * Frame layout (7 bytes):
 *   [0] ris_raw low byte   (IPROPI1 raw 12-bit ADC count, 0-4095)
 *   [1] ris_raw high byte
 *   [2] lis_raw low byte   (IPROPI2 raw 12-bit ADC count, 0-4095)
 *   [3] lis_raw high byte
 *   [4] sol_raw low byte   (SOL_CS_CURRENT raw 12-bit ADC count, 0-4095)
 *   [5] sol_raw high byte
 *   [6] sol_status bitmask:
 *         bit0 FAULT_SOL_OPEN   (relay ON, current < ON threshold)
 *         bit1 FAULT_SOL_WELDED (relay OFF, current > ON threshold → safe state)
 *         bit2 FAULT_SOL_OC     (current > OC threshold → safe state)
 *         bit3 SOL_INFERRED_ON  (current > ON threshold, informational)
 *
 * Motor I conversion: I_mA = raw × 3300 × 1500 / (4096 × 360) ≈ raw × 3.357 mA/count
 * Sol I conversion:   I_A  = raw × 3300 × 6444 / (4096 × 4700 × 1000) ≈ raw × 0.001104 A/count
 */
void CanIo_serviceTx2(uint32_t now_ms, uint16_t ris_raw, uint16_t lis_raw,
                      uint16_t sol_raw, uint8_t sol_status);

#endif
