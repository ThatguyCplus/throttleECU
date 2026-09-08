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
 *   [0] mode(bits1:0) | relay(bit4)
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
                     uint16_t raw_angle_hundredths);

#endif
