#include "can_io.h"

#include <stdbool.h>

#include "throttle_config.h"
#include "safety.h"

#include "device.h"
#include "driverlib.h"

extern void Throttle_CanRxApply(uint8_t flags, uint8_t throttle_pct, uint8_t seq);

#define CAN_BASE_CFG  CANA_BASE

#define CAN_RX_MB     CFG_CAN_RX_MAILBOX
#define CAN_TX_MB     CFG_CAN_TX_MAILBOX

static uint32_t s_lastTxMs      = 0U;
static uint8_t  s_txSeq        = 0U;
static uint8_t  s_busOffLatched = 0U;

static void can_pinmux(void)
{
#if (CFG_CAN_PINMUX == 0U)
    GPIO_setPinConfig(GPIO_32_CANA_RX);
    GPIO_setPinConfig(GPIO_33_CANA_TX);
#elif (CFG_CAN_PINMUX == 1U)
    GPIO_setPinConfig(GPIO_4_CANA_RX);
    GPIO_setPinConfig(GPIO_5_CANA_TX);
#else
    GPIO_setPinConfig(GPIO_30_CANA_RX);
    GPIO_setPinConfig(GPIO_31_CANA_TX);
#endif
}

void CanIo_init(void)
{
    can_pinmux();

    CAN_initModule(CAN_BASE_CFG);
    CAN_setBitRate(CAN_BASE_CFG, DEVICE_SYSCLK_FREQ, CFG_CAN_BITRATE, 20U);
    CAN_setAutoBusOnTime(CAN_BASE_CFG, 10U);
    CAN_enableAutoBusOn(CAN_BASE_CFG);

    CAN_setupMessageObject(CAN_BASE_CFG, CAN_RX_MB, CFG_CAN_RX_ID,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX,
                           0x7FFU,
                           CAN_MSG_OBJ_USE_ID_FILTER,
                           CFG_CAN_RX_DLC);

    CAN_setupMessageObject(CAN_BASE_CFG, CAN_TX_MB, CFG_CAN_TX_ID,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX,
                           0U,
                           CAN_MSG_OBJ_NO_FLAGS,
                           CFG_CAN_TX_DLC);

    CAN_startModule(CAN_BASE_CFG);

    s_lastTxMs      = 0U;
    s_txSeq         = 0U;
    s_busOffLatched = 0U;
}

void CanIo_serviceRx(uint32_t now_ms)
{
    uint32_t st = CAN_getStatus(CAN_BASE_CFG);

    if ((st & CAN_STATUS_BUS_OFF) != 0U) {
        if (s_busOffLatched == 0U) {
            s_busOffLatched = 1U;
            safe_can_set_bus_off_fault();
        }
    } else {
        if (s_busOffLatched != 0U) {
            s_busOffLatched = 0U;
            safe_can_clear_bus_off_fault();
        }
    }

    for (;;) {
        uint16_t     msgData[8];
        uint32_t     rxId       = 0U;
        CAN_MsgFrameType frame   = CAN_MSG_FRAME_STD;
        bool         got;

        got = CAN_readMessageWithID(CAN_BASE_CFG, CAN_RX_MB, &frame, &rxId, msgData);
        if (!got) {
            break;
        }

        if ((frame != CAN_MSG_FRAME_STD) || (rxId != CFG_CAN_RX_ID)) {
            continue;
        }

        {
            uint8_t b0 = (uint8_t)(msgData[0] & 0xFFU);
            uint8_t b1 = (uint8_t)(msgData[1] & 0xFFU);
            uint8_t b3 = (uint8_t)(msgData[3] & 0xFFU);

            if ((b0 & CFG_CAN_FLAG_ESTOP) != 0U) {
                safe_enter_safe_state("CAN ESTOP");
            } else {
                safe_can_mark_rx(now_ms);
                Throttle_CanRxApply(b0, b1, b3);
            }
        }
    }

    safe_can_check_timeout(now_ms);
}

void CanIo_serviceTx(uint32_t now_ms,
                     uint8_t mode_u8,
                     uint8_t act_pct_spi,
                     uint8_t tgt_pct,
                     uint8_t fault_flags,
                     int16_t motor_cmd,
                     uint8_t relay_on)
{
    if ((now_ms - s_lastTxMs) < CFG_CAN_TX_RATE_MS) {
        return;
    }
    s_lastTxMs = now_ms;

    {
        uint16_t tx[8];
        uint16_t m = (uint16_t)((uint16_t)motor_cmd & 0xFFFFU);

        tx[0] = (uint16_t)mode_u8;
        tx[1] = (uint16_t)act_pct_spi;
        tx[2] = (uint16_t)tgt_pct;
        tx[3] = (uint16_t)fault_flags;
        tx[4] = (uint16_t)(m & 0xFFU);
        tx[5] = (uint16_t)((m >> 8) & 0xFFU);
        tx[6] = (uint16_t)relay_on;
        tx[7] = (uint16_t)s_txSeq++;

        CAN_sendMessage(CAN_BASE_CFG, CAN_TX_MB, CFG_CAN_TX_DLC, tx);
    }
}
