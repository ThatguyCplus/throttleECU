#include "driverlib.h"
#include "device.h"
#include <stdlib.h>
#include "encoder_gpio.h"
#include "board.h"
#include "throttle_config.h"

static volatile uint32_t s_encRiseCycles = 0U;
static volatile uint32_t s_encHighUs    = 0U;
static volatile uint32_t s_encPeriodUs  = 0U;
static volatile bool     s_encValid    = false;

#define ENC_AVG_SIZE 8
static int32_t  s_encBuf[ENC_AVG_SIZE];
static uint8_t  s_encBufIdx = 0U;
static uint8_t  s_encBufCnt = 0U;

#pragma CODE_SECTION(encoderIsr, ".TI.ramfunc");
__interrupt void encoderIsr(void)
{
    uint32_t now = Board_cycleCounter();
    uint16_t lev = GPIO_readPin(CFG_ENC_GPIO_PIN);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    if (lev != 0U) {
        if (s_encRiseCycles != 0U) {
            s_encPeriodUs = Board_cyclesToUs(now - s_encRiseCycles);
        }
        s_encRiseCycles = now;
    } else {
        s_encHighUs = Board_cyclesToUs(now - s_encRiseCycles);
        s_encValid  = (s_encPeriodUs > 0U);
    }
}

static int32_t getAngleRaw(void)
{
    uint32_t h;
    uint32_t p;
    bool     v;

    asm(" DINT");
    h = s_encHighUs;
    p = s_encPeriodUs;
    v = s_encValid;
    asm(" EINT");

    if (!v || p == 0U) {
        return -1;
    }

    int32_t raw = (int32_t)((uint64_t)h * (uint64_t)CFG_ENC_PERIOD_CLKS / (uint64_t)p)
                  - (int32_t)CFG_ENC_OFFSET;
    if (raw < 0) {
        raw = 0;
    }
    if (raw > 4094) {
        raw = 4094;
    }

    return (int32_t)((uint32_t)raw * 36000UL / (uint32_t)CFG_ENC_DATA);
}

void EncoderGpio_init(void)
{
    asm(" DINT");
    s_encRiseCycles = 0U;
    s_encHighUs     = 0U;
    s_encPeriodUs   = 0U;
    s_encValid      = false;
    asm(" EINT");

    GPIO_setPinConfig(CFG_ENC_PIN_CONFIG);
    GPIO_setDirectionMode(CFG_ENC_GPIO_PIN, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(CFG_ENC_GPIO_PIN, GPIO_PIN_TYPE_STD);

    GPIO_setInterruptPin(CFG_ENC_GPIO_PIN, GPIO_INT_XINT1);
    GPIO_setInterruptType(GPIO_INT_XINT1, GPIO_INT_TYPE_BOTH_EDGES);
    GPIO_enableInterrupt(GPIO_INT_XINT1);

    Interrupt_register(INT_XINT1, &encoderIsr);
    Interrupt_enable(INT_XINT1);
}

int32_t EncoderGpio_getAngle(void)
{
    int32_t raw = getAngleRaw();
    if (raw < 0) {
        return -1;
    }

    if (s_encBufCnt >= 3U) {
        int32_t sum = 0;
        uint8_t i;
        for (i = 0U; i < s_encBufCnt; i++) {
            sum += s_encBuf[i];
        }
        int32_t avg = sum / (int32_t)s_encBufCnt;

        if (abs(raw - avg) > (int32_t)CFG_ENC_SPIKE_THRESH) {
            return avg;
        }
    }

    s_encBuf[s_encBufIdx] = raw;
    s_encBufIdx           = (uint8_t)((s_encBufIdx + 1U) % ENC_AVG_SIZE);
    if (s_encBufCnt < ENC_AVG_SIZE) {
        s_encBufCnt++;
    }

    {
        int32_t sum = 0;
        uint8_t i;
        for (i = 0U; i < s_encBufCnt; i++) {
            sum += s_encBuf[i];
        }
        return sum / (int32_t)s_encBufCnt;
    }
}
