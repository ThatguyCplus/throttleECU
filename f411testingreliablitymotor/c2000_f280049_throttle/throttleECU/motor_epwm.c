#include "driverlib.h"
#include "device.h"
#include "motor_epwm.h"
#include "throttle_config.h"

static uint16_t s_tbprd = 1U;

static void initOneEPWM(uint32_t base, uint16_t tbprd)
{
    EPWM_setTimeBasePeriod(base, tbprd);
    EPWM_setPhaseShift(base, 0U);
    EPWM_setTimeBaseCounter(base, 0U);
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, 0U);

    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
}

void MotorEPwm_init(void)
{
    uint32_t sys = DEVICE_SYSCLK_FREQ;
    uint32_t f   = (uint32_t)CFG_PWM_FREQ_HZ;
    uint32_t prd = (f > 0U) ? (sys / f) : 5000U;
    if (prd < 3U) {
        prd = 3U;
    }
    s_tbprd = (uint16_t)(prd - 1U);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);

    GPIO_setPinConfig(CFG_RPWM_PIN_CONFIG);
    GPIO_setPinConfig(CFG_LPWM_PIN_CONFIG);

    initOneEPWM(CFG_EPWM_R_BASE, s_tbprd);
    initOneEPWM(CFG_EPWM_L_BASE, s_tbprd);
}

void MotorEPwm_setCommand(int32_t cmd, int pwmMax)
{
    uint16_t tbprd = s_tbprd;
    uint32_t baseR = CFG_EPWM_R_BASE;
    uint32_t baseL = CFG_EPWM_L_BASE;

    if (pwmMax <= 0) {
        pwmMax = 1;
    }

    if (cmd > 0) {
        int64_t cmp64 = ((int64_t)cmd * (int64_t)tbprd) / (int64_t)pwmMax;
        uint16_t cmp  = (cmp64 > (int64_t)tbprd) ? tbprd : (uint16_t)cmp64;
        EPWM_setCounterCompareValue(baseR, EPWM_COUNTER_COMPARE_A, cmp);
        EPWM_setCounterCompareValue(baseL, EPWM_COUNTER_COMPARE_A, 0U);
    } else if (cmd < 0) {
        int64_t cmp64 = ((int64_t)(-cmd) * (int64_t)tbprd) / (int64_t)pwmMax;
        uint16_t cmp  = (cmp64 > (int64_t)tbprd) ? tbprd : (uint16_t)cmp64;
        EPWM_setCounterCompareValue(baseL, EPWM_COUNTER_COMPARE_A, cmp);
        EPWM_setCounterCompareValue(baseR, EPWM_COUNTER_COMPARE_A, 0U);
    } else {
        EPWM_setCounterCompareValue(baseR, EPWM_COUNTER_COMPARE_A, 0U);
        EPWM_setCounterCompareValue(baseL, EPWM_COUNTER_COMPARE_A, 0U);
    }
}
