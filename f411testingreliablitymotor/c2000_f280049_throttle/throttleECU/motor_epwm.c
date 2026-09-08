#include "driverlib.h"
#include "device.h"
#include "motor_epwm.h"
#include "throttle_config.h"

/* DRV8873H — PH/EN mode
 *   EN/IN1 (GPIO0 / EPWM1A) : speed PWM  (0 = coast, 100% = full speed)
 *   PH/IN2 (GPIO2 / GPIO)   : direction  (HIGH = forward, LOW = reverse)
 */

static uint16_t s_tbprd = 1U;

void MotorEPwm_init(void)
{
    uint32_t sys = DEVICE_SYSCLK_FREQ;
    uint32_t f   = (uint32_t)CFG_PWM_FREQ_HZ;
    uint32_t prd = (f > 0U) ? (sys / f) : 5000U;
    if (prd < 3U) {
        prd = 3U;
    }
    s_tbprd = (uint16_t)(prd - 1U);

    /* EN/IN1 — EPWM1A on GPIO0 */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    GPIO_setPinConfig(CFG_LPWM_PIN_CONFIG);

    EPWM_setTimeBasePeriod(CFG_EPWM_L_BASE, s_tbprd);
    EPWM_setPhaseShift(CFG_EPWM_L_BASE, 0U);
    EPWM_setTimeBaseCounter(CFG_EPWM_L_BASE, 0U);
    EPWM_setTimeBaseCounterMode(CFG_EPWM_L_BASE, EPWM_COUNTER_MODE_UP);
    EPWM_setClockPrescaler(CFG_EPWM_L_BASE,
                           EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_disablePhaseShiftLoad(CFG_EPWM_L_BASE);
    EPWM_setCounterCompareValue(CFG_EPWM_L_BASE, EPWM_COUNTER_COMPARE_A, 0U);

    EPWM_setActionQualifierAction(CFG_EPWM_L_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(CFG_EPWM_L_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(CFG_EPWM_L_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

    /* PH/IN2 — GPIO2 as plain direction output, init LOW (reverse safe) */
    GPIO_setPinConfig(CFG_MOT_DIR_PIN_CONFIG);
    GPIO_setDirectionMode(CFG_MOT_DIR_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(CFG_MOT_DIR_PIN, GPIO_PIN_TYPE_STD);
    GPIO_writePin(CFG_MOT_DIR_PIN, 0U);
}

void MotorEPwm_setCommand(int32_t cmd, int pwmMax)
{
    uint16_t tbprd = s_tbprd;

    if (pwmMax <= 0) {
        pwmMax = 1;
    }

    if (cmd == 0) {
        /* Coast: EN duty = 0% — set CMPA = TBPRD so no HIGH pulse */
        EPWM_setCounterCompareValue(CFG_EPWM_L_BASE, EPWM_COUNTER_COMPARE_A, tbprd);
        GPIO_writePin(CFG_MOT_DIR_PIN, 0U);
        return;
    }

    /* Direction */
    GPIO_writePin(CFG_MOT_DIR_PIN, (cmd > 0) ? 1U : 0U);

    /* Speed: AQ = LOW@ZERO, HIGH@CMPA, LOW@PRD → duty = (TBPRD-CMPA)/TBPRD
     * To get duty = mag/pwmMax we need CMPA = TBPRD - mag*TBPRD/pwmMax */
    int32_t mag   = (cmd > 0) ? cmd : -cmd;
    int64_t cmp64 = (int64_t)tbprd - ((int64_t)mag * (int64_t)tbprd) / (int64_t)pwmMax;
    if (cmp64 < 0) { cmp64 = 0; }
    uint16_t cmp  = (cmp64 >= (int64_t)tbprd) ? tbprd : (uint16_t)cmp64;
    EPWM_setCounterCompareValue(CFG_EPWM_L_BASE, EPWM_COUNTER_COMPARE_A, cmp);
}
