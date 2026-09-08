/* drv8873_test.c — bare-metal DRV8873HPWPR bring-up test
 *
 * Bypasses all throttle ECU logic.  Directly sequences NSLEEP/DISABLE/DIR
 * and EPWM1A to verify the motor driver chip powers up and drives the motor.
 *
 * Pin usage (matches throttle_config.h):
 *   GPIO3  — NSLEEP  (HIGH = awake)
 *   GPIO4  — DISABLE (LOW  = outputs enabled)
 *   GPIO2  — PH/IN2  direction (HIGH = forward)
 *   GPIO0  — EN/IN1  EPWM1A speed PWM
 *
 * Sequence on power-up:
 *   1. All outputs safe (NSLEEP=0, DISABLE=1, PWM=0%)
 *   2. Assert NSLEEP HIGH, wait 5 ms (DRV8873 tSLEEP typ 1 ms)
 *   3. Assert DISABLE LOW  — outputs enabled
 *   4. Ramp: 25% → 50% → 75% → 0% → repeat indefinitely
 *
 * Expected result with motor connected and 12 V supply:
 *   Motor spins forward at three increasing speeds, then coasts 1 s, repeats.
 *
 * Call DRV8873_runTest() from main() — it never returns.
 */

#include "driverlib.h"
#include "device.h"
#include "throttle_config.h"

/* 20 kHz PWM: TBPRD = SYSCLK/Fpwm - 1 = 100e6/20000 - 1 = 4999 */
#define DRV_TBPRD  4999U

/* ── Spin-wait ────────────────────────────────────────────────────────────── */
static void spin_ms(uint32_t ms)
{
    /* Conservative: 200 000 loop iterations ≈ 2 ms @ 100 MHz (NOP + loop overhead).
     * Intentionally over-estimated — we only need "long enough", not precision. */
    while (ms-- > 0U) {
        volatile uint32_t i = 200000U;
        while (i-- > 0U) { __asm(" NOP"); }
    }
}

/* ── ePWM1A setup ─────────────────────────────────────────────────────────── */
static void drv_epwm_init(void)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    GPIO_setPinConfig(GPIO_0_EPWM1_A);   /* EN/IN1 → EPWM1A */

    EPWM_setTimeBasePeriod(EPWM1_BASE, DRV_TBPRD);
    EPWM_setPhaseShift(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP);
    EPWM_setClockPrescaler(EPWM1_BASE,
                           EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_disablePhaseShiftLoad(EPWM1_BASE);

    /* Action-qualifier: LOW@ZERO, HIGH@CMPA_UP, LOW@PRD
     * → duty = (TBPRD - CMPA) / TBPRD  (same as motor_epwm.c) */
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

    /* Start at 0% (CMPA = TBPRD → no HIGH pulse) */
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, DRV_TBPRD);
}

/* ── Set PWM duty 0-100 % ─────────────────────────────────────────────────── */
static void drv_set_duty(uint16_t pct)
{
    if (pct > 100U) { pct = 100U; }
    uint16_t cmpa = (uint16_t)(DRV_TBPRD - (uint32_t)pct * DRV_TBPRD / 100U);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, cmpa);
}

/* ── Public entry point ───────────────────────────────────────────────────── */
void DRV8873_runTest(void)
{
    /* ── Step 1: GPIO directions, safe defaults ──────────────────────────── */

    /* NSLEEP — start LOW (chip sleeping) */
    GPIO_setPinConfig(CFG_MOT_NSLEEP_PIN_CONFIG);
    GPIO_setDirectionMode(CFG_MOT_NSLEEP_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(CFG_MOT_NSLEEP_PIN,     GPIO_PIN_TYPE_STD);
    GPIO_writePin(CFG_MOT_NSLEEP_PIN, 0U);

    /* DISABLE — start HIGH (outputs tri-stated) */
    GPIO_setPinConfig(CFG_MOT_DISABLE_PIN_CONFIG);
    GPIO_setDirectionMode(CFG_MOT_DISABLE_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(CFG_MOT_DISABLE_PIN,     GPIO_PIN_TYPE_STD);
    GPIO_writePin(CFG_MOT_DISABLE_PIN, 1U);

    /* DIR — forward (HIGH) */
    GPIO_setPinConfig(CFG_MOT_DIR_PIN_CONFIG);
    GPIO_setDirectionMode(CFG_MOT_DIR_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(CFG_MOT_DIR_PIN,     GPIO_PIN_TYPE_STD);
    GPIO_writePin(CFG_MOT_DIR_PIN, 1U);

    /* ── Step 2: PWM — 20 kHz, 0% ───────────────────────────────────────── */
    drv_epwm_init();

    /* ── Step 3: Wake DRV8873H ───────────────────────────────────────────── */
    GPIO_writePin(CFG_MOT_NSLEEP_PIN, 1U);
    spin_ms(5U);   /* > tSLEEP (1 ms typ) */

    /* ── Step 4: Enable outputs ──────────────────────────────────────────── */
    GPIO_writePin(CFG_MOT_DISABLE_PIN, 0U);
    spin_ms(2U);

    /* ── Step 5: Ramp loop — observe motor and probe with multimeter/scope ─ */
    for (;;) {
        /* 25% — slow spin (2 s) */
        drv_set_duty(25U);
        spin_ms(2000U);

        /* 50% — medium spin (2 s) */
        drv_set_duty(50U);
        spin_ms(2000U);

        /* 75% — fast spin (2 s) */
        drv_set_duty(75U);
        spin_ms(2000U);

        /* 0% / coast (1 s) */
        drv_set_duty(0U);
        spin_ms(1000U);
    }
}
