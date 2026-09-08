#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "throttle_config.h"

#define PIN_RELAY 7U   /* SOL_GATE → TPS1H100B IN */

static volatile uint32_t g_millis = 0U;

#pragma CODE_SECTION(cpuTimer0ISR, ".TI.ramfunc");
__interrupt void cpuTimer0ISR(void)
{
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);
    g_millis++;
}

uint32_t Board_millis(void)
{
    return g_millis;
}

uint32_t Board_cycleCounter(void)
{
    return (0xFFFFFFFFUL - CPUTimer_getTimerCount(CPUTIMER1_BASE));
}

uint32_t Board_cyclesToUs(uint32_t deltaCycles)
{
    uint32_t sysMhz = (uint32_t)(DEVICE_SYSCLK_FREQ / 1000000UL);
    if (sysMhz == 0U) {
        sysMhz = 1U;
    }
    return deltaCycles / sysMhz;
}

void Board_initHW(void)
{
    /* Solenoid gate — TPS1H100B IN (active high, init off) */
    GPIO_setPinConfig(CFG_RELAY_PIN_CONFIG);
    GPIO_setDirectionMode(PIN_RELAY, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(PIN_RELAY, GPIO_PIN_TYPE_STD);
    GPIO_writePin(PIN_RELAY, 0U);

    /* DRV8873H NSLEEP — HIGH = awake, init sleeping */
    GPIO_setPinConfig(CFG_MOT_NSLEEP_PIN_CONFIG);
    GPIO_setDirectionMode(CFG_MOT_NSLEEP_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(CFG_MOT_NSLEEP_PIN, GPIO_PIN_TYPE_STD);
    GPIO_writePin(CFG_MOT_NSLEEP_PIN, 1U);  /* wake immediately; 1 ms hold below */

    /* DRV8873H DISABLE — HIGH = outputs disabled, init disabled */
    GPIO_setPinConfig(CFG_MOT_DISABLE_PIN_CONFIG);
    GPIO_setDirectionMode(CFG_MOT_DISABLE_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(CFG_MOT_DISABLE_PIN, GPIO_PIN_TYPE_STD);
    GPIO_writePin(CFG_MOT_DISABLE_PIN, 1U);

    /* DRV8873H tSLEEP = 1 ms (typ) before outputs can be enabled.
     * Spin here during init — avoids needing a delay in Board_digitalEnables(). */
    {
        uint32_t t = (DEVICE_SYSCLK_FREQ / 1000U) * 2U;  /* ~2 ms margin */
        while (t-- > 0U) { __asm(" NOP"); }
    }

    /* GPIO5 = SENSOR_PWM (AS5147U W output) — input only; encoder_gpio.c owns this */

    /* 1 ms tick timer */
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0U);
    CPUTimer_setPeriod(CPUTIMER0_BASE, (DEVICE_SYSCLK_FREQ / 1000U) - 1U);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    CPUTimer_startTimer(CPUTIMER0_BASE);

    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    Interrupt_enable(INT_TIMER0);

    /* Free-running cycle counter for timing */
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0U);
    CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFFUL);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_startTimer(CPUTIMER1_BASE);
}

void Board_digitalRelay(uint16_t on)
{
    GPIO_writePin(PIN_RELAY, on ? 1U : 0U);
}

void Board_digitalEnables(uint16_t on)
{
    /* NSLEEP is held HIGH permanently (woken at startup with tSLEEP delay).
     * Only DISABLE is toggled here: LOW = outputs enabled, HIGH = disabled. */
    GPIO_writePin(CFG_MOT_DISABLE_PIN, on ? 0U : 1U);
}
