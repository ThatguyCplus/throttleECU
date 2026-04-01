#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "throttle_config.h"

#define PIN_REN   6U
#define PIN_LEN   5U
#define PIN_RELAY 7U

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
    GPIO_setPinConfig(CFG_REN_PIN_CONFIG);
    GPIO_setPinConfig(CFG_LEN_PIN_CONFIG);
    GPIO_setPinConfig(CFG_RELAY_PIN_CONFIG);
    GPIO_setDirectionMode(PIN_REN, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(PIN_LEN, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(PIN_RELAY, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(PIN_REN, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(PIN_LEN, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(PIN_RELAY, GPIO_PIN_TYPE_STD);
    GPIO_writePin(PIN_REN, 1U);
    GPIO_writePin(PIN_LEN, 1U);
    GPIO_writePin(PIN_RELAY, 0U);

    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0U);
    CPUTimer_setPeriod(CPUTIMER0_BASE, (DEVICE_SYSCLK_FREQ / 1000U) - 1U);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    CPUTimer_startTimer(CPUTIMER0_BASE);

    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    Interrupt_enable(INT_TIMER0);

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
    GPIO_writePin(PIN_REN, on ? 1U : 0U);
    GPIO_writePin(PIN_LEN, on ? 1U : 0U);
}
