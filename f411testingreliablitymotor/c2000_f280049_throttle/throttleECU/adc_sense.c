#include "driverlib.h"
#include "device.h"
#include "adc_sense.h"
#include "throttle_config.h"

/* Diagnostic globals — inspect in debugger at any time (same pattern as g_enc_raw_last).
 * ISO26262 volatile audit (2026-09-11): written in ADC read helpers called from the
 * main loop; not shared with any ISR. volatile prevents the compiler from eliminating
 * the stores as dead writes, ensuring debugger visibility. Confirmed correct. */
volatile uint16_t g_adc_ris_last = 0U;  /* last IPROPI1 raw ADC count (0-4095), ADCA_IN0, pin 23 */
volatile uint16_t g_adc_lis_last = 0U;  /* last IPROPI2 raw ADC count (0-4095), ADCB_IN0, pin 41 */
volatile uint16_t g_adc_sol_last = 0U;  /* last SOL_CS_CURRENT raw ADC count (0-4095), ADCC_IN1, pin 29 */
volatile uint16_t g_adc_brk_last = 0U;  /* last BRK_SENSE raw ADC count (0-4095), ADCC_IN0, pin 19 */

void AdcSense_init(void)
{
    /* ADCA — IPROPI1 on pin 23 (ADCINA0 = A0).
     * Internal VREF: VREFHIA not connected on this PCB. */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_enableConverter(ADCA_BASE);

    /* ADCB — IPROPI2 on pin 41 (ADCINB0 = B0).
     * Internal VREF: VREFHIB not connected on this PCB. */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCB);
    ADC_setVREF(ADCB_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_4_0);
    ADC_enableConverter(ADCB_BASE);

    /* ADCC — SOL_CS_CURRENT on pin 29 (ADCINC1 = C1), via JP10.
     * Internal VREF: VREFHIC not connected on this PCB. */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCC);
    ADC_setVREF(ADCC_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_4_0);
    ADC_enableConverter(ADCC_BASE);

    DEVICE_DELAY_US(1000U);

    ADC_setupSOC(ADCA_BASE, CFG_ADC_RIS_SOC, ADC_TRIGGER_SW_ONLY,
                 CFG_ADC_RIS_CH, 10U);
    ADC_setupSOC(ADCB_BASE, CFG_ADC_LIS_SOC, ADC_TRIGGER_SW_ONLY,
                 CFG_ADC_LIS_CH, 10U);
    ADC_setupSOC(ADCC_BASE, CFG_ADC_SOL_SOC, ADC_TRIGGER_SW_ONLY,
                 CFG_ADC_SOL_CH, 10U);
    ADC_setupSOC(ADCC_BASE, CFG_ADC_BRK_SOC, ADC_TRIGGER_SW_ONLY,
                 CFG_ADC_BRK_CH, 10U);
}

static uint16_t readAvgSOC(uint32_t adcBase, uint32_t resultBase,
                            ADC_SOCNumber soc, uint32_t oversample)
{
    uint32_t sum = 0U;
    uint32_t i;

    for (i = 0U; i < oversample; i++) {
        ADC_forceSOC(adcBase, soc);
        /* Timeout prevents blocking forever if ADC is disrupted by motor noise */
        uint32_t timeout = 50000U;
        while (ADC_isBusy(adcBase) && (timeout > 0U)) {
            timeout--;
        }
        {
            uint16_t raw = ADC_readResult(resultBase, soc);
            /* ISO26262: The F280049C ADC is 12-bit; the result register is 16
             * bits but only bits [11:0] are valid (0-4095). A corrupted ADC
             * result register (e.g. due to RAM/bus fault) could read > 4095 and
             * confuse the safety threshold comparisons. Clamp here to prevent
             * any out-of-range value from propagating to fault detection logic. */
            if (raw > 4095U) {
                raw = 4095U;
            }
            sum += raw;
        }
    }
    return (uint16_t)(sum / oversample);
}

void AdcSense_readCurrents(uint16_t *risOut, uint16_t *lisOut)
{
    uint32_t n = (uint32_t)CFG_ADC_OVERSAMPLE;
    if (n < 1U) {
        n = 1U;
    }
    if (risOut != NULL) {
        *risOut = readAvgSOC(ADCA_BASE, ADCARESULT_BASE, CFG_ADC_RIS_SOC, n);
        g_adc_ris_last = *risOut;
    }
    if (lisOut != NULL) {
        *lisOut = readAvgSOC(ADCB_BASE, ADCBRESULT_BASE, CFG_ADC_LIS_SOC, n);
        g_adc_lis_last = *lisOut;
    }
}

void AdcSense_readSolenoid(uint16_t *solOut)
{
    uint32_t n = (uint32_t)CFG_ADC_OVERSAMPLE;
    if (n < 1U) {
        n = 1U;
    }
    if (solOut != NULL) {
        *solOut = readAvgSOC(ADCC_BASE, ADCCRESULT_BASE, CFG_ADC_SOL_SOC, n);
        g_adc_sol_last = *solOut;
    }
}

void AdcSense_readBrake(uint16_t *brkOut)
{
    uint32_t n = (uint32_t)CFG_ADC_OVERSAMPLE;
    if (n < 1U) {
        n = 1U;
    }
    if (brkOut != NULL) {
        *brkOut = readAvgSOC(ADCC_BASE, ADCCRESULT_BASE, CFG_ADC_BRK_SOC, n);
        g_adc_brk_last = *brkOut;
    }
}
