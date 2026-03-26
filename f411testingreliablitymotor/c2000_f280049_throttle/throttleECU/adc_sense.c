#include "driverlib.h"
#include "device.h"
#include "adc_sense.h"
#include "throttle_config.h"

void AdcSense_init(void)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);

    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_EXTERNAL, ADC_REFERENCE_3_3V);
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000U);

    ADC_setupSOC(ADCA_BASE, CFG_ADC_RIS_SOC, ADC_TRIGGER_SW_ONLY,
                 CFG_ADC_RIS_CH, 10U);
    ADC_setupSOC(ADCA_BASE, CFG_ADC_LIS_SOC, ADC_TRIGGER_SW_ONLY,
                 CFG_ADC_LIS_CH, 10U);
}

static uint16_t readAvgSOC(ADC_SOCNumber soc, uint32_t oversample)
{
    uint32_t sum = 0U;
    uint32_t i;

    for (i = 0U; i < oversample; i++) {
        ADC_forceSOC(ADCA_BASE, soc);
        /* Timeout prevents blocking forever if ADC is disrupted by motor noise */
        uint32_t timeout = 50000U;
        while (ADC_isBusy(ADCA_BASE) && (timeout > 0U)) {
            timeout--;
        }
        sum += ADC_readResult(ADCARESULT_BASE, soc);
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
        *risOut = readAvgSOC(CFG_ADC_RIS_SOC, n);
    }
    if (lisOut != NULL) {
        *lisOut = readAvgSOC(CFG_ADC_LIS_SOC, n);
    }
}
