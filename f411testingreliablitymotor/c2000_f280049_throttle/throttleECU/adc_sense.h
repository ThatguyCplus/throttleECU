#ifndef ADC_SENSE_H
#define ADC_SENSE_H

#include <stdint.h>

void AdcSense_init(void);

/* Motor phase currents — IPROPI1 (ADCA_IN0) and IPROPI2 (ADCB_IN0) */
void AdcSense_readCurrents(uint16_t *risOut, uint16_t *lisOut);

/* Solenoid current sense — SOL_CS_CURRENT (ADCC_IN1, pin 29) */
void AdcSense_readSolenoid(uint16_t *solOut);

/* Brake sense — BRK_SENSE (ADCC_IN0, pin 19); returns raw ADC count */
void AdcSense_readBrake(uint16_t *brkOut);

/* Diagnostic globals — inspect in debugger without halting */
extern volatile uint16_t g_adc_ris_last;
extern volatile uint16_t g_adc_lis_last;
extern volatile uint16_t g_adc_sol_last;
extern volatile uint16_t g_adc_brk_last;

#endif
