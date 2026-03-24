#ifndef ADC_SENSE_H
#define ADC_SENSE_H

#include <stdint.h>

void AdcSense_init(void);
void AdcSense_readCurrents(uint16_t *risOut, uint16_t *lisOut);

#endif
