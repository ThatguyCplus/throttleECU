#ifndef THROTTLE_ECU_H
#define THROTTLE_ECU_H

#include <stdint.h>
#include "device.h"

void Throttle_init(void);
void Throttle_runOnce(void);

void Throttle_CanRxApply(uint8_t flags, uint8_t throttle_pct, uint8_t seq);

#endif
