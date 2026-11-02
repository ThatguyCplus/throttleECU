#ifndef MOTOR_EPWM_H
#define MOTOR_EPWM_H

#include <stdint.h>

void MotorEPwm_init(void);
void MotorEPwm_setCommand(int32_t cmd, int pwmMax);

/* Returns 1 if DRV8873H nFAULT is asserted (GPIO1 LOW = fault), 0 if normal. */
uint8_t MotorEPwm_isNFault(void);

#endif
