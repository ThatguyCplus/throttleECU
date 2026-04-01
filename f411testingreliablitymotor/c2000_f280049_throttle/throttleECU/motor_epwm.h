#ifndef MOTOR_EPWM_H
#define MOTOR_EPWM_H

#include <stdint.h>

void MotorEPwm_init(void);
void MotorEPwm_setCommand(int32_t cmd, int pwmMax);

#endif
