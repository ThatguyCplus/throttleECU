#ifndef PID_H
#define PID_H

#include <stdint.h>

void Pid_reset(void);
int32_t Pid_run(int32_t current, int32_t target, int32_t pwmMax,
                int32_t deadband, uint16_t minDutyThresh,
                float kp, float ki, float kd, float iLimit,
                uint32_t nowUs);

#endif
