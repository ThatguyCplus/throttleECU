#ifndef PID_H
#define PID_H

#include <stdint.h>

void Pid_reset(void);

/* Pid_resetIntegral — clear the integral accumulator only.
 * Unlike Pid_reset(), this preserves s_prevE (derivative memory) and s_lastMs
 * (dt timestamp), so derivative action and timing are unaffected.
 *
 * ISO26262: call this during setpoint slew steps to prevent integral windup
 * accumulating against a moving target. The integral re-accumulates immediately
 * once the slew completes and the target is fixed. */
void Pid_resetIntegral(void);

/* nowMs: Board_millis() — millisecond timestamp. Using ms avoids the uint32_t
 * overflow that occurs when converting ms→µs (overflows after ~71 minutes). */
int32_t Pid_run(int32_t current, int32_t target, int32_t pwmMax,
                int32_t deadband, uint16_t minDutyThresh,
                float kp, float ki, float kd, float iLimit,
                uint32_t nowMs);

#endif
