#include <stdlib.h>
#include "pid.h"

static float    s_int    = 0.0f;
static int32_t  s_prevE  = 0;
static uint32_t s_lastMs = 0U;

void Pid_reset(void)
{
    s_int    = 0.0f;
    s_prevE  = 0;
    s_lastMs = 0U;
}

int32_t Pid_run(int32_t current, int32_t target, int32_t pwmMax,
                int32_t deadband, uint16_t minDutyThresh,
                float kp, float ki, float kd, float iLimit,
                uint32_t nowMs)
{
    /* ISO26262: use millisecond timestamps to avoid uint32_t overflow that
     * occurred at ~71 min when converting ms→µs with now*1000. Unsigned
     * subtraction handles Board_millis() rollover correctly. */
    float dt = (s_lastMs == 0U) ? 0.005f
                                : (float)(nowMs - s_lastMs) * 0.001f;
    s_lastMs = nowMs;

    if (dt > 0.05f) {
        dt = 0.05f;
    }
    if (dt <= 0.0f) {
        dt = 0.001f;
    }

    int32_t err = target - current;

    if (abs(err) < deadband) {
        s_int   = 0.0f;
        s_prevE = 0;
        return 0;
    }

    float pTerm = kp * (float)err;
    s_int += ki * (float)err * dt;
    if (s_int > iLimit) {
        s_int = iLimit;
    }
    if (s_int < -iLimit) {
        s_int = -iLimit;
    }

    float dTerm = (dt > 0.0f) ? kd * (float)(err - s_prevE) / dt : 0.0f;
    s_prevE     = err;

    float out = pTerm + s_int + dTerm;
    int32_t cmd;
    if (out > (float)pwmMax) {
        cmd = pwmMax;
    } else if (out < (float)(-pwmMax)) {
        cmd = -pwmMax;
    } else {
        cmd = (int32_t)out;
    }

    if (abs(cmd) < (int32_t)minDutyThresh) {
        cmd = 0;
    }

    return cmd;
}
