#include "throttle_ecu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "throttle_config.h"
#include "board.h"
#include "encoder_gpio.h"
#include "motor_epwm.h"
#include "adc_sense.h"
#include "sci_io.h"
#include "pid.h"
#include "safety.h"

typedef enum {
    MODE_MANUAL = 0,
    MODE_PID    = 1,
    MODE_SAFE   = 2
} RunMode;

static RunMode  s_mode         = MODE_MANUAL;
static int8_t   s_dir          = 1;
static uint16_t s_duty         = 0U;
static int32_t  s_targetAngle  = CFG_ANGLE_MIN;
static int16_t  s_throttlePct  = 0;

static float s_kp = CFG_KP_DEFAULT;
static float s_ki = CFG_KI_DEFAULT;
static float s_kd = CFG_KD_DEFAULT;

static bool     s_settled     = false;
static uint32_t s_settleStart = 0U;

static uint32_t s_lastPrint    = 0U;
static uint32_t s_lastSafeTick = 0U;

static const int32_t s_angleRange = (int32_t)CFG_ANGLE_MAX - (int32_t)CFG_ANGLE_MIN;
static const int32_t s_usableMin  =
    (int32_t)CFG_ANGLE_MIN + ((int32_t)CFG_ANGLE_MAX - (int32_t)CFG_ANGLE_MIN) * 5 / 100;
static const int32_t s_usableMax  =
    (int32_t)CFG_ANGLE_MIN + ((int32_t)CFG_ANGLE_MAX - (int32_t)CFG_ANGLE_MIN) * 95 / 100;

static void printBoth(const char *msg)
{
    SciIo_printLine(msg);
}

static int str_eq_ic(const char *a, const char *b)
{
    if (a == NULL || b == NULL) {
        return 0;
    }
    while (*a != '\0' && *b != '\0') {
        char ca = *a;
        char cb = *b;
        if (isupper((int)ca) != 0) {
            ca = (char)tolower((int)ca);
        }
        if (isupper((int)cb) != 0) {
            cb = (char)tolower((int)cb);
        }
        if (ca != cb) {
            return 0;
        }
        a++;
        b++;
    }
    return (*a == *b) ? 1 : 0;
}

static int32_t throttleToAngle(int16_t pct)
{
    int32_t p = (int32_t)pct;
    if (p < 0) {
        p = 0;
    }
    if (p > 100) {
        p = 100;
    }
    return s_usableMin + (int32_t)(((int64_t)(s_usableMax - s_usableMin) * p) / 100);
}

static int16_t angleToThrottle(int32_t angle)
{
    if (angle < s_usableMin) {
        return 0;
    }
    if (angle > s_usableMax) {
        return 100;
    }
    return (int16_t)(((int64_t)(angle - s_usableMin) * 100) / (s_usableMax - s_usableMin));
}

static void setMotor(int32_t cmd)
{
    const int32_t pwmMax = (int32_t)CFG_PWM_MAX;

    if (s_mode == MODE_SAFE) {
        MotorEPwm_setCommand(0, pwmMax);
        Board_digitalEnables(0U);
        return;
    }

    if (cmd > pwmMax) {
        cmd = pwmMax;
    }
    if (cmd < -pwmMax) {
        cmd = -pwmMax;
    }

    if (cmd > 0) {
        Board_digitalEnables(1U);
        MotorEPwm_setCommand(cmd, pwmMax);
    } else if (cmd < 0) {
        Board_digitalEnables(1U);
        MotorEPwm_setCommand(cmd, pwmMax);
    } else {
        MotorEPwm_setCommand(0, pwmMax);
        Board_digitalEnables(0U);
    }
}

static void enterSafeStateEc(const char *reason)
{
    s_mode = MODE_SAFE;
    MotorEPwm_setCommand(0, (int32_t)CFG_PWM_MAX);
    Board_digitalEnables(0U);
    Board_digitalRelay(0U);
    Pid_reset();
    s_duty        = 0U;
    s_throttlePct = 0;
    s_targetAngle = CFG_ANGLE_MIN;
    s_settled     = false;
    s_settleStart = 0U;

    {
        char msg[72];
        snprintf(msg, sizeof(msg), "[SAFE] %s", (reason != NULL) ? reason : "?");
        printBoth(msg);
    }
}

static void processCmd(char *s)
{
    if (str_eq_ic(s, "f")) {
        s_dir = 1;
    } else if (str_eq_ic(s, "r")) {
        s_dir = -1;
    } else if (str_eq_ic(s, "s")) {
        s_duty = 0U;
        s_mode = MODE_MANUAL;
        Pid_reset();
    } else if (str_eq_ic(s, "reset")) {
        if (s_mode == MODE_SAFE) {
            safe_clear_faults();
            s_mode = MODE_MANUAL;
            s_duty = 0U;
            Board_digitalEnables(1U);
            printBoth("Safe state cleared — mode=MAN");
        }
    } else if (str_eq_ic(s, "on")) {
        Board_digitalRelay(1U);
        printBoth("Relay ON");
    } else if (str_eq_ic(s, "off")) {
        Board_digitalRelay(0U);
        printBoth("Relay OFF");
    } else if (str_eq_ic(s, "diag")) {
        safe_print_status(printBoth);
    } else if (str_eq_ic(s, "clearfaults")) {
        safe_clear_faults();
        printBoth("Faults cleared");
    } else if (str_eq_ic(s, "config")) {
        char buf[88];
        printBoth("=================================");
        printBoth("  Active Configuration (C2000)");
        printBoth("=================================");
        snprintf(buf, sizeof(buf), "PWM: %luHz max=%u",
                 (unsigned long)CFG_PWM_FREQ_HZ, (unsigned)CFG_PWM_MAX);
        printBoth(buf);
        snprintf(buf, sizeof(buf), "Usable (5-95%%): %.2f - %.2f (0.01 deg)",
                 (float)s_usableMin / 100.0f, (float)s_usableMax / 100.0f);
        printBoth(buf);
        snprintf(buf, sizeof(buf), "PID: Kp=%.2f Ki=%.2f Kd=%.2f",
                 s_kp, s_ki, s_kd);
        printBoth(buf);
        printBoth("=================================");
    } else if ((s[0] == 't') || (s[0] == 'T')) {
        if (s_mode == MODE_SAFE) {
            printBoth("[SAFE] Cmd rejected");
            return;
        }
        {
            long pct = strtol(s + 1, NULL, 10);
            if (pct < 0L) {
                pct = 0L;
            }
            if (pct > 100L) {
                pct = 100L;
            }
            s_throttlePct  = (int16_t)pct;
            s_targetAngle  = throttleToAngle(s_throttlePct);
            s_mode         = MODE_PID;
            s_settled      = false;
            s_settleStart  = 0U;
            Pid_reset();
        }
    } else if ((s[0] == 'd') || (s[0] == 'D')) {
        if (s_mode == MODE_SAFE) {
            printBoth("[SAFE] Cmd rejected");
            return;
        }
        {
            long v = strtol(s + 1, NULL, 10);
            long mx = (long)CFG_PWM_MAX;
            if (v < 0L) {
                v = 0L;
            }
            if (v > mx) {
                v = mx;
            }
            s_duty = (uint16_t)v;
            s_mode = MODE_MANUAL;
            Pid_reset();
        }
    } else if ((s[0] == 'p') || (s[0] == 'P')) {
        s_kp = (float)strtod(s + 1, NULL);
        {
            char buf[32];
            snprintf(buf, sizeof(buf), "Kp=%.2f", (double)s_kp);
            printBoth(buf);
        }
    } else if ((s[0] == 'i') || (s[0] == 'I')) {
        s_ki = (float)strtod(s + 1, NULL);
        {
            char buf[32];
            snprintf(buf, sizeof(buf), "Ki=%.2f", (double)s_ki);
            printBoth(buf);
        }
    } else if ((s[0] == 'k') || (s[0] == 'K')) {
        s_kd = (float)strtod(s + 1, NULL);
        {
            char buf[32];
            snprintf(buf, sizeof(buf), "Kd=%.2f", (double)s_kd);
            printBoth(buf);
        }
    }
}

void Throttle_init(void)
{
    SciIo_init();
    AdcSense_init();
    MotorEPwm_init();
    EncoderGpio_init();
    safe_init();

    setMotor(0);

    printBoth("=================================");
    printBoth("  Throttle ECU — C2000 F280049");
    printBoth("=================================");
    s_targetAngle = s_usableMin;
    printBoth("Commands: t0-t100, d0-d4095, f, r, s");
    printBoth("         p## i## k##, on, off, reset");
    printBoth("         diag, clearfaults, config");

    if (safe_was_reset_by_watchdog()) {
        printBoth("[WARN] Recovered from watchdog reset!");
    }

    s_lastPrint    = Board_millis();
    s_lastSafeTick = Board_millis();
}

void Throttle_runOnce(void)
{
    char line[64];

    if (SciIo_readLine(line, sizeof(line))) {
        processCmd(line);
    }

    {
        int32_t  angle = EncoderGpio_getAngle();
        uint32_t now   = Board_millis();
        uint32_t nowUs = now * 1000U;

        uint16_t ris = 0U;
        uint16_t lis = 0U;
        AdcSense_readCurrents(&ris, &lis);

        safe_check_encoder(angle >= 0, now);
        safe_check_current(ris, lis, now);

        if (g_safety.safe_state_active && (s_mode != MODE_SAFE)) {
            enterSafeStateEc(g_safety.last_reason);
        }

        if ((now - s_lastSafeTick) >= 100U) {
            s_lastSafeTick = now;
            safe_tick(now);
        }

        switch (s_mode) {
        case MODE_PID:
            if (angle >= 0) {
                int32_t clamped = s_targetAngle;
                if (clamped < s_usableMin) {
                    clamped = s_usableMin;
                }
                if (clamped > s_usableMax) {
                    clamped = s_usableMax;
                }
                {
                    int32_t posErr = clamped - angle;
                    if (posErr < 0) {
                        posErr = -posErr;
                    }

                    if (posErr < (int32_t)CFG_SETTLE_WINDOW) {
                        if (!s_settled) {
                            if (s_settleStart == 0U) {
                                s_settleStart = now;
                            } else if ((now - s_settleStart) >= CFG_SETTLE_TIME_MS) {
                                s_settled = true;
                            }
                        }
                    } else {
                        s_settled     = false;
                        s_settleStart = 0U;
                    }
                }

                {
                    int32_t pidCmd = 0;
                    if (!s_settled) {
                        pidCmd = Pid_run(angle, clamped, (int32_t)CFG_PWM_MAX,
                                         (int32_t)CFG_PID_DEADBAND, CFG_MIN_DUTY_THRESH,
                                         s_kp, s_ki, s_kd, CFG_PID_INTEGRAL_LIMIT, nowUs);
                    }
                    setMotor(pidCmd);
                }
            } else {
                setMotor(0);
            }
            break;

        case MODE_MANUAL:
            if (s_duty == 0U) {
                setMotor(0);
            } else if (s_dir > 0) {
                setMotor((int32_t)s_duty);
            } else {
                setMotor(-(int32_t)s_duty);
            }
            break;

        case MODE_SAFE:
            setMotor(0);
            break;
        }

        if ((now - s_lastPrint) >= CFG_TELEMETRY_RATE_MS) {
            s_lastPrint = now;

            int16_t actPct = -1;
            if (angle >= 0) {
                actPct = angleToThrottle(angle);
            }

            {
                uint8_t errFlags = safe_get_fault_flags();
                static const char *modeStr[] = {"MAN", "PID", "SAFE"};
                char out[128];

                if (angle >= 0) {
                    int32_t frac = angle % 100;
                    if (frac < 0) {
                        frac = -frac;
                    }
                    snprintf(out, sizeof(out),
                             "mode=%s dir=%s duty=%u RIS=%u LIS=%u pos=%ld.%02ld thr=%d%% tgt=%d%% err=0x%02X",
                             modeStr[s_mode], (s_dir > 0) ? "F" : "R",
                             (unsigned)s_duty, (unsigned)ris, (unsigned)lis,
                             (long)(angle / 100), (long)frac,
                             (int)actPct, (int)s_throttlePct, (unsigned)errFlags);
                } else {
                    snprintf(out, sizeof(out),
                             "mode=%s dir=%s duty=%u RIS=%u LIS=%u pos=--- thr=---%% tgt=%d%% err=0x%02X",
                             modeStr[s_mode], (s_dir > 0) ? "F" : "R",
                             (unsigned)s_duty, (unsigned)ris, (unsigned)lis,
                             (int)s_throttlePct, (unsigned)errFlags);
                }
                printBoth(out);
            }
        }
    }
}
