/*
 * throttle_ecu.c — F280049C LaunchXL Throttle ECU  (PROTOTYPE / EXPERIMENTAL)
 *
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  WARNING — PROTOTYPE FIRMWARE — NOT FOR PRODUCTION USE                  ║
 * ║  This firmware is provided for development and evaluation purposes only. ║
 * ║  It has NOT been independently validated, safety-certified, or tested    ║
 * ║  for use in any safety-critical or road-going application.               ║
 * ║  Use at your own risk. Always maintain a physical kill-switch and        ║
 * ║  manual override accessible at all times during testing.                 ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * Changelog
 * ─────────
 * v2.2  2026-09-14  (PROTOTYPE)
 *   - SAFETY: setRelay() now enforces the ON permissive itself (refuses when
 *             mode==SAFE, safety module active, or brake pressed). Previously
 *             only the CAN path was gated; serial "on" bypassed all checks.
 *   - SAFETY: Setpoint and slew target park at CLOSED (s_usableMax) on boot
 *             and safe-state entry. Were parked at CFG_ANGLE_MIN (= fully open),
 *             causing an open-blip on the first PID engage after boot/RESET.
 *   - SAFETY: Stuck encoder line (SPI word 0xFFFF or 0x0000) is now rejected
 *             as invalid; EncoderGpio_getAngle() returns -1 instead of the
 *             rolling average so the fault is visible to safe_check_encoder().
 *   - SAFETY: RAM canary check gated on POST-done flag, not canary != 0, so a
 *             canary zeroed by SRAM corruption is detected.
 *   - BUGFIX: safe_clear_faults() clears FAULT_CAN_TIMEOUT / FAULT_WATCHDOG_RESET.
 *             Nothing else ever cleared them, so any safe state involving either
 *             bit could not be exited by RESET — power cycle only.
 *   - BUGFIX: CAN ESTOP frame refreshes the heartbeat; holding ESTOP > 200 ms
 *             no longer stacks a CAN_TIMEOUT fault on top.
 *   - BUGFIX: Convergence deadline re-armed on relay OFF→ON edge in PID mode.
 *             A target set before the relay was enabled could expire silently
 *             and trip "position not reached" the instant the relay came on.
 *   - Serial "on" reports "[SAFE] Relay ON refused" when the gate blocks it.
 *
 * v1.8  2026-09-13
 *   - SAFETY: Immediate safe state on SOL_OPEN detection (no motor noise while
 *             trying to reach unreachable %, previously let PID run for 2 s)
 *   - SAFETY: Position drift monitor — after convergence, if actual drifts
 *             >CFG_DRIFT_THRESH_PCT for >CFG_DRIFT_TIMEOUT_MS → safe state
 *   - BUGFIX: Solenoid interlock regression — was triggering safe state when
 *             relay is simply OFF (startup, post-RESET); now only fires on
 *             FAULT_SOL_OPEN (relay ON + open circuit)
 *   - BUGFIX: processCmd 't' branch missing s_targetReached/s_driftStart reset;
 *             could cause false drift fault mid-travel after serial command
 *   - BUGFIX: Brake PRESSED block missing drift/convergence state reset;
 *             could cause false drift fault if relay re-enabled post-brake
 *   - Flash linker script: .const scatter-load to SEC4|SEC8 (was overflowing)
 *   - Build serial: CFG_BUILD_SERIAL packed from __TIME__ for unique ID on boot
 *
 * v1.7  (previous release)
 *   - Convergence deadline check (CFG_CONV_TIMEOUT_MS / CFG_CONV_REACH_PCT)
 *   - Setpoint slew rate limiter (CFG_SLEW_RATE_MS / CFG_SLEW_STEP)
 *   - Spring return feed-forward (CFG_FF_SPRING_GAIN)
 *   - SOL current sense fault (FAULT_SOL_OPEN / FAULT_SOL_OC) with debounce
 *   - CAN heartbeat timeout (CFG_CAN_HEARTBEAT_EN / CFG_CAN_RX_TIMEOUT_MS)
 *   - Brake sense via ADCC_IN0 with hold-off/seq-gate anti-bounce
 *   - Encoder spike filter (CFG_ENC_SPIKE_THRESH / CFG_ENC_SPIKE_CONSEC)
 *   - Encoder GPIO pin-map corrected (pkg-pin 65=GPIO56, 66=57, 67=58, 92=59)
 */
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
#include "can_io.h"

typedef enum {
    MODE_MANUAL = 0,
    MODE_PID    = 1,
    MODE_SAFE   = 2
} RunMode;

static RunMode  s_mode         = MODE_MANUAL;
static int8_t   s_dir          = 1;
static uint16_t s_duty         = 0U;
/* Rest position for the setpoint is the CLOSED end (CFG_ANGLE_MAX — larger
 * angle = more closed on this sensor). Previously parked at CFG_ANGLE_MIN
 * (= 100% open), which made the first PID engage after boot or RESET start
 * with a "go fully open" error and blip the throttle open before the slew
 * limiter caught up. Refined to s_usableMax in Throttle_init(). */
static int32_t  s_targetAngle  = CFG_ANGLE_MAX;
static int16_t  s_throttlePct  = 0;

static float s_kp = CFG_KP_DEFAULT;
static float s_ki = CFG_KI_DEFAULT;
static float s_kd = CFG_KD_DEFAULT;

/* s_slewTarget — working angle that ramps toward s_targetAngle each slew tick.
 * The PID tracks s_slewTarget, not s_targetAngle directly. */
static int32_t  s_slewTarget  = (int32_t)CFG_ANGLE_MAX;  /* parked closed, see above */
static uint32_t s_lastSlew    = 0U;

static bool     s_settled     = false;
static uint32_t s_settleStart = 0U;

static uint32_t s_lastPrint    = 0U;
static uint32_t s_lastSafeTick = 0U;

static int32_t  s_motorCmd     = 0;   /* last command sent to setMotor() for telemetry */
static uint8_t  s_relayOn      = 0U;  /* relay state — motor must not run when relay is off */

/* ── Brake state machine ─────────────────────────────────────────────────── */
static uint8_t  s_brake_active   = 0U;  /* 1 while BRK_SENSE reads pressed          */
static uint8_t  s_brake_holdoff  = 0U;  /* 1 after release: ignore until fresh seq   */
static uint8_t  s_brake_last_seq = 0U;  /* CAN seq captured at moment of release     */
static uint8_t  s_last_rx_seq    = 0U;  /* most recent seq from any valid RX frame   */
#define CFG_BRAKE_DEBOUNCE_MS  20U      /* ignore transitions shorter than this      */
static uint32_t s_brake_debounce_start = 0U;
static uint8_t  s_brake_debounced      = 0U;  /* debounced brake state                */

static uint8_t  s_sol_interlock  = 0U;  /* 1 while solenoid-not-conducting interlock is active */

/* Convergence deadline: Board_millis() by which position must reach target.
 * Set when a new PID target is commanded; 0 = no active check. */
static uint32_t s_convDeadline  = 0U;

/* Position drift monitor: after target is reached, watch for unexpected movement.
 * s_targetReached: set when convergence succeeds; cleared on new target or safe state.
 * s_driftStart:   Board_millis() when drift was first detected; 0 = no drift. */
static uint8_t  s_targetReached = 0U;
static uint32_t s_driftStart    = 0U;

/* ISO26262: guard against miscalibration where MIN==MAX — angleToThrottle
 * would divide by zero. Minimum 10° (1000 units) of travel required. */
_Static_assert((int32_t)CFG_ANGLE_MAX - (int32_t)CFG_ANGLE_MIN >= 1000,
               "CFG_ANGLE_MAX - CFG_ANGLE_MIN too small: check calibration (min 10 deg range)");

static const int32_t s_usableMin  =
    (int32_t)CFG_ANGLE_MIN + ((int32_t)CFG_ANGLE_MAX - (int32_t)CFG_ANGLE_MIN) * 5 / 100;
static const int32_t s_usableMax  =
    (int32_t)CFG_ANGLE_MIN + ((int32_t)CFG_ANGLE_MAX - (int32_t)CFG_ANGLE_MIN) * 95 / 100;

/* setRelay — single point of control for the solenoid relay.
 *
 * ISO26262: the ON permissive is enforced HERE, not in the callers, so every
 * path (CAN frame, serial "on", any future caller) is gated identically —
 * same pattern as setMotor(). An ON request is refused when:
 *   - s_mode == MODE_SAFE                (local mode already SAFE)
 *   - g_safety.safe_state_active         (safety module tripped this tick but
 *                                         s_mode not yet switched — closes the
 *                                         one-loop window before line ~585)
 *   - s_brake_debounced != 0             (brake pedal pressed)
 * OFF requests are never refused. s_relayOn always reflects the pin.
 *
 * On an OFF→ON edge while already in PID mode, the convergence deadline is
 * re-armed. The convergence check only runs while the relay is on, so a
 * target commanded before the relay (serial "t50" then "on", or CAN PID=1
 * with RELAY=0 for > CFG_CONV_TIMEOUT_MS) would otherwise expire silently
 * and trip "position not reached" the instant the relay came on. */
static void setRelay(uint8_t on)
{
    uint8_t prev = s_relayOn;

    if (on != 0U) {
        if ((s_mode == MODE_SAFE) ||
            g_safety.safe_state_active ||
            (s_brake_debounced != 0U)) {
            on = 0U;
        }
    }
    s_relayOn = on ? 1U : 0U;
    Board_digitalRelay(s_relayOn);

    if ((prev == 0U) && (s_relayOn != 0U) && (s_mode == MODE_PID)) {
        s_convDeadline  = Board_millis() + (uint32_t)CFG_CONV_TIMEOUT_MS;
        s_targetReached = 0U;
        s_driftStart    = 0U;
    }
}

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

/* Sensor is inverted: smaller angle = more open.
 * 100% throttle = open = s_usableMin (~80.4°)
 *   0% throttle = closed = s_usableMax (~127.1°) */
static int32_t throttleToAngle(int16_t pct)
{
    int32_t p = (int32_t)pct;
    if (p < 0) {
        p = 0;
    }
    if (p > 100) {
        p = 100;
    }
    return s_usableMax - (int32_t)(((int64_t)(s_usableMax - s_usableMin) * p) / 100);
}

static int16_t angleToThrottle(int32_t angle)
{
    if (angle < s_usableMin) {
        return 100;   /* beyond open end — cap at 100% */
    }
    if (angle > s_usableMax) {
        return 0;     /* beyond closed end — cap at 0% */
    }
    return (int16_t)(100 - ((int64_t)(angle - s_usableMin) * 100) / (s_usableMax - s_usableMin));
}

static void setMotor(int32_t cmd)
{
    s_motorCmd = cmd;
    const int32_t pwmMax = (int32_t)CFG_PWM_MAX;

    if (s_mode == MODE_SAFE || !s_relayOn) {
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
    /* ISO26262: always go through the canonical safety module so that
     * g_safety.safe_state_active, g_safety.last_reason, and the transition
     * counter are updated regardless of which call path triggered safe state.
     * safe_enter_safe_state() is idempotent — safe to call when already active. */
    safe_enter_safe_state(reason);

    s_mode = MODE_SAFE;
    s_motorCmd = 0;                               /* ISO26262: clear stale cmd before CAN TX */
    MotorEPwm_setCommand(0, (int32_t)CFG_PWM_MAX);
    Board_digitalEnables(0U);
    setRelay(0U);
    Pid_reset();
    s_duty         = 0U;
    s_throttlePct  = 0;
    /* Park setpoint at CLOSED (s_usableMax), matching the brake path. The
     * throttle is spring-returned to closed while the motor is off, so the
     * next PID engage starts with ~zero error instead of "go fully open". */
    s_targetAngle  = s_usableMax;
    s_slewTarget   = s_usableMax;
    s_settled      = false;
    s_settleStart  = 0U;
    s_convDeadline  = 0U;
    s_targetReached = 0U;
    s_driftStart    = 0U;

    {
        char msg[72];
        snprintf(msg, sizeof(msg), "[SAFE] %s", (reason != NULL) ? reason : "?");
        printBoth(msg);
    }
}

void Throttle_CanRxApply(uint8_t flags, uint8_t throttle_pct, uint8_t seq)
{
    s_last_rx_seq = seq;   /* always track latest seq regardless of brake state */

    /* ISO26262: ESTOP is handled in CanIo_serviceRx() before this function is
     * called — the ESTOP branch in can_io.c calls safe_enter_safe_state() directly
     * and does NOT forward the frame to Throttle_CanRxApply(). Any ESTOP frame
     * therefore never reaches this point. No check needed here. */

    /* Allow CAN reset to exit safe state before the MODE_SAFE guard.
     * ISO26262: clear s_motorCmd before returning to MANUAL so the next CAN TX
     * does not replay a stale non-zero motor command in the telemetry byte. */
    if ((flags & CFG_CAN_FLAG_RESET) != 0U) {
        if (s_mode == MODE_SAFE) {
            safe_clear_faults();
            s_mode     = MODE_MANUAL;
            s_duty     = 0U;
            s_motorCmd = 0;           /* ISO26262: clear stale cmd before next telemetry TX */
            Board_digitalEnables(0U); /* bridge disabled — operator must send explicit PID/duty cmd */
            printBoth("Safe state cleared via CAN — mode=MAN");
        }
        return;
    }

    if (s_mode == MODE_SAFE) {
        return;
    }

    /* Post-brake holdoff: ignore frames until a new seq arrives.
     * This prevents the pre-brake throttle command from re-engaging
     * the moment the brake is released. */
    if (s_brake_holdoff != 0U) {
        if (seq == s_brake_last_seq) {
            return;   /* stale frame — same seq as when brake released */
        }
        s_brake_holdoff = 0U;   /* fresh seq received — holdoff cleared */
    }

    /* While brake is physically pressed: solenoid and throttle are owned by
     * the brake state machine in the main loop. Reject relay-on and all
     * throttle commands. ESTOP/RESET above still pass through. */
    if (s_brake_active != 0U) {
        /* Ensure relay stays off even if CAN tries to turn it on */
        setRelay(0U);
        return;
    }

    setRelay(((flags & CFG_CAN_FLAG_RELAY) != 0U) ? 1U : 0U);

    if ((flags & CFG_CAN_FLAG_PID) != 0U) {
        int16_t pct = (int16_t)throttle_pct;
        if (pct < 0) {
            pct = 0;
        }
        if (pct > 100) {
            pct = 100;
        }
        /* Only restart the convergence deadline when the target actually changes
         * or we are transitioning into PID mode. The CAN controller sends at 50Hz
         * continuously with the same target — resetting on every frame would push
         * the deadline forward forever and the check would never fire.
         * Also skip reset while sol interlock is active: the interlock overwrites
         * s_throttlePct to 0 each tick, which would make pct != s_throttlePct true
         * every frame and perpetually reset the deadline. */
        if ((pct != s_throttlePct || s_mode != MODE_PID) && (s_sol_interlock == 0U)) {
            s_convDeadline  = Board_millis() + (uint32_t)CFG_CONV_TIMEOUT_MS;
            s_targetReached = 0U;   /* new target — drift monitor resets */
            s_driftStart    = 0U;
            Pid_reset();
        }
        s_throttlePct  = pct;
        s_targetAngle  = throttleToAngle(s_throttlePct);
        s_mode         = MODE_PID;
        s_settled      = false;
        s_settleStart  = 0U;
    } else {
        s_duty = 0U;
        s_mode = MODE_MANUAL;
        Pid_reset();
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
            s_mode     = MODE_MANUAL;
            s_duty     = 0U;
            s_motorCmd = 0;           /* ISO26262: clear stale cmd before next telemetry TX */
            Board_digitalEnables(0U); /* ISO26262: bridge DISABLED after reset — operator must
                                       * send an explicit d## or PID command to re-engage motor.
                                       * Previously (1U) re-enabled bridge immediately on reset,
                                       * inconsistent with CAN RESET path which uses (0U). */
            printBoth("Safe state cleared — mode=MAN");
        }
    } else if (str_eq_ic(s, "on")) {
        setRelay(1U);
        /* setRelay() may refuse (safe state / brake) — report the real result */
        printBoth((s_relayOn != 0U) ? "Relay ON" : "[SAFE] Relay ON refused");
    } else if (str_eq_ic(s, "off")) {
        setRelay(0U);
        setMotor(0);
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
            s_throttlePct   = (int16_t)pct;
            s_targetAngle   = throttleToAngle(s_throttlePct);
            s_mode          = MODE_PID;
            s_settled       = false;
            s_settleStart   = 0U;
            s_convDeadline  = Board_millis() + (uint32_t)CFG_CONV_TIMEOUT_MS;
            s_targetReached = 0U;
            s_driftStart    = 0U;
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
        /* ISO26262: clamp gains to sane range — unclamped values cause immediate
         * PWM saturation and violent uncontrolled throttle motion. */
        float v = (float)strtod(s + 1, NULL);
        if (v < 0.0f) { v = 0.0f; }
        if (v > 200.0f) { v = 200.0f; }
        s_kp = v;
        {
            char buf[32];
            snprintf(buf, sizeof(buf), "Kp=%.2f", (double)s_kp);
            printBoth(buf);
        }
    } else if ((s[0] == 'i') || (s[0] == 'I')) {
        float v = (float)strtod(s + 1, NULL);
        if (v < 0.0f) { v = 0.0f; }
        if (v > 50.0f) { v = 50.0f; }
        s_ki = v;
        {
            char buf[32];
            snprintf(buf, sizeof(buf), "Ki=%.2f", (double)s_ki);
            printBoth(buf);
        }
    } else if ((s[0] == 'k') || (s[0] == 'K')) {
        float v = (float)strtod(s + 1, NULL);
        if (v < 0.0f) { v = 0.0f; }
        if (v > 100.0f) { v = 100.0f; }
        s_kd = v;
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
    CanIo_init();

    setMotor(0);

    printBoth("=================================");
    printBoth("  Throttle ECU — C2000 F280049");
    {
        char vbuf[48];
        snprintf(vbuf, sizeof(vbuf), "  FW v%u.%u  Serial: 0x%06lX",
                 (unsigned)CFG_FW_VERSION_MAJOR,
                 (unsigned)CFG_FW_VERSION_MINOR,
                 (unsigned long)CFG_BUILD_SERIAL);
        printBoth(vbuf);
    }
    printBoth("=================================");
    /* Park setpoint and slew target at CLOSED (0%) — consistent with
     * s_throttlePct = 0 and with the spring-returned rest position. */
    s_targetAngle = s_usableMax;
    s_slewTarget  = s_usableMax;
    printBoth("Commands: t0-t100, d0-d4095, f, r, s");
    printBoth("         p## i## k##, on, off, reset");
    printBoth("         diag, clearfaults, config");
    {
        char cbuf[96];
        snprintf(cbuf, sizeof(cbuf),
                 "CAN: cmd 0x%03lX telem 0x%03lX %lukbps GPIO%u/%u",
                 (unsigned long)CFG_CAN_RX_ID, (unsigned long)CFG_CAN_TX_ID,
                 (unsigned long)(CFG_CAN_BITRATE / 1000UL),
                 (unsigned)CFG_CAN_RX_PIN, (unsigned)CFG_CAN_TX_PIN);
        printBoth(cbuf);
    }

    if (safe_was_reset_by_watchdog()) {
        printBoth("[WARN] Recovered from watchdog reset!");
    }

    s_lastPrint    = Board_millis();
    s_lastSafeTick = Board_millis();

    /* ISO26262: POST runs last, after all hardware is initialised and startup
     * messages are printed, so the PASS/FAIL lines appear at the end of boot
     * output. Failure enters safe state immediately and halts control. */
    safe_post(printBoth);
}

void Throttle_runOnce(void)
{
    /* ISO26262: loop timing monitor — record start time so we can detect if
     * any single iteration runs longer than CFG_LOOP_MAX_MS (50ms). The
     * hardware WDT handles a completely stuck loop; this catches soft overruns
     * (e.g. ADC timeout spinning, blocking UART). Counter visible via "diag". */
    uint32_t s_loopStart = Board_millis();

    /* Kick hardware WDT every iteration (~few ms apart) so the 840ms timeout
     * can only fire if the CPU is completely halted, not merely slow. */
    SysCtl_serviceWatchdog();

    char line[64];

    if (SciIo_readLine(line, sizeof(line))) {
        processCmd(line);
    }

    {
        int32_t  angle = EncoderGpio_getAngle();
        uint32_t now   = Board_millis();

        uint16_t ris = 0U;
        uint16_t lis = 0U;
        uint16_t sol = 0U;
        uint16_t brk_raw = 0U;
        AdcSense_readCurrents(&ris, &lis);
        AdcSense_readSolenoid(&sol);
        AdcSense_readBrake(&brk_raw);
        uint8_t brake_raw = (brk_raw >= CFG_ADC_BRK_THRESH) ? 1U : 0U;

        /* Debounce brake sensor: require CFG_BRAKE_DEBOUNCE_MS stable before transition */
        if (brake_raw != s_brake_debounced) {
            if (s_brake_debounce_start == 0U) {
                s_brake_debounce_start = now;
            } else if ((now - s_brake_debounce_start) >= CFG_BRAKE_DEBOUNCE_MS) {
                s_brake_debounced      = brake_raw;
                s_brake_debounce_start = 0U;
            }
        } else {
            s_brake_debounce_start = 0U;
        }

        CanIo_serviceRx(now);  /* update s_last_rx_seq before brake state machine */

        /* ── Brake state machine ─────────────────────────────────────────── */
        if (s_brake_debounced != 0U) {
            /* BRAKE PRESSED ------------------------------------------------ */
            if (s_brake_active == 0U) {
                s_brake_active = 1U;  /* rising edge */
            }
            /* Solenoid must be off */
            if (s_relayOn != 0U) {
                setRelay(0U);
            }
            /* Drive motor to 0% (fully closed) — bypass slew for immediacy.
             * Only override if not already in safe state (safe state owns motor). */
            if (s_mode != MODE_SAFE) {
                if (s_mode != MODE_PID) {
                    Pid_reset();
                    s_mode = MODE_PID;
                }
                s_throttlePct   = 0;
                s_targetAngle   = s_usableMax;
                s_slewTarget    = s_usableMax;  /* skip slew — immediate close */
                s_settled       = false;
                s_targetReached = 0U;   /* disarm drift monitor — new commanded target */
                s_driftStart    = 0U;
                s_convDeadline  = 0U;
            }
        } else {
            /* BRAKE RELEASED ----------------------------------------------- */
            if (s_brake_active != 0U) {
                /* Falling edge: enter holdoff so stale CAN frames are rejected.
                 * s_last_rx_seq was just updated by CanIo_serviceRx above.    */
                s_brake_active  = 0U;
                s_brake_holdoff = 1U;
                s_brake_last_seq = s_last_rx_seq;
                /* Drop to MANUAL so throttle doesn't re-engage until operator
                 * explicitly sends a fresh PID command.                        */
                s_mode        = MODE_MANUAL;
                s_throttlePct = 0;
                s_duty        = 0U;
                Pid_reset();
            }
        }

        safe_check_encoder(angle >= 0, now);
        safe_check_current(ris, lis, now);
        safe_check_solenoid(sol, s_relayOn);
        /* nFAULT check disabled — GPIO1 picks up PWM switching noise; motor confirmed working. */

        if (g_safety.safe_state_active && (s_mode != MODE_SAFE)) {
            enterSafeStateEc(g_safety.last_reason);
        }

        /* Solenoid interlock: if relay is ON but solenoid current is absent
         * (FAULT_SOL_OPEN = open circuit / disconnected), the H-bridge can still
         * drive the motor → enter safe state immediately.
         *
         * Relay simply being OFF is NOT a fault here — setMotor() already blocks
         * the H-bridge when relay is off, so no safe state is needed in that case.
         * Treating relay-OFF as a fault would trigger safe state on every boot
         * and after every RESET before the operator enables the relay. */
        if (s_mode != MODE_SAFE) {
            uint8_t sol_open = (s_relayOn != 0U) &&
                               ((g_safety.sol_faults & FAULT_SOL_OPEN) != 0U);
            if (sol_open != 0U) {
                if (s_sol_interlock == 0U) {
                    s_sol_interlock = 1U;
                    g_safety.sol_faults_latched |= FAULT_SOL_OPEN;
                    enterSafeStateEc("sol open");
                }
            } else {
                s_sol_interlock = 0U;   /* relay OFF or fault cleared — reset */
            }
        }

        if ((now - s_lastSafeTick) >= 100U) {
            s_lastSafeTick = now;
            safe_tick(now);
        }

        /* ── Setpoint slew: ramp s_slewTarget toward s_targetAngle ─────────── */
#if CFG_SLEW_STEP > 0
        if ((now - s_lastSlew) >= CFG_SLEW_RATE_MS) {
            s_lastSlew = now;
            int32_t diff = s_targetAngle - s_slewTarget;
            if (diff > (int32_t)CFG_SLEW_STEP) {
                s_slewTarget += (int32_t)CFG_SLEW_STEP;
                /* ISO26262: reset integral every slew step to prevent windup
                 * accumulating against the moving target. The integral
                 * re-accumulates from zero once slew completes. Derivative
                 * memory (s_prevE) and timing (s_lastMs) are preserved so
                 * derivative action remains effective during the ramp. */
                Pid_resetIntegral();
            } else if (diff < -(int32_t)CFG_SLEW_STEP) {
                s_slewTarget -= (int32_t)CFG_SLEW_STEP;
                Pid_resetIntegral();  /* ISO26262: anti-windup during slew */
            } else {
                s_slewTarget = s_targetAngle;
            }
        }
#else
        s_slewTarget = s_targetAngle;
#endif

        switch (s_mode) {
        case MODE_PID:
            if (angle >= 0) {
                int32_t clamped = s_slewTarget;
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
                                         s_kp, s_ki, s_kd, CFG_PID_INTEGRAL_LIMIT, now);
                    }

                    /* Spring return feed-forward: apply a holding force proportional
                     * to how open the throttle is (smaller angle = more open = more spring).
                     * FF is always negative (toward open) to oppose the return spring.
                     * Applied even when settled so the motor holds against spring load. */
#if CFG_FF_SPRING_GAIN > 0
                    {
                        int32_t range = s_usableMax - s_usableMin;
                        if (range > 0) {
                            /* At angle=usableMin (100% open) → ff = -CFG_FF_SPRING_GAIN
                             * At angle=usableMax (0% closed)  → ff = 0 */
                            int32_t ff = -((int32_t)CFG_FF_SPRING_GAIN *
                                           (s_usableMax - angle)) / range;
                            pidCmd += ff;
                        }
                    }
#endif
                    setMotor(pidCmd);
                }

                /* ── Convergence check ──────────────────────────────────────────
                 * Did actual position reach within CFG_CONV_REACH_PCT of the
                 * commanded target within CFG_CONV_TIMEOUT_MS of the target being
                 * set? If not → motor is disconnected/stalled → enter safe state. */
                if ((s_convDeadline != 0U) && (s_relayOn != 0U)) {
                    int16_t actualPct = angleToThrottle(angle);
                    int16_t errPct    = actualPct - s_throttlePct;
                    if (errPct < 0) { errPct = -errPct; }
                    if ((int32_t)errPct <= (int32_t)CFG_CONV_REACH_PCT) {
                        s_convDeadline  = 0U;   /* reached — disable until next target */
                        s_targetReached = 1U;   /* arm drift monitor */
                        s_driftStart    = 0U;
                    } else if (now > s_convDeadline) {
                        g_safety.sol_faults        |= FAULT_POSITION_ERROR;
                        g_safety.sol_faults_latched |= FAULT_POSITION_ERROR;
                        enterSafeStateEc("position not reached");
                    }
                }

                /* ── Position drift monitor ──────────────────────────────────────
                 * After the target has been reached (s_targetReached), watch for
                 * unexpected movement — e.g. motor driver runaway, mechanical slip.
                 * If |actual - commanded| > CFG_DRIFT_THRESH_PCT for
                 * CFG_DRIFT_TIMEOUT_MS → safe state. */
                if ((s_targetReached != 0U) && (s_relayOn != 0U)) {
                    int16_t actualPct = angleToThrottle(angle);
                    int16_t driftPct  = actualPct - s_throttlePct;
                    if (driftPct < 0) { driftPct = -driftPct; }
                    if ((int32_t)driftPct > (int32_t)CFG_DRIFT_THRESH_PCT) {
                        if (s_driftStart == 0U) {
                            s_driftStart = now;
                        } else if ((now - s_driftStart) >= (uint32_t)CFG_DRIFT_TIMEOUT_MS) {
                            g_safety.sol_faults        |= FAULT_POSITION_ERROR;
                            g_safety.sol_faults_latched |= FAULT_POSITION_ERROR;
                            enterSafeStateEc("position drift");
                        }
                    } else {
                        s_driftStart = 0U;  /* back in range — reset timer */
                    }
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

        default:
            /* ISO26262: s_mode holds a value not in {MODE_MANUAL, MODE_PID,
             * MODE_SAFE} — most likely caused by SRAM bit-flip or stack
             * corruption. Enter safe state immediately rather than leaving the
             * motor in its last commanded state (which could be non-zero). */
            enterSafeStateEc("invalid mode");
            break;
        }

        {
            uint8_t actSpi = 0xFFU;
            if (angle >= 0) {
                int16_t ap = angleToThrottle(angle);
                if (ap < 0) {
                    ap = 0;
                }
                if (ap > 100) {
                    ap = 100;
                }
                actSpi = (uint8_t)ap;
            }
            /* raw_angle_hundredths: 0xFFFF when no valid angle */
            uint16_t rawHundredths = (angle >= 0)
                ? (uint16_t)((uint32_t)angle & 0xFFFFU)
                : 0xFFFFU;
            {
                /* sol_status byte[6] of 0x102: fault bits + inferred-on indicator */
                uint8_t solSt = safe_get_sol_faults();
                if (sol >= (uint16_t)CFG_SOL_ON_THRESH) {
                    solSt |= 0x08U;  /* bit3: SOL_INFERRED_ON — current above threshold */
                }
                CanIo_serviceTx(now, (uint8_t)s_mode, actSpi,
                                (uint8_t)s_throttlePct, safe_get_fault_flags(),
                                (int16_t)s_motorCmd, s_relayOn, rawHundredths,
                                s_brake_debounced);
                CanIo_serviceTx2(now, ris, lis, sol, solSt);
            }
        }

        if ((now - s_lastPrint) >= CFG_TELEMETRY_RATE_MS) {
            s_lastPrint = now;

            int16_t actPct = -1;
            if (angle >= 0) {
                actPct = angleToThrottle(angle);
            }

            {
                uint16_t errFlags = (uint16_t)safe_get_fault_flags();
                static const char *modeStr[] = {"MAN", "PID", "SAFE"};
                char out[128];

                if (angle >= 0) {
                    int32_t frac = angle % 100;
                    if (frac < 0) {
                        frac = -frac;
                    }
                    snprintf(out, sizeof(out),
                             "mode=%s cmd=%ld RIS=%u LIS=%u pos=%ld.%02ld thr=%d%% tgt=%d%% err=0x%02X",
                             modeStr[s_mode], (long)s_motorCmd,
                             (unsigned)ris, (unsigned)lis,
                             (long)(angle / 100), (long)frac,
                             (int)actPct, (int)s_throttlePct, (unsigned)errFlags);
                } else {
                    snprintf(out, sizeof(out),
                             "mode=%s cmd=%ld RIS=%u LIS=%u pos=--- thr=---%% tgt=%d%% err=0x%02X",
                             modeStr[s_mode], (long)s_motorCmd,
                             (unsigned)ris, (unsigned)lis,
                             (int)s_throttlePct, (unsigned)errFlags);
                }
                printBoth(out);
            }
        }
    }

    /* ISO26262: loop overrun check — measure wall-clock cost of this iteration.
     * Increment counter if > CFG_LOOP_MAX_MS (50ms). The counter is shown in
     * "diag" output and transmitted via CAN (visible in GUI log).
     * A non-zero count indicates ADC blocking, UART stall, or encoder SPI hang. */
    {
        uint32_t loopDur = Board_millis() - s_loopStart;
        if (loopDur > (uint32_t)CFG_LOOP_MAX_MS) {
            g_safety.loop_overrun_count++;
        }
    }
}
