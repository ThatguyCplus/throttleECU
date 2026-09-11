#include "driverlib.h"
#include "device.h"
#include <stdio.h>
#include <string.h>
#include "safety.h"
#include "board.h"
#include "throttle_config.h"

#if CFG_CAN_HEARTBEAT_EN
static uint32_t s_canLastRxMs      = 0U;
static uint8_t  s_canHbArmed     = 0U;
#endif

SafetyState g_safety;

#ifndef SYSCTL_CAUSE_WDRS
#define SYSCTL_CAUSE_WDRS  (1UL << 6)
#endif

void safe_init(void)
{
    memset(&g_safety, 0, sizeof(g_safety));

    {
        uint32_t rc = SysCtl_getResetCause();
        if ((rc & SYSCTL_CAUSE_WDRS) != 0U) {
            g_safety.faults |= FAULT_WATCHDOG_RESET;
            g_safety.faults_latched |= FAULT_WATCHDOG_RESET;
        }
        SysCtl_clearResetCause(rc);
    }

    /* Enable hardware watchdog (~840 ms timeout at INTOSC2/512/64/256).
     * If the main loop stops running for any reason (ESTOP0, hang, etc.)
     * the WDT resets the MCU, which boots with relay and bridge off. */
    SysCtl_setWatchdogPrescaler(SYSCTL_WD_PRESCALE_64);
    SysCtl_enableWatchdog();
    SysCtl_serviceWatchdog();
}

void safe_kick_watchdog(void)
{
    g_safety.watchdog_kick_count++;
    SysCtl_serviceWatchdog();   /* reset hardware WDT countdown */
}

bool safe_was_reset_by_watchdog(void)
{
    return (g_safety.faults_latched & FAULT_WATCHDOG_RESET) != 0U;
}

void safe_check_encoder(bool is_valid, uint32_t now_ms)
{
    if (is_valid) {
        g_safety.last_encoder_update = now_ms;
        g_safety.fault_count[0] = 0U;
        if (!g_safety.safe_state_active) {
            g_safety.faults &= (uint8_t)(~(FAULT_ENCODER_STALE | FAULT_ENCODER_INVALID));
        }
    } else {
        if ((now_ms - g_safety.last_encoder_update) > CFG_ENCODER_TIMEOUT_MS) {
            if (g_safety.fault_count[0] < CFG_ENCODER_DEBOUNCE) {
                g_safety.fault_count[0]++;
                if (g_safety.fault_count[0] >= CFG_ENCODER_DEBOUNCE) {
                    g_safety.faults |= (FAULT_ENCODER_STALE | FAULT_ENCODER_INVALID);
                    g_safety.faults_latched |= FAULT_ENCODER_INVALID;
                    g_safety.encoder_timeouts++;
                    safe_enter_safe_state("Encoder timeout");
                }
            }
        }
    }
}

void safe_check_current(uint16_t ris, uint16_t lis, uint32_t now_ms)
{
    (void)now_ms;

    if (lis > CFG_OVERCURRENT_THRESH) {
        if (g_safety.fault_count[1] < CFG_OVERCURRENT_DEBOUNCE) {
            g_safety.fault_count[1]++;
            if (g_safety.fault_count[1] >= CFG_OVERCURRENT_DEBOUNCE) {
                g_safety.faults |= FAULT_OVERCURRENT_L;
                g_safety.faults_latched |= FAULT_OVERCURRENT_L;
                g_safety.overcurrent_events++;
                safe_enter_safe_state("Overcurrent L");
            }
        }
    } else {
        g_safety.fault_count[1] = 0U;
        if (!g_safety.safe_state_active) {
            g_safety.faults &= (uint8_t)(~FAULT_OVERCURRENT_L);
        }
    }

    if (ris > CFG_OVERCURRENT_THRESH) {
        if (g_safety.fault_count[2] < CFG_OVERCURRENT_DEBOUNCE) {
            g_safety.fault_count[2]++;
            if (g_safety.fault_count[2] >= CFG_OVERCURRENT_DEBOUNCE) {
                g_safety.faults |= FAULT_OVERCURRENT_R;
                g_safety.faults_latched |= FAULT_OVERCURRENT_R;
                g_safety.overcurrent_events++;
                safe_enter_safe_state("Overcurrent R");
            }
        }
    } else {
        g_safety.fault_count[2] = 0U;
        if (!g_safety.safe_state_active) {
            g_safety.faults &= (uint8_t)(~FAULT_OVERCURRENT_R);
        }
    }
}

/* safe_check_power() — supply voltage under-voltage check.
 *
 * ISO26262 NOTE: This function is defined and ready but is NOT currently called
 * from throttle_ecu.c because the current PCB revision does not have a supply-
 * voltage sense line wired to any ADC input. All four ADCA/B/C channels are
 * allocated to IPROPI1, IPROPI2, SOL_CS_CURRENT, and BRK_SENSE.
 *
 * To activate: connect VM or a resistor-divided version of the supply rail to
 * a spare ADC pin, add an AdcSense_readSupply() helper in adc_sense.c, then
 * call safe_check_power(supply_mv) inside Throttle_runOnce() after the other
 * safe_check_*() calls.
 *
 * Until then, FAULT_POWER_LOW will never be raised. This is an accepted residual
 * risk — documented in FMEA. */
void safe_check_power(uint16_t supply_mv)
{
    if (supply_mv < CFG_POWER_LOW_MV) {
        g_safety.fault_count[3]++;
        if (g_safety.fault_count[3] >= CFG_POWER_DEBOUNCE) {
            g_safety.faults |= FAULT_POWER_LOW;
            g_safety.faults_latched |= FAULT_POWER_LOW;
            g_safety.power_low_events++;
            safe_enter_safe_state("Power low");
        }
    } else {
        g_safety.fault_count[3] = 0U;
        if (!g_safety.safe_state_active) {
            g_safety.faults &= (uint8_t)(~FAULT_POWER_LOW);
        }
    }
}

void safe_enter_safe_state(const char *reason)
{
    if (g_safety.safe_state_active) {
        return;
    }

    g_safety.safe_state_active = true;
    g_safety.safe_state_entered = Board_millis();
    g_safety.safe_transitions++;

    if (reason != NULL) {
        /* ISO26262: strncpy does not guarantee NUL termination when src length
         * equals (sizeof(dst) - 1). The explicit NUL write on the next line
         * ensures last_reason is always NUL-terminated regardless of src length.
         * This satisfies MISRA C:2012 Rule 21.14 and ISO 26262 data integrity
         * requirements for diagnostic strings transmitted over CAN/UART. */
        strncpy(g_safety.last_reason, reason, sizeof(g_safety.last_reason) - 1U);
        g_safety.last_reason[sizeof(g_safety.last_reason) - 1U] = '\0';  /* explicit NUL — ISO26262 compliant */
    }
}

bool safe_can_recover(void)
{
    if (!g_safety.safe_state_active) {
        return false;
    }
    {
        uint32_t elapsed = Board_millis() - g_safety.safe_state_entered;
        return (g_safety.faults == 0U) && (elapsed > CFG_RECOVERY_DELAY_MS);
    }
}

void safe_attempt_recovery(void)
{
    if (safe_can_recover()) {
        /* ISO26262: auto-recovery path analysis (2026-09-11).
         *
         * Setting safe_state_active=false here does NOT automatically re-engage
         * the throttle. The RunMode (s_mode) in throttle_ecu.c remains MODE_SAFE
         * after this call because:
         *   1. enterSafeStateEc() set s_mode = MODE_SAFE when safe state was entered.
         *   2. safe_attempt_recovery() only clears safe_state_active — it does NOT
         *      modify s_mode.
         *   3. In throttle_ecu.c the guard `if (g_safety.safe_state_active && ...)`
         *      will no longer call enterSafeStateEc() (because active=false), but
         *      s_mode stays MODE_SAFE.
         *   4. The switch(s_mode) case MODE_SAFE calls setMotor(0) — motor stays off.
         *   5. The only way out of MODE_SAFE is an explicit CAN RESET frame (flag
         *      CFG_CAN_FLAG_RESET) or UART "reset" command, which calls
         *      safe_clear_faults() and sets s_mode = MODE_MANUAL.
         *
         * This means: after CAN timeout recovers and auto-recovery fires, the
         * throttle DOES NOT re-engage at the previous target. The operator must
         * send an explicit RESET frame followed by a fresh PID command. This is
         * intentional and safe by design — it prevents automatic re-engagement
         * after a reconnect without operator intent.
         *
         * ASIL rationale: satisfies ISO 26262 requirement for recovery to require
         * positive operator action (no automatic re-engagement after loss of control). */
        g_safety.safe_state_active = false;
        g_safety.recovery_attempts++;
        g_safety.last_encoder_update = Board_millis();
    }
}

void safe_clear_faults(void)
{
    /* ISO26262: do not clear active (non-latched) faults or debounce counters
     * while underlying conditions still hold. The safety checks will re-set
     * faults immediately in the next tick if conditions persist, but clearing
     * the debounce counter resets the hysteresis and allows one tick of
     * undetected fault exposure. Only clear when g_safety.faults is already 0
     * (i.e. checks have naturally cleared the active condition).
     * Latched faults are always cleared — they are historical records only. */
    if (g_safety.faults == 0U) {
        memset(g_safety.fault_count, 0, sizeof(g_safety.fault_count));
        g_safety.safe_state_active = false;
#if CFG_CAN_HEARTBEAT_EN
        s_canHbArmed = 0U;
#endif
    }
    /* Always clear latched/historical records so operator can acknowledge */
    g_safety.faults_latched     = 0U;
    g_safety.sol_faults         = 0U;
    g_safety.sol_faults_latched = 0U;
}

void safe_tick(uint32_t now_ms)
{
    (void)now_ms;
    safe_kick_watchdog();

    /* ISO26262: RAM canary check — detect SRAM corruption between POST and now.
     * The canary word is written once by safe_post() and should never change.
     * A corrupted value indicates SRAM bit-flip, stack overflow into the struct,
     * or a C bug writing to the wrong address. Enter safe state immediately. */
    if ((g_safety.ram_canary != 0U) &&
        (g_safety.ram_canary != SAFETY_RAM_CANARY)) {
        g_safety.post_result |= POST_CANARY_FAIL;
        safe_enter_safe_state("RAM canary corrupt");
    }

    safe_attempt_recovery();
}

uint8_t safe_post(void (*print_fn)(const char *))
{
    uint8_t result = 0U;
    char    buf[80];

    if (print_fn != NULL) { print_fn("[POST] Power-On Self-Test..."); }

    /* ── Test 1: RAM march test ──────────────────────────────────────────────
     * Write four alternating patterns to a scratchpad region and verify
     * readback. Catches stuck-at-1, stuck-at-0, and data-coupling faults.
     *
     * volatile: prevents the C compiler from eliminating the stores as dead
     * writes since the variable is never read outside this function.
     * C28x note: uint16_t is the native word; 32 words = 64 bytes of SRAM.
     * static: placed in .bss (global SRAM), not on the stack, so the test is
     * genuinely exercising SRAM rather than the current stack frame. */
    {
        static volatile uint16_t scratch[32];
        const uint16_t patterns[4] = {0x5555U, 0xAAAAU, 0x0000U, 0xFFFFU};
        uint16_t p, i;
        uint8_t  ramOk = 1U;

        for (p = 0U; p < 4U; p++) {
            for (i = 0U; i < 32U; i++) { scratch[i] = patterns[p]; }
            for (i = 0U; i < 32U; i++) {
                if (scratch[i] != patterns[p]) {
                    ramOk = 0U;
                    break;
                }
            }
            if (ramOk == 0U) { break; }
        }

        if (ramOk != 0U) {
            if (print_fn != NULL) { print_fn("[POST] RAM march:   PASS"); }
        } else {
            result |= POST_RAM_FAIL;
            if (print_fn != NULL) { print_fn("[POST] RAM march:   FAIL ***"); }
        }
    }

    /* ── Test 2: Configuration range sanity ──────────────────────────────────
     * Verify that calibration constants in throttle_config.h are within
     * physically possible bounds. Static asserts catch MIN==MAX at compile
     * time; this catches plausible-but-wrong runtime values (e.g. limits
     * accidentally swapped after recalibration, or PWM range zeroed out).
     * No hardware access — pure arithmetic. */
    {
        uint8_t cfgOk = 1U;

        /* Angle limits: MIN < MAX, and MAX ≤ 35999 (0.01° units, 360° max) */
        if ((int32_t)CFG_ANGLE_MIN >= (int32_t)CFG_ANGLE_MAX) { cfgOk = 0U; }
        if ((uint32_t)CFG_ANGLE_MAX > 35999U)                  { cfgOk = 0U; }

        /* PWM: non-zero, fits in 12-bit ePWM compare register */
        if (CFG_PWM_MAX == 0U)                                  { cfgOk = 0U; }
        if ((uint32_t)CFG_PWM_MAX > 4095U)                     { cfgOk = 0U; }

        /* PID: positive Kp and integral limit required for stable control */
        if (CFG_KP_DEFAULT <= 0.0f)                             { cfgOk = 0U; }
        if (CFG_PID_INTEGRAL_LIMIT <= 0.0f)                    { cfgOk = 0U; }

        /* Solenoid thresholds: ON < OC (inverted = nonsensical) */
        if ((uint32_t)CFG_SOL_ON_THRESH >= (uint32_t)CFG_SOL_OC_THRESH) { cfgOk = 0U; }

        /* Heartbeat timeout must be longer than TX rate (otherwise always trips) */
        if ((uint32_t)CFG_CAN_RX_TIMEOUT_MS <= (uint32_t)CFG_CAN_TX_RATE_MS) { cfgOk = 0U; }

        if (cfgOk != 0U) {
            if (print_fn != NULL) { print_fn("[POST] Config check: PASS"); }
        } else {
            result |= POST_CFG_FAIL;
            if (print_fn != NULL) { print_fn("[POST] Config check: FAIL *** check throttle_config.h"); }
        }
    }

    /* ── Test 3: Plant RAM canary ────────────────────────────────────────────
     * Write the known-good pattern. safe_tick() verifies this every 100ms.
     * If it changes, safe_tick() sets POST_CANARY_FAIL and enters safe state. */
    g_safety.ram_canary = SAFETY_RAM_CANARY;
    if (print_fn != NULL) { print_fn("[POST] RAM canary:  planted (checked every 100ms)"); }

    /* ── Final summary ───────────────────────────────────────────────────── */
    g_safety.post_result = result;
    if (result == 0U) {
        if (print_fn != NULL) { print_fn("[POST] ALL PASS"); }
    } else {
        snprintf(buf, sizeof(buf), "[POST] FAILED (0x%02X) — safe state entered", (unsigned)result);
        if (print_fn != NULL) { print_fn(buf); }
        safe_enter_safe_state("POST failed");
    }
    return result;
}

const char *safe_fault_name(uint8_t fault_bit)
{
    switch (fault_bit) {
    case FAULT_ENCODER_STALE:
        return "ENCODER_STALE";
    case FAULT_ENCODER_INVALID:
        return "ENCODER_INVALID";
    case FAULT_OVERCURRENT_L:
        return "OVERCURR_L";
    case FAULT_OVERCURRENT_R:
        return "OVERCURR_R";
    case FAULT_POWER_LOW:
        return "POWER_LOW";
    case FAULT_WATCHDOG_RESET:
        return "WATCHDOG_RST";
    case FAULT_CAN_TIMEOUT:
        return "CAN_TIMEOUT";
    case FAULT_CAN_BUS_OFF:
        return "CAN_BUS_OFF";
    default:
        return "UNKNOWN";
    }
}

void safe_can_mark_rx(uint32_t now_ms)
{
#if CFG_CAN_HEARTBEAT_EN
    s_canLastRxMs  = now_ms;
    s_canHbArmed   = 1U;
#else
    (void)now_ms;
#endif
}

void safe_can_check_timeout(uint32_t now_ms)
{
#if CFG_CAN_HEARTBEAT_EN
    if ((s_canHbArmed == 0U) || g_safety.safe_state_active) {
        return;
    }
    if ((now_ms - s_canLastRxMs) > CFG_CAN_RX_TIMEOUT_MS) {
        g_safety.faults |= FAULT_CAN_TIMEOUT;
        g_safety.faults_latched |= FAULT_CAN_TIMEOUT;
        safe_enter_safe_state("CAN heartbeat timeout");
    }
#else
    (void)now_ms;
#endif
}

void safe_can_set_bus_off_fault(void)
{
    g_safety.faults |= FAULT_CAN_BUS_OFF;
    g_safety.faults_latched |= FAULT_CAN_BUS_OFF;
    safe_enter_safe_state("CAN bus-off");
}

void safe_can_clear_bus_off_fault(void)
{
    if (!g_safety.safe_state_active) {
        g_safety.faults &= (uint8_t)(~FAULT_CAN_BUS_OFF);
    }
}

uint8_t safe_get_fault_flags(void)
{
    return g_safety.faults;
}

uint8_t safe_get_sol_faults(void)
{
    return g_safety.sol_faults;
}

/* DRV8873H nFAULT check — diagnostic only, does NOT enter safe state.
 * nFAULT asserts LOW for UVLO, OCP, OTS, OTW. These conditions are shown in the
 * GUI via sol_status bit4 (DRV_NFAULT=0x10) for operator awareness.
 * Safe-state entry removed because UVLO at idle (VM absent) is not a run-time fault
 * and OCP auto-retry on the DRV8873H recovers without intervention. */
#define CFG_DRV_NFAULT_DEBOUNCE  10U
void safe_check_drv_nfault(uint8_t nfault_asserted)
{
    if (nfault_asserted != 0U) {
        if (g_safety.fault_count[7] < CFG_DRV_NFAULT_DEBOUNCE) {
            g_safety.fault_count[7]++;
        }
        if (g_safety.fault_count[7] >= CFG_DRV_NFAULT_DEBOUNCE) {
            /* Set warning flag for GUI display — no safe state entry */
            g_safety.sol_faults        |= FAULT_DRV_NFAULT;
            g_safety.sol_faults_latched |= FAULT_DRV_NFAULT;
        }
    } else {
        g_safety.fault_count[7] = 0U;
        g_safety.sol_faults &= (uint8_t)(~FAULT_DRV_NFAULT);
    }
}

/* Solenoid current check — simple on/off confirmation only.
 * When relay is commanded ON, verify current is above the ON threshold.
 * SOL_OPEN = relay ON but no current detected (open circuit / not connected).
 * All other SOL fault checks removed — current sense is reference/diagnostic only.
 * fault_count[4] = SOL_OPEN debounce
 */
void safe_check_solenoid(uint16_t sol_raw, uint8_t relay_on)
{
    if (relay_on != 0U) {
        if (sol_raw < (uint16_t)CFG_SOL_ON_THRESH) {
            if (g_safety.fault_count[4] < (uint8_t)CFG_SOL_FAULT_DEBOUNCE) {
                g_safety.fault_count[4]++;
            }
            if (g_safety.fault_count[4] >= (uint8_t)CFG_SOL_FAULT_DEBOUNCE) {
                g_safety.sol_faults        |= FAULT_SOL_OPEN;
                g_safety.sol_faults_latched |= FAULT_SOL_OPEN;
            }
        } else {
            g_safety.fault_count[4] = 0U;
            g_safety.sol_faults &= (uint8_t)(~FAULT_SOL_OPEN);
        }
    } else {
        g_safety.fault_count[4] = 0U;
        g_safety.sol_faults &= (uint8_t)(~(FAULT_SOL_OPEN | FAULT_SOL_WELDED | FAULT_SOL_OC));
    }
}

void safe_print_status(void (*print_fn)(const char *))
{
    char buf[128];

    print_fn("=================================");
    print_fn("Safety Status:");
    print_fn("=================================");

    snprintf(buf, sizeof(buf), "Safe State: %s", g_safety.safe_state_active ? "YES" : "NO");
    print_fn(buf);

    snprintf(buf, sizeof(buf), "Active Faults: 0x%02X", (unsigned)g_safety.faults);
    print_fn(buf);

    if (g_safety.faults != 0U) {
        uint8_t i;
        for (i = 0U; i < 8U; i++) {
            uint8_t m = (uint8_t)(1U << i);
            if ((g_safety.faults & m) != 0U) {
                snprintf(buf, sizeof(buf), "  - %s", safe_fault_name(m));
                print_fn(buf);
            }
        }
    } else {
        print_fn("  (none)");
    }

    snprintf(buf, sizeof(buf), "Latched Faults: 0x%02X", (unsigned)g_safety.faults_latched);
    print_fn(buf);

    snprintf(buf, sizeof(buf), "Watchdog Kicks: %lu", (unsigned long)g_safety.watchdog_kick_count);
    print_fn(buf);

    snprintf(buf, sizeof(buf), "Safe Transitions: %lu", (unsigned long)g_safety.safe_transitions);
    print_fn(buf);

    snprintf(buf, sizeof(buf), "Encoder Timeouts: %lu", (unsigned long)g_safety.encoder_timeouts);
    print_fn(buf);

    snprintf(buf, sizeof(buf), "Overcurrent Events: %lu", (unsigned long)g_safety.overcurrent_events);
    print_fn(buf);

    snprintf(buf, sizeof(buf), "Power Low Events: %lu", (unsigned long)g_safety.power_low_events);
    print_fn(buf);

    snprintf(buf, sizeof(buf), "Recovery Attempts: %lu", (unsigned long)g_safety.recovery_attempts);
    print_fn(buf);

    snprintf(buf, sizeof(buf), "SOL Faults: 0x%02X (latched: 0x%02X)",
             (unsigned)g_safety.sol_faults, (unsigned)g_safety.sol_faults_latched);
    print_fn(buf);
    if (g_safety.sol_faults & FAULT_SOL_OPEN) {
        print_fn("  - SOL_OPEN (relay ON, no current detected)");
    }
    if (g_safety.sol_faults & FAULT_DRV_NFAULT) {
        print_fn("  - DRV_NFAULT (DRV8873H fault pin asserted)");
    }
    if (g_safety.sol_faults & FAULT_POSITION_ERROR) {
        print_fn("  - POSITION_ERROR (|pos_error| > 15% for 2s — motor stuck or encoder)");
    }

    /* ISO26262: hardware WDT is ACTIVE (~840ms timeout), enabled in safe_init(). */
    print_fn("Watchdog: HW WDT ACTIVE (~840ms timeout, enabled in safe_init)");

    snprintf(buf, sizeof(buf), "POST result: 0x%02X (%s)",
             (unsigned)g_safety.post_result,
             (g_safety.post_result == 0U) ? "PASS" : "FAIL");
    print_fn(buf);
    if (g_safety.post_result & POST_RAM_FAIL)    { print_fn("  - POST_RAM_FAIL (RAM march test failed at boot)"); }
    if (g_safety.post_result & POST_CFG_FAIL)    { print_fn("  - POST_CFG_FAIL (throttle_config.h value out of range)"); }
    if (g_safety.post_result & POST_CANARY_FAIL) { print_fn("  - POST_CANARY_FAIL (RAM canary corrupted since boot)"); }

    snprintf(buf, sizeof(buf), "Loop overruns (>50ms): %lu", (unsigned long)g_safety.loop_overrun_count);
    print_fn(buf);

    print_fn("=================================");
}
