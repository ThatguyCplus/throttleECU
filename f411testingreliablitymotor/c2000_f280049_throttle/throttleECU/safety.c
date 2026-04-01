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
        strncpy(g_safety.last_reason, reason, sizeof(g_safety.last_reason) - 1U);
        g_safety.last_reason[sizeof(g_safety.last_reason) - 1U] = '\0';
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
        g_safety.safe_state_active = false;
        g_safety.recovery_attempts++;
        g_safety.last_encoder_update = Board_millis();
    }
}

void safe_clear_faults(void)
{
    g_safety.faults         = 0U;
    g_safety.faults_latched = 0U;
    memset(g_safety.fault_count, 0, sizeof(g_safety.fault_count));
    g_safety.safe_state_active = false;
#if CFG_CAN_HEARTBEAT_EN
    s_canHbArmed = 0U;
#endif
}

void safe_tick(uint32_t now_ms)
{
    (void)now_ms;
    safe_kick_watchdog();
    safe_attempt_recovery();
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

    print_fn("Watchdog: optional — enable in safety.c");
    print_fn("=================================");
}
