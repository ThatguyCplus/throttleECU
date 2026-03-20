// ═══════════════════════════════════════════════════════════════════════
// Throttle ECU Safety Monitor — Implementation
// Uses STM32duino built-in IWatchdog library (no custom register defs)
// ═══════════════════════════════════════════════════════════════════════

#include "initialsafe.h"
#include <Arduino.h>
#include <IWatchdog.h>
#include <stdio.h>
#include <string.h>

// ═══════════════════════════════════
// GLOBAL STATE
// ═══════════════════════════════════
SafetyState g_safety = {0};

// ═══════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════
void safe_init(void) {
  memset(&g_safety, 0, sizeof(SafetyState));

  // Check if MCU was reset by watchdog (pass true to auto-clear the flag)
  if (IWatchdog.isReset(true)) {
    g_safety.faults |= FAULT_WATCHDOG_RESET;
    g_safety.faults_latched |= FAULT_WATCHDOG_RESET;
  }

  // Initialize watchdog with 200ms timeout (200000 microseconds)
  IWatchdog.begin(SAFE_WATCHDOG_TIMEOUT_US);
}

// ═══════════════════════════════════
// WATCHDOG MANAGEMENT
// ═══════════════════════════════════
void safe_kick_watchdog(void) {
  // Always feed watchdog — safe state is a controlled state, not a hang.
  // Watchdog should only bite if the loop truly stops running.
  IWatchdog.reload();
  g_safety.watchdog_kick_count++;
}

bool safe_was_reset_by_watchdog(void) {
  return (g_safety.faults_latched & FAULT_WATCHDOG_RESET) != 0;
}

// ═══════════════════════════════════
// ENCODER MONITORING
// ═══════════════════════════════════
void safe_check_encoder(bool is_valid, uint32_t now_ms) {
  if (is_valid) {
    g_safety.last_encoder_update = now_ms;
    g_safety.fault_count[0] = 0;
    // Only clear flags if NOT in safe state — keep them visible until manual reset
    if (!g_safety.safe_state_active) {
      g_safety.faults &= ~FAULT_ENCODER_STALE;
      g_safety.faults &= ~FAULT_ENCODER_INVALID;
    }
  } else {
    if ((now_ms - g_safety.last_encoder_update) > SAFE_ENCODER_TIMEOUT_MS) {
      if (g_safety.fault_count[0] < 2) {
        g_safety.fault_count[0]++;
        if (g_safety.fault_count[0] >= 2) {
          g_safety.faults |= (FAULT_ENCODER_STALE | FAULT_ENCODER_INVALID);
          g_safety.faults_latched |= FAULT_ENCODER_INVALID;
          g_safety.encoder_timeouts++;
          safe_enter_safe_state("Encoder timeout");
        }
      }
    }
  }
}

// ═══════════════════════════════════
// CURRENT MONITORING (Overcurrent)
// ═══════════════════════════════════
void safe_check_current(uint16_t ris, uint16_t lis, uint32_t now_ms) {
  g_safety.last_current_check = now_ms;

  // Check left motor current (PA0)
  if (lis > SAFE_OVERCURRENT_THRESH) {
    if (g_safety.fault_count[1] < 3) {
      g_safety.fault_count[1]++;
      if (g_safety.fault_count[1] >= 3) {
        g_safety.faults |= FAULT_OVERCURRENT_L;
        g_safety.faults_latched |= FAULT_OVERCURRENT_L;
        g_safety.overcurrent_events++;
        safe_enter_safe_state("Overcurrent L");
      }
    }
  } else {
    g_safety.fault_count[1] = 0;
    if (!g_safety.safe_state_active) {
      g_safety.faults &= ~FAULT_OVERCURRENT_L;
    }
  }

  // Check right motor current (PA1)
  if (ris > SAFE_OVERCURRENT_THRESH) {
    if (g_safety.fault_count[2] < 3) {
      g_safety.fault_count[2]++;
      if (g_safety.fault_count[2] >= 3) {
        g_safety.faults |= FAULT_OVERCURRENT_R;
        g_safety.faults_latched |= FAULT_OVERCURRENT_R;
        g_safety.overcurrent_events++;
        safe_enter_safe_state("Overcurrent R");
      }
    }
  } else {
    g_safety.fault_count[2] = 0;
    if (!g_safety.safe_state_active) {
      g_safety.faults &= ~FAULT_OVERCURRENT_R;
    }
  }
}

// ═══════════════════════════════════
// POWER SUPPLY MONITORING
// ═══════════════════════════════════
void safe_check_power(uint16_t supply_mv) {
  if (supply_mv < SAFE_POWER_LOW_MV) {
    g_safety.fault_count[3]++;
    if (g_safety.fault_count[3] >= 4) {
      g_safety.faults |= FAULT_POWER_LOW;
      g_safety.faults_latched |= FAULT_POWER_LOW;
      g_safety.power_low_events++;
      safe_enter_safe_state("Power low");
    }
  } else {
    g_safety.fault_count[3] = 0;
    if (!g_safety.safe_state_active) {
      g_safety.faults &= ~FAULT_POWER_LOW;
    }
  }
}

// ═══════════════════════════════════
// SAFE STATE MANAGEMENT
// ═══════════════════════════════════
void safe_enter_safe_state(const char* reason) {
  if (g_safety.safe_state_active) {
    return;
  }

  g_safety.safe_state_active = true;
  g_safety.safe_state_entered = millis();
  g_safety.safe_transitions++;
  strncpy(g_safety.last_reason, reason, sizeof(g_safety.last_reason) - 1);
  g_safety.last_reason[sizeof(g_safety.last_reason) - 1] = '\0';
}

bool safe_can_recover(void) {
  if (!g_safety.safe_state_active) {
    return false;
  }

  uint32_t elapsed = millis() - g_safety.safe_state_entered;
  return (g_safety.faults == 0) && (elapsed > SAFE_RECOVERY_DELAY_MS);
}

void safe_attempt_recovery(void) {
  if (safe_can_recover()) {
    g_safety.safe_state_active = false;
    g_safety.recovery_attempts++;
    g_safety.last_encoder_update = millis();
  }
}

void safe_clear_faults(void) {
  g_safety.faults = 0;
  g_safety.faults_latched = 0;
  memset(g_safety.fault_count, 0, sizeof(g_safety.fault_count));
  g_safety.safe_state_active = false;
}

// ═══════════════════════════════════
// PERIODIC TICK (call ~100ms from main loop)
// ═══════════════════════════════════
void safe_tick(uint32_t now_ms) {
  // Always kick watchdog — safe state is controlled, not a hang
  safe_kick_watchdog();
}

// ═══════════════════════════════════
// DIAGNOSTICS
// ═══════════════════════════════════
const char* safe_fault_name(uint8_t fault_bit) {
  switch (fault_bit) {
    case 0x01: return "ENCODER_STALE";
    case 0x02: return "ENCODER_INVALID";
    case 0x04: return "OVERCURR_L";
    case 0x08: return "OVERCURR_R";
    case 0x10: return "POWER_LOW";
    case 0x20: return "WATCHDOG_RST";
    case 0x40: return "PID_SAT";
    case 0x80: return "TEMP_HIGH";
    default:   return "UNKNOWN";
  }
}

uint8_t safe_get_fault_flags(void) {
  return g_safety.faults;
}

void safe_print_status(void (*print_fn)(const char*)) {
  char buf[128];

  print_fn("═════════════════════════════════");
  print_fn("Safety Status:");
  print_fn("═════════════════════════════════");

  snprintf(buf, sizeof(buf), "Safe State: %s", g_safety.safe_state_active ? "YES" : "NO");
  print_fn(buf);

  snprintf(buf, sizeof(buf), "Active Faults: 0x%02X", g_safety.faults);
  print_fn(buf);

  if (g_safety.faults == 0) {
    print_fn("  (none)");
  } else {
    for (uint8_t i = 0; i < 8; i++) {
      if (g_safety.faults & (1u << i)) {
        snprintf(buf, sizeof(buf), "  - %s", safe_fault_name(1u << i));
        print_fn(buf);
      }
    }
  }

  snprintf(buf, sizeof(buf), "Latched Faults: 0x%02X", g_safety.faults_latched);
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

  snprintf(buf, sizeof(buf), "Watchdog Enabled: %s", IWatchdog.isEnabled() ? "YES" : "NO");
  print_fn(buf);

  print_fn("═════════════════════════════════");
}
