// ═══════════════════════════════════════════════════════════════════════
// Throttle ECU Safety Monitor — Fault Detection & Recovery
// Detects: encoder loss, power failures, overcurrent, watchdog resets
// Target: STM32F411CE (WeAct BlackPill)
// Uses STM32duino built-in IWatchdog library
// ═══════════════════════════════════════════════════════════════════════

#ifndef INITIALSAFE_H
#define INITIALSAFE_H

#include <stdint.h>
#include <stdbool.h>

// ═══════════════════════════════════
// FAULT FLAG DEFINITIONS
// ═══════════════════════════════════
#define FAULT_ENCODER_STALE    0x01   // Encoder data not updated in timeout window
#define FAULT_ENCODER_INVALID  0x02   // Encoder signal lost/malformed
#define FAULT_OVERCURRENT_L    0x04   // Left motor overcurrent detected
#define FAULT_OVERCURRENT_R    0x08   // Right motor overcurrent detected
#define FAULT_POWER_LOW        0x10   // Supply voltage below threshold
#define FAULT_WATCHDOG_RESET   0x20   // MCU recovered from watchdog reset
#define FAULT_PID_SAT          0x40   // PID output saturated (windup)
#define FAULT_TEMP_HIGH        0x80   // Temperature sensor above threshold (future)

// ═══════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════
#define SAFE_ENCODER_TIMEOUT_MS    500    // Stale data timeout
#define SAFE_OVERCURRENT_THRESH    3950   // ADC counts (~5A — raise if still false-tripping)
#define SAFE_POWER_LOW_MV          9000   // 9.0V minimum supply
#define SAFE_RECOVERY_DELAY_MS     1000   // Time before recovering from safe state
#define SAFE_WATCHDOG_TIMEOUT_US   200000 // 200ms in microseconds

// ═══════════════════════════════════
// SAFETY STATE STRUCTURE
// ═══════════════════════════════════
typedef struct {
  uint8_t  faults;                 // Current fault flags (bitmask)
  uint8_t  faults_latched;         // Latched faults (persist until cleared)
  uint32_t fault_timestamp[8];     // Timestamp each fault was detected
  uint8_t  fault_count[8];         // Debounce counter for each fault

  uint32_t last_encoder_update;    // Timestamp of last valid encoder read
  uint32_t last_current_check;     // Timestamp of last current measurement

  bool     safe_state_active;      // True if in safe state
  uint32_t safe_state_entered;     // Timestamp when safe state was entered

  char     last_reason[24];        // Human-readable reason for last safe state entry

  uint32_t watchdog_kick_count;    // Incremented each kick (diagnostics)
  uint32_t recovery_attempts;      // Number of recovery attempts made

  // Diagnostic counters
  uint32_t encoder_timeouts;
  uint32_t overcurrent_events;
  uint32_t power_low_events;
  uint32_t safe_transitions;
} SafetyState;

// ═══════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════
extern SafetyState g_safety;

// Initialize safety module (call in setup, after Serial.begin)
void safe_init(void);

// Periodic check — call from main loop ~100ms
void safe_tick(uint32_t now_ms);

// Fault detection functions
void safe_check_encoder(bool is_valid, uint32_t now_ms);
void safe_check_current(uint16_t ris, uint16_t lis, uint32_t now_ms);
void safe_check_power(uint16_t supply_mv);

// Safe state management
void safe_enter_safe_state(const char* reason);
bool safe_can_recover(void);
void safe_attempt_recovery(void);
void safe_clear_faults(void);

// Watchdog functions
void safe_kick_watchdog(void);
bool safe_was_reset_by_watchdog(void);

// Diagnostics
void safe_print_status(void (*print_fn)(const char*));
uint8_t safe_get_fault_flags(void);
const char* safe_fault_name(uint8_t fault_bit);

#endif // INITIALSAFE_H
