// ═══════════════════════════════════════════════════════════════════════
// Throttle ECU Safety Monitor — Implementation
// ═══════════════════════════════════════════════════════════════════════

#include "initialsafe.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

// ═══════════════════════════════════
// GLOBAL STATE
// ═══════════════════════════════════
SafetyState g_safety = {0};

// STM32F411CE Watchdog registers (direct access)
typedef struct {
  volatile uint32_t KR;   // 0x00 - Key register
  volatile uint32_t PR;   // 0x04 - Prescaler
  volatile uint32_t RLR;  // 0x08 - Reload register
  volatile uint32_t SR;   // 0x0C - Status register
} IWDG_TypeDef;

#define IWDG_BASE     0x40003000
#define IWDG          ((IWDG_TypeDef*)IWDG_BASE)
#define IWDG_KEY_UNLOCK 0x5555
#define IWDG_KEY_RELOAD 0xAAAA
#define IWDG_KEY_START  0xCCCC

// RCC CSR (Clock Control / Status Register) for watchdog reset flag
typedef struct {
  volatile uint32_t CR;      // 0x00
  volatile uint32_t PLLCFGR; // 0x04
  volatile uint32_t CFGR;    // 0x08
  volatile uint32_t CIR;     // 0x0C
  volatile uint32_t AHB1RSTR;// 0x10
  volatile uint32_t AHB2RSTR;// 0x14
  volatile uint32_t APB1RSTR;// 0x18
  volatile uint32_t APB2RSTR;// 0x1C
  volatile uint32_t AHB1ENR; // 0x20
  volatile uint32_t AHB2ENR; // 0x24
  volatile uint32_t APB1ENR; // 0x28
  volatile uint32_t APB2ENR; // 0x2C
  volatile uint32_t AHB1LPENR;
  volatile uint32_t AHB2LPENR;
  volatile uint32_t APB1LPENR;
  volatile uint32_t APB2LPENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;     // 0x74 - contains reset flags
} RCC_TypeDef;

#define RCC_BASE      0x40023800
#define RCC           ((RCC_TypeDef*)RCC_BASE)
#define RCC_CSR_IWDGRSTF  (1u << 29)  // Independent watchdog reset flag
#define RCC_CSR_WWDGRSTF  (1u << 30)  // Window watchdog reset flag
#define RCC_CSR_RMVF      (1u << 24)  // Remove reset flags

// ═══════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════
void safe_init(void) {
  memset(&g_safety, 0, sizeof(SafetyState));

  // Check if MCU was reset by watchdog
  if (RCC->CSR & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) {
    g_safety.faults |= FAULT_WATCHDOG_RESET;
    g_safety.faults_latched |= FAULT_WATCHDOG_RESET;
  }

  // Clear reset flags
  RCC->CSR |= RCC_CSR_RMVF;

  // Initialize watchdog (200ms timeout)
  safe_init_watchdog(200);
}

// ═══════════════════════════════════
// WATCHDOG MANAGEMENT
// ═══════════════════════════════════
void safe_init_watchdog(uint32_t timeout_ms) {
  // Watchdog clock: LSI = 32 kHz
  // Prescaler options: /4, /8, /16, /32, /64, /128, /256
  // Reload: 1-4095
  // Timeout = (Prescaler * Reload) / 32000

  // For 200ms: need (200 * 32) = 6400 counts
  // With prescaler /32: 6400 / 32 = 200 counts (fits in RLR)

  IWDG->KR = IWDG_KEY_UNLOCK;       // Unlock registers
  IWDG->PR = 3;                      // Prescaler /32
  IWDG->RLR = 200;                   // Reload value (200ms)
  IWDG->KR = IWDG_KEY_START;         // Start watchdog
}

void safe_kick_watchdog(void) {
  if (g_safety.safe_state_active) {
    // Don't feed watchdog in safe state — let it bite
    return;
  }
  IWDG->KR = IWDG_KEY_RELOAD;
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
    g_safety.fault_count[0] = 0;  // Clear debounce
    g_safety.faults &= ~FAULT_ENCODER_STALE;
    g_safety.faults &= ~FAULT_ENCODER_INVALID;
  } else {
    // Check for stale data (no update in 500ms)
    if ((now_ms - g_safety.last_encoder_update) > SAFE_ENCODER_TIMEOUT_MS) {
      g_safety.fault_count[0]++;
      if (g_safety.fault_count[0] >= 2) {  // Debounce: 2 ticks = ~200ms
        g_safety.faults |= (FAULT_ENCODER_STALE | FAULT_ENCODER_INVALID);
        g_safety.faults_latched |= FAULT_ENCODER_INVALID;
        g_safety.encoder_timeouts++;
        if (!(g_safety.faults & 0x7E)) {  // Not already in safe state for other reasons
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
    g_safety.fault_count[1]++;
    if (g_safety.fault_count[1] >= 3) {  // Debounce: 3 ticks = ~300ms
      g_safety.faults |= FAULT_OVERCURRENT_L;
      g_safety.faults_latched |= FAULT_OVERCURRENT_L;
      g_safety.overcurrent_events++;
      safe_enter_safe_state("Overcurrent L");
    }
  } else {
    g_safety.fault_count[1] = 0;
    g_safety.faults &= ~FAULT_OVERCURRENT_L;
  }

  // Check right motor current (PA1)
  if (ris > SAFE_OVERCURRENT_THRESH) {
    g_safety.fault_count[2]++;
    if (g_safety.fault_count[2] >= 3) {
      g_safety.faults |= FAULT_OVERCURRENT_R;
      g_safety.faults_latched |= FAULT_OVERCURRENT_R;
      g_safety.overcurrent_events++;
      safe_enter_safe_state("Overcurrent R");
    }
  } else {
    g_safety.fault_count[2] = 0;
    g_safety.faults &= ~FAULT_OVERCURRENT_R;
  }
}

// ═══════════════════════════════════
// POWER SUPPLY MONITORING
// ═══════════════════════════════════
void safe_check_power(uint16_t supply_mv) {
  if (supply_mv < SAFE_POWER_LOW_MV) {
    g_safety.fault_count[3]++;
    if (g_safety.fault_count[3] >= 4) {  // Debounce: 4 ticks = ~400ms
      g_safety.faults |= FAULT_POWER_LOW;
      g_safety.faults_latched |= FAULT_POWER_LOW;
      g_safety.power_low_events++;
      safe_enter_safe_state("Power low");
    }
  } else {
    g_safety.fault_count[3] = 0;
    g_safety.faults &= ~FAULT_POWER_LOW;
  }
}

// ═══════════════════════════════════
// SAFE STATE MANAGEMENT
// ═══════════════════════════════════
void safe_enter_safe_state(const char* reason) {
  if (g_safety.safe_state_active) {
    return;  // Already in safe state
  }

  g_safety.safe_state_active = true;
  g_safety.safe_state_entered = millis();
  g_safety.safe_transitions++;

  // Application code should respond to this and actually cut power
  // (motor pins go LOW, relay goes LOW, etc.)

  char msg[64];
  snprintf(msg, sizeof(msg), "[SAFE] %s", reason);
  // Would be called via callback or printf in main loop
}

bool safe_can_recover(void) {
  if (!g_safety.safe_state_active) {
    return false;
  }

  // Only allow recovery if:
  // 1. No active faults (only latched ones remain)
  // 2. Safe state has been active for minimum time (recovery delay)
  uint32_t elapsed = millis() - g_safety.safe_state_entered;

  return (g_safety.faults == 0) && (elapsed > SAFE_RECOVERY_DELAY_MS);
}

void safe_attempt_recovery(void) {
  if (safe_can_recover()) {
    g_safety.safe_state_active = false;
    g_safety.recovery_attempts++;
    g_safety.last_encoder_update = millis();
    // Application should detect this and re-enable outputs
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
  // Kick watchdog only if not in safe state
  if (!g_safety.safe_state_active) {
    safe_kick_watchdog();
  }

  // Attempt recovery if conditions met
  if (safe_can_recover()) {
    safe_attempt_recovery();
  }
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
  char buf[256];

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

  snprintf(buf, sizeof(buf), "Watchdog Kicks: %u", g_safety.watchdog_kick_count);
  print_fn(buf);

  snprintf(buf, sizeof(buf), "Safe Transitions: %u", g_safety.safe_transitions);
  print_fn(buf);

  snprintf(buf, sizeof(buf), "Encoder Timeouts: %u", g_safety.encoder_timeouts);
  print_fn(buf);

  snprintf(buf, sizeof(buf), "Overcurrent Events: %u", g_safety.overcurrent_events);
  print_fn(buf);

  snprintf(buf, sizeof(buf), "Power Low Events: %u", g_safety.power_low_events);
  print_fn(buf);

  snprintf(buf, sizeof(buf), "Recovery Attempts: %u", g_safety.recovery_attempts);
  print_fn(buf);

  print_fn("═════════════════════════════════");
}
