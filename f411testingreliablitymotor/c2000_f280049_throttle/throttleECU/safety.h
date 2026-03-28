#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>
#include <stdbool.h>

#define FAULT_ENCODER_STALE    0x01U
#define FAULT_ENCODER_INVALID  0x02U
#define FAULT_OVERCURRENT_L    0x04U
#define FAULT_OVERCURRENT_R    0x08U
#define FAULT_POWER_LOW        0x10U
#define FAULT_WATCHDOG_RESET   0x20U
#define FAULT_CAN_TIMEOUT      0x40U
#define FAULT_CAN_BUS_OFF      0x80U

typedef struct {
    uint8_t  faults;
    uint8_t  faults_latched;
    uint32_t last_encoder_update;
    uint8_t  fault_count[8];
    bool     safe_state_active;
    uint32_t safe_state_entered;
    char     last_reason[24];
    uint32_t watchdog_kick_count;
    uint32_t recovery_attempts;
    uint32_t encoder_timeouts;
    uint32_t overcurrent_events;
    uint32_t power_low_events;
    uint32_t safe_transitions;
} SafetyState;

extern SafetyState g_safety;

void safe_init(void);
void safe_tick(uint32_t now_ms);
void safe_check_encoder(bool is_valid, uint32_t now_ms);
void safe_check_current(uint16_t ris, uint16_t lis, uint32_t now_ms);
void safe_check_power(uint16_t supply_mv);
void safe_enter_safe_state(const char *reason);
bool safe_can_recover(void);
void safe_attempt_recovery(void);
void safe_clear_faults(void);
void safe_kick_watchdog(void);
bool safe_was_reset_by_watchdog(void);
void safe_print_status(void (*print_fn)(const char *));
uint8_t safe_get_fault_flags(void);

void safe_can_mark_rx(uint32_t now_ms);
void safe_can_check_timeout(uint32_t now_ms);
void safe_can_set_bus_off_fault(void);
void safe_can_clear_bus_off_fault(void);

#endif
