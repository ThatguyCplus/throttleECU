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
/* ISO26262: position error safe state now sets this bit in sol_faults (byte 6
 * of 0x102) so the GUI/controller sees why safe state was entered.
 * Previously enterSafeStateEc() set no fault bits → faults=0x00 mode=SAFE. */
#define FAULT_POSITION_ERROR   0x20U   /* bit in sol_faults: |pos_error| > 15% for 2s */

/* Extended fault flags — stored in sol_faults byte, transmitted in 0x102 byte[6].
 * bit3 of the CAN byte = SOL_INFERRED_ON status (not a fault, added in throttle_ecu.c).
 *
 * Solenoid current faults (bits 0-2): */
#define FAULT_SOL_OPEN    0x01U   /* relay ON, current below ON threshold (open/disconnected) */
#define FAULT_SOL_WELDED  0x02U   /* relay OFF, current above ON threshold (welded contact)   */
#define FAULT_SOL_OC      0x04U   /* current above OC threshold (short/overcurrent)           */
/* Motor driver fault (bit 4): */
#define FAULT_DRV_NFAULT  0x10U   /* DRV8873H nFAULT asserted LOW (OCP/OTW/OTS/UVLO)         */

/* POST (Power-On Self-Test) result bits — stored in g_safety.post_result.
 * safe_post() runs once at end of Throttle_init() before the main loop starts.
 * Failure causes immediate safe state entry and is visible via UART "diag". */
#define POST_RAM_FAIL     0x01U  /* RAM march test: stuck-bit or coupling error */
#define POST_CFG_FAIL     0x02U  /* Config range sanity: impossible parameter detected */
#define POST_CANARY_FAIL  0x04U  /* Runtime: g_safety.ram_canary corrupted since POST */

/* Known-good 32-bit word written to g_safety.ram_canary by safe_post().
 * Chosen to be distinct from 0x00000000 and 0xFFFFFFFF (all-zeros/ones
 * are the two most common RAM failure modes). Checked every 100ms in safe_tick(). */
#define SAFETY_RAM_CANARY  0xDEAD5AFEU

typedef struct {
    uint8_t  faults;
    uint8_t  faults_latched;
    uint8_t  sol_faults;
    uint8_t  sol_faults_latched;
    uint32_t last_encoder_update;
    uint8_t  fault_count[8];  /* [0]=enc [1]=OC_L [2]=OC_R [3]=pwr [4]=SOL_OPEN [5]=SOL_WELD [6]=SOL_OC [7]=DRV */
    bool     safe_state_active;
    uint32_t safe_state_entered;
    char     last_reason[24];
    uint32_t watchdog_kick_count;
    uint32_t recovery_attempts;
    uint32_t encoder_timeouts;
    uint32_t overcurrent_events;
    uint32_t power_low_events;
    uint32_t safe_transitions;
    /* ── POST and runtime integrity monitoring ─────────────────────────── */
    uint8_t  post_result;         /* POST failure bitmap (POST_* bits); 0 = all pass */
    uint32_t ram_canary;          /* Written SAFETY_RAM_CANARY at POST; checked in safe_tick() */
    uint32_t loop_overrun_count;  /* Main loop iterations that exceeded CFG_LOOP_MAX_MS */
} SafetyState;

extern SafetyState g_safety;

void safe_init(void);
void safe_tick(uint32_t now_ms);

/* safe_post — Power-On Self-Test. Call once at end of Throttle_init(), after
 * all hardware is initialised. Runs RAM march test, config sanity check, and
 * plants the RAM canary. Prints PASS/FAIL to print_fn (may be NULL to skip
 * printing). Enters safe state on failure; returns POST_* failure bitmap. */
uint8_t safe_post(void (*print_fn)(const char *));
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

/* Solenoid current monitoring — call after AdcSense_readSolenoid() each loop.
 * relay_on: current commanded relay state (s_relayOn).
 * SOL_WELDED and SOL_OC trigger safe state. SOL_OPEN is a latched warning only. */
void safe_check_solenoid(uint16_t sol_raw, uint8_t relay_on);

/* DRV8873H nFAULT monitoring — call each loop with MotorEPwm_isNFault().
 * nfault_asserted=1 means GPIO1 reads LOW (fault). Enters safe state when debounced. */
void safe_check_drv_nfault(uint8_t nfault_asserted);

uint8_t safe_get_sol_faults(void);

#endif
