// ═══════════════════════════════════════════════════════════════════════════════
//  config.h — ALL USER-CONFIGURABLE PARAMETERS IN ONE PLACE
// ═══════════════════════════════════════════════════════════════════════════════
//
//  Change values here to adapt the firmware to different:
//    • Motors (current limits, PWM freq, deadband)
//    • Throttle bodies (angle range, PID gains)
//    • Sensors (encoder type, ADC thresholds)
//    • Safety requirements (timeouts, debounce counts)
//
//  After changing any value, recompile and reflash.
//  No other files need to be edited.
//
// ═══════════════════════════════════════════════════════════════════════════════

#ifndef CONFIG_H
#define CONFIG_H

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 1: PIN ASSIGNMENTS
// ─────────────────────────────────────────────────────────────────────────────
//  Which STM32 pins connect to what hardware.
//  Only change these if you physically rewire the board.
//
//  PIN_RPWM / PIN_LPWM  → H-bridge PWM inputs (right / left direction)
//  PIN_REN  / PIN_LEN   → H-bridge enable pins (must be HIGH to drive)
//  PIN_RIS  / PIN_LIS   → H-bridge current sense analog outputs
//  PIN_RELAY            → MOSFET relay gate trigger (digital HIGH = on)
//  PIN_ENC              → AS5048A encoder PWM output
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_PIN_RPWM    PA9
#define CFG_PIN_LPWM    PA8
#define CFG_PIN_REN     PB0
#define CFG_PIN_LEN     PB1
#define CFG_PIN_RIS     PA1     // Right current sense (ADC)
#define CFG_PIN_LIS     PA0     // Left current sense  (ADC)
#define CFG_PIN_RELAY   PB10    // MOSFET relay trigger
#define CFG_PIN_ENC     PB4     // Encoder PWM input

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 2: MOTOR / H-BRIDGE
// ─────────────────────────────────────────────────────────────────────────────
//  PWM_FREQ_HZ   → Switching frequency in Hz
//                   20000 = 20 kHz (ultrasonic, quiet)
//                   Lower = louder but more torque at low duty
//                   Range: 1000–40000
//
//  PWM_BITS      → Resolution of the PWM duty cycle
//                   12 = 0–4095 steps, 8 = 0–255 steps
//                   Higher = finer motor control
//                   Range: 8–16 (12 recommended for most applications)
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_PWM_FREQ_HZ     20000
#define CFG_PWM_BITS         12

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 3: AS5048A MAGNETIC ENCODER
// ─────────────────────────────────────────────────────────────────────────────
//  These come from the AS5048A datasheet and should NOT be changed
//  unless you are using a different encoder model.
//
//  ENC_OFFSET       → Number of init/error clock cycles to strip from PWM
//  ENC_DATA         → Number of data clock cycles (12-bit PWM = 4095)
//                      NOTE: 14-bit (16384) is SPI only. PWM is 12-bit.
//  ENC_PERIOD_CLKS  → Total PWM period in clock cycles
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_ENC_OFFSET        16
#define CFG_ENC_DATA          4095
#define CFG_ENC_PERIOD_CLKS   4119

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 4: THROTTLE CALIBRATION
// ─────────────────────────────────────────────────────────────────────────────
//  PASSIVE END-STOP LEARNING:
//    Starts with the hardcoded ANGLE_MIN/MAX as initial estimates.
//    During normal PID operation, if the motor is driving at high duty
//    but the angle has stopped changing, we've hit a physical end stop.
//    The firmware quietly tightens the range to match reality.
//
//    NO active sweep, NO calibration routine, NEVER moves the throttle
//    on its own. Only learns from commands the driver is already sending.
//
//  ANGLE_MIN / ANGLE_MAX → Initial angle estimates (0.01° units).
//                           Measure roughly with serial monitor.
//                           These are the STARTING values — the firmware
//                           will tighten them if it detects end stops.
//
//  ENDSTOP_DUTY_THRESH → PID duty must be above this % of PWM_MAX to
//                          consider the motor "driving hard."
//                          80 = 80% of 4095 = 3276. If duty is below
//                          this, the motor isn't stalled, just cruising.
//
//  ENDSTOP_STALL_MS    → How long (ms) the angle must stop changing
//                          while duty is high before we believe it's a
//                          real mechanical stop (not just slow movement).
//                          500 = half second. Longer = more conservative.
//
//  ENDSTOP_STALL_THRESH → Maximum angle change (0.01° units) per tick
//                          that counts as "not moving."
//                          30 = 0.30° — if angle changes less than this
//                          while duty is saturated, we're at a stop.
//
//  ENDSTOP_MARGIN      → Safety margin (0.01° units) applied inward
//                          from a learned end stop. Prevents PID from
//                          commanding exactly at the mechanical limit.
//                          50 = 0.50°
//
//  ENDSTOP_SANITY      → Maximum allowed deviation (0.01° units) from
//                          the hardcoded ANGLE_MIN/MAX. Rejects garbage
//                          readings from sensor glitches.
//                          500 = 5.00° — learned stop must be within
//                          5° of the initial estimate or it's rejected.
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_ANGLE_MIN             7032    // 70.32° closed (initial estimate)
#define CFG_ANGLE_MAX             17940   // 179.40° wide open (initial estimate)

#define CFG_ENDSTOP_DUTY_THRESH   80      // % of PWM_MAX (80% = driving hard)
#define CFG_ENDSTOP_STALL_MS      500     // ms of no movement while duty high
#define CFG_ENDSTOP_STALL_THRESH  30      // 0.30° movement = "not moving"
#define CFG_ENDSTOP_MARGIN        50      // 0.50° inward from learned stop
#define CFG_ENDSTOP_SANITY        500     // 5.00° max deviation from initial

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 5: PID CONTROLLER
// ─────────────────────────────────────────────────────────────────────────────
//  These control how aggressively the motor tracks the target angle.
//  Can also be tuned LIVE over serial (p##, i##, k## commands)
//  without reflashing — but values here set the power-on defaults.
//
//  Kp  → Proportional gain. Higher = faster response, more overshoot.
//         Start at 8–15. If it overshoots, lower it.
//
//  Ki  → Integral gain. Eliminates steady-state error (offset).
//         Start at 0.1–0.5. If it oscillates slowly, lower it.
//
//  Kd  → Derivative gain. Dampens overshoot.
//         Start at 1.0–3.0. If it's jerky, lower it.
//
//  PID_INTEGRAL_LIMIT → Anti-windup clamp on the integral term.
//                        In PWM duty units. 2000 = half of 4095 max.
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_KP_DEFAULT          12.0f
#define CFG_KI_DEFAULT          0.3f
#define CFG_KD_DEFAULT          1.5f
#define CFG_PID_INTEGRAL_LIMIT  2000.0f

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 6: NOISE REDUCTION / MOTOR COMFORT
// ─────────────────────────────────────────────────────────────────────────────
//  These prevent the motor from buzzing/hunting when holding position.
//
//  PID_DEADBAND      → Error (in 0.01° units) below which the PID
//                       outputs zero. Wider = quieter but less precise.
//                       50 = 0.50°, 100 = 1.00°
//
//  MIN_DUTY_THRESH   → Minimum PWM duty that actually moves the motor.
//                       Below this, the motor vibrates but doesn't turn.
//                       Set by experiment: increase until buzzing stops.
//                       Range: 20–150 (depends on motor + load)
//
//  SETTLE_TIME_MS    → After reaching target, wait this long before
//                       cutting motor power (coast mode). Prevents
//                       re-engaging on every tiny vibration.
//                       500 = half second
//
//  SETTLE_WINDOW     → Position must stay within this many 0.01° units
//                       of the target for SETTLE_TIME_MS to count.
//                       100 = 1.00°
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_PID_DEADBAND      100      // 1°
#define CFG_MIN_DUTY_THRESH   50      // PWM counts
#define CFG_SETTLE_TIME_MS    500     // ms
#define CFG_SETTLE_WINDOW     100     // 0.01° units (1.00°)

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 7: SAFETY — OVERCURRENT
// ─────────────────────────────────────────────────────────────────────────────
//  Motor current is read as a 12-bit ADC value (0–4095).
//  The actual amps depend on your H-bridge current sense resistor.
//
//  OVERCURRENT_THRESH  → ADC count above which overcurrent is flagged.
//                         Set to 4095 to DISABLE (current sense needs
//                         resistor swap before this works properly).
//
//                         BTS7960 current sense math (kILIS = 8500:1):
//                           V_per_amp = R_sense / 8500
//                           ADC_count = (V_per_amp * amps / 3.3) * 4095
//
//                         With 10kΩ sense resistor (factory default):
//                           1A = 1460 counts, 2A = 2920, 2.8A = 4095 (MAX)
//                           Cannot measure above ~2.8A!
//
//                         With 2.2kΩ sense resistor (recommended for 8A motors):
//                           5A = 1374 counts, 8A = 2199, 10A = 2749
//                           Set threshold to ~2750 for 10A trip
//
//                         With 3.3kΩ parallel to 10kΩ (effective 2.48kΩ):
//                           5A = 1548 counts, 8A = 2477, 10A = 3096
//                           Set threshold to ~3100 for 10A trip
//
//  OVERCURRENT_DEBOUNCE → Number of consecutive loop iterations the
//                          current must exceed the threshold before
//                          tripping safe state. Prevents false alarms
//                          from inrush spikes.
//                          3 ticks ≈ 30ms at 10kHz loop rate
//                          Range: 1–10
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_OVERCURRENT_THRESH    4095    // DISABLED — swap sense resistor first
#define CFG_OVERCURRENT_DEBOUNCE  10      // consecutive ticks

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 8: SAFETY — ENCODER TIMEOUT
// ─────────────────────────────────────────────────────────────────────────────
//  If the encoder stops sending valid data for this long, the motor
//  is stopped immediately (can't control what you can't measure).
//
//  ENCODER_TIMEOUT_MS   → Stale data timeout in milliseconds.
//                          500 = half second (good for 1kHz encoder)
//                          Lower = faster fault detection
//                          Higher = more tolerant of intermittent signal
//
//  ENCODER_DEBOUNCE     → Number of consecutive checks the encoder
//                          must be invalid before tripping.
//                          2 = 2 ticks ≈ 200ms
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_ENCODER_TIMEOUT_MS    500     // ms
#define CFG_ENCODER_DEBOUNCE      2       // consecutive ticks

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 9: SAFETY — POWER SUPPLY
// ─────────────────────────────────────────────────────────────────────────────
//  Minimum supply voltage before entering safe state.
//  Only active if you connect a voltage divider to an ADC pin.
//  (Not connected by default — set to 0 to disable)
//
//  POWER_LOW_MV        → Minimum voltage in millivolts.
//                         12000 = 12.0V, 9000 = 9.0V, 0 = disabled
//
//  POWER_DEBOUNCE      → Consecutive ticks below threshold to trip.
//                         4 = 4 ticks ≈ 400ms
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_POWER_LOW_MV          9000    // millivolts (0 = disabled)
#define CFG_POWER_DEBOUNCE        4       // consecutive ticks

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 10: SAFETY — WATCHDOG TIMER
// ─────────────────────────────────────────────────────────────────────────────
//  Hardware watchdog resets the entire MCU if the main loop hangs.
//  This catches infinite loops, hard faults, and code crashes.
//
//  WATCHDOG_TIMEOUT_US  → Timeout in MICROSECONDS.
//                          200000 = 200ms (recommended)
//                          Must be longer than your worst-case loop time.
//                          Range: 1000–32000000 (1ms–32s)
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_WATCHDOG_TIMEOUT_US   200000  // 200ms

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 11: SAFETY — RECOVERY
// ─────────────────────────────────────────────────────────────────────────────
//  After entering safe state, the system waits this long before
//  allowing recovery (prevents rapid fault/recover cycling).
//
//  RECOVERY_DELAY_MS   → Minimum time in safe state before reset allowed.
//                         1000 = 1 second
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_RECOVERY_DELAY_MS     1000    // ms

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 12: TELEMETRY / SERIAL
// ─────────────────────────────────────────────────────────────────────────────
//  SERIAL_BAUD        → Baud rate for both USB CDC and UART Serial1.
//                        115200 is standard. 230400 for faster updates.
//
//  TELEMETRY_RATE_MS  → How often telemetry prints to serial.
//                        200 = 5 Hz (good for GUI)
//                        100 = 10 Hz (faster response, more data)
//                        1000 = 1 Hz (minimal, for logging)
//
//  ADC_OVERSAMPLE     → Number of ADC reads to average per measurement.
//                        16 = good noise rejection, ~50us
//                        4  = faster, noisier
//                        32 = slower, cleaner
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_SERIAL_BAUD         115200
#define CFG_TELEMETRY_RATE_MS   200     // ms
#define CFG_ADC_OVERSAMPLE      16      // readings to average

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 13: USB CDC STARTUP
// ─────────────────────────────────────────────────────────────────────────────
//  USB_SETTLE_MS  → Delay after Serial.begin() for USB CDC to enumerate.
//                    250 = works for most PCs
//                    Increase to 500–1000 if serial doesn't appear on boot.
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_USB_SETTLE_MS       250     // ms

#endif // CONFIG_H
