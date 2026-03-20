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
//  ENC_DATA         → Number of data clock cycles (14-bit = 4095)
//  ENC_PERIOD_CLKS  → Total PWM period in clock cycles
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_ENC_OFFSET        16
#define CFG_ENC_DATA          4095
#define CFG_ENC_PERIOD_CLKS   4119

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 4: THROTTLE CALIBRATION
// ─────────────────────────────────────────────────────────────────────────────
//  These define the mechanical limits of YOUR throttle body.
//  Values are in 0.01° units (hundredths of a degree).
//
//  HOW TO CALIBRATE:
//    1. Flash firmware, open serial monitor at 115200
//    2. Manually push throttle to fully CLOSED → read pos=XX.XX
//    3. Manually push throttle to fully OPEN   → read pos=XX.XX
//    4. Multiply each by 100 and enter below
//       Example: 70.32° → 7032,  179.40° → 17940
//
//  ANGLE_MIN  → Encoder angle when throttle is fully CLOSED  (0%)
//  ANGLE_MAX  → Encoder angle when throttle is fully OPEN  (100%)
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_ANGLE_MIN    7032    // 70.32° closed
#define CFG_ANGLE_MAX    17940   // 179.40° wide open

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
#define CFG_PID_DEADBAND      50      // 0.50°
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
//                         3950 ≈ 5A on most BTS7960 modules.
//                         Lower = more sensitive, higher = more tolerant.
//                         MAX possible: 4095 (disables overcurrent check)
//
//  OVERCURRENT_DEBOUNCE → Number of consecutive loop iterations the
//                          current must exceed the threshold before
//                          tripping safe state. Prevents false alarms
//                          from inrush spikes.
//                          3 ticks ≈ 30ms at 10kHz loop rate
//                          Range: 1–10
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_OVERCURRENT_THRESH    3950    // ADC counts (~5A)
#define CFG_OVERCURRENT_DEBOUNCE  3       // consecutive ticks

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
