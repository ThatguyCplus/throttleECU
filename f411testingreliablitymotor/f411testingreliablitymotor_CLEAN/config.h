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

//  ENC_SPIKE_THRESH → Maximum allowed jump (in 0.01° units) between
//                       consecutive readings before it's rejected as noise.
//                       500 = 5.00° — any reading more than 5° from the
//                       rolling average is ignored (likely an EMI spike).
//                       Lower = more aggressive filtering.
//                       Higher = allows faster real movement.
//                       Range: 200–2000
#define CFG_ENC_SPIKE_THRESH  500

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 4: THROTTLE CALIBRATION
// ─────────────────────────────────────────────────────────────────────────────
//  ANGLE_MIN / ANGLE_MAX → Physical end stops of the throttle in 0.01° units.
//                           Measure with serial monitor: drive forward at
//                           d4095 → that's your max, reverse d4095 → that's min.
//
//  The firmware uses only 5-95% of this range for actual throttle control.
//  This avoids noisy encoder readings near the mechanical stops and
//  prevents the PID from chasing an angle the hardware can't reach.
//
//  Example: with min=70.32° and max=179.40°, the usable range is:
//    0% throttle = 75.77°  (5% from closed)
//    100% throttle = 173.95° (95% from closed)
//
//  This means t0 = 75.77° and t100 = 173.95°, with ~5° margin at each end.
// ─────────────────────────────────────────────────────────────────────────────
#define CFG_ANGLE_MIN             7032    // 70.32° fully closed
#define CFG_ANGLE_MAX             17940   // 179.40° fully open

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
//  SECTION 9: SAFETY — POWER SUPPLY VOLTAGE MONITORING
// ─────────────────────────────────────────────────────────────────────────────
//  Monitors motor supply voltage (12V battery / power supply) via a resistive
//  voltage divider connected to an ADC pin. If voltage drops below threshold,
//  the relay is disabled and safe state is entered.
//
//  *** STATUS: NOT YET WIRED — uncomment CFG_POWER_SENSE_ENABLED when ready ***
//
//  HARDWARE REQUIRED:
//    Wire a voltage divider from your 12V motor supply to an ADC pin:
//
//      Motor 12V+ ─── R1 (47kΩ) ───┬─── R2 (10kΩ) ─── GND
//                                   │
//                                   ├─── 100nF cap ─── GND  (noise filter)
//                                   │
//                                   └─── PA3 (ADC pin)
//
//    Parts needed:
//      R1 = 47kΩ  1% metal film 0805
//      R2 = 10kΩ  1% metal film 0805
//      C1 = 100nF ceramic 0805
//      Optional: BAT54S Schottky clamp diode (transient protection)
//      Optional: SMBJ18A TVS diode at input (load dump protection)
//
//    Divider ratio = 10 / (47 + 10) = 0.1754
//      14.4V (running) → 2.53V at ADC
//      12.0V (resting) → 2.11V at ADC
//       9.0V (low)     → 1.58V at ADC → ~1960 ADC counts
//      16.0V (max)     → 2.81V at ADC (safe, under 3.3V)
//
//    Conversion formula:
//      V_batt_mV = ADC_count * 3300 * (47 + 10) / (10 * 4095)
//      V_batt_mV = ADC_count * 188100 / 40950
//      V_batt_mV = ADC_count * 4593 / 1000  (simplified)
//
//  CONFIGURATION:
//    PIN_VBATT_SENSE   → ADC pin connected to voltage divider midpoint
//    R1_KOHM / R2_KOHM → Resistor values (for conversion formula)
//    POWER_LOW_MV      → Below this voltage, relay disables + safe state
//    POWER_DEBOUNCE    → Consecutive ticks below threshold to trip
//
//  TO ENABLE:
//    1. Wire the voltage divider (see above)
//    2. Uncomment the #define CFG_POWER_SENSE_ENABLED line below
//    3. Recompile and reflash
// ─────────────────────────────────────────────────────────────────────────────

// Uncomment this line when the voltage divider is wired up:
// #define CFG_POWER_SENSE_ENABLED

#define CFG_PIN_VBATT_SENSE   PA3     // ADC pin for voltage divider
#define CFG_VBATT_R1_KOHM     47      // Top resistor (kΩ)
#define CFG_VBATT_R2_KOHM     10      // Bottom resistor (kΩ)
#define CFG_POWER_LOW_MV      9000    // 9.0V — relay disables below this
#define CFG_POWER_DEBOUNCE    4       // consecutive ticks (~400ms)

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
