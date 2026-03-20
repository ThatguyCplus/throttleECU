# Safety Module Integration Guide

## Files Created

- **initialsafe.h** — Header with fault definitions, API, and types
- **initialsafe.cpp** — Implementation with fault detection logic

## Features

### Fault Detection
- **Encoder Stale/Invalid** — No position update for 500ms
- **Overcurrent (L/R)** — Current sensor exceeds 3800 ADC counts (~2.5A)
- **Power Low** — Supply voltage drops below 9.0V
- **Watchdog Reset** — MCU recovered from watchdog bite
- **PID Saturation** — Motor command at limits (detected externally)

### Safety Actions
- **Debouncing** — Faults must persist 2-4 ticks (~200-400ms) before triggering
- **Motor Cutout** — Immediate stop on fault detection
- **Relay Disable** — Kill MOSFET trigger to disable actuator
- **Safe State Lock** — No recovery until conditions clear + 1000ms delay
- **Watchdog Guardian** — 200ms timeout; starves watchdog in safe state

### Diagnostics
- Real-time fault flags (bitmask)
- Latched fault history
- Event counters (timeouts, overcurrents, safe transitions)
- Recoverable vs. permanent faults

## Integration Steps

### 1. Add Files to Your Arduino Sketch

In Arduino IDE:
- Add `initialsafe.h` and `initialsafe.cpp` to the same directory as your `.ino`
- Or: Sketch → Add File → browse to `initialsafe.cpp`

Arduino will automatically compile both files.

### 2. Include in Your Main Sketch

At the top of your `.ino` file:

```cpp
#include "initialsafe.h"
```

### 3. Initialize in setup()

```cpp
void setup() {
  Serial.begin(115200);

  // ... other init code ...

  safe_init();  // Initialize safety module

  // Check if we recovered from watchdog
  if (safe_was_reset_by_watchdog()) {
    Serial.println("[WARN] Recovered from watchdog reset!");
  }
}
```

### 4. Call Safety Checks in loop()

```cpp
void loop() {
  // ... existing code ...

  int32_t angle = getAngle();
  uint32_t now = millis();

  // Check encoder vitals
  safe_check_encoder(angle >= 0, now);  // Pass true if angle is valid

  // Read current sensors
  uint16_t ris = adcAvg(PIN_RIS);
  uint16_t lis = adcAvg(PIN_LIS);
  safe_check_current(ris, lis, now);

  // Check power supply (example: using ADC on PA2)
  // uint16_t supply_mv = (adcRead(PA2) * 3300 * 11) / 4095;  // 11:1 divider
  // safe_check_power(supply_mv);

  // Safety state machine
  if (g_safety.safe_state_active) {
    // HARD STOP: zero all outputs
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, 0);
    digitalWrite(PIN_RELAY, LOW);
    digitalWrite(PIN_REN, LOW);
    digitalWrite(PIN_LEN, LOW);
    mode = MODE_SAFE;
  } else {
    // Normal operation (existing motor control code)
    // ...
  }

  // Periodic safety tick (~100ms)
  static uint32_t last_safe_tick = 0;
  if ((now - last_safe_tick) >= 100) {
    last_safe_tick = now;
    safe_tick(now);
  }

  // Optional: Emit fault flags in telemetry
  // Format: mode=PID dir=F duty=1234 RIS=456 LIS=789 pos=123.45 thr=50% tgt=50% err=0x05
  static uint32_t lastPrint = 0;
  if ((now - lastPrint) >= 200) {
    lastPrint = now;
    // ... your existing telemetry code ...
    uint8_t err = safe_get_fault_flags();
    printf(" err=0x%02X\n", err);
  }
}
```

### 5. Add Diagnostic Commands (Optional)

In your `processCmd()` function:

```cpp
else if (strcasecmp(s, "diag") == 0) {
  // Print safety diagnostics
  safe_print_status(printBoth);
}
else if (strcasecmp(s, "clearlog") == 0) {
  // Clear latched faults
  safe_clear_faults();
  printBoth("Faults cleared");
}
```

## Fault Codes (err=0x##)

| Code | Meaning | Action |
|------|---------|--------|
| 0x01 | Encoder Stale | Motor stops, safe state |
| 0x02 | Encoder Invalid | Motor stops, safe state |
| 0x04 | Overcurrent (Left) | Motor stops, safe state |
| 0x08 | Overcurrent (Right) | Motor stops, safe state |
| 0x10 | Power Low | Motor stops, safe state |
| 0x20 | Watchdog Reset | Logged, recovery after 1s |
| 0x40 | PID Saturation | Warning, may continue |
| 0x80 | Temperature High | Reserved for future use |

## Testing Safety Logic

### Test 1: Encoder Loss
- Disconnect AS5048A wire
- Send `t50` (throttle command)
- Expect: Motor stops within 500ms, safe state triggered, `err=0x03`

### Test 2: Overcurrent
- Lock the throttle shaft mechanically
- Send `d2048` (manual duty)
- Expect: Motor cuts within 300ms, `err=0x04` or `0x08`

### Test 3: Watchdog Recovery
- Send `diag` to show watchdog kick count
- Unplug and replug USB (simulates hard reset)
- Should recover to manual mode with `err=0x20` latched

### Test 4: Safe State
- Trigger any fault (e.g., encoder loss)
- Should enter safe state
- Send `reset` command — should refuse if faults active
- After fault clears + 1s, sends `reset` should work

## Configuration

Edit these constants in `initialsafe.h`:

```cpp
#define SAFE_ENCODER_TIMEOUT_MS    500    // Stale data timeout
#define SAFE_OVERCURRENT_THRESH    3800   // ADC counts (2.5A @ 10mOhm)
#define SAFE_POWER_LOW_MV          9000   // 9.0V minimum
#define SAFE_RECOVERY_DELAY_MS     1000   // 1 second delay before recovery
```

Then recompile.

## GUI Integration

Update `throttle_gui.py` to parse the `err=0x##` field:

```python
TELEM_RE = re.compile(
    r"mode=(\w+)\s+dir=([FR])\s+duty=(\d+)\s+RIS=(\d+)\s+LIS=(\d+)\s+"
    r"pos=([\d.]+|---)\s+thr=([\d]+|---)%\s+tgt=(\d+)%\s+err=0x([0-9A-Fa-f]+)"
)
```

The GUI error flags panel will display active faults in real time.

## Troubleshooting

### Watchdog Resets Happening Too Soon
- Increase `SAFE_RECOVERY_DELAY_MS` to give system more time to stabilize
- Check if main loop is blocking (serial reads, delays, etc.)

### False Overcurrent Alarms
- Increase `SAFE_OVERCURRENT_THRESH` if your motor naturally draws peak current
- Increase debounce counter (current uses 3 ticks = ~300ms)

### Can't Recover from Safe State
- Check that faults have actually cleared (use `diag` command)
- Ensure 1000ms has passed since safe state was entered
- Send `reset` command explicitly if needed

## Future Enhancements

- **Brown-out detector** — Catch partial power loss
- **Temperature sensor** — Thermal shutdown
- **CAN bus watchdog** — Detect lost heartbeat frames
- **Motor stall detection** — Current + zero movement
- **Redundant encoder** — Dual sensors for voting
