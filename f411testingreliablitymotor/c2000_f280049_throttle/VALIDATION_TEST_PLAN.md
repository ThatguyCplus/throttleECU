# Throttle ECU — Safety Validation Test Plan
**Firmware:** v1.6 — F280049C
**Date:** 2026-09-11
**Standard:** ISO 26262 functional safety claim validation

---

## 1. Equipment Required

| Item | Purpose |
|------|---------|
| CCS debugger (XDS110 on-board or external) | Memory inspection, breakpoints, live watch — no source changes |
| CAN interface (PEAK PCAN-USB / CANable / SocketCAN) | Inject and capture CAN frames |
| `throttle_gui.py` or candump + cansend | CAN frame sender/viewer |
| Bench variable PSU (0–30 V, current limited) | Simulate brake signal; solenoid supply isolation |
| Digital multimeter | Measure DISABLE pin, relay coil, motor terminal voltage |
| Oscilloscope (optional) | Verify PWM output goes to 0 on safe state |
| Jumper wires / switches | Physically break/connect signal lines |
| Mechanical throttle clamp (vice-grip or strap) | Force mechanical stall for position-error test |
| Stopwatch | Time-based tests (encoder timeout, position error) |
| Serial terminal (115200 8N1) | Observe UART output and send commands |

---

## 2. Test Setup

```
     [Bench PSU 12V] ──────── Motor VM rail
          │
     [Bench PSU 12V] ──────── Solenoid VM rail (optional separate supply)
          │
     [JTAG / XDS110] ─────── F280049C LaunchXL (JTAG header)
          │
     [CAN Interface] ─────── CAN-H / CAN-L on throttle PCB
          │
     [Serial Terminal] ───── USB-UART backchannel (GPIO28/29, 115200 baud)
```

All tests assume:
- Relay starts **OFF** (`s_relayOn = 0`)
- Motor bridge **DISABLED** at boot
- ECU reaches main loop (UART prints "Throttle ECU — C2000 F280049")

**Safety during testing:** Keep motor bench-mounted with throttle plate free to move. For mechanical stall tests, ensure the applied force is limited — the motor will draw locked-rotor current. The DRV8873H OCP will protect the driver; test duration should be kept under 5 seconds.

---

## 3. Test Record Format

For each test, record:
```
TEST ID:   [e.g. POST-01]
Date/Time:
Tester:
UART output (screenshot or copy):
CAN trace (if applicable):
Debugger observation (if applicable):
RESULT: PASS / FAIL / SKIP
Notes:
```

---

## 4. POST Tests (Power-On Self-Test)

### POST-01 — RAM march test PASS (healthy hardware)
**Claim:** safe_post() RAM march writes four patterns and reads back correctly on healthy SRAM.
**Prerequisite:** Normal bench power-up.
**Procedure:**
1. Power cycle the ECU.
2. Watch UART output during boot.

**Pass criteria:** Within 2 seconds of boot, UART prints:
```
[POST] Power-On Self-Test...
[POST] RAM march:   PASS
[POST] Config check: PASS
[POST] RAM canary:  planted (checked every 100ms)
[POST] ALL PASS
```

---

### POST-02 — RAM march test FAIL (debugger-injected corruption)
**Claim:** POST catches SRAM bit errors and enters safe state before the main loop runs.
**Prerequisite:** ECU connected to JTAG debugger, CCS open.
**Method:** Corrupt memory at runtime during the march test — no source code change.

**Procedure:**
1. In CCS, open `safety.c`.
2. Set a **breakpoint** on the first `for` loop inside `safe_post()`, after the first write pass (`for (i = 0U; i < 32U; i++) { scratch[i] = patterns[p]; }` — the line immediately after the first write loop).
3. Power cycle. Let MCU halt at the breakpoint.
4. In CCS **Memory Browser**, find the address of `scratch` (add `&scratch[0]` to the Expressions panel to reveal its address).
5. In Memory Browser, write `0x0000` to `scratch[0]` (overriding the `0x5555` just written).
6. Click **Resume** (F8) — the read-back verification loop will now see a mismatch.

**Pass criteria:**
- UART prints `[POST] RAM march:   FAIL ***`
- UART prints `[POST] FAILED (0x01) — safe state entered`
- `g_safety.post_result` shows `0x01` (POST_RAM_FAIL) in CCS Expressions
- `g_safety.safe_state_active` shows `true`
- ECU does **not** proceed to the main control loop

---

### POST-03 — Config range check FAIL (test build only)
**Claim:** POST catches impossible calibration values.
**Method:** This test requires a one-time temporary change to `throttle_config.h` — **restore after test**.

**Procedure:**
1. In `throttle_config.h`, temporarily swap: `#define CFG_ANGLE_MIN 13062` and `#define CFG_ANGLE_MAX 2411` (MIN > MAX).
2. Build and flash.
3. Power cycle.

**Pass criteria:**
- UART prints `[POST] Config check: FAIL *** check throttle_config.h`
- UART prints `[POST] FAILED (0x02) — safe state entered`
- `g_safety.post_result` shows `0x02` (POST_CFG_FAIL)

**Restore:** Revert `throttle_config.h` to original values and reflash.

---

## 5. Watchdog Tests

### WDT-01 — Hardware watchdog resets a halted CPU
**Claim:** If `Throttle_runOnce()` stops executing for >840ms, the hardware WDT resets the MCU and the system boots with motor and relay off.
**Prerequisite:** ECU running (past boot), JTAG debugger connected.

**Procedure:**
1. With ECU in any run mode, in CCS click **Suspend** (Pause button) to halt the CPU.
2. Start a stopwatch.
3. Wait **900ms** (do not resume).
4. The MCU resets automatically (CCS will lose contact / show "Target disconnected").
5. Allow the MCU to finish its reset sequence.
6. Reconnect CCS (Run → Connect Target).

**Pass criteria:**
- After reset, UART prints: `[WARN] Recovered from watchdog reset!`
- `g_safety.faults_latched` has bit `FAULT_WATCHDOG_RESET (0x20)` set
- Relay is OFF (measure relay coil: ~0V)
- Motor does not run (DISABLE pin = HIGH)
- `g_safety.watchdog_kick_count` restarted from 0 (fresh boot)

**Note:** Do not wait >5 seconds while halted — repeated WDT resets can complicate debugging.

---

### WDT-02 — Watchdog continues kicking while in safe state
**Claim:** Entering safe state does not stop the WDT kick — the main loop continues running.

**Procedure:**
1. Trigger any safe state (e.g., send CAN ESTOP frame byte[0]=0x04).
2. In CCS Expressions panel, add watch on `g_safety.watchdog_kick_count`.
3. Observe the value for 60 seconds.

**Pass criteria:**
- `watchdog_kick_count` increments continuously throughout the 60 seconds
- No watchdog reset occurs (UART stays quiet, no `[WARN] Recovered from watchdog reset!`)

---

## 6. Encoder Tests

### ENC-01 — Encoder wire disconnect → FAULT_ENCODER_STALE
**Claim:** Losing the encoder SPI connection enters safe state after `CFG_ENCODER_TIMEOUT_MS` (30 s).

**Procedure:**
1. ECU powered on, PID mode active (`t50` command), observe angle in telemetry.
2. Physically **unplug** the encoder CS wire (GPIO59) or MISO wire (GPIO57) from the connector.
3. Start stopwatch.
4. Continue watching UART telemetry.

**Pass criteria:**
- Within 30 seconds + 2× debounce, UART prints: `[SAFE] Encoder timeout`
- `g_safety.faults` has `FAULT_ENCODER_STALE (0x01)` set
- `g_safety.encoder_timeouts` increments by 1
- Motor stops, relay turns off
- CAN 0x101 byte[3] (fault_flags) shows 0x03

**Note:** The timeout is 30 s by default (`CFG_ENCODER_TIMEOUT_MS`). This is intentionally long to prevent nuisance faults from transient encoder glitches.

---

### ENC-02 — Encoder fault stays latched until operator reset
**Claim:** After an encoder fault, reconnecting the encoder does NOT automatically clear the fault.

**Procedure (immediately after ENC-01):**
1. Reconnect the encoder wire.
2. Observe UART telemetry for 30 seconds.
3. Do NOT send any reset command.

**Pass criteria:**
- Safe state remains active
- `g_safety.safe_state_active` stays `true` in CCS Expressions
- Motor stays off
- Fault only clears after explicit UART `reset` or CAN RESET frame (CFG_CAN_FLAG_RESET=0x08)

---

### ENC-03 — Encoder EF flag triggers angle hold
**Claim:** When the AS5147U reports EF=1 (out-of-range field), the firmware returns an invalid sentinel (0xFFFF / -1) rather than passing garbage angle data to the PID.

**Procedure:**
1. ECU in PID mode, encoder reading normally.
2. Hold a strong external magnet (e.g., neodymium) close to but above/beside the AS5147U IC — aim to saturate the field without aligning it.
3. Watch `g_enc_ef_count` in CCS live watch.

**Pass criteria:**
- `g_enc_ef_count` increments (EF events counted)
- UART telemetry shows `pos=---` (no valid angle) OR holds last good angle (average path)
- ECU does NOT crash to safe state from EF alone (EF is transient; timeout check runs separately)
- If EF persists > `CFG_ENCODER_TIMEOUT_MS`, safe state is entered (covered by ENC-01)

---

## 7. Overcurrent Tests

> **⚠ Important firmware note:** `CFG_OVERCURRENT_THRESH` is currently `4095U` (maximum ADC value). Because ADC results are clamped to 4095, the software OC threshold can never be exceeded in normal operation — the DRV8873H hardware OCP (triggered by SRP/SRN resistors) handles current limiting. To test the **software** overcurrent path, the threshold must be temporarily lowered in `throttle_config.h` to a value within the expected current sense range (e.g., `500U` ≈ 0.55 A). **Restore after test.**

### OC-01 — Software overcurrent L → safe state (test build)
**Claim:** `safe_check_current()` detects LIS (left current sense) above threshold and enters safe state.

**Procedure (requires temporary config change):**
1. In `throttle_config.h`, set `CFG_OVERCURRENT_THRESH 500U`. Build and flash.
2. ECU in MODE_MANUAL, send `d3000` (75% duty), relay ON (`on` command).
3. Physically stall the motor shaft (grip with insulated gloves or strap to bench).
4. Observe UART.

**Pass criteria:**
- UART prints `[SAFE] Overcurrent L` (or R, depending on drive direction)
- `g_safety.faults` shows `FAULT_OVERCURRENT_L (0x04)` or `FAULT_OVERCURRENT_R (0x08)`
- `g_safety.overcurrent_events` increments
- Motor stops, relay off

**Restore:** Revert threshold to `4095U` and reflash.

---

### OC-02 — DRV8873H hardware OCP (no firmware change needed)
**Claim:** The DRV8873H independently limits motor current through its own OCP circuit (SRP/SRN, charge pump), without relying on firmware.

**Procedure:**
1. ECU in MODE_MANUAL, `d4095` (full duty), relay ON.
2. Hard-stall the motor shaft for 3 seconds.
3. Release.

**Pass criteria:**
- Motor current is limited by DRV8873H auto-retry (you may hear the motor chirping/stuttering — that is OCP retry)
- The motor resumes after shaft release without any firmware intervention
- No safe state is entered (firmware OC check is at max threshold — this verifies hardware-only OCP)
- DRV8873H nFAULT LED (if fitted on PCB) or GPIO1 can be observed going LOW during OCP

---

## 8. CAN Protocol Tests

### CAN-01 — ESTOP frame immediately enters safe state
**Claim:** A CAN frame with ESTOP flag (byte[0] bit2 = 0x04) triggers immediate safe state, even from PID mode.

**Procedure:**
1. ECU in PID mode, throttle active at 50% (`t50`), relay ON.
2. Using CAN tool, send: `cansend can0 100#04.32.00.00` (flags=ESTOP, throttle=50%, seq=0)
3. Observe.

**Pass criteria:**
- Motor stops within one CAN RX service interval (<20ms)
- Relay turns off
- UART prints `[SAFE] CAN ESTOP`
- CAN 0x101: mode byte shows 0x02 (MODE_SAFE), fault_flags shows CAN state

---

### CAN-02 — CAN heartbeat timeout → FAULT_CAN_TIMEOUT
**Claim:** After the first valid RX frame arms the heartbeat, silence >200ms enters safe state.

**Procedure:**
1. Send one valid CAN command frame (e.g., relay on + PID 50%): `cansend can0 100#03.32.00.AA`
2. Stop sending all CAN frames.
3. Start stopwatch.

**Pass criteria:**
- Within 200–250ms of last frame, UART prints `[SAFE] CAN heartbeat timeout`
- `g_safety.faults` has `FAULT_CAN_TIMEOUT (0x40)` set
- Motor stops, relay off
- CAN 0x101 telemetry continues (ECU still transmits — only RX is timed out)

---

### CAN-03 — Auto-recovery does NOT re-engage throttle
**Claim:** After a CAN timeout, auto-recovery clears `safe_state_active` but `s_mode` stays `MODE_SAFE`. The throttle cannot re-engage without an explicit RESET frame.

**Procedure (immediately after CAN-02):**
1. Resume sending valid CAN frames (relay ON + PID 50%): send every 100ms.
2. Watch CCS live: `g_safety.safe_state_active`, `g_safety.recovery_attempts`.
3. Wait 1100ms (1000ms recovery delay + margin).
4. Continue sending PID frames for another 5 seconds.

**Pass criteria:**
- `g_safety.recovery_attempts` increments (auto-recovery fired)
- `g_safety.safe_state_active` becomes `false`
- BUT throttle does NOT move — motor stays off
- CAN 0x101 byte[0] mode bits still show `0x02` (MODE_SAFE)
- Only after sending RESET frame (byte[0]=0x08) does mode change to MANUAL (0x00)

---

### CAN-04 — CAN bus-off recovery and latching
**Claim:** A CAN bus fault (bus-off state) is detected and latched; it sets `FAULT_CAN_BUS_OFF` and enters safe state.

**Procedure:**
1. ECU running, CAN heartbeat active.
2. Briefly short **CAN-H to CAN-L** with a jumper wire for ~200ms, then remove the short.
3. The DCAN peripheral will attempt auto bus-on after 10 transmit error counts (`CAN_setAutoBusOnTime` = 10).

**Pass criteria:**
- During the short: safe state entered, `FAULT_CAN_BUS_OFF (0x80)` set
- After short removed and auto bus-on completes: `g_safety.faults` may clear `CAN_BUS_OFF` (via `safe_can_clear_bus_off_fault()`), but `faults_latched` retains it
- `diag` command shows `Latched Faults: 0x80`

---

### CAN-05 — RESET frame exits safe state with bridge disabled
**Claim:** CAN RESET (byte[0]=0x08) clears safe state but leaves motor bridge DISABLED until operator sends a drive command.

**Procedure:**
1. Enter safe state by any method (e.g., CAN-01 ESTOP).
2. Measure GPIO4 (DISABLE pin) — should be HIGH (bridge off).
3. Send RESET frame: `cansend can0 100#08.00.00.00`
4. Re-measure GPIO4.
5. Without sending any further commands, observe motor.

**Pass criteria:**
- UART prints: `Safe state cleared via CAN — mode=MAN`
- GPIO4 (DISABLE) remains HIGH after reset (bridge stays off)
- `s_motorCmd` is 0 (verify in CCS Expressions)
- Motor does NOT move — operator must explicitly send a PID or duty command
- After sending `d2000` UART command: GPIO4 goes LOW (bridge enabled), motor responds

---

### CAN-06 — UART "reset" matches CAN RESET bridge behavior
**Claim:** UART `reset` command also leaves bridge DISABLED post-reset (fixed in v1.6, was re-enabling bridge).

**Procedure:**
1. Enter safe state (any method).
2. Measure GPIO4 (DISABLE) — HIGH.
3. Type `reset` in UART terminal.
4. Re-measure GPIO4 immediately after.

**Pass criteria:**
- UART prints: `Safe state cleared — mode=MAN`
- GPIO4 remains HIGH after reset
- Motor does not move until explicit drive command is sent

---

## 9. Brake Interlock Tests

### BRK-01 — Brake input forces relay off and throttle to zero
**Claim:** A high brake sense signal (above `CFG_ADC_BRK_THRESH`) forces the relay off and drives throttle to fully-closed, regardless of active CAN commands.

**Hardware setup:** The BRK_SENSE input (GPIO ADCC_IN0, pin 19) goes through a resistor divider. At 12V brake input, the divided voltage is ~2.16V at the ADC pin. To simulate brake-ON without external brake pedal hardware:
- Apply **~1.5V** directly to the BRK_SENSE ADC pin (after the voltage divider) using a bench PSU. This exceeds the threshold (~1.0V / 1241 counts).
- Alternatively, if the brake input has a connector, apply 12V to the brake signal line (top of the divider) as designed.

**Procedure:**
1. ECU in PID mode, relay ON, throttle at 50%.
2. Apply brake signal voltage (method above) — maintain it.
3. Observe UART and CAN telemetry.

**Pass criteria:**
- Within CFG_BRAKE_DEBOUNCE_MS (20ms) of stable brake signal: relay turns off (measure relay coil: ~0V)
- Throttle plate drives toward fully-closed (angle → s_usableMax)
- CAN 0x101 byte[0]: bit5 (brake_on) = 1
- Even if you send CAN relay-ON frames while brake is applied, relay stays off

---

### BRK-02 — Brake holdoff rejects stale CAN frames after release
**Claim:** After brake is released, CAN frames with the same sequence number as the pre-release frame are rejected, preventing immediate re-engagement.

**Procedure (immediately after BRK-01):**
1. Note the current CAN RX seq number (byte[3] of 0x100 frames, or watch `s_last_rx_seq` in CCS).
2. Remove the brake signal voltage.
3. Continue sending CAN PID frames using the **same seq byte** as during brake.
4. Observe throttle.

**Pass criteria:**
- Mode drops to MANUAL after brake release
- Throttle does NOT re-engage while seq is unchanged
- Only after sending a frame with a **different seq byte** does the holdoff clear
- Throttle re-engages normally after holdoff clears and relay command arrives

---

## 10. Solenoid Tests

### SOL-01 — Relay ON with no solenoid → FAULT_SOL_OPEN warning
**Claim:** If relay is commanded ON but no solenoid current is detected, `FAULT_SOL_OPEN` is set in sol_faults (latched warning, does not enter safe state).

**Procedure:**
1. Disconnect the solenoid wires (unplug solenoid connector from PCB).
2. Send relay ON command: UART `on` or CAN with relay flag set.
3. Wait `CFG_SOL_FAULT_DEBOUNCE` iterations (50 main loop ticks ≈ 50ms at ~1 kHz).
4. Type `diag` in UART.

**Pass criteria:**
- UART `diag` shows: `SOL Faults: 0x01 (latched: 0x01)`
- Line `- SOL_OPEN (relay ON, no current detected)` is printed
- CAN 0x102 byte[6] has bit0 set (0x01)
- ECU does NOT enter safe state (SOL_OPEN is a warning only)
- ECU continues running normally

---

### SOL-02 — Solenoid ON confirmed by current
**Claim:** When relay is ON and solenoid is connected, `FAULT_SOL_OPEN` clears.

**Procedure (immediately after SOL-01):**
1. Reconnect the solenoid wires.
2. Wait 50ms.
3. Type `diag`.

**Pass criteria:**
- `SOL Faults: 0x00` (active faults cleared)
- `Latched Faults: 0x01` (history preserved until `clearfaults` command)
- CAN 0x102 byte[6] shows bit3 (SOL_INFERRED_ON = 0x08) set — current above ON threshold

---

## 11. Position Error Tests

### POS-01 — Sustained large position error → safe state
**Claim:** If `|actual_angle - target_angle| > 15%` of usable range persists for >2 seconds, `FAULT_POSITION_ERROR` is set and safe state is entered.

**Prerequisite:** Relay ON, ECU in PID mode.

**Procedure:**
1. Command throttle to 0% (`t0`) — target angle = s_usableMax (fully closed).
2. Manually grip/clamp the throttle plate so it **cannot move** to the target — hold it at roughly 50% position.
3. Start stopwatch when you apply the clamp.
4. Hold for 2100ms (just past the 2-second timeout).
5. Release.

**Pass criteria:**
- After 2 seconds of clamped error: UART prints `[SAFE] pos error >15% for 2s`
- CAN 0x102 byte[6] has `FAULT_POSITION_ERROR (0x20)` set
- Motor stops, relay turns off
- `g_safety.sol_faults_latched` shows `0x20` in CCS Expressions

---

## 12. PID and Control Tests

### PID-01 — Gain clamping via UART
**Claim:** Out-of-range gain commands are clamped to safe maximums (Kp≤200, Ki≤50, Kd≤100).

**Procedure:**
1. Send `p999` via UART (Kp = 999, above max 200).
2. Send `i999` (Ki = 999, above max 50).
3. Send `k999` (Kd = 999, above max 100).
4. Type `config` to read back values.

**Pass criteria:**
- `config` shows: `PID: Kp=200.00 Ki=50.00 Kd=100.00`
- Motor does not make violent movements during the commands

---

### PID-02 — Setpoint slew rate limits step changes
**Claim:** A large instantaneous setpoint change (e.g., 0%→100%) is slew-rate limited to `CFG_SLEW_STEP` (200 units) per `CFG_SLEW_RATE_MS` (10ms).

**Procedure:**
1. In CCS, add `s_slewTarget` to live Expressions watch. (Access via Expressions panel; static vars need the full file path prefix — type `'throttle_ecu.c'::s_slewTarget` or find via Memory Browser.)
2. ECU in PID mode, command `t0` (fully closed).
3. Let throttle settle.
4. Command `t100` (fully open).
5. Watch `s_slewTarget` in CCS live watch for several seconds.

**Pass criteria:**
- `s_slewTarget` changes by at most 200 per 10ms (≈ 20,000 units/sec = ~200°/sec)
- The throttle plate moves smoothly over ~0.5s from closed to open, NOT instantaneously
- No large current spike visible on motor PSU ammeter

---

## 13. Memory Integrity Tests

### MEM-01 — RAM canary corruption detected at runtime
**Claim:** If `g_safety.ram_canary` is overwritten, `safe_tick()` detects it within 100ms and enters safe state.

**Prerequisite:** ECU running normally, CCS connected.

**Procedure:**
1. In CCS Expressions panel, add `g_safety.ram_canary`.
2. Verify it reads `0xDEAD5AFE`.
3. In **Memory Browser**, navigate to the address of `g_safety.ram_canary`.
4. Write `0x00000000` to that address (two 16-bit writes: `0x0000` at addr, `0x0000` at addr+1).
5. Wait 100–200ms.

**Pass criteria:**
- Within 100ms: safe state entered with reason `"RAM canary corrupt"`
- `g_safety.post_result` has `POST_CANARY_FAIL (0x04)` set
- `g_safety.safe_state_active` = `true`
- UART prints `[SAFE] RAM canary corrupt`

---

### MEM-02 — Invalid `s_mode` value → safe state
**Claim:** The `default:` case in `switch(s_mode)` catches RAM-corrupted mode values and enters safe state.

**Prerequisite:** ECU in MODE_MANUAL, CCS connected.

**Procedure:**
1. In CCS Expressions panel, find `s_mode`. (Static var — use `'throttle_ecu.c'::s_mode` or find via Memory Browser with map file.)
2. The current value should be `0` (MODE_MANUAL) or `1` (MODE_PID).
3. Using Memory Browser, write value `0x05` (not a valid RunMode) to `s_mode`'s address.
4. Wait for the next `Throttle_runOnce()` iteration (< 5ms).

**Pass criteria:**
- Immediate safe state: UART prints `[SAFE] invalid mode`
- Motor stops
- `g_safety.safe_state_active` = `true`

---

### MEM-03 — Loop overrun counter increments
**Claim:** `g_safety.loop_overrun_count` increments whenever a single loop iteration exceeds 50ms.

**Prerequisite:** ECU running, CCS connected.

**Procedure:**
1. Add `g_safety.loop_overrun_count` to CCS Expressions, note current value.
2. Set a breakpoint at the very start of `Throttle_runOnce()` (after the `loopStart = Board_millis()` line).
3. Hit the breakpoint — CPU halts.
4. Wait exactly **60ms** (stopwatch — do NOT exceed 800ms or WDT fires).
5. Resume execution.

**Pass criteria:**
- `g_safety.loop_overrun_count` increments by 1
- No watchdog reset (waited < 840ms)
- ECU continues running normally

---

## 14. Operator Reset Tests

### RST-01 — `safe_clear_faults()` guarded: debounce not cleared while fault active
**Claim:** Sending `clearfaults` while an active fault condition exists does NOT reset the debounce counters (prevents one-tick window of undetected fault).

**Procedure:**
1. Unplug encoder MISO (to cause ENC fault condition).
2. Before the 30s timeout fires, send UART command `clearfaults`.
3. Watch `g_safety.fault_count[0]` (encoder debounce counter) in CCS.

**Pass criteria:**
- `fault_count[0]` is NOT reset to 0 while the encoder wire is unplugged and `g_safety.faults != 0`
- Latched faults are cleared (historical records)
- Active fault debounce counter is preserved

---

### RST-02 — Auto-recovery requires explicit operator RESET
**Claim:** After conditions resolve and auto-recovery fires, `s_mode` stays `MODE_SAFE`. Only an explicit RESET frame or command transitions to MANUAL.

**Procedure:**
1. Trigger CAN heartbeat timeout (send one frame, stop sending for 200ms).
2. Resume sending frames (heartbeat restores). Wait 1100ms for `safe_attempt_recovery()` to fire.
3. In CCS, verify `g_safety.safe_state_active` has become `false`, but check CAN 0x101 mode byte.
4. Attempt to command throttle with PID frame — observe.

**Pass criteria:**
- `g_safety.safe_state_active` = `false` (auto-recovery cleared it)
- CAN 0x101 mode byte still = `0x02` (MODE_SAFE — s_mode unchanged)
- Motor stays off despite PID frames
- Only after CAN RESET frame (byte[0]=0x08) does mode become MANUAL (0x00)

---

## 15. Diag Output Validation

### DIAG-01 — `diag` command shows complete status
**Procedure:**
1. After a successful boot (POST passed), type `diag` in UART terminal.

**Pass criteria:** Output includes ALL of the following sections:
- `Safe State: NO`
- `Active Faults: 0x00` with `(none)` underneath
- `Latched Faults: 0x00`
- `Watchdog kicks: [N]`
- `Safe Transitions: 0`
- `SOL Faults: 0x00 (latched: 0x00)`
- `Watchdog: HW WDT ACTIVE (~840ms timeout, enabled in safe_init)`
- `POST result: 0x00 (PASS)`
- `Loop overruns (>50ms): 0`
- `=================================`

---

## 16. Regression Tests (Run After Any Firmware Change)

Run these as a minimum regression suite after any code modification:

| ID | Test | Expected |
|----|------|----------|
| POST-01 | Normal boot | ALL PASS on UART |
| WDT-01 | Halt >840ms via JTAG | Reset + `[WARN] Recovered from watchdog reset!` |
| CAN-01 | ESTOP frame | Immediate safe state |
| CAN-02 | Stop CAN frames | Heartbeat timeout within 200ms |
| CAN-05 | RESET frame | Bridge stays disabled post-reset |
| POS-01 | Mechanical stall 2s | `FAULT_POSITION_ERROR` + safe state |
| BRK-01 | Brake signal | Relay off, throttle to 0% |
| DIAG-01 | `diag` command | All fields present and correct |

---

## 17. Known Limitations and Accepted Residual Risks

| Item | Detail |
|------|--------|
| Software OC threshold disabled | `CFG_OVERCURRENT_THRESH = 4095U` — software OC check never fires. Hardware DRV8873H OCP is the sole current protection. Testing OC-01 requires a temporary threshold change (test build). |
| `safe_check_power()` not called | No supply voltage ADC pin on current PCB. `FAULT_POWER_LOW` cannot be triggered. See `safety.c` comment for activation steps. |
| CAN DLC validation | `CAN_readMessageWithID()` does not return received DLC. Short frames may replay stale seq byte. Full mitigation requires IF2MCTL register read (documented in `can_io.c`). RPN=70 per FMEA. |
| EF flag test reliability | AS5147U EF condition depends on magnet geometry; artificial EF induction may not be reproducible without a controlled fixture. |
| Encoder timeout (30s) | Long timeout minimises nuisance faults; a genuine encoder loss in the first 30s of control is undetected until timeout fires. Accepted per FMEA. |

---

*End of validation test plan. Retain completed test records alongside the firmware release artefact.*
