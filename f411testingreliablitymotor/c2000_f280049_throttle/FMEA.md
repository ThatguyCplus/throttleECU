# Failure Mode and Effects Analysis (FMEA)
## Throttle-by-Wire ECU — C2000 F280049C
### Document Version: 1.0 | Firmware Version: v1.5 | Date: 2026-09-11

---

## 1. System Overview

### 1.1 System Description

The throttle-by-wire (TBW) ECU controls a mechanical throttle body actuated by a DC motor (DRV8873H H-bridge driver). An AS5147U magnetic angle sensor provides absolute position feedback over bit-bang SPI. The ECU receives throttle commands from an external controller via CAN bus (500 kbps, ISO 11898) and transmits position/fault telemetry back at 50 Hz. A high-side solenoid (TPS1H100B) acts as a hardware relay enabling motor power. A brake sensor overrides all throttle commands and drives the valve closed immediately.

**Key hardware components:**
- Microcontroller: TI F280049C (C28x 100 MHz, 256 KB Flash, 100 KB SRAM)
- Position sensor: ams AS5147U (14-bit absolute, SPI, EF error flag)
- Motor driver: TI DRV8873H (PH/EN mode, OCP/OTW/OTS/UVLO, nFAULT pin)
- Motor power relay: TI TPS1H100B (high-side switch, solenoid gate)
- CAN transceiver: IS2062A (ISO 11898-2, connected via external module)
- Brake sensor: voltage divider on 12 V brake signal → ADCC_IN0

**Operating profile:**
- Supply: 12 V automotive (VM for motor, 3.3 V MCU logic)
- Ambient: −20 °C to +85 °C (engine bay adjacent)
- Duty cycle: continuous operation during vehicle power-on
- Safety-critical output: throttle position directly controls air/fuel to engine

### 1.2 ASIL Rating Rationale

**Target ASIL: C (aspiring toward D for production)**

ISO 26262 ASIL determination (HARA):

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Severity (S) | S3 | Unintended full throttle or stuck throttle → vehicle acceleration → serious injury/death possible |
| Exposure (E) | E4 | Vehicle operating continuously — throttle is always active when engine running |
| Controllability (C) | C2 | Driver has limited ability to override: braking may counter moderate runaway, but high-speed or sustained throttle opening can be fatal |

S3 + E4 + C2 → **ASIL C** per ISO 26262-1 Table B.1.

Justification for not claiming ASIL D: the brake input provides an independent hardware path to close the throttle (brake sensor → immediate close command, relay off). This independent mechanism reduces residual risk. Production systems should implement independent hardware brake interlock (relay de-energised by brake signal directly) to achieve ASIL D.

### 1.3 Safe State Definition

The system's **safe state** is: motor outputs disabled (PWM = 0, DRV8873H DISABLE pin HIGH), relay (TPS1H100B) de-energised, throttle valve driven closed by return spring. This is fail-safe by design — spring return ensures closed throttle on power loss.

---

## 2. FMEA Table

**Severity scale:** 10=death or serious injury, 7-9=serious harm (hospitalisation), 4-6=degraded operation (stranded/vehicle damage), 1-3=minor inconvenience (nuisance fault)

**Occurrence scale:** 10=inevitable, 7-9=frequent (multiple times per year in fleet), 4-6=occasional (once per year per vehicle), 1-3=rare (once per 10 years per vehicle)

**Detection scale:** 10=undetectable, 7-9=hard to detect (detected only in field), 4-6=some detection (detected in test or by telemetry), 1-3=easily detected (detected in-loop within 100 ms)

**RPN = Severity × Occurrence × Detection**

---

### 2.1 Throttle Position Sensor — AS5147U Magnetic Encoder

| # | Item | Function | Failure Mode | Local Effect | System Effect | End User Effect | S | O | D | RPN | Current Controls | Recommended Action | Status |
|---|------|----------|-------------|-------------|---------------|-----------------|---|---|---|-----|------------------|-------------------|--------|
| 1.1 | AS5147U | Provide 14-bit absolute angle | EF flag asserted (CORDIC overflow / field out of range) | read_angle_raw() returns 0xFFFF | EncoderGpio_getAngle() holds last-good average; safe_check_encoder() sees invalid angle | Throttle holds last commanded position; after CFG_ENCODER_TIMEOUT_MS (30 s) → safe state | 7 | 3 | 2 | 42 | EF bit checked every read; 8-sample rolling average; safe_state triggered on sustained loss | Reduce CFG_ENCODER_TIMEOUT_MS to <2 s; add EF count to CAN telemetry | Open |
| 1.2 | AS5147U / Magnet | Magnetic field out of operational range (magnet moved/missing) | EF flag set permanently | Encoder always reports 0xFFFF | No valid position; after 30 s timeout, encoder fault → safe state | Vehicle stranded — throttle closes | 7 | 2 | 2 | 28 | EF flag detection; timeout → safe state | Magnet retention design review; reduce timeout to 2 s | Open |
| 1.3 | AS5147U / SPI bus | SPI communication loss (CS stuck, CLK noise, MISO floating) | Spurious or stuck angle readings | Raw angle reads garbage | Spike rejection filter may accept corrupt data; angle outliers cause position errors | Throttle jerks, large-error fault (15%/2s) → safe state | 8 | 2 | 4 | 64 | 8-sample average; 500 unit spike reject (CFG_ENC_SPIKE_THRESH); 3-consecutive-spike flush; EF bit | Add SPI parity/CRC verification; hardware SPI with DMA checksum | Open |
| 1.4 | AS5147U | Stuck angle value (frozen output, internal latch) | Same angle returned every read | Spike filter accepts it (no spike = no rejection) | PID sees zero error; motor holds current position at wrong angle | Throttle stuck at incorrect position up to 2 s before large-error fault fires | 8 | 2 | 4 | 64 | Large-error fault: if pos_error >15% for 2 s → safe state | Add angular velocity check: if angle unchanging during commanded motion for >500 ms → fault | Open |
| 1.5 | AS5147U / Calibration | CFG_ANGLE_MIN == CFG_ANGLE_MAX (miscalibration) | Zero-range denominator | _Static_assert fires at compile time | Build fails — not deployable with miscal | N/A — caught at compile | 9 | 1 | 1 | 9 | `_Static_assert((MAX-MIN) >= 1000)` | Keep static assert; add runtime cross-check at init | Implemented |

---

### 2.2 Motor Driver — DRV8873H

| # | Item | Function | Failure Mode | Local Effect | System Effect | End User Effect | S | O | D | RPN | Current Controls | Recommended Action | Status |
|---|------|----------|-------------|-------------|---------------|-----------------|---|---|---|-----|------------------|-------------------|--------|
| 2.1 | DRV8873H | Drive motor via EN/DIR (PH/EN mode) | Overcurrent protection (OCP) triggers | nFAULT asserts LOW, outputs disabled (auto-retry) | Motor stops mid-travel; safe_check_drv_nfault() sets FAULT_DRV_NFAULT in sol_faults (warning, no safe state) | Throttle holds position during OCP; may recover after auto-retry cycle | 6 | 3 | 3 | 54 | nFAULT debounced (10 counts), set as diagnostic warning; IPROPI current monitoring | Elevate OCP after N retries to safe state; add retry counter | Open |
| 2.2 | DRV8873H | UVLO — VM below undervoltage lockout (~6 V) | nFAULT asserts; outputs disabled | Motor loses drive; throttle closes via return spring | Throttle closes — safe by design | Momentary loss of throttle (engine stumble) during cranking | 5 | 4 | 3 | 60 | nFAULT monitored; sol_faults bit set; motor disable = fail-safe (spring closes) | UVLO at idle (VM absent) not a fault; no action required | Implemented (documented) |
| 2.3 | DRV8873H | Thermal shutdown (OTS > 150 °C) | nFAULT asserts, outputs disabled | Motor stops | Throttle held by spring; position error may accumulate; nFAULT warning | Vehicle may derate; throttle position unknown | 7 | 2 | 3 | 42 | nFAULT sets diagnostic warning; large-error fault fires if position diverges for 2 s | Enter safe state on thermal shutdown (not just warning) after confirmed OTS via temp sensor | Open |
| 2.4 | DRV8873H / GPIO0 | PWM signal loss (CPU halted, EPWM peripheral reset) | PWM duty = 0 by default (safe output) | Motor coasts, then throttle closes via spring | Throttle closes — safe by design | Vehicle decelerates | 6 | 2 | 2 | 24 | Watchdog resets MCU if main loop halts; EPWM AQ configured LOW on reset | WDT timeout → MCU reset → relay off, DISABLE high | Implemented |
| 2.5 | DRV8873H / GPIO2 | Direction (DIR) signal corruption (RAM fault writes wrong GPIO state) | Motor driven in wrong direction | Throttle moves away from setpoint | Large-error fault fires after 2 s | Vehicle sees unexpected throttle motion | 8 | 2 | 4 | 64 | Large-error fault (15%/2 s) → safe state | ECC on GPIO shadow register; readback verify after write | Open |
| 2.6 | DRV8873H / GPIO4 | DISABLE stuck LOW (outputs permanently enabled) | Motor cannot be disabled by software | setMotor(0) call doesn't disable outputs | Motor remains powered in safe state | No fail-safe — spring return still works but motor may fight it | 8 | 1 | 6 | 48 | Spring return closes throttle even if motor runs; relay off cuts power path | Verify DISABLE pin readback after write in safe state | Open |

---

### 2.3 CAN Bus

| # | Item | Function | Failure Mode | Local Effect | System Effect | End User Effect | S | O | D | RPN | Current Controls | Recommended Action | Status |
|---|------|----------|-------------|-------------|---------------|-----------------|---|---|---|-----|------------------|-------------------|--------|
| 3.1 | CAN bus | Receive throttle commands | Timeout — no frames for >200 ms | FAULT_CAN_TIMEOUT set | safe_enter_safe_state("CAN heartbeat timeout") → relay off, motor off | Throttle closes; vehicle decelerates | 4 | 3 | 2 | 24 | Heartbeat: CFG_CAN_HEARTBEAT_EN; 200 ms timeout → safe state | Timeout already implemented; verify timeout on both sides | Implemented |
| 3.2 | CAN bus | Data integrity | Bus-off state (TEC > 255 — dominant error storm) | CAN_STATUS_BUS_OFF detected | safe_can_set_bus_off_fault() → safe state immediately | Throttle closes | 5 | 2 | 2 | 20 | Bus-off detection + auto-recovery (setAutoBusOnTime 10 ms); FAULT_CAN_BUS_OFF set | Already implemented | Implemented |
| 3.3 | CAN / frame | Data integrity | Frame corruption (CRC error, bit error) | CAN hardware discards frame (CRC is in hardware) | Discarded frame counts as missed; sufficient consecutive misses → timeout | Throttle holds position briefly, then closes on timeout | 5 | 3 | 2 | 30 | Hardware CRC filtering; timeout fault; sequence byte allows gap detection | Add application-level E2E CRC (e.g. CRC-8 in byte 3) for additional coverage | Open |
| 3.4 | CAN | Security | Spoofed command frame (attacker transmits 0x100 with ESTOP or RELAY) | ESTOP: safe state triggered; RELAY: relay enabled against brake state | Unintended safe state or relay on | Nuisance: unexpected safe state; safety concern: relay on while brake active | 7 | 1 | 7 | 49 | Brake state machine overrides relay; ESTOP → safe state (safe direction); sequence byte provides some replay detection | Add message authentication (CMAC); restrict CAN gateway access | Open |
| 3.5 | CAN / RX frame | Protocol conformance | Wrong DLC (short frame — e.g. 2 bytes received) | msgData[3] (sequence byte) is uninitialised memory | Sequence byte comparison unreliable; brake holdoff logic may clear prematurely | Brake holdoff cleared on old seq — throttle may re-engage after brake release | 7 | 2 | 5 | 70 | CAN_setupMessageObject with CFG_CAN_RX_DLC=4 filters at message object level; short frames still received (C2000 DCAN does not hard-reject by DLC in RX object config) | CAN_readMessageWithID API does not return DLC; add DLC read from IF register or document limitation | Open (comment added — see 2a) |
| 3.6 | CAN / TX | Communication | TX flood (CAN_sendMessage called every loop) | Bus load excessive if loop rate > 50 Hz TX rate | None — rate-limited by CFG_CAN_TX_RATE_MS=20 ms | None observable | 2 | 1 | 2 | 4 | TX rate gate; s_lastTxMs guard | Already rate-limited | Implemented |

---

### 2.4 PID Controller

| # | Item | Function | Failure Mode | Local Effect | System Effect | End User Effect | S | O | D | RPN | Current Controls | Recommended Action | Status |
|---|------|----------|-------------|-------------|---------------|-----------------|---|---|---|-----|------------------|-------------------|--------|
| 4.1 | PID — integral | Reduce steady-state error | Integral windup during setpoint step | s_int accumulates up to ±CFG_PID_INTEGRAL_LIMIT (2000) | Overshoot when setpoint changes; motor saturates briefly | Throttle overshoots, recovers within ~0.5 s | 5 | 4 | 4 | 80 | iLimit clamped to ±2000; Pid_reset() on mode change; Pid_resetIntegral() during slew steps | Pid_resetIntegral() implemented during slew; anti-windup clamping already present | Implemented |
| 4.2 | PID — dt overflow | Time delta for integral/derivative | dt overflow at 71 min (ms→µs conversion) | dt becomes very large or wraps; integral diverges | PID output saturates; motor runs at max duty | Uncontrolled throttle motion → safe state via large-error fault | 9 | 3 | 6 | 162 | Fixed: nowMs used directly (ms, not µs); uint32_t subtraction handles rollover; dt clamped to 50 ms max | Already fixed in v1.5 | Implemented |
| 4.3 | PID — gains | Control stability | Gain misconfiguration (Kp=200, Ki=50 entered by operator) | Oscillation, current spikes | Overcurrent fault possible; large-error fault if oscillation persists | Throttle oscillates, system enters safe state | 6 | 3 | 3 | 54 | UART PID gain clamped: Kp≤200, Ki≤50, Kd≤100; CFG_KP/I/D_DEFAULT are tuned safe values | Gains clamped; add non-volatile storage for tuned gains to prevent re-entry | Implemented |
| 4.4 | PID — deadband | Avoid motor jitter at setpoint | Deadband too wide (CFG_PID_DEADBAND = 100 units = 1°) | Large dead zone; position accuracy poor | Throttle settles 1° from target; acceptable for most uses | Vehicle AFR slightly off at light throttle | 3 | 2 | 3 | 18 | s_settled flag + 500 ms settle window; spring FF compensates at open end | Deadband acceptable for current hardware; reduce if mechanical hysteresis reduced | No action |
| 4.5 | PID — slew | Setpoint rate limiting | Slew too fast (CFG_SLEW_STEP=200, 200°/s) | Large current spike if throttle jumps full range in 0.5 s | VM sag; UVLO possible | Momentary power droop during step | 4 | 3 | 4 | 48 | Slew limiter: 200 units per 10 ms; Pid_resetIntegral() clears windup per slew step | Slew rate acceptable; reduce if VM sag observed | No action needed |

---

### 2.5 Power Supply

| # | Item | Function | Failure Mode | Local Effect | System Effect | End User Effect | S | O | D | RPN | Current Controls | Recommended Action | Status |
|---|------|----------|-------------|-------------|---------------|-----------------|---|---|---|-----|------------------|-------------------|--------|
| 5.1 | 12 V supply / VM | Power motor via DRV8873H | Undervoltage (VM < 9 V) | UVLO on DRV8873H; nFAULT asserted; also FAULT_POWER_LOW from safe_check_power() | Safe state if 9 V threshold exceeded; throttle spring closes | Vehicle stumble or stall — throttle closes during low-voltage event | 6 | 3 | 3 | 54 | FAULT_POWER_LOW at CFG_POWER_LOW_MV=9000 with 4-count debounce; nFAULT via DRV8873H | safe_check_power() not called in main loop (no supply ADC fitted) — note this path is inactive | Open |
| 5.2 | 12 V supply | Overvoltage (VM > 18 V load dump) | DRV8873H PVDD clamp activated; possible driver damage | Driver damage → motor disabled | Throttle spring closes | Vehicle loss of throttle | 6 | 2 | 6 | 72 | DRV8873H has internal clamp diodes; PCB TVS or MOV recommended | Add TVS diode on VM rail; supplier review | Open |
| 5.3 | 3.3 V MCU supply | Power MCU logic | 3.3 V brownout | MCU reset (BOR) | Relay off, DISABLE high on reset — spring closes throttle | Unexpected throttle close | 5 | 2 | 3 | 30 | F280049C internal BOR reset; boot with relay/motor disabled | No additional action needed | Implemented |
| 5.4 | 3.3 V / VM | VM present, 3.3 V absent | MCU off — no control signals | PWM = 0 (no drive); relay not energised | Throttle spring closes | Vehicle decelerates | 3 | 1 | 2 | 6 | Fail-safe by design (relay requires active MCU command) | OK | OK |

---

### 2.6 Solenoid / High-Side Relay (TPS1H100B)

| # | Item | Function | Failure Mode | Local Effect | System Effect | End User Effect | S | O | D | RPN | Current Controls | Recommended Action | Status |
|---|------|----------|-------------|-------------|---------------|-----------------|---|---|---|-----|------------------|-------------------|--------|
| 6.1 | TPS1H100B | Enable motor power path | Open circuit (output fails off) | Motor loses power; solenoid current = 0 while relay commanded ON | FAULT_SOL_OPEN set after 50 ms debounce; motor cannot drive throttle | Throttle closed by spring; safe state | 4 | 2 | 2 | 16 | SOL_OPEN detection from ADC current sense; latched warning in sol_faults | Already implemented | Implemented |
| 6.2 | TPS1H100B | De-energise on safe state | Welded / stuck-on output | Motor remains powered when relay commanded off | FAULT_SOL_WELDED check removed (current sense too noisy); throttle remains controllable but relay interlock lost | Motor fights spring on safe state; safety depends on PWM = 0 | 8 | 1 | 7 | 56 | PWM=0 in safe state (motor coasts even if relay on); spring return closes throttle; FAULT_SOL_WELDED latched (welded check disabled in current code) | Re-enable FAULT_SOL_WELDED detection with improved filtering; add hardware interlock | Open |
| 6.3 | TPS1H100B | Relay chatter (oscillation between on/off) | Relay switches rapidly | SOL current oscillates; brake debounce may not settle | FAULT_SOL_OPEN triggers repeatedly | Operator sees oscillating fault; possible relay damage | 4 | 2 | 4 | 32 | 50-count debounce on SOL_OPEN (CFG_SOL_FAULT_DEBOUNCE=50) | Increase debounce if chatter observed; add rate limiter on relay commands | No action |
| 6.4 | TPS1H100B | Control via GPIO7 | GPIO7 stuck high (relay always on) | Motor path permanently enabled | Relay interlock ineffective; safe state requires PWM=0 to stop motor | Motor may run during safe state; spring return is backup | 7 | 1 | 6 | 42 | PWM disabled in safe state; motor DISABLE pin still active | GPIO readback verify; hardware relay monitor via SOL_CS current | Open |

---

### 2.7 Brake Sensor

| # | Item | Function | Failure Mode | Local Effect | System Effect | End User Effect | S | O | D | RPN | Current Controls | Recommended Action | Status |
|---|------|----------|-------------|-------------|---------------|-----------------|---|---|---|-----|------------------|-------------------|--------|
| 7.1 | BRK_SENSE / ADCC_IN0 | Detect brake press | ADC noise causing false brake assertion | Brake state machine activates; throttle closes; relay off | Nuisance false brake (single shot) quickly debounced | Momentary throttle dip; not safety-critical | 3 | 3 | 2 | 18 | 20 ms debounce (CFG_BRAKE_DEBOUNCE_MS); ADC 16× oversampling | Debounce adequate | Implemented |
| 7.2 | BRK_SENSE | Detect brake press | ADC stuck at 0 (short to GND) | Brake always reads not-pressed | Brake interlock not active; CAN throttle commands pass through during brake press | Vehicle throttle not auto-closed on brake — depends on driver | 8 | 2 | 6 | 96 | ADC value clamped to 4095 max; no stuck-low detection | Add plausibility check: if sol is enabled and brake sensor reads 0 for >5s despite brake press (via driver model), flag fault | Open |
| 7.3 | BRK_SENSE | Detect brake release | Holdoff sequence byte stale | If s_last_rx_seq never changes (controller stopped sending), holdoff never clears | Throttle stays in MANUAL mode indefinitely post-brake — safe but prevents re-engagement | Driver cannot re-engage throttle via CAN until fresh frame arrives | 3 | 2 | 3 | 18 | Holdoff clears on any new seq; timeout or fresh CAN frame clears it | Add holdoff timeout (max 5 s) after which holdoff clears regardless | Open |
| 7.4 | BRK_SENSE / resistor divider | Scale 12 V brake signal to ADC range | Resistor open (R1 or R7 fails open) | ADCC_IN0 reads 0 — brake appears always-released | Brake interlock inactive during brake press | Driver applies brake; ECU does not close throttle; collision risk | 9 | 1 | 7 | 63 | None (no redundancy on brake sensor path) | Add second brake sense channel or hardware brake-relay interlock (relay driven by brake signal directly) | Open |

---

### 2.8 Microcontroller

| # | Item | Function | Failure Mode | Local Effect | System Effect | End User Effect | S | O | D | RPN | Current Controls | Recommended Action | Status |
|---|------|----------|-------------|-------------|---------------|-----------------|---|---|---|-----|------------------|-------------------|--------|
| 8.1 | F280049C | Program execution | Watchdog timeout (main loop halts) | Hardware WDT (840 ms timeout) resets MCU | MCU reboots; relay off, motor off; throttle spring closes | Vehicle decelerates; driver warned | 5 | 2 | 2 | 20 | WDT enabled with prescale 64; SysCtl_serviceWatchdog() every loop; WDT reset detected in safe_init() | WDT already implemented | Implemented |
| 8.2 | F280049C / Flash | Store firmware | Flash bit error (single-bit correctable, multi-bit uncorrectable) | MCU executes corrupted instructions or hard faults | Unpredictable behaviour; likely MCU exception → reset | Vehicle loses throttle | 7 | 1 | 7 | 49 | F280049C has Flash ECC (hardware); MCU reset on uncorrectable ECC error | Add ROM checksum verification at startup (CRC32 over .text section) | Open |
| 8.3 | F280049C / SRAM | Hold g_safety, stack, data | RAM bit error (single-bit or multi-bit) | Corrupted fault flags, mode enum, or stack | s_mode may become invalid value (not 0/1/2); `default:` case → safe state | Safe state triggered by corrupted mode | 8 | 1 | 5 | 40 | `default:` case in RunMode switch → safe state; F280049C has SRAM ECC (hardware) | Add periodic RAM scrubbing; ECC error interrupt to safe state | Open (default case implemented) |
| 8.4 | F280049C | Interrupt handling | Stack overflow (ISR nested deeper than stack depth) | Stack wraps into data region; corrupts variables | g_safety or s_mode corrupted; unpredictable | Unpredictable — likely reset or stuck throttle | 8 | 1 | 7 | 56 | C28x has small ISR set (1 ISR: Timer0 ISR); no nested interrupts enabled | Add stack canary; linker section check at startup | Open |
| 8.5 | F280049C / Timer0 ISR | Maintain g_millis 1 ms tick | Timer ISR fails to fire (INT_TIMER0 disabled) | Board_millis() frozen at last value; all time-based checks fail | Watchdog times not checked; safe state never entered; motor runs indefinitely | Uncontrolled throttle if setpoint drifts | 9 | 1 | 7 | 63 | g_millis is volatile; DINT/EINT around read; timer ISR registered at boot | Monitor millis increment: if unchanged for >50 ms in main loop, treat as fault | Open |

---

### 2.9 Safe State Transitions

| # | Item | Function | Failure Mode | Local Effect | System Effect | End User Effect | S | O | D | RPN | Current Controls | Recommended Action | Status |
|---|------|----------|-------------|-------------|---------------|-----------------|---|---|---|-----|------------------|-------------------|--------|
| 9.1 | safe_enter_safe_state() | Enter safe state on fault | Stuck in safe state — auto-recovery never fires | g_safety.faults never clears (active condition persists) | safe_can_recover() returns false; vehicle stays in safe state | Throttle permanently closed; vehicle stranded | 3 | 3 | 2 | 18 | safe_attempt_recovery() in safe_tick() checks faults==0 and 1s elapsed | Already handled — safe by design | Implemented |
| 9.2 | safe_attempt_recovery() | Auto-exit safe state when faults cleared | Spurious safe state exit (recovery fires while condition persists) | safe_state_active=false; mode stays MODE_SAFE; CAN RESET needed to re-engage | s_mode remains MODE_SAFE so throttle stays closed until operator RESET | Vehicle cannot re-engage until driver sends RESET frame | 5 | 2 | 3 | 30 | safe_can_recover() requires faults==0 AND 1s elapsed; s_mode stays MODE_SAFE after auto-recovery | Confirm safe by design — s_mode stays SAFE after auto-recovery (see 2g documentation) | Implemented |
| 9.3 | Throttle_CanRxApply | CAN RESET without fault clearance | safe_clear_faults() called when active faults still present | safe_clear_faults() guards: only clears if g_safety.faults == 0 | Clear rejected silently; safe state persists until real fault clears | Operator sends RESET, no effect — may be confusing | 4 | 3 | 4 | 48 | safe_clear_faults() guards against clearing active faults; returns without clearing | Add UART/CAN error response explaining why RESET was rejected | Open |
| 9.4 | enterSafeStateEc() | Unified safe state entry | enterSafeStateEc() called concurrently from ISR and main loop | No ISR calls enterSafeStateEc; all paths are from main loop only | No race condition | N/A | 2 | 1 | 2 | 4 | All safe state entry from main loop only; no ISR path | Verify no future ISR paths added without DINT guard | Implemented |

---

### 2.10 Firmware Update

| # | Item | Function | Failure Mode | Local Effect | System Effect | End User Effect | S | O | D | RPN | Current Controls | Recommended Action | Status |
|---|------|----------|-------------|-------------|---------------|-----------------|---|---|---|-----|------------------|-------------------|--------|
| 10.1 | Flash programmer | Load new firmware | Partial flash write (power loss during update) | Flash partially written; MCU boots into invalid state | Likely hard fault or WDT reset loop; throttle inoperable | Vehicle requires re-flashing; cannot drive | 4 | 2 | 6 | 48 | F280049C Flash ECC detects uncorrectable errors; WDT reset on hang | Implement dual-bank bootloader with version rollback | Open |
| 10.2 | Firmware version | Protocol compatibility | Version mismatch between ECU and GUI/controller (wrong DLC or byte layout) | CAN RX parses bytes in wrong positions | Wrong throttle % or flags applied from mangled frame | Incorrect throttle command; position error fault fires | 6 | 3 | 5 | 90 | CFG_FW_VERSION transmitted in 0x102 byte[7]; GUI can read and warn | Add version handshake: ECU rejects commands if no version ACK within 1 s of startup | Open |
| 10.3 | Firmware version | Version tracking | Version not bumped after code change | GUI shows stale version; operator unaware of changed behavior | No safety effect; diagnostic confusion | None safety-related | 2 | 4 | 5 | 40 | CFG_FW_VERSION_MINOR must be manually bumped; no enforcement | Add CI lint rule to require version bump with certain file changes | Open |

---

## 3. Risk Matrix

Grouped by Severity × Occurrence (S × O) without considering detection. High-RPN items (S×O ≥ 24) are highlighted.

```
           Occurrence
           O1-3 (Rare)    O4-6 (Occasional)    O7-9 (Frequent)
         ┌──────────────┬──────────────────────┬──────────────────┐
S7-10    │ 1.2, 2.5,    │ 1.1, 1.3, 1.4,       │  (none)          │
(High    │ 2.6, 7.4,    │ 2.1, 2.3, 3.5,       │                  │
 Severity│ 8.2, 8.4,    │ 4.1, 4.2, 5.2, 7.2   │                  │
 ≥7)     │ 8.5, 10.2    │                       │                  │
         ├──────────────┼──────────────────────┼──────────────────┤
S4-6     │ 2.4, 3.2,    │ 2.2, 3.1, 4.3, 4.5,  │  4.1 (also ≥7)  │
(Medium  │ 5.3, 5.4,    │ 5.1, 9.2             │                  │
 Severity│ 6.1, 10.1    │                       │                  │
 4-6)    │              │                       │                  │
         ├──────────────┼──────────────────────┼──────────────────┤
S1-3     │ 1.5, 9.4     │ 7.3, 9.3             │  3.6, 7.1, 10.3  │
(Low     │              │                       │                  │
 Severity│              │                       │                  │
 1-3)    │              │                       │                  │
         └──────────────┴──────────────────────┴──────────────────┘
```

**Highest-RPN items (RPN ≥ 60) requiring priority action:**

| RPN | ID | Item | Primary Risk |
|-----|----|------|-------------|
| 162 | 4.2 | PID dt overflow | Already fixed v1.5 |
| 96 | 7.2 | Brake sensor stuck low | No redundancy on brake sense |
| 90 | 10.2 | Firmware version mismatch | Protocol confusion |
| 80 | 4.1 | Integral windup | Addressed via Pid_resetIntegral() |
| 72 | 5.2 | VM overvoltage | Need TVS diode |
| 70 | 3.5 | CAN wrong DLC | API limitation documented |
| 64 | 1.3 | SPI bus corruption | Need hardware SPI + CRC |
| 64 | 1.4 | Stuck encoder value | Need velocity check |
| 64 | 2.5 | DIR signal corruption | Need GPIO readback |
| 63 | 7.4 | Brake sense resistor open | Need hardware redundancy |
| 63 | 8.5 | Timer ISR disabled | Need millis stall detection |

---

## 4. Remaining Open Items

These items have been identified during FMEA but are not yet implemented in firmware v1.5. They are listed in priority order.

### High Priority (RPN ≥ 63, Safety-Critical)

1. **Hardware brake interlock** (items 7.2, 7.4): Brake sensor is single-point; R1/R7 open-circuit failure silently disables brake interlock. Implement: relay driven by brake signal via hardware AND gate, independent of MCU. ASIL D prerequisite.

2. **Encoder timeout reduction** (items 1.1, 1.2): CFG_ENCODER_TIMEOUT_MS = 30,000 ms is far too long for a safety-critical position sensor. Reduce to 2,000 ms maximum. A position sensor that has been invalid for 30 seconds should have triggered safe state in 2 seconds.

3. **Encoder velocity stall check** (item 1.4): A frozen encoder value is undetectable by the existing spike filter. Add: if in MODE_PID and motor_cmd > MIN_DUTY_THRESH and |position_change| < STALL_THRESH for >500 ms, flag encoder stall fault.

4. **Timer ISR stall detection** (item 8.5): If g_millis does not increment between consecutive main loop iterations, the 1 ms tick has died. Add a cycle-counter cross-check: read Board_cycleCounter(); if millis unchanged after 5000 cycles (~50 µs), enter safe state.

### Medium Priority (RPN 42-62, Operational Safety)

5. **VM overvoltage protection** (item 5.2): Add external TVS or MOV on VM rail. PCB review required. DRV8873H clamps may not be sufficient for 40 V load dump.

6. **ROM checksum at startup** (item 8.2): Compute CRC32 over .text Flash section in safe_init(). If CRC fails, do not proceed — hold MCU in reset with relay off.

7. **Stack canary** (item 8.4): Place known pattern at stack bottom in linker CMD file; check it in safe_tick() every 100 ms. If corrupted, enter safe state.

8. **Firmware version handshake** (item 10.2): ECU should transmit version in first CAN frame and optionally reject commands until controller acknowledges matching version.

### Low Priority (RPN < 42, Diagnostic/Operational)

9. **E2E CRC on CAN RX** (item 3.3): Add CRC-8 in byte 3 of 0x100 frame (currently used for sequence number — would require protocol revision). Provides protection against frame corruption that passes hardware CRC.

10. **FAULT_SOL_WELDED re-enable** (item 6.2): Re-enable welded contact detection with improved filtering once current sense noise characterised.

11. **DLC validation via register read** (item 3.5): After CAN_readMessageWithID, read IF2MCTL register DLC field directly to validate received DLC == CFG_CAN_RX_DLC. Reject frame if mismatch.

12. **Brake holdoff timeout** (item 7.3): Add a 5-second maximum holdoff: if no new seq received within 5 s of brake release, clear holdoff regardless. Prevents permanent MANUAL lockout if CAN controller stops sending.

13. **PID auto-recovery operator re-enable** (item 9.2): After auto-recovery from CAN timeout, document clearly that s_mode stays MODE_SAFE (safe by design) — this is already the case, but should be verified by regression test.

14. **Dual-bank bootloader** (item 10.1): Implement A/B firmware partition with rollback on failed boot. Prevents brick on partial flash write.

---

## 5. Document Sign-Off

| Role | Name | Date |
|------|------|------|
| Author | Claude Sonnet 4.6 (AI assistant) | 2026-09-11 |
| Review | (Pending — must be reviewed by qualified Functional Safety Engineer) | — |
| Approval | (Pending — required before ASIL C claim) | — |

**Note:** This FMEA was generated from source code analysis and is intended as a starting point for formal safety review. It does not constitute a certified ISO 26262 safety analysis. Independent review by a qualified Functional Safety Engineer is required before any production deployment claim.
