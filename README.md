# ESP32_MCPWM

Arduino-friendly dual-input H-bridge motor control for ESP32 using the **MCPWM** peripheral rather than LEDC.
The library keeps the control surface small while providing predictable coast, hard-brake, dither-brake, setup validation, runtime retuning, software fault handling, and GPIO edge capture.

- ✅ Arduino Library Manager and PlatformIO package format
- ✅ Beginner alias: `Motor` → `HBridgeMotor`
- ✅ Designed for dual-input bridges such as BTS7960 / IBT-2-style modules
- ✅ Deterministic startup, shutdown, fault, and reconfiguration states
- ✅ FreeRTOS-friendly timer task with tiny GPIO ISRs
- ✅ MIT licensed

Current release: **v1.3.0**

---

## Targets and ESP32 Scope

This is intentionally an **ESP32 / Arduino-ESP32** library. It uses Espressif MCPWM (`driver/mcpwm.h`), `esp_timer`, FreeRTOS critical sections, and Arduino-ESP32 GPIO interrupts. It is not intended as a generic AVR, SAMD, RP2040, or ESP8266 motor API.

- **Primary demonstration MCU:** ESP32-S3 DevKitC-1.
- **Supported target family:** ESP32-family targets where Arduino-ESP32 provides the legacy MCPWM driver used by this release.
- **ESP32-only features:** MCPWM signal routing, timer/counter selection, center-aligned PWM, optional dead-time, runtime frequency retuning, soft-brake scheduling, and GPIO fault/capture fallbacks.
- **Example pins:** GPIO4 through GPIO11. These are exposed general-purpose I/O on the ESP32-S3 DevKitC-1 headers and avoid the S3 strapping pins, USB GPIO19/20, and flash/PSRAM GPIO ranges.

Pin maps differ across the ESP32 family. In particular, GPIO34–39 are input-only on the original ESP32, while some ESP32-S3 module variants reserve GPIO35–37 for octal flash/PSRAM. Always check the exact module and board schematic before copying example pin assignments.

---

## Install

**Arduino IDE:** search for **ESP32_MCPWM** in Library Manager.

**Manual Arduino install:** download the release ZIP, then use _Sketch → Include Library → Add .ZIP Library…_

**PlatformIO:** add `ESP32_MCPWM` to `lib_deps`, install the release archive, or reference the Git repository.

The public include is:

```cpp
#include <ESP32_MCPWM.h>
```

That header provides the shared interface/configuration types, `HBridgeMotor`, and the beginner-friendly alias:

```cpp
using Motor = HBridgeMotor;
```

---

## Quick Start

This example uses the normal ESP32-S3 DevKitC-1 demonstration pins:

```cpp
#include <ESP32_MCPWM.h>

MotorMCPWMConfig hardware{
  4,              // LPWM
  5,              // RPWM
  6,              // EN (-1 if unused)
  MCPWM_UNIT_0,
  MCPWM_TIMER_0,
  MCPWM0A,
  MCPWM0B
};

Motor motor;

void setup()
{
  Serial.begin(115200);
  motor.setup(hardware);

  if (!motor.isSetupComplete())
  {
    Serial.print("Motor setup failed, error code: ");
    Serial.println(static_cast<int>(motor.getLastSetupError()));
    while (true)
      delay(1000);
  }
}

void loop()
{
  motor.setSpeedPercent(50, Dir::CW);
  delay(2000);

  motor.setFreewheel();
  delay(2000);

  motor.setSpeedPercent(50, Dir::CCW);
  delay(2000);

  motor.setFreewheel();
  delay(2000);
}
```

`setup()` remains `void` for source compatibility. Check `isSetupComplete()` before allowing the rest of the application to issue motor commands. Failed setup leaves the object inactive, and control methods are safe no-ops until a valid setup succeeds.

---

## Features

- **Concrete ESP32 driver:** `HBridgeMotor` using two independently routed MCPWM outputs.
- **Shared interface:** `IMotorDriver` for applications that should not depend on the concrete driver type.
- **Logical input range:** configurable with `input_max` and available through `getMaxPwmInput()`.
- **Freewheel modes:** `HiZ`, `HiZ_Awake`, and `DitherBrake`.
- **Hard brake:** A/B at 100% with EN asserted when available.
- **Soft/dither brake:** alternating brake and coast phases at a configured frequency.
- **True Hi-Z dither coast:** optional EN-low coast phase for bridges where enabled A/B=0 is braking.
- **Center-aligned PWM:** supported through `MCPWM_UP_DOWN_COUNTER`.
- **Optional dead-time:** available for suitable half-bridge hardware, but normally wrong for IBT-2/BTS7960 dual-input modules.
- **Runtime PWM retuning:** guarded `reconfigureFrequency(new_hz)` with a deterministic safe transition.
- **Setup validation:** pin, collision, frequency, input-range, and dither-timing checks without fatal resets for bad user configuration.
- **Software fault fallback:** one-shot or level-follow persistence with selectable coast, output-disable, or hard-brake action.
- **Capture fallback:** GPIO edge-interval measurement with callback and non-callback access.
- **Concurrency protection:** dither/output sequence tracking prevents stale timer work from replacing a newer command.
- **Safe lifecycle:** repeated setup, stop, destruction, and failed initialization leave outputs inactive or in the configured safety state.

---

## API Overview

### Types

- `Motor` — beginner alias for `HBridgeMotor`.
- `HBridgeMotor` — concrete MCPWM motor driver.
- `IMotorDriver` — shared abstract interface with safe defaults for optional capabilities.
- `MotorMCPWMConfig` — pins, MCPWM routing, frequency, counter mode, input scaling, and dead-time.
- `MotorBehaviorConfig` — freewheel and dither behavior.
- `MotorSafetyConfig` — optional software fault input and response.
- `MotorCaptureConfig` — optional GPIO edge capture.
- `MotorSetupError` — result of the most recent setup attempt.
- `FaultAction` — `Coast`, `DisableOutputs`, or `HardBrake`.
- `Dir` — `CW` or `CCW`.
- `FreewheelMode` — `HiZ`, `HiZ_Awake`, or `DitherBrake`.
- `CaptureEdge` — `Rising`, `Falling`, or `Both`.
- `CaptureCallback` — GPIO ISR-context edge-interval callback.
- `FaultCallback` — task-context fault notification callback run from `pollFaults()`.

### Methods

```cpp
// Setup and status
void setup(const MotorMCPWMConfig& hw);
void setup(const MotorMCPWMConfig& hw, const MotorBehaviorConfig& beh);
void setup(const MotorMCPWMConfig& hw, const MotorBehaviorConfig& beh,
           const MotorSafetyConfig& safety, const MotorCaptureConfig& cap);
bool isSetupComplete() const noexcept;
MotorSetupError getLastSetupError() const noexcept;

// Control
void setSpeed(int speed, Dir dir) noexcept;
void setSpeedPercent(float percent, Dir dir) noexcept;
void setFreewheel() noexcept;
void setHardBrake() noexcept;
void setSoftBrakePWM(uint16_t pwm) noexcept;
void softBrakeNow(uint16_t pwm) noexcept;  // HBridgeMotor convenience method

// Behavior
void setFreewheelMode(FreewheelMode mode) noexcept;
void applyFreewheel(FreewheelMode mode) noexcept;

// Lifecycle and runtime configuration
void start() noexcept;
void stop() noexcept;
bool reconfigureFrequency(int new_hz) noexcept;

// Safety and diagnostics
bool hasFault() const noexcept;
bool hasEnableControl() const noexcept;
void clearFault() noexcept;
void pollFaults() noexcept;
void setFaultCallback(FaultCallback cb, void* ctx) noexcept;
void forceOutputs(bool a_high, bool b_high) noexcept;

// Information and capture
int getMaxPwmInput() const noexcept;
uint32_t getLastCapturePeriodUs() const noexcept;

// Shared direction helper (call as IMotorDriver::changeDir(...))
static Dir changeDir(Dir dir) noexcept;
```

`pollFaults()` is available through `IMotorDriver` with a safe default implementation, so application code can call it through an interface pointer or reference without forcing unrelated driver implementations to add boilerplate.

`forceOutputs()` is intended for short, controlled diagnostics. It immediately requests 100% on the selected sides and should not be used as a normal speed or direction API.

---

## Configuration

### `MotorMCPWMConfig`

```cpp
MotorMCPWMConfig hw{
  /* lpwm_pin */ 4,
  /* rpwm_pin */ 5,
  /* en_pin   */ 6,              // -1 if EN is not controlled
  /* unit     */ MCPWM_UNIT_0,
  /* timer    */ MCPWM_TIMER_0,
  /* sig_l    */ MCPWM0A,
  /* sig_r    */ MCPWM0B
};

// Optional fields and defaults
hw.pwm_freq_hz     = 20000;
hw.input_max       = 1023;
hw.counter         = MCPWM_UP_COUNTER;
hw.use_deadtime    = false;
hw.deadtime_type   = MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE;
hw.deadtime_red_ns = 500;
hw.deadtime_fed_ns = 500;
```

**Fields**

- `lpwm_pin` / `rpwm_pin` — required output-capable GPIOs for the two H-bridge inputs.
- `en_pin` — optional output-capable bridge enable pin; use `-1` when the library cannot control EN.
- `unit`, `timer` — MCPWM hardware resources owned by this motor instance.
- `sig_l`, `sig_r` — separate MCPWM signals routed to LPWM/RPWM.
- `pwm_freq_hz` — drive PWM frequency; accepted setup range is 1 to 1,000,000 Hz. Hardware may reject a value that cannot be produced for the selected mode.
- `input_max` — maximum logical speed/dither input, valid from 1 to 65,535.
- `counter` — edge-aligned `MCPWM_UP_COUNTER` or another supported MCPWM counter mode such as `MCPWM_UP_DOWN_COUNTER`.
- `use_deadtime` and dead-time fields — optional MCPWM dead-time configuration for suitable hardware.

### `MotorBehaviorConfig`

```cpp
MotorBehaviorConfig behavior{
  /* freewheel_mode          */ FreewheelMode::HiZ,
  /* soft_brake_hz           */ 100,
  /* dither_pwm              */ 80,
  /* default_soft_brake_pwm  */ 0,
  /* min_phase_us            */ 50,
  /* dither_coast_hi_z       */ true
};
```

The default values are `HiZ`, 300 Hz, dither strength 30, initial soft-brake strength 50, a 50 µs minimum phase, and enabled A/B=0 dither coast (`dither_coast_hi_z = false`).

**Fields**

- `freewheel_mode` — strategy applied by `setFreewheel()`.
- `soft_brake_hz` — requested dither frequency, valid from 1 to 10,000 Hz.
- `dither_pwm` — strength loaded when `setFreewheel()` applies `DitherBrake` mode.
- `default_soft_brake_pwm` — initial soft-brake strength used by `setSpeed(0, ...)`.
- `min_phase_us` — requested minimum non-endpoint brake/coast duration. The default is 50 µs.
- `dither_coast_hi_z` — when true, dither coast deasserts EN; when false, coast retains EN and writes A/B=0.

Dither levels above `input_max` are clamped to the configured logical range. The same clamping applies to runtime speed, percentage, and soft-brake requests.

### `MotorSafetyConfig`

```cpp
MotorSafetyConfig safety;
safety.fault_gpio        = 7;                  // -1 disables fault monitoring
safety.fault_active_high = false;              // active-low input
safety.oneshot           = true;               // latch until a valid clear
safety.fault_action      = FaultAction::Coast;
```

`fault_action` and `oneshot` are independent. The action selects the bridge response; `oneshot` selects whether an assertion remains latched or follows the input level. `HardBrake` remains the compatibility default for existing three-field safety configurations.

### `MotorCaptureConfig`

```cpp
MotorCaptureConfig capture;
capture.cap_gpio   = 8;                    // -1 disables capture
capture.edge       = CaptureEdge::Rising;
capture.on_capture = nullptr;              // optional ISR callback
capture.user       = nullptr;
```

The callback is optional because the most recent interval is also available through `getLastCapturePeriodUs()`.

---

## Setup Validation and Status

Setup validates obvious configuration errors before MCPWM initialization. Bad user configuration returns cleanly rather than passing the invalid request to `ESP_ERROR_CHECK()`.

The checks include:

- LPWM and RPWM are present and output-capable.
- LPWM and RPWM are different pins.
- EN is output-capable when configured and does not collide with LPWM/RPWM.
- LPWM and RPWM use different MCPWM output signals.
- Optional fault and capture inputs are valid GPIOs.
- Fault/capture inputs do not collide with output pins or each other.
- Drive PWM frequency is within 1 to 1,000,000 Hz.
- `input_max` is within 1 to 65,535.
- Dither frequency/timing can form a valid period.

### `MotorSetupError`

| Value                 | Meaning                                                                         |
| --------------------- | ------------------------------------------------------------------------------- |
| `None`                | Setup completed successfully.                                                   |
| `InvalidPwmPin`       | LPWM or RPWM is not a usable output pin.                                        |
| `DuplicatePwmPin`     | LPWM and RPWM use the same GPIO.                                                |
| `PinConflict`         | EN, fault, capture, or MCPWM output-signal assignments conflict or are invalid. |
| `InvalidPwmFrequency` | Drive PWM frequency is outside the accepted range.                              |
| `InvalidInputRange`   | Logical input range is invalid.                                                 |
| `InvalidDitherConfig` | Dither timing cannot form a valid period.                                       |
| `HardwareInitFailed`  | MCPWM routing, initialization, duty mode, or dead-time setup failed.            |
| `TimerInitFailed`     | The soft-brake timer could not be created.                                      |

```cpp
motor.setup(hw, behavior);

if (!motor.isSetupComplete())
{
  const MotorSetupError error = motor.getLastSetupError();
  // Report the error and keep the application in its safe state.
}
```

On failure, EN remains inactive when available. If MCPWM initialization had already begun, A/B are returned to zero and MCPWM is stopped. Motor-output methods do nothing until setup succeeds.

Calling `setup()` again is supported. The previous fault/capture interrupts are detached, dither is invalidated and stopped, EN/A/B are made inactive, previous dead-time is disabled, and the old MCPWM timer is stopped before applying the new configuration. A failed repeated setup leaves the object inactive and requires a later valid setup attempt.

---

## Coast, Brake, and Output States

The electrical meaning of A/B=0 is not universal. Check the connected bridge truth table rather than relying only on the labels printed on a module.

| State                      | Library output                                     | Practical meaning                                                                        |
| -------------------------- | -------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| `FreewheelMode::HiZ`       | EN low when controlled, A/B=0                      | Intended high-impedance coast. Without controllable EN, physical Hi-Z is not guaranteed. |
| `FreewheelMode::HiZ_Awake` | EN high, A/B=0                                     | Enabled zero-input state; may coast or brake depending on the bridge.                    |
| Hard brake                 | EN high, A/B=100%                                  | Dynamic electronic brake.                                                                |
| Dither brake               | Alternating hard-brake and configured coast phases | Adjustable average braking force.                                                        |

`setFreewheel()` applies the currently selected mode. `applyFreewheel(mode)` selects and immediately applies a mode.

When the simple `setup(hw)` overload is used without an EN pin (`en_pin = -1`), the library selects `HiZ_Awake` as the default mode because A/B=0 is the only coast-like state it can command. This does not imply that the connected bridge is physically high impedance.

Calling `setFreewheelMode()` with the current mode is a true no-op: it does not change output state, stop/restart dither, or touch the timer. Changing the mode while dither is active stops the old cycle in the configured dither coast state. Call `setFreewheel()` to apply the newly selected mode.

When both MCPWM channels must change together, the library temporarily deasserts a controlled EN pin around the sequential duty writes. This avoids exposing a one-sided drive pulse between the A and B updates.

---

## Dither Braking — Deep Dive

### What It Does

Dither brake alternates between:

- **Brake phase:** EN asserted and both A/B at 100%.
- **Coast phase:** A/B at 0%, with EN either deasserted for Hi-Z or retained for enabled coast.

This produces a repeatable average drag without requiring a background task or timing loop in the sketch.

On many IBT-2/BTS7960-style boards, enabled A/B=0 can behave as dynamic brake. Use `dither_coast_hi_z = true` when the coast half-cycle must lower a correctly wired EN signal.

### Zero and Full Strength

The endpoint behavior is deliberate:

- A zero soft-brake level applies the configured dither **Coast** phase and does not start the timer.
- A full soft-brake level applies steady **Brake** and does not start the timer.
- Only non-zero, non-full levels run the alternating timer cycle.

This applies consistently to `setSpeed(0, ...)`, `softBrakeNow(0)`, an active cycle changed with `setSoftBrakePWM(0)`, DitherBrake freewheel with `dither_pwm = 0`, and setup applying a zero-strength DitherBrake default.

### Period and Phase Calculation

The requested integer period is calculated once from `soft_brake_hz`:

```text
period_us = 1,000,000 / soft_brake_hz
```

For a non-endpoint request:

```text
requested_brake_us = round(period_us × soft_brake_pwm / input_max)
effective_min_us   = clamp(min_phase_us, 1, period_us / 2)
brake_us           = clamp(requested_brake_us,
                           effective_min_us,
                           period_us - effective_min_us)
coast_us           = period_us - brake_us
```

The important invariant is:

```text
brake_us + coast_us == period_us
```

The phase minimum is never applied independently in a way that extends the requested cycle. If `min_phase_us` is too large to fit twice, the effective minimum is reduced to half the period. That fallback preserves frequency but limits the smallest achievable brake fraction.

The practical minimum fraction is approximately:

```text
f_min = effective_min_us / period_us
```

For gentler braking, reduce `min_phase_us`, reduce `soft_brake_hz`, or both. The default 50 µs minimum is conservative for `esp_timer` while remaining much less intrusive than the previous 1500 µs default.

### Runtime Control

```cpp
// Start an 8% dither brake immediately.
motor.softBrakeNow(motor.getMaxPwmInput() * 8 / 100);

// Update the strength while dither is active.
// The old timer sequence is invalidated and a clean cycle restarts from coast.
motor.setSoftBrakePWM(motor.getMaxPwmInput() * 4 / 100);

// Zero strength returns to the configured dither coast with no timer running.
motor.setSoftBrakePWM(0);
```

### Pulse-Skipped Feather-Light Braking

Very small average braking can be demonstrated without another library feature by alternating one dither period with several normal Hi-Z periods:

```cpp
const uint32_t dither_period_ms = 1000UL / 100; // 100 Hz

motor.softBrakeNow(motor.getMaxPwmInput() * 2 / 100);
delay(dither_period_ms);

motor.applyFreewheel(FreewheelMode::HiZ);
delay(dither_period_ms * 9);
```

Keep application-level pulse skipping simple and use it only when that visible low-rate behavior is useful to the project or demonstration.

---

## Dead-Time and Center-Aligned PWM

Center-aligned PWM and dead-time are separate features:

```cpp
hw.counter = MCPWM_UP_DOWN_COUNTER; // center-aligned PWM
hw.use_deadtime = false;            // normal choice for IBT-2/BTS7960
```

MCPWM dead-time modes commonly derive complementary high-side/low-side signals for one half-bridge. That is appropriate when the ESP32 directly controls a suitable gate-driver arrangement, but it breaks the independent LPWM/RPWM truth table expected by many dual-input H-bridge modules.

- **Symptom:** unexpected coupling, surging, or direction behavior after dead-time is enabled.
- **IBT-2/BTS7960 guidance:** leave `use_deadtime = false` unless the exact hardware and waveform have been verified.
- **Repeated setup:** if a previous configuration enabled dead-time, the library explicitly disables it during teardown before applying the next configuration.

---

## Runtime Frequency Changes

`reconfigureFrequency(new_hz)` changes the drive PWM frequency, not the dither frequency.

The call returns `false` when:

- setup has not completed successfully;
- the request is outside 1 to 1,000,000 Hz; or
- the MCPWM driver rejects the request.

For a normal active output, reconfiguration first invalidates dither, deasserts EN when controlled, and writes A/B=0. If dither was active and the frequency change succeeds, dither restarts from its coast phase. Other operating states remain at disabled zero output and require a deliberate new drive command.

```cpp
if (motor.reconfigureFrequency(25000))
{
  Serial.println("Drive PWM changed to 25 kHz.");
  motor.setSpeedPercent(50, Dir::CW); // deliberate new request
}
else
{
  Serial.println("Retune failed; outputs remain inactive.");
}
```

If a fault is active or latched, its configured output action remains authoritative. Frequency configuration does not clear or override the fault.

---

## Fault Handling — Software Fallback

The optional fault input is a **software safety fallback**. It is not MCPWM hardware trip-zone support and does not provide hardware-trip response latency.

### Action and Persistence

`FaultAction` selects the low-level response:

- `Coast` — stop dither, deassert EN when available, write A/B=0, and preserve the current MCPWM running/stopped lifecycle state.
- `DisableOutputs` — stop dither, deassert EN, write A/B=0, and stop MCPWM generation.
- `HardBrake` — stop dither, assert EN, write A/B=100%, and start/keep MCPWM running. This is the compatibility default.

`oneshot` independently selects persistence:

- `true` — latch the first active assertion until `clearFault()` succeeds after the physical input becomes inactive.
- `false` — follow the sampled input level; the inactive transition is processed by `pollFaults()`.

In level-follow mode, `hasFault()` reflects the most recently sampled ISR level rather than performing a separate live GPIO read. Once an inactive edge is sampled, continuously submitted control commands may become eligible again before the deferred clear work runs. Use one-shot mode when recovery must always require an explicit authority/re-arm decision.

### Deferred Processing

The GPIO ISR only reads the fault level and posts state. It does not call MCPWM, `esp_timer`, callbacks, logging, allocation, or delay functions.

Call `pollFaults()` frequently from normal task context:

```cpp
void loop()
{
  motor.pollFaults();

  if (motor.hasFault())
  {
    // Remove the physical fault and apply application-specific re-arm policy.
  }
}
```

`pollFaults()` is virtual on `IMotorDriver`, so the same loop works through an interface reference:

```cpp
void serviceMotor(IMotorDriver& driver)
{
  driver.pollFaults();
}
```

Response latency is bounded by how frequently the application calls `pollFaults()`. `hasFault()` may become true as soon as the ISR samples an assertion, before the task-context output action has run.

Debounce or filter noisy fault sources at the source. The ISR is intentionally small and does not implement a general-purpose debounce policy.

### Fault Active During Setup

Setup installs the input interrupt and samples the configured level. If the input is already active, the selected fault action is applied before `setup()` returns. User callback notification remains deferred until `pollFaults()`.

### Output Inhibition

While `hasFault()` is true, normal output requests cannot override the selected action. This includes speed, percentage, freewheel application, hard brake, immediate dither, raw outputs, start, and stop. Configuration-only calls do not release the fault.

### Clear Behavior

`clearFault()` performs a fresh physical input check:

1. If the input remains active, clear is rejected and the fault action is reapplied.
2. Otherwise pending fault work is discarded and dither is invalidated.
3. EN is deasserted when available and A/B are written to zero.
4. MCPWM is restarted at zero if the fault action had stopped it.
5. The fault latch is released.

Clear never restores a previous speed, direction, duty, or freewheel request. A separate deliberate drive command is required after the application decides that re-arming is safe.

### EN Capability Limits

`hasEnableControl()` reports whether the library was configured with an EN pin. It cannot verify wiring or the bridge truth table.

Without controllable EN:

- `Coast` still writes A/B=0 but cannot guarantee physical high impedance.
- `DisableOutputs` still stops MCPWM but cannot guarantee the bridge output stage is disabled.
- The application must confirm the connected module’s behavior.

---

## Capture — Edge-Interval Measurement

Capture measures the interval between selected GPIO edges using wrap-safe `micros()` arithmetic.

### Non-Callback Access

```cpp
MotorCaptureConfig capture;
capture.cap_gpio = 8;
capture.edge = CaptureEdge::Both;

motor.setup(hw, behavior, MotorSafetyConfig{}, capture);

void loop()
{
  const uint32_t edge_interval_us = motor.getLastCapturePeriodUs();
  if (edge_interval_us != 0)
    Serial.println(edge_interval_us);
}
```

`getLastCapturePeriodUs()` returns zero until two selected edges have been observed. The historical method name contains “Period”, but the stored value is the interval between selected edges.

### Edge Selection

- `Rising` — interval between rising edges; normally one full period for a regular waveform.
- `Falling` — interval between falling edges; normally one full period.
- `Both` — interval between adjacent edges; normally half a period for a symmetrical square wave.

For example, both-edge capture of a symmetrical 1 kHz square wave normally reports about 500 µs rather than the full 1000 µs period.

### Optional ISR Callback

```cpp
volatile uint32_t last_edge_interval_us = 0;

static void IRAM_ATTR onCapture(uint32_t interval_us, void*)
{
  last_edge_interval_us = interval_us;
}

MotorCaptureConfig capture;
capture.cap_gpio   = 8;
capture.edge       = CaptureEdge::Rising;
capture.on_capture = &onCapture;
capture.user       = nullptr;
```

The callback runs in GPIO interrupt context. Keep it IRAM-safe, short, and non-blocking: no `Serial`, allocation, `delay()`, or locks. The first selected edge is tracked explicitly, so a valid first edge at `micros() == 0` is not mistaken for “no data”.

This release intentionally keeps capture as a simple GPIO fallback rather than adding a larger encoder or closed-loop control framework.

---

## Lifecycle and Concurrency Notes

- `start()` restarts MCPWM and reapplies the configured freewheel state.
- `stop()` invalidates dither, deasserts EN, writes A/B=0, and stops MCPWM.
- Destruction detaches interrupts, invalidates dither, disables outputs, disables configured dead-time, stops MCPWM, and deletes the timer.
- Output duty writes are cached to avoid unnecessary MCPWM calls.
- Soft-brake state uses a small critical section; slow GPIO/MCPWM writes are performed outside it.
- Dither, output, and timer sequence counters ensure that stale callback work cannot become the final output after a newer drive, freewheel, fault, reconfiguration, or teardown command.
- Timer start failure falls back to the configured dither coast rather than leaving an unknown phase active.
- All safety-sensitive control methods are safe before setup and while a fault is active.

---

## Numbered Examples

The release examples follow a simple learning sequence and use ESP32-S3 DevKitC-1-compatible pins.

### `01_BasicMotorControl`

One motor, forward/reverse drive, checked setup, and ordinary coast. This is the first sketch to type and wire.

### `02_FreewheelAndDitherBrake`

Compares Hi-Z coast, hard brake, gentle dither brake, and the corrected zero-strength dither behavior.

### `03_FaultInputSafety`

Uses an active-low GPIO fault, `FaultAction::Coast`, regular `pollFaults()`, and separate clear/re-arm commands. The bench test uses only a logic-level button to ground and does not encourage high-current fault testing.

### `04_RuntimeFrequencyChange`

Checks the return value from `reconfigureFrequency()` and submits a new drive request after a successful safe retune.

### `05_CaptureInput`

Generates a repeatable 1 kHz test signal, captures both edges, and demonstrates why the measured interval is about 500 µs.

### `06_TwoMotors`

Extends the basic API to two motors using separate MCPWM timers and signal pairs.

---

## Notes and Practical Guidance

- **Bridge truth table:** do not assume that A/B=0 means coast. Verify EN and input behavior for the exact module.
- **Input range:** `getMaxPwmInput()` mirrors `input_max`; the default is 1023.
- **Percentage control:** `setSpeedPercent()` clamps finite values to 0–100; invalid non-finite input becomes zero.
- **Direction changes:** a controlled EN pin is temporarily lowered when both channels must change, avoiding a one-sided intermediate drive state.
- **Time types:** use fixed-width types such as `uint32_t` for `millis()`/`micros()` arithmetic.
- **Fault callbacks:** run from `pollFaults()` in task context.
- **Capture callbacks:** run from GPIO interrupt context.
- **Dither sound:** audible behavior depends on frequency, minimum phase, motor mechanics, and bridge switching behavior.
- **EN absent:** the library can still command A/B but cannot promise true Hi-Z coast or physical output disable.

---

## Troubleshooting

- **Setup does nothing:** check `isSetupComplete()` and inspect `getLastSetupError()` before debugging motor wiring.
- **`InvalidPwmPin`:** LPWM/RPWM must be present and output-capable on the selected ESP32 target.
- **`PinConflict`:** check EN, fault, capture, and MCPWM signal assignments for duplication.
- **Coast feels like brake on IBT-2:** use `FreewheelMode::HiZ`, provide a working EN pin, and set `dither_coast_hi_z = true` for dither.
- **Zero dither still seems to brake:** confirm that EN low actually disables the connected bridge; A/B state alone cannot guarantee Hi-Z.
- **Dither feels too strong:** reduce `soft_brake_hz`, `min_phase_us`, or the soft-brake level. Inspect the effective minimum fraction described above.
- **Dither frequency seems wrong:** remember that drive PWM frequency and `soft_brake_hz` are independent. The dither phase sum always equals the requested integer dither period.
- **Runtime retune returns false:** setup may be incomplete, the frequency may be outside the accepted range, or the MCPWM driver may not support the requested value/mode.
- **Fault action seems delayed:** call `pollFaults()` more frequently. The GPIO fallback is deliberately deferred and is not hardware trip-zone protection.
- **Fault will not clear:** the configured physical fault level is still active.
- **Both-edge capture reads twice the expected frequency:** `Both` measures adjacent-edge intervals, commonly half the full square-wave period.
- **Unexpected behavior with dead-time:** disable it for IBT-2/BTS7960-style independent dual-input modules.
- **Nothing moves:** verify common ground, motor supply, EN wiring, LPWM/RPWM mapping, and setup status before using `forceOutputs()` for a brief controlled diagnostic.

---

## Testing and Release Checks

Run deterministic host regressions from the repository root:

```sh
sh test/run_native_tests.sh
```

Compile an example and the library for the ESP32-S3 target:

```sh
pio ci -c platformio.ini --lib=. examples/01_BasicMotorControl/01_BasicMotorControl.ino
```

The host suite covers startup and teardown, zero/full dither endpoints, period-preserving phase math, same-mode idempotence, stale callback interruption, runtime retuning, invalid configuration, hardware/timer failures, fault actions and persistence, interface access, and capture semantics.

The release checklist is maintained in [RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md), and release history is recorded in [CHANGELOG.md](CHANGELOG.md).

---

## License

MIT © 2026 Little Man Builds (Darren Osborne)
