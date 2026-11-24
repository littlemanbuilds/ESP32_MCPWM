# ESP32_MCPWM

A focused Arduino-ESP32 motor-control library for driving an H-bridge with the ESP32 **MCPWM** peripheral.

ESP32_MCPWM is intended for projects that have grown beyond "write a PWM value to two pins" but still want a small, understandable driver. It provides explicit drive, coast, disable, hard-brake and dither-brake operations; validated setup; lifecycle results; software fault observation; optional MCPWM peripheral fault actions; capture timing; runtime frequency changes; and coherent diagnostics.

The library does **not** decide whether a vehicle is safe to move. It is an actuator driver. Authority, command freshness, current limits, obstacle policy, reversal permission and emergency-stop architecture belong above it.

> **Version 2.0.0:** v2 deliberately removes the old hidden meaning of `setSpeed(0)`. A zero numeric drive request is rejected. Choose the physical action explicitly with `coast()`, `disableOutputs()`, `setHardBrake()` or `softBrakeNow()`.

---

## Contents

- [Why ESP32_MCPWM exists](#why-esp32_mcpwm-exists)
- [Design boundaries](#design-boundaries)
- [Installation](#installation)
- [Supported targets](#supported-targets)
- [Beginner path](#beginner-path)
- [Core control model](#core-control-model)
- [Setup and configuration](#setup-and-configuration)
- [Operation results](#operation-results)
- [Coast, disable and braking](#coast-disable-and-braking)
- [Fault handling](#fault-handling)
- [Status and readback](#status-and-readback)
- [Runtime PWM frequency](#runtime-pwm-frequency)
- [Capture input](#capture-input)
- [Concurrency model](#concurrency-model)
- [Commissioning-only raw outputs](#commissioning-only-raw-outputs)
- [Examples](#examples)
- [Testing](#testing)
- [Physical validation](#physical-validation)
- [Migration from v1.3](#migration-from-v13)
- [Application integration notes](#application-integration-notes)
- [Repository structure](#repository-structure)
- [Limitations](#limitations)
- [Version](#version)
- [License](#license)

---

## Why ESP32_MCPWM exists

A motor command is more than a duty cycle. Real H-bridge control also needs to answer questions such as:

- Which input is PWM for each direction?
- What does "zero" physically mean: coast, disabled bridge, brake, or something else?
- Is EN asserted before or after the PWM outputs change?
- What happens if an MCPWM API call fails?
- Is the peripheral actually running after `start()`?
- What should happen when a fault input changes?
- Is that fault response performed by hardware or delayed until software runs?
- How can a higher-level controller tell what the driver last requested?

ESP32_MCPWM gives those concepts names and keeps them at the motor-driver boundary.

It is deliberately **not** a motion planner, current controller, steering controller, application authority manager or system state machine.

---

## Design boundaries

### The library owns

- MCPWM A/B routing and duty writes;
- optional EN GPIO sequencing;
- configured coast/freewheel behavior;
- explicit hard and dither braking;
- MCPWM start/stop lifecycle;
- setup validation and structured error reporting;
- a scheduler-dependent software fault observer;
- an optional MCPWM peripheral fault path;
- edge-interval capture;
- coherent software status and available hardware readback.

### The application owns

- whether movement is permitted;
- direction-change policy;
- neutral/reversal settle time;
- current, temperature and wheel-speed limits;
- stale-command handling;
- obstacle response;
- child/owner authority;
- emergency-stop power architecture;
- whether electronic braking is mechanically/electrically safe.

### No LMB-library dependencies

ESP32_MCPWM uses Arduino-ESP32 and ESP-IDF. Applications can use it directly or wrap it with their own adapters.

---

## Installation

### Arduino Library Manager

When the release is published to the Arduino Library Manager:

1. Open **Arduino IDE**.
2. Open **Library Manager**.
3. Search for **ESP32_MCPWM**.
4. Install the library.

### ZIP installation

1. Download the release ZIP.
2. In Arduino IDE choose **Sketch → Include Library → Add .ZIP Library**.
3. Include:

```cpp
#include <ESP32_MCPWM.h>
```

### PlatformIO

Add the library to the project or reference the repository from `lib_deps`.

## The included `platformio.ini` is for library development and CI, not a requirement for users.

## Supported targets

ESP32_MCPWM is intentionally **ESP32-specific** because it uses the MCPWM peripheral and ESP timer facilities.

The primary project and example target is:

- ESP32-S3 DevKitC-1.

The routine CI matrix covers ESP32-S3 DevKitC-1. Original ESP32 Dev Module
compilation was also recorded for v2.1.0; it is not a separate routine CI gate.

ESP32-S2 and ESP32-C3 are **not** listed as supported targets for this release because the pinned legacy MCPWM API is not available for those targets in this release configuration.

The pinned release CI baseline is:

- PlatformIO Core 6.1.19;
- pioarduino Espressif32 55.03.38;
- Arduino-ESP32 3.3.8.

The current ESP32-S3 gate uses that pinned platform/framework baseline. A floating/latest build is not release evidence.

A successful compile does not prove that every peripheral signal can be routed to every physical board pin; check the selected SoC and board documentation.

### ESP-IDF driver generation

The library uses the legacy `driver/mcpwm.h` API. Isolated ESP32-S3 shutdown
qualification covered Arduino-ESP32 2.0.17 / ESP-IDF 4.4.7 and Arduino-ESP32
3.3.8 / ESP-IDF 5.5.4. ESP-IDF 6 removes that legacy driver in favor of the MCPWM prelude/object API.

That future backend migration should be handled as a dedicated library release rather than mixing two driver architectures into this release. The public motor concepts introduced in v2 are intentionally suitable for either backend.

---

## Beginner path

If this is your first time using MCPWM, start here and ignore the advanced fault/capture sections initially.

### 1. Wire one H-bridge

The first example uses an ESP32-S3 DevKitC-1:

```text
ESP32-S3 GPIO 4  -> bridge LPWM / IN1
ESP32-S3 GPIO 5  -> bridge RPWM / IN2
ESP32-S3 GPIO 6  -> bridge EN
ESP32 GND        -> bridge logic GND
```

**Check the truth table and logic voltage for your actual bridge.** "BTS7960" boards are not all electrically identical at the connector/interface level.

### 2. Describe the hardware

```cpp
#include <ESP32_MCPWM.h>

MotorMCPWMConfig hardware{
    4, 5, 6,
    MCPWM_UNIT_0,
    MCPWM_TIMER_0,
    MCPWM0A,
    MCPWM0B
};
```

For a first test, the defaults use a 20 kHz drive frequency and a logical input range of 0–1023.

### 3. Create and set up the motor

```cpp
Motor motor;

void setup()
{
    Serial.begin(115200);

    const MotorSetupResult result = motor.setup(hardware);
    if (!result.ok())
    {
        Serial.println("Motor setup failed.");
        while (true)
            delay(1000);
    }
}
```

Checking the returned result is preferable to assuming the peripheral initialized correctly.

### 4. Drive with a positive request

```cpp
motor.drivePercent(50.0f, Dir::CW);
```

### 5. Stop asking for drive by choosing a physical action

For ordinary configured freewheel/coast:

```cpp
motor.coast();
```

To disable the bridge and stop MCPWM generation:

```cpp
motor.disableOutputs();
```

Do **not** write:

```cpp
motor.setSpeed(0, Dir::CW);
```

v2 rejects that command because `0` does not tell the driver which physical stop behavior you want.

### 6. Move on to braking only after basic drive/coast works

Run **02_FreewheelAndDitherBrake** at low power and validate the actual bridge behavior before using braking in a larger machine.

---

## Core control model

The public API distinguishes **drive demand** from **output state**.

### Positive drive

```cpp
motor.drive(600, Dir::CW);
motor.drivePercent(55.0f, Dir::CCW);
```

Drive requests must be positive. Requests above the configured maximum are clamped; zero/negative requests are rejected.

### Explicit physical actions

```cpp
motor.coast();
motor.disableOutputs();
motor.setHardBrake();
motor.softBrakeNow(200);
```

This is intentional. In a motor system, these states are not interchangeable.

### Direction

```cpp
Dir::CW
Dir::CCW
```

The library does not insert a reversal pause. A higher-level controller must remove torque, verify its reversal conditions and then issue the new direction.

---

## Setup and configuration

### `MotorMCPWMConfig`

Controls the physical MCPWM mapping:

```cpp
MotorMCPWMConfig hardware{
    LPWM_PIN,
    RPWM_PIN,
    EN_PIN,
    MCPWM_UNIT_0,
    MCPWM_TIMER_0,
    MCPWM0A,
    MCPWM0B
};

hardware.pwm_freq_hz = 20000;
hardware.input_max = 1023;
```

`pwm_freq_hz` specifies the intended GPIO waveform frequency. The pinned
ESP32 and ESP32-S3 configurations use a 1 MHz timer for edge-aligned up or
down counting and a 2 MHz timer for center-aligned up/down counting. Setup
and runtime accept 16–500,000 Hz in all three drive modes. Freeze mode is
rejected. The upper limit preserves a usable timer period. The driver chooses
the closest frequency representable through the legacy timer API. Duty and
frequency quantization become coarse near the upper limit. Other users of
the MCPWM group must preserve its default clock and leave owned timer events,
output routing and resources under this driver's control.

Setup and teardown hold both PWM pins low at the GPIO matrix while changing
timer state, even without an EN pin. Timer reuse waits for a fresh hardware
stop event. After restart, the driver observes peak and zero events before
routing the PWM pins back to MCPWM. These waits can span timer cycles and
run in the calling task. A failed or timed-out stop prevents resource reuse.
Retry `stop()` or `setup()` after a recoverable failure. A failed destructor
teardown quarantines the occupied resources until reboot. Explicit stop and
teardown on an EN-less bridge switch the two GPIO routes separately, so validate
how the bridge responds to the intermediate input state.

A successful `stop()` keeps ownership and drives PWM A, PWM B and configured EN
low. Successful resource release, including destruction or teardown before
re-setup, switches both PWM pads to inputs with weak internal pull-downs and
releases EN the same way after the PWM pads. The library then relinquishes its
reservation. Internal pulls provide only weak bias and cannot guarantee a low
voltage against external circuitry or throughout reset and power sequencing.
For safety-relevant installations, use appropriate external pull-downs and
default-disable circuitry that keeps the bridge inactive independently of MCU
pin state.

Optional MCPWM dead-time configuration remains available through the same structure.
Despite their historical `_ns` names, `deadtime_red_ns` and `deadtime_fed_ns`
are passed unchanged to the legacy MCPWM API as RED/FED counts. They are not
converted from nanoseconds; the actual delay depends on the MCPWM group
resolution. Values must be in 0..65535; larger counts are rejected before routing
to avoid register truncation. A future nanosecond conversion API is a separate
compatibility decision. Validate the resulting waveform before enabling dead time.

Setup rejects invalid unit/timer bounds, mismatched or swapped A/B signal pairs,
conflicting pins, invalid ranges and frequencies before routing a fresh configuration.
Timer 0 requires MCPWM0A/MCPWM0B, timer 1 requires MCPWM1A/MCPWM1B, and timer 2
requires MCPWM2A/MCPWM2B on supported hardware.

Each configured instance reserves its timer/operator and adapter-used GPIOs.
Ordinary instances can use separate timers in a unit. Hardware-fault configuration
reserves the entire unit: the legacy SDK deinitialization clears one-shot state on
all its operators. Capture uses a GPIO interrupt, not an MCPWM capture channel.
Reservations are atomic across instances; they do not coordinate arbitrary ESP-IDF
callers. Stop retains ownership because the instance can restart. Re-setup first
tears down the old configuration, then validates and reserves the replacement;
it is not transactional. Safe teardown disconnects old PWM routes and releases
ownership. Setup/teardown failure retains reservations until cleanup succeeds;
failed destruction quarantines those resources until reboot without retaining an
object pointer. ContainmentFailed reports incomplete cleanup while the object lives.

### `MotorBehaviorConfig`

Controls explicit freewheel and dither behavior:

```cpp
MotorBehaviorConfig behavior;
behavior.freewheel_mode = FreewheelMode::HiZ;
behavior.soft_brake_hz = 300;
behavior.dither_pwm = 30;
behavior.default_soft_brake_pwm = 50;
behavior.min_phase_us = 50;
behavior.dither_coast_hi_z = false;
```

The three freewheel modes are:

- `FreewheelMode::HiZ` — A/B zero and EN deasserted where EN exists;
- `FreewheelMode::HiZ_Awake` — A/B zero while EN remains asserted;
- `FreewheelMode::DitherBrake` — repeated configured brake/coast phases.

The physical meaning of `HiZ_Awake` depends on the H-bridge truth table. The name describes what the library drives, not a universal motor-current guarantee.

For ordinary EN-controlled PWM, HiZ coast withdraws EN, routes both PWM pads to
low GPIO latches, then stages zero duty. The same sequence applies to a software
fault configured to coast when the application handles it with `pollFaults()`.
The timer keeps running. An explicit new drive or brake command confirms a quiet
timer boundary before reconnecting PWM. Elapsed time to resume can therefore
depend on the configured PWM period. EN-less coast, timed dither phases, and
ordinary coast with configured peripheral fault or dead-time actions retain their
generator behavior. Lifecycle stop withdraws EN where available and disconnects
both pads before staging zero duty and stopping the timer. Pad suppression is an
electrical signal operation. Bridge coasting depends on the driver truth table.

### Setup result

Every `setup()` overload returns:

```cpp
MotorSetupResult
```

with:

```cpp
result.ok()
result.error
result.software_fault_enabled
result.hardware_fault_enabled
```

Representative errors include invalid pins/frequency/range, dither timing, MCPWM initialization, timer initialization and hardware-fault configuration. `Unsupported` means a custom driver does not implement requested optional setup behavior. `ContainmentFailed` takes precedence when replacing or rolling back a configuration could not reliably contain the previous/partial output.

---

## Operation results

Runtime operations return `MotorOperationResult` rather than silently assuming the hardware changed state.

```cpp
const MotorOperationResult result = motor.start();

if (!result.ok())
{
    // result.error explains why it failed.
}
```

The structure contains:

```cpp
operation
error
changed
sequence
```

`sequence` is a monotonic successful-operation counter. A rejected operation does not pretend that the hardware changed.

`changed` reports an observable driver or output change. A failed operation can change state when containment or a partial update succeeds. The operation sequence advances on successful changes. Reapplying an already-current drive, coast, disable, brake, freewheel selection or lifecycle state returns success with `changed == false`, even if the driver deliberately reasserts hardware for safety.

Representative errors include:

- `NotSetup`;
- `Unsupported`;
- `FaultActive`;
- `InvalidCommand`;
- `HardwareWriteFailed`;
- `HardwareStartFailed`;
- `HardwareStopFailed`;
- `TimerFailed`;
- `FrequencyChangeFailed`;
- `HardwareFaultClearFailed`;
- `CommissioningDisabled`.

A caller may still ignore a returned value in a simple sketch, but vehicle/application adapters should check it.

---

## Coast, disable and braking

### Coast

```cpp
motor.coast();
```

Applies the configured `FreewheelMode`.

`setFreewheel()` remains as a v1 compatibility alias.

### Disable outputs

```cpp
motor.disableOutputs();
```

This calls the motor lifecycle stop path: dither stops, EN is deasserted where available, A/B are commanded to zero, and MCPWM generation is stopped.

`stop()` remains available when the lifecycle wording is more appropriate.

### Hard brake

```cpp
motor.setHardBrake();
```

The current implementation commands EN asserted and both A/B generators to 100%.

That is an **electrical action**, not a generic promise of safe braking. Depending on the bridge, motor, battery and mechanics it can create high current, regeneration and shock. Use it only after bench validation.

### Soft / dither brake

```cpp
motor.softBrakeNow(200);
```

Dither brake alternates explicit brake and coast phases using `esp_timer`.

```cpp
motor.setSoftBrakePWM(200);
```

changes the configured explicit soft-brake level without making a zero drive request secretly enter that mode.

The implementation preserves the configured dither period even at very small or large requested brake strengths, subject to the practical minimum phase time.

While lifecycle-stopped, ordinary `HiZ` and `HiZ_Awake` coast requests can update the static A/B and EN state without restarting MCPWM. `DitherBrake` requires a running MCPWM/timer path, so `coast()` returns `InvalidCommand` while stopped. Call `start()` first if dither braking is required.

---

## Fault handling

ESP32_MCPWM v2 intentionally exposes **two different fault mechanisms** because they provide different guarantees.

### 1. Software fault observer — `MotorSafetyConfig`

```cpp
MotorSafetyConfig safety;
safety.fault_gpio = 7;
safety.fault_active_high = false;
safety.oneshot = true;
safety.fault_action = FaultAction::DisableOutputs;
```

The GPIO ISR records synchronized fault state. The configured bridge action is applied later when the application calls:

```cpp
motor.pollFaults();
```

This path is therefore **scheduler-dependent**.

Use it for:

- observation;
- diagnostics/logging;
- controlled software containment;
- application notification.

Do **not** describe it as a hardware emergency stop.

The conservative v2 default is:

```cpp
FaultAction::DisableOutputs
```

rather than hard braking.

Available software actions are:

- `Coast`;
- `DisableOutputs`;
- `HardBrake` — explicit opt-in only.

In one-shot mode, the software observer latches the fault. `clearFault()` rejects active inputs and keeps `hasFault()` true through quiet-output staging, peripheral re-arm and timer restart. Any failure retains the recovery inhibit and blocks drive, brake, start and commissioning requests; stop/disable remain available. Only successful recovery with no newer fault event releases the barrier. Success leaves zero demand and EN low, and never restores an old drive or dither command. Keep the application's recovery result checks and require a separate new drive command. Level-follow software release stays inhibited until `pollFaults()` completes the same quiet recovery; failed polling can be retried with `clearFault()`. Pending assertions are notified before recovery; assertions arriving during recovery remain pending and supersede that attempt.

### 2. MCPWM peripheral fault — `MotorHardwareFaultConfig`

Where the legacy ESP32 MCPWM hardware supports it, the selected GPIO can be routed directly into MCPWM fault input F0/F1/F2:

```cpp
MotorHardwareFaultConfig fault;
fault.fault_gpio = 12;
fault.input = HardwareFaultInput::Fault0;
fault.mode = HardwareFaultMode::OneShot;
fault.active_high = true;
fault.action_a = HardwareFaultOutputAction::ForceLow;
fault.action_b = HardwareFaultOutputAction::ForceLow;
```

Pass it to the full setup overload:

```cpp
motor.setup(hardware, behavior, safety, capture, fault);
```

The selected MCPWM generator action is configured in the peripheral. It does not wait for `pollFaults()` to run.

Modes:

```cpp
HardwareFaultMode::Disabled
HardwareFaultMode::CycleByCycle
HardwareFaultMode::OneShot
```

Generator actions are intentionally named electrically:

```cpp
HardwareFaultOutputAction::Hold
HardwareFaultOutputAction::ForceLow
HardwareFaultOutputAction::ForceHigh
```

The library does not call these actions "brake" or "coast" because only the hardware truth table can establish what they mean physically.

#### Active level limitation

The legacy MCPWM fault API used by this release does not provide the required active-low fault behavior consistently, so v2 rejects active-low hardware-fault configuration rather than pretending it is supported.

The hardware-fault GPIO is configured as a plain input. Provide an external pull-up or pull-down appropriate to the chosen inactive level and the real fault source; do not rely on a floating input for a containment signal.

#### Hardware fault is still not complete emergency-stop architecture

The MCPWM fault peripheral can change its generators without waiting for the scheduler. That is valuable containment.

It does **not**, by itself, prove that:

- bridge EN is physically removed;
- motor supply is disconnected;
- stored/rotating energy is removed;
- regenerative current is safe;
- the external H-bridge responds as assumed.

A safety-critical emergency stop should have an independent hardware path appropriate to the real power stage.

### Software and hardware fault paths can coexist

They are deliberately configured separately. A project may use the hardware path for immediate PWM containment and the software observer for logging/diagnostics or additional task-context behavior.

---

## Status and readback

### Coherent software status

```cpp
const MotorDriverStatus status = motor.status();
```

Includes:

- setup state;
- cached MCPWM running state;
- EN capability and commanded state;
- software fault configured/active/pending;
- hardware fault configured/active/latched;
- unresolved recovery inhibit (also included in `hasFault()`);
- dither activity;
- named `MotorOutputMode`;
- commanded A/B duty;
- configured frequency;
- operation/fault/capture sequences;
- most recent operation and error.

ISR-shared fields are protected by the ESP32 critical-section mechanism rather than `volatile` alone.

### Hardware/API readback

```cpp
const MotorHardwareReadback readback = motor.readback();
```

Frequency is a whole-Hz estimate from the programmed period and the driver
clock setting. A zero value means the timer is stopped, transitioning,
inhibited, or the legacy SDK's reported quotient disagrees. The SDK does not
read the active divider directly. Duty values are derived from comparator
registers and may reflect shadow values before a PWM boundary. They describe
the generator setting before dead time, hardware fault actions and GPIO routing.
Measure the GPIO waveform when a tolerance matters.

`running_cached` and `status().mcpwm_running` remain conservative lifecycle
indicators. The driver leaves them true during a pending stop or stop timeout.
Use the operation result and recovery inhibit to distinguish an unconfirmed
stop from completed containment. EN readback and the hardware-fault GPIO
level are read when those inputs are available.

---

## Runtime PWM frequency

```cpp
const MotorOperationResult result = motor.reconfigureFrequency(25000);
```

For a supported change, the driver stops dither, lowers EN before remapping
either PWM GPIO, drives both PWM GPIOs low and waits for hardware stop. Then it writes the
period, stages zero compare values and restarts only if the timer was running
before the call. A running timer reconnects to the PWM pins after fresh peak
and zero events. Drive stays off until the next command. Previously active
dither may resume after the transition. A stopped timer stays stopped.

Active faults reject the request with `FaultActive`. A running timer without
EN rejects a change with `Unsupported` because the two PWM pins are remapped
separately. Stop it before retuning. Peripheral fault actions and dead time
also reject changes because those features own output behavior while GPIOs
would otherwise be isolated. Select the frequency at setup for those configurations.
An unchanged valid request leaves the hardware alone when no fault is active.

`status().pwm_frequency_hz` records the last request successfully programmed
into the timer. An SDK frequency-write failure retains the previous request
and leaves the outputs disabled. A subsequent compare, restart, or dither
failure can return an error after the new request was programmed. Check both
the operation result and status, and recover before commanding drive.

When upgrading from v2.1.0 to v2.1.1, rebuild with the new version. After HiZ
coast or successful recovery, issue fresh drive demand and check the result.
Allow for a quiet timer boundary when PWM resumes. After stop, use the existing
`start()` lifecycle before requesting drive.

If migrating from a version before v2.1.0 with the earlier frequency behavior,
remove any doubled center-aligned frequency compensation. Check the 16–500,000 Hz
limits, freeze-mode rejection, and runtime restrictions above.

---

## Capture input

The optional capture helper measures the microsecond interval between selected GPIO edges:

```cpp
MotorCaptureConfig capture;
capture.cap_gpio = 8;
capture.edge = CaptureEdge::Both;
```

Read the last interval using:

```cpp
const uint32_t interval = motor.getLastCapturePeriodUs();
```

or supply an ISR callback.

The capture timestamp/period and sequence are synchronized with task-context readers. Unsigned timestamp subtraction preserves normal `micros()` rollover behavior.

This helper measures edge intervals; it does not interpret wheel speed, RPM or vehicle motion for the application.

---

## Concurrency model

ESP32_MCPWM can be touched by:

- normal application/task code;
- GPIO fault ISR;
- hardware-fault observation ISR;
- capture ISR;
- `esp_timer` dither callback.

v2 removes the old reliance on `volatile` as a cross-core synchronization mechanism. Shared state is protected by one `portMUX_TYPE` critical section with the task and ISR variants of the FreeRTOS/ESP32 critical-section API.

The host test suite mirrors that behavior with a mutex-backed test implementation and includes a ThreadSanitizer ISR/task stress test.

This does not prove real-world interrupt latency; it verifies the C++ shared-state behavior.

Internal synchronization protects ISR/task status and prevents stale deferred dither work from committing after a newer completed command. It does **not** make one motor instance a safe multi-writer actuator. Use one application command owner/arbitrator per motor instance; command authority, rearming and system safety policy remain application-owned.

---

## Commissioning-only raw outputs

Raw A/B forcing is intentionally disabled in production builds by default.

```cpp
motor.forceOutputs(true, false);
```

returns:

```cpp
MotorOperationError::CommissioningDisabled
```

unless the library is compiled with:

```cpp
#define ESP32_MCPWM_ENABLE_COMMISSIONING_API 1
```

The compile-time gate exists because raw output forcing bypasses the normal drive/coast/brake model.
Enabled calls require setup, a running timer and no fault or recovery inhibit.
Rejected calls do not restart MCPWM or assert EN. Success reports
`MotorOutputMode::Commissioning`, raw commanded duties and EN intent; identical
requests return `changed == false`. Write failure reports `Uncertain`; normal
stop and drive commands return to their usual modes.

Use it only for controlled bench commissioning where directly forcing the two bridge inputs is deliberate and understood.

---

## API reference

The public entry point is `<ESP32_MCPWM.h>`. The sections above document the configuration records, setup/result types, `Motor` lifecycle and drive/coast/brake operations, fault/status readback, runtime-frequency support, and optional capture input. See the public headers for the complete signatures and Doxygen contracts.

## Examples

The existing six-example progression is retained.

### 01_BasicMotorControl

First motor test: forward, coast, reverse, coast.

### 02_FreewheelAndDitherBrake

Compares explicit Hi-Z coast, hard brake and dither brake. Use low power and validate the bridge behavior before applying this to a loaded mechanism.

### 03_FaultInputSafety

Demonstrates the **software fault observer** and `pollFaults()`. The example intentionally states that this is deferred software containment rather than a hardware E-stop.

### 04_RuntimeFrequencyChange

Changes the drive PWM frequency and checks the structured operation outcome.

### 05_CaptureInput

Measures edge intervals with an ESP32-S3 test signal/jumper.

### 06_TwoMotors

Uses independent MCPWM timers/channels to control two H-bridges. The application
assigns distinct timers and matching A/B signal pairs; setup checks those mappings
and rejects overlapping resources before altering another instance.

The examples remain intentionally small. The README is the technical reference; the examples are there to get working code on a bench quickly.

---

## Testing and validation

The repository uses a dedicated `test/` folder rather than the old `platformio.ci.ini` pattern.

### Native deterministic suite

```bash
bash test/run_native_tests.sh
```

Runs under C++11 with strict warnings and deterministic Arduino/MCPWM/GPIO/timer mocks.

Coverage includes:

- setup output ordering;
- all freewheel modes;
- explicit zero-command behavior;
- dither timing and stale callback protection;
- stale-callback physical event ordering, phase-write failure containment, and
  timer-scheduling failure containment;
- repeated-setup containment-failure precedence;
- literal `changed`/idempotency contracts;
- custom-driver mandatory/optional interface contracts;
- repeated setup/destruction;
- drive/brake/coast/disable lifecycle;
- software fault actions and latching;
- default fault containment;
- fault recovery;
- EN-present and EN-absent behavior;
- setup validation and injected hardware failures;
- runtime frequency quiet-state transitions, idempotency, failures, and readback;
- hardware MCPWM one-shot and cycle-by-cycle fault configuration;
- hardware fault re-arm and setup failure;
- structured operation results;
- commissioning API build gate;
- capture and rollover behavior.

### GCC and Clang

The native suite is run with both compilers where available using:

```text
-Wall
-Wextra
-Wpedantic
-Wconversion
-Wsign-conversion
-Wshadow
-Werror
```

### Sanitizers

```bash
bash test/run_sanitizers.sh
```

Runs AddressSanitizer and UndefinedBehaviorSanitizer on the deterministic suite.

### ThreadSanitizer

```bash
bash test/run_tsan.sh
```

Stresses synchronized ISR/task status and capture state while ThreadSanitizer watches for host-visible data races.

### Example syntax checks

```bash
bash test/check_examples_host.sh
```

Compiles the public sketch surfaces against lightweight ESP32 host stubs. Real target compilation remains a separate CI gate.

### Documentation and release contracts

```bash
bash test/check_release_contracts.sh
```

The release contract checks version/toolchain consistency, README sections and
CI commands. Documentation quality also receives a manual release review.

Run the complete host suite with `sh test/run_host_checks.sh`. It creates a
deterministic source ZIP, checks the extracted bytes, and runs GNU GCC, Clang,
sanitizers, example syntax and release checks from the extracted copy with all
executable bits removed. Set `GCC_CXX` to your genuine GNU C++ compiler and
`CLANG_CXX` to Clang when their command names differ from `g++` and `clang++`;
on macOS, `g++` is commonly an alias for Clang.

### Target CI

GitHub Actions installs PlatformIO Core 6.1.19 and contains pinned pioarduino Espressif32 55.03.38 / Arduino-ESP32 3.3.8 library compile gates using `pio ci --project-conf platformio.ini --lib=.` for the ESP32-S3 smoke sketch and every public example.

Local host tests are **not** a substitute for those target compiler gates.

---

## Physical validation

Host tests can prove software state transitions and error reporting. They cannot prove how a real motor power stage behaves.

Before calling a vehicle/machine integration validated, measure the real system.

### Bridge truth table

With the actual module, confirm what the motor terminals do for:

|  EN |   A |   B | Expected library name        | Must be measured         |
| --: | --: | --: | ---------------------------- | ------------------------ |
|   0 |   0 |   0 | disabled / Hi-Z intent       | terminal voltage/current |
|   1 |   0 |   0 | awake zero / coast intent    | actual coast/brake state |
|   1 | PWM |   0 | CW drive                     | polarity/current         |
|   1 |   0 | PWM | CCW drive                    | polarity/current         |
|   1 |   1 |   1 | hard-brake electrical action | current/regeneration     |

Do not assume the labels above are mechanically true until the actual board is measured.

### Oscilloscope checks

Validate at minimum:

- drive PWM frequency and duty;
- A/B overlap during direction/output transitions;
- EN sequencing;
- configured dead time where used;
- dither brake/coast phase timing;
- runtime frequency change behavior;
- hardware fault A/B latency and exact levels;
- software fault latency under worst-case scheduler load.

### Current / energy checks

Use an appropriate current probe/current sensor to measure:

- hard-brake peak and duration;
- dither-brake current;
- regeneration into the supply/battery;
- reversal current;
- fault response current;
- bridge and connector thermal behavior.

### Emergency-stop checks

Validate the independent hardware path that removes/contains propulsion even if the ESP32 task scheduler, application logic or software fault polling is unavailable.

Until those tests are complete, this library should be described as **software-validated**, not vehicle-safety-validated.

---

## Migration from v1.3

v2 keeps the familiar HBridgeMotor/Motor configuration style, but several behavior changes are deliberate.

### 1. Version macro

Preferred spelling:

```cpp
ESP32_MCPWM_VERSION
```

`ESP32_MCPWM_MOTOR_VERSION` remains as a compatibility alias.

### 2. Setup and runtime operations return structured results

v1:

```cpp
motor.setup(hardware);
motor.start();
```

v2:

```cpp
const MotorSetupResult setup = motor.setup(hardware);
const MotorOperationResult started = motor.start();
```

Existing sketches may ignore the returned values. Higher-level adapters should use them.

### 3. `setSpeed(0)` / `setSpeedPercent(0)` no longer choose a stop mode

v1 could interpret zero through the configured freewheel/dither behavior.

v2 returns `InvalidCommand` and leaves the existing output request untouched.

Replace zero-demand calls with the physical intention:

```cpp
motor.coast();
// or
motor.disableOutputs();
// or
motor.setHardBrake();
// or
motor.softBrakeNow(strength);
```

### 4. Default software fault action changed

v1 default:

```cpp
FaultAction::HardBrake
```

v2 default:

```cpp
FaultAction::DisableOutputs
```

Hard brake must now be explicitly selected after hardware validation.

### 5. Hardware fault path is separate

`MotorSafetyConfig` remains the GPIO ISR + `pollFaults()` software observer.

Use `MotorHardwareFaultConfig` when you deliberately want the MCPWM peripheral fault mechanism.

### 6. `forceOutputs()` is gated

Production builds return `CommissioningDisabled` unless `ESP32_MCPWM_ENABLE_COMMISSIONING_API=1` is explicitly defined.

### 7. Shared ISR state is synchronized

No user migration is required. This is an internal correctness change.

### 8. Custom `IMotorDriver` implementations are truthful by construction

Custom drivers must now implement lifecycle-critical `start()` and `stop()` methods, along with setup-state/error and input-range reporting. Inherited optional operations such as soft-brake configuration, fault polling, freewheel-mode selection, fault clearing and frequency changes return `Unsupported` unless overridden.

Default setup overloads delegate only when additional behavior/fault/capture configuration is disabled or default. A requested unsupported capability returns `MotorSetupError::Unsupported` instead of being silently discarded. `MotorSetupError::ContainmentFailed`, `MotorOperationError::Unsupported` and `MotorOutputMode::Uncertain` are new v2 result/status values.

`MotorOperationResult::changed` now consistently reports a public semantic/output state transition, not merely that a hardware write was attempted.

---

## Application integration notes

Applications that wrap ESP32_MCPWM in a motor adapter should use its operation results and diagnostics to track confirmed lifecycle transitions.

### Operation results

Instead of:

```cpp
motor.start();
outputs_running = true;
```

use the returned `MotorOperationResult` to decide whether the adapter is actually available/running.

### Reversal dither

Applications migrating from v1 zero-speed reversal dither should now request the intended output action explicitly:

```cpp
motor.softBrakeNow(reversal_dither_pwm);
```

or another explicit project-selected output action.

### Fault architecture

An application must not treat the software `pollFaults()` path as its sole emergency containment. Application-level safety coordination and independent hardware stop/bridge-disable design remain responsible for final propulsion safety.

### Readback

The adapter can use:

```cpp
status()
readback()
```

to improve startup and fault diagnostics and avoid setting local lifecycle flags after an unverified operation.

---

## Deliberate limitations

- ESP32-specific; this is not a portable generic Arduino PWM library.
- v2.1.1 target qualification used ESP32-S3. The original ESP32 Dev Module compile was recorded for v2.1.0.
- An ESP-IDF 6 backend requires dedicated migration from the legacy MCPWM driver.
- Software fault actions require `pollFaults()` and are scheduler-dependent.
- MCPWM hardware fault actions control generator outputs; they do not guarantee external bridge power removal.
- The legacy hardware fault path in this release is restricted to active-high triggering.
- `readback().running_cached` is cached because the legacy API provides no equivalent direct timer-running getter.
- Capture measures edge intervals, not interpreted motor/vehicle speed.
- Hard brake and dither behavior are hardware-dependent and require physical validation.
- No library API can make an unknown H-bridge truth table safe by itself.

---

## Repository structure

```text
ESP32_MCPWM/
├── .github/workflows/ci.yml
├── examples/
│   ├── 01_BasicMotorControl/
│   ├── 02_FreewheelAndDitherBrake/
│   ├── 03_FaultInputSafety/
│   ├── 04_RuntimeFrequencyChange/
│   ├── 05_CaptureInput/
│   └── 06_TwoMotors/
├── src/
│   ├── ESP32_MCPWM.h
│   ├── HBridgeMotor.h
│   ├── HBridgeMotor.cpp
│   └── IMotorDriver.h
├── test/
│   ├── host_stubs/
│   ├── mocks/
│   ├── native/
│   ├── compile_smoke/
│   └── validation scripts
├── CHANGELOG.md
├── RELEASE_CHECKLIST.md
├── keywords.txt
├── library.json
├── library.properties
├── platformio.ini
└── README.md
```

---

## Version history

Current version:

```text
2.1.1
```

See [CHANGELOG.md](CHANGELOG.md) for detailed release history.

---

## License

ESP32_MCPWM is released under the **MIT License**. See [LICENSE](LICENSE).

Copyright © 2026 Little Man Builds (Darren Osborne).
