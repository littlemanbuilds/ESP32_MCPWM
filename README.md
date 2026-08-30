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
- [PW_PVT integration notes](#pw_pvt-integration-notes)
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

It is deliberately **not** a motion planner, SafetyCore, current controller, steering controller, authority manager or vehicle state machine.

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

ESP32_MCPWM does not depend on SnapshotBus, RCLink, SwitchBank, SafetyCore, SteerCore or any other LMB library. A project may wrap it with adapters, but the motor library remains independently usable.

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

The included `platformio.ini` is for library development and CI, not a requirement for users.

---

## Supported targets

ESP32_MCPWM is intentionally **ESP32-specific** because it uses the MCPWM peripheral and ESP timer facilities.

The primary project and example target is:

- ESP32-S3 DevKitC-1.

The release compile matrix currently covers:

- ESP32-S3 DevKitC-1;
- original ESP32 Dev Module.

ESP32-S2 and ESP32-C3 are **not** listed as supported targets for this release because the pinned legacy MCPWM API is not available for those targets in this release configuration.

The authoritative v2.0.0 release baseline is:

- PlatformIO Core 6.1.19;
- pioarduino Espressif32 55.03.38;
- Arduino-ESP32 3.3.8.

Both ESP32-S3 and original ESP32 builds use that same pinned platform/framework baseline. A floating/latest build is not release evidence.

A successful compile does not prove that every peripheral signal can be routed to every physical board pin; check the selected SoC and board documentation.

### ESP-IDF driver generation

v2.0.0 uses the legacy `driver/mcpwm.h` API currently provided by Arduino-ESP32 3.x / ESP-IDF 5.x. ESP-IDF 6 removes that legacy driver in favor of the newer MCPWM prelude/object API.

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

Optional MCPWM dead-time configuration remains available through the same structure.

Setup rejects invalid or conflicting pins, duplicate PWM signals, invalid input ranges and invalid drive frequencies before activating the bridge.

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

`changed` is literal: it is `true` only when a successful request changes the public semantic/output state. Reapplying an already-current drive, coast, disable, brake, freewheel selection or lifecycle state returns success with `changed == false`, even if the driver deliberately reasserts hardware for safety.

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

For one-shot mode, the fault remains latched until the physical input is inactive and `clearFault()` succeeds. For level-follow mode, release returns the driver to a quiet zero-output state; it never restores the previous drive command.

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

Where the legacy driver exposes the information, this reads:

- MCPWM frequency;
- A duty;
- B duty;
- physical EN GPIO level;
- hardware-fault GPIO level.

`running_cached` remains explicitly named **cached** because the legacy API does not expose an equivalent direct hardware getter for timer running state.

The distinction matters: readback reports what can genuinely be read back, and does not manufacture a hardware confirmation that the SDK cannot provide.

---

## Runtime PWM frequency

```cpp
const MotorOperationResult result = motor.reconfigureFrequency(25000);
```

The operation:

1. validates the requested frequency;
2. moves the bridge to a quiet output where possible;
3. asks MCPWM to change frequency;
4. records the new value only after success;
5. restores an active dither state when appropriate.

A failed frequency change is reported and does not silently update the driver's configured frequency.

Use `readback()` if the application wants the frequency reported by the MCPWM API.

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

Internal synchronization protects ISR/task status and prevents stale deferred dither work from committing after a newer completed command. It does **not** make one motor instance a safe multi-writer actuator. Use one application command owner/arbitrator per motor instance; command authority and SafetyCore policy remain application-owned.

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

Uses independent MCPWM timers/channels to control two H-bridges.

The examples remain intentionally small. The README is the technical reference; the examples are there to get working code on a bench quickly.

---

## Testing and validation

The repository uses a dedicated `test/` folder rather than the old `platformio.ci.ini` pattern.

### Native deterministic suite

```bash
./test/run_native_tests.sh
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
./test/run_sanitizers.sh
```

Runs AddressSanitizer and UndefinedBehaviorSanitizer on the deterministic suite.

### ThreadSanitizer

```bash
./test/run_tsan.sh
```

Stresses synchronized ISR/task status and capture state while ThreadSanitizer watches for host-visible data races.

### Example syntax checks

```bash
./test/check_examples_host.sh
```

Compiles the public sketch surfaces against lightweight ESP32 host stubs. Real target compilation remains a separate CI gate.

### LMB presentation and release contracts

```bash
python3 ./test/check_lmb_style.py
./test/check_release_contracts.sh
```

The presentation check audits the complete public source/example surface and
runs deterministic negative probes. The release contract separately verifies
version/toolchain consistency and the strict Git-aware distribution inventory.
Semantic documentation quality still receives a manual release review.

### Target CI

GitHub Actions installs PlatformIO Core 6.1.19 and contains pinned pioarduino Espressif32 55.03.38 / Arduino-ESP32 3.3.8 compile gates for the ESP32-S3/original-ESP32 targets and the public ESP32-S3 examples.

Local host tests are **not** a substitute for those target compiler gates.

---

## Physical validation

Host tests can prove software state transitions and error reporting. They cannot prove how a real motor power stage behaves.

Before calling a vehicle/machine integration validated, measure the real system.

### Bridge truth table

With the actual module, confirm what the motor terminals do for:

| EN | A | B | Expected library name | Must be measured |
|---:|---:|---:|---|---|
| 0 | 0 | 0 | disabled / Hi-Z intent | terminal voltage/current |
| 1 | 0 | 0 | awake zero / coast intent | actual coast/brake state |
| 1 | PWM | 0 | CW drive | polarity/current |
| 1 | 0 | PWM | CCW drive | polarity/current |
| 1 | 1 | 1 | hard-brake electrical action | current/regeneration |

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

## PW_PVT integration notes

PW_PVT currently uses ESP32_MCPWM through a motor adapter. v2 intentionally remains independent of the project, but the later PW_PVT update should use the stronger operation results and diagnostics.

### Operation results

Instead of:

```cpp
motor.start();
outputs_running = true;
```

use the returned `MotorOperationResult` to decide whether the adapter is actually available/running.

### Reversal dither

The current project uses a zero speed request to enter its configured dither state during reversal handling.

That v1 behavior is intentionally removed. The later PW_PVT migration should use:

```cpp
motor.softBrakeNow(reversal_dither_pwm);
```

or another explicit project-selected output action.

### Fault architecture

PW_PVT should not treat the software `pollFaults()` path as its sole emergency containment. The project-level SafetyCore and independent hardware stop/bridge-disable design remain responsible for final propulsion safety.

### Readback

The adapter can use:

```cpp
status()
readback()
```

to improve StartupSupervisor/SafetyCore diagnostics and avoid setting local lifecycle flags after an unverified operation.

No PW_PVT source is modified by this library release.

---

## Deliberate limitations

- ESP32-specific; this is not a portable generic Arduino PWM library.
- Verified target compiles are limited to ESP32-S3 and original ESP32 for this release.
- v2.0.0 uses the legacy ESP-IDF MCPWM driver exposed by current Arduino-ESP32 3.x. A future ESP-IDF 6 backend requires dedicated migration work.
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

Current release:

```text
2.0.0
```

This is a **major** release because zero-drive behavior and the default fault action intentionally change, structured operation results are added to the driver interface, and raw output forcing is gated.

See [CHANGELOG.md](CHANGELOG.md) for the release history.

---

## License

MIT License. See [LICENSE](LICENSE).

Copyright (c) 2026 Little Man Builds.
