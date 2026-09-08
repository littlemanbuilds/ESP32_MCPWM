# Changelog

All notable changes to ESP32_MCPWM are documented here.

## [Unreleased]

## [2.1.0] - 2026-11-22

### Added

- Setup diagnostics for invalid timer/signal mappings, conflicting resources and out-of-range legacy deadtime counts.
- `MotorDriverStatus::recovery_inhibited` to expose pending, in-progress or failed fault recovery.
- `MotorOutputMode::Commissioning` to distinguish raw A/B output commands from normal motor commands.
- Regression coverage for resource conflicts, atomic competing claims, cleanup/reuse/quarantine, failed recovery, newer fault assertions, commissioning state and exact deadtime forwarding.
- Deterministic source ZIP validation with host checks run against the unpacked package.

### Changed

- Clarified lifecycle, output-state, fault-recovery, deadtime and commissioning contracts in the public headers and README.
- Simplified the two-motor example while retaining distinct matching timer/A/B resources.
- Standardized repository presentation, source documentation and release-check instructions.
- Unified CI host checks across genuine GNU GCC, Clang, sanitizers, concurrency, example syntax and distribution validation; aligned PlatformIO package builds and release-contract checks.
- Declared `ESP32_MCPWM.h` as the package's public umbrella header.
- Removed tracked local VS Code configuration and updated repository ignore rules.

- Interpret PWM requests as GPIO waveform hertz in all drive modes, choose the
  nearest representable rate from 16 to 500,000 Hz, and reject freeze mode.
  Frequency readback is derived from the timer period.

### Fixed

- Reject invalid unit/timer bounds, mismatched or swapped timer/A/B signal mappings, and conflicting instance resources before fresh setup changes hardware.
- Reserve hardware-fault units exclusively to prevent shared legacy fault cleanup from affecting another instance. Stop retains ownership; successful teardown disconnects old PWM routes before reuse.
- Retain reservations through failed cleanup so another instance cannot reuse uncertain resources; failed destruction quarantines those resources until reboot without retaining an object pointer.
- Keep failed fault recovery inhibited through quiet output writes, peripheral re-arm and timer restart. Preserve newer assertions and pending notifications; successful recovery leaves EN low and zero demand until a separate command.
- Require the fault example to observe successful recovery before accepting a separate drive request, retain application recovery intent after failure, and print recovery success only after a successful result.
- Require a running, recovered timer for enabled commissioning. Report raw output mode, commanded duties and literal change detection; report uncertainty on write failure.
- Reject deadtime counts above 65535 before routing to prevent register truncation.

- Hold both PWM pins low during setup, stop and retuning. Confirm hardware stop
  before changing the period or releasing timer ownership.
- Release PWM A, PWM B and configured EN as inputs with weak pull-downs after
  successful teardown. Explicit stop retains ownership and drives the pins low.
- Round percentage drive demand to the nearest logical count with ties rounded
  up. At the default input range of 0–1023, a 50% request matches raw demand 512.

### Compatibility notes

- Existing public API spellings and enum values are retained; new diagnostics and output mode values are appended.
- `ESP32_MCPWM_MOTOR_VERSION` remains a compatibility alias for `ESP32_MCPWM_VERSION`.
- Previously accepted conflicting or incoherent configurations are now rejected. Direct ESP-IDF peripheral users must still coordinate resource use with the application.
- The historical `deadtime_red_ns` and `deadtime_fed_ns` fields still forward legacy RED/FED counts unchanged; this release does not convert nanoseconds to counts.
- Remove any doubled center-aligned frequency compensation used with the
  earlier behavior. Running retuning requires EN and is rejected with active
  faults, peripheral fault actions or dead time.
- Commissioning remains disabled by default. Isolated ESP32-S3 bench checks cover
  selected waveforms and transitions. Consult the release checklist for open
  timing and loaded-system validation.

## [2.0.0] - 2026-08-30

### Changed

- Reworked public control behavior so positive drive, coast, output disable, hard brake and dither brake are explicit operations.
- `setSpeed(0)` and `setSpeedPercent(0)` now return `InvalidCommand` instead of selecting a configured physical stop mode.
- Changed the default software fault action from `HardBrake` to the more conservative `DisableOutputs`.
- Setup, drive/control, lifecycle and frequency operations now return structured results with detailed error information.
- Replaced ISR/task `volatile` sharing with synchronized critical-section state.
- Clarified `start()`/`stop()` lifecycle and retained configured freewheel behavior on successful start.
- Level-follow software fault recovery now restores only a quiet peripheral state and never restores an old drive command.
- Made `start()` and `stop()` mandatory for custom `IMotorDriver` implementations; inherited optional operations now return `Unsupported`.
- Defined `MotorOperationResult::changed` as a literal public semantic/output-state transition.

### Added

- `drive()` and `drivePercent()` explicit positive-demand APIs.
- `coast()` and `disableOutputs()` explicit output APIs while retaining `setFreewheel()`/`stop()` compatibility.
- `MotorSetupResult`, `MotorOperationResult`, `MotorDriverStatus` and `MotorHardwareReadback`.
- Separate `MotorHardwareFaultConfig` for MCPWM peripheral fault input/action configuration.
- Hardware fault one-shot and cycle-by-cycle modes with exact A/B actions: hold, force low or force high.
- Hardware-fault observation/latch sequencing and explicit one-shot re-arm through `clearFault()`.
- Build-time `ESP32_MCPWM_ENABLE_COMMISSIONING_API` gate for `forceOutputs()`.
- Single source version macros in `ESP32_MCPWM.h`.
- Dedicated C++11 native, sanitizer, ThreadSanitizer, example-syntax and release-check tests.
- ESP32-S3/original-ESP32 PlatformIO compile matrix plus an ESP32-S3 public-example compile gate in GitHub Actions.
- Beginner path, physical-validation checklist and detailed v1.3 migration guide in the README.

### Fixed

- Software-polled faults are no longer presented as an emergency-stop-equivalent mechanism.
- Generic fault containment no longer defaults to a potentially high-current hard brake.
- Runtime callers can now detect MCPWM start, stop, duty-write, timer and frequency-change failures.
- Capture, software-fault and hardware-fault ISR state now have defined cross-core synchronization.
- Hardware fault setup rejects unsupported active-low configuration rather than silently misconfiguring it.
- Hardware one-shot recovery stages a zero/disabled base output before re-arming the peripheral so pre-fault compare values cannot briefly reappear.
- Dither setup failure is surfaced as setup failure instead of leaving a partially initialized configured state.
- Deferred dither commits are serialized with newer commands so obsolete work cannot write after a newer successful operation returns.
- Repeated setup and rollback now report `ContainmentFailed` ahead of lower-consequence validation errors when old outputs cannot be reliably contained.
- Failed dither phase writes terminate the generation, stop future scheduling and report contained or uncertain output state truthfully.
- Failed dither timer scheduling now terminates the generation, preserves the
  `TimerFailed` diagnosis, and reports successful containment or `Uncertain`.
- Runtime frequency changes now publish their quiet zero-duty transition,
  preserve truthful contained/uncertain failure state, and treat an
  already-current frequency as unchanged success.

### Compatibility notes

- Existing positive `setSpeed()` / `setSpeedPercent()` calls remain source-compatible.
- Existing `setFreewheel()` remains an alias for `coast()`.
- `ESP32_MCPWM_MOTOR_VERSION` remains as a compatibility alias for `ESP32_MCPWM_VERSION`.
- Applications that intentionally used zero speed to enter freewheel/dither must migrate to an explicit output operation.
- Custom classes implementing `IMotorDriver` must implement lifecycle-critical methods and handle inherited `Unsupported` results for optional capabilities.

## [1.3.0] - 2026-06-22

### Bug fixes and safety

- Corrected zero-strength dither so it uses the configured coast phase, including EN-low Hi-Z coast, without leaving a timer running.
- Preserved the requested integer dither period while enforcing practical non-zero phase durations; reduced the default minimum phase to 50 us.
- Made re-selecting the current freewheel mode a true no-op and made changed-mode dither cancellation finish in a coherent coast state.
- Added lightweight sequence protection so a stale timer callback cannot overwrite a newer drive, freewheel, fault, or teardown command.
- Disabled EN around two-channel MCPWM transitions when possible to avoid a one-sided drive pulse between sequential duty writes.
- Guarded runtime frequency changes before setup and for invalid values; active dither now stops safely and restarts only after a successful change.
- Added non-fatal setup validation for PWM pins, pin collisions, frequency, logical input range, and dither timing, with safe failure teardown.
- Kept startup, repeated setup, stop, destruction, and active-at-startup fault handling in deterministic inactive or configured-safe states.
- Enforced active fault output inhibition across drive, brake, freewheel, raw-output, dither, and lifecycle calls.

### API and usability

- Added compatible `isSetupComplete()` and `getLastSetupError()` status APIs while retaining the existing `void setup(...)` signatures.
- Added a default `IMotorDriver::pollFaults()` and marked the concrete implementation as an override.
- Added `getLastCapturePeriodUs()`; it returns the stored selected-edge interval or zero before a valid capture.
- Added `FaultAction::Coast`, `DisableOutputs`, and `HardBrake`; hard brake remains the compatibility default.
- Added `hasEnableControl()` so applications can identify whether the library can command bridge EN.
- Clarified that both-edge capture commonly reports half-cycle intervals on symmetrical waveforms.
- Corrected capture validity when the first selected edge arrives exactly at `micros() == 0`.

### Examples, tests, and release files

- Replaced combined sketches with six focused, numbered ESP32-S3 examples covering basic control, braking, fault input, frequency changes, capture, and two motors.
- Added deterministic host regression coverage for dither timing and zero strength, idempotent mode selection, stale callbacks, setup validation/failure, runtime reconfiguration, faults, teardown, interface access, and capture intervals.
- Expanded the README around configuration, safety behavior, dither timing, capture semantics, examples, and troubleshooting; synchronized Arduino keywords and package descriptions with the public API.
- Updated release metadata to 1.3.0, expanded release ignores, and added a short release checklist.

### Deferred

- MCPWM hardware trip-zone support remains outside this release. Fault GPIO actions are the documented software fallback and require regular `pollFaults()` calls.
