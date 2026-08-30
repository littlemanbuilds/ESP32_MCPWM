# Changelog

All notable changes to ESP32_MCPWM are documented here.

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

- Matured setup validation, freewheel/dither behavior, software fault handling, runtime frequency changes, capture support and native host regression coverage.
