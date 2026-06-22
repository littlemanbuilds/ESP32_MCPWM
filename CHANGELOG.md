# Changelog

## 1.3.0

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
