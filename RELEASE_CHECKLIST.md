# ESP32_MCPWM release checklist

Use this checklist before publishing an ESP32_MCPWM release.

## Source and API

- [x] Version agrees across `ESP32_MCPWM.h`, `library.properties`, and `library.json`.
- [x] Positive v1 drive calls remain source-compatible.
- [x] Zero drive requests do not choose an implicit physical stop mode.
- [x] `coast()`, `disableOutputs()`, hard brake and dither brake are explicit.
- [x] Software fault default is `DisableOutputs`, not hard brake.
- [x] Software fault observer is documented and implemented as scheduler-dependent.
- [x] MCPWM peripheral hardware fault configuration is separate from the software observer.
- [x] ISR/task shared state uses defined critical-section synchronization.
- [x] Setup/start/stop/output/frequency operations return structured results.
- [x] Custom-driver mandatory methods and unsupported optional defaults are regression-tested.
- [x] Stale dither writes are linearized against newer completed commands.
- [x] Repeated-setup containment failures take precedence over new validation errors.
- [x] Failed dither work terminates and reports contained/uncertain state truthfully.
- [x] Failed dither timer scheduling preserves `TimerFailed` and leaves no future phase queued.
- [x] Runtime frequency changes publish quiet/uncertain state and same-frequency requests are unchanged success.
- [x] `changed` is literal for principal idempotent output/lifecycle states.
- [x] `status()` and `readback()` expose only states/readbacks the implementation can justify.
- [x] `forceOutputs()` is disabled unless the commissioning build macro is explicitly enabled.

## Documentation and examples

- [x] README follows the introduction / contents / installation / beginner path / technical reference / testing / migration flow.
- [x] README has a dedicated physical-validation section and clear evidence boundary.
- [x] All source and example files use Doxygen file headers.
- [x] Public/private function contracts describe parameters, returns and safety boundaries.
- [x] Historical v1.3 Doxygen/provenance and the complete public source/example surface were reviewed.
- [x] Existing six-example progression is retained.
- [x] Example GPIO choices remain appropriate for the ESP32-S3 DevKitC-1 teaching target.
- [x] Fault example explicitly identifies `pollFaults()` as software-deferred containment.
- [x] `keywords.txt` and package metadata match the v2 public API.

## Automated validation for the next package

- [ ] Native C++11 deterministic suite with genuine GNU GCC.
- [ ] Native C++11 deterministic suite with Clang.
- [ ] Strict warnings: `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror`.
- [ ] AddressSanitizer + UndefinedBehaviorSanitizer.
- [ ] ThreadSanitizer ISR/task shared-state stress.
- [ ] Commissioning API tested both disabled and explicitly enabled.
- [ ] Strict host syntax compile of all public examples.
- [ ] Documentation/style checks and source ZIP validation.
- [ ] Pinned PlatformIO 6.1.19 compile smoke for ESP32-S3 DevKitC-1.
- [ ] Pinned PlatformIO compile for all six public examples on ESP32-S3 DevKitC-1.
- [ ] Release/package-hygiene checks.
- [ ] Final release ZIP unpacked and retested from a clean directory.

## Remote CI / hardware gates

- [ ] GitHub Actions ESP32-S3 PlatformIO gate completed remotely.
- [ ] ESP32-S3 public examples completed remotely in GitHub Actions.
- [ ] Physical ESP32-S3 drive/coast/disable output truth table validated.
- [ ] Physical EN sequencing validated with an oscilloscope.
- [ ] Dead-time behavior validated where used.
- [ ] Hard-brake and dither current/regeneration validated with the actual bridge and motor.
- [ ] MCPWM peripheral fault latency and A/B action validated on real silicon.
- [ ] Software fault worst-case scheduler latency measured.
- [ ] Independent emergency-stop/bridge-disable hardware validated.

Hardware and remote CI gates must not be reported as passed until they have actually run.
