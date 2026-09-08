# ESP32_MCPWM release checklist

Target release **2.1.0**. Planned publication **2026-11-22**. Confirm the date before tagging and update the changelog and release metadata if the date changes.

Publication is on hold pending final-commit CI and the open release gates below. Checked source, API, example and documentation items record earlier review of the relevant source and examples. They carry no hardware or publication claim.

## Release preparation

- [x] Version string, numeric macros and both package manifests agree on 2.1.0.
- [x] README current version and migration direction agree with the target release.
- [x] One dated v2.1.0 changelog entry covers the consolidated changes and `[Unreleased]` is empty. The v2.0.0, v1.3.0 and earlier entries are preserved.
- [ ] Confirm the actual publication date, proposed tag and final release notes against both manifests before tagging.
- [ ] Confirm the final public diff, source ZIP and registry package against the exact release commit.

## Source and API

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

- [x] Invalid timer/A/B mappings and resource conflicts are rejected before fresh setup alters hardware.
- [x] Ownership is retained through stop and failed cleanup; successful teardown disconnects routes before reuse, and failed destruction quarantines resources.
- [x] Hardware-fault configurations exclude other owners of the same MCPWM unit.
- [x] Failed recovery blocks energizing commands until successful quiet recovery; newer fault assertions and pending notifications are preserved.
- [x] Enabled commissioning requires a running, recovered timer and reports raw command state and write uncertainty.
- [x] Legacy deadtime counts are forwarded unchanged and bounded to 0..65535.
- [x] Existing enum values and version alias are retained alongside appended diagnostics.

## Documentation and examples

- [x] README follows the introduction / contents / installation / beginner path / technical reference / testing / migration flow.
- [x] README has a dedicated physical-validation section and clear evidence boundary.
- [x] All source and example files use Doxygen file headers.
- [x] Public/private function contracts describe parameters, returns and safety boundaries.
- [x] Original source/example creation dates are preserved and all public source files and examples were reviewed.
- [x] Existing six-example progression is retained.
- [x] Example GPIO choices remain appropriate for the ESP32-S3 DevKitC-1 teaching target.
- [x] Fault example identifies `pollFaults()` as software-deferred containment and requires successful recovery before a separate drive command.
- [x] `keywords.txt` and package metadata match the v2 public API.

## Automated validation

The earlier qualified implementation passed strict C++11 GNU GCC and Clang tests, ASan/UBSan, TSan, commissioning with the API enabled and disabled, host syntax checks for all six examples, and pinned ESP32-S3 and classic ESP32 smoke builds. The implementation and public examples are unchanged. Previous remote CI also compiled the six ESP32-S3 examples. Packaging and final-commit CI require their own evidence.

- [ ] `final-source-zip` — exact final source ZIP unpacked and validated, including host checks and release contracts.
- [ ] `final-registry-package` — exact final PlatformIO archive reviewed for public files, contents and package version.
- [ ] `final-remote-ci` — host and source ZIP checks, ESP32-S3 smoke build and all six public examples pass on the final amended commit with Python 3.11 and PlatformIO Core 6.1.19.

## Isolated ESP32-S3 bench

Earlier logic-only observations include center-aligned 20 kHz steady PWM, immediate 16 Hz timer reuse, EN-less running-retune rejection, software and peripheral fault transitions, and GPIO teardown. Separate destruction-marker captures showed EN and each PWM pin low in the observed windows. The 500 kHz percentage midpoint passed in up- and down-count modes with operator-reported 2.000 µs period and 1.000 µs HIGH on PWM B. These observations cover the captured windows and setup only, including no measurement of pull-down strength or simultaneous three-pin release.

Release hardware items below remain open. A completed item requires evidence for the full stated scope. No deferral has been approved.

- [ ] `hw-output` — complete logic-level drive/coast/disable truth table and remaining supported-mode frequency and duty endpoints.
- [ ] `hw-reuse` — characterize low-frequency lifecycle, resource reuse, failed-recovery containment and enabled commissioning on hardware.
- [ ] `hw-transitions` — resolve short transients, EN sequencing and EN-less pad-to-pad shutdown skew.
- [ ] `hw-deadtime` — verify physical delay where legacy deadtime counts are used. The API forwards counts without a nanosecond conversion.
- [ ] `hw-peripheral-fault` — measure peripheral fault latency and A/B action on real silicon.
- [ ] `hw-software-fault` — measure worst-case software fault latency for the intended scheduler workload.

## Powered-system integration

These checks require the actual bridge, motor, power supply or machine safety circuit. They do not apply to the USB-only bench setup. Their release disposition remains open until the operator and independent reviewer confirm the applicable scope under the release policy.

- [ ] `system-energy` — validate braking and dither current and regeneration with the actual bridge and motor.
- [ ] `system-disable` — validate the independent emergency stop and bridge-disable hardware on the intended machine.
