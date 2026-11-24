# ESP32_MCPWM tests

The test folder separates host-verifiable software behavior from target and physical validation.

## Native suite

Run:

```bash
bash test/run_native_tests.sh
```

The suite builds as C++11 with strict warnings. It uses deterministic Arduino, GPIO, MCPWM and `esp_timer` mocks and covers setup/containment, physical output ordering, custom-driver interface truthfulness, explicit drive/coast/brake behavior, literal changed semantics, lifecycle failures, software faults, MCPWM hardware-fault configuration, dither failure containment, capture, readback and commissioning API gating.

Focused regressions also cover timer/signal mapping, instance ownership/reuse/quarantine, shared hardware-fault unit exclusion, GPIO conflicts, failed recovery admission, concurrent fault assertions and exact legacy deadtime forwarding. Commissioning tests cover stopped/faulted rejection, raw status, failure and return to normal commands.

The same script also runs a synchronized ISR/task concurrency stress case.

The shutdown-aperture regression runs the production backend with a buffered
comparator model. It checks EN and pad routing before compare writes, including
pending demand, both directions, timer phases, stop and software fault coast.
Separate stale-compare timing simulation demonstrates the released failure
condition with an injected write delay. That delay is a model input. On
restoration, the quiet-route witness rejects reconnecting with a nonzero active
compare, and fault or timer-event loss must retain suppression.

The isolated target fixture in `hardware/shutdown` adds shared-clock GPIO edge
capture and independent pulse counting. Separate capture registers time rising
and falling edges on the active PWM route. The opposite route keeps an
independent pulse counter, so activity there rejects the trial. Its schedule covers six forward
command/operation pairs across all eight measured phase bins, followed by targeted
reverse-route checks from quiet. It checks at most one rising edge after EN
withdrawal within one 20 kHz period, idle and wiring checks, quiet recovery, and
final inactivity. It is intended for an unloaded ESP32-S3 with USB logic power
and the loopbacks encoded in the fixture.

## Sanitizers

```bash
bash test/run_sanitizers.sh
bash test/run_tsan.sh
```

AddressSanitizer/UndefinedBehaviorSanitizer exercise the deterministic suite. ThreadSanitizer exercises the shared ISR/task status behavior with a mutex-backed host equivalent of the ESP32 critical section.

## Public examples

```bash
bash test/check_examples_host.sh
```

This is a strict host syntax gate only. It does not emulate MCPWM silicon.

## Release checks

```bash
bash test/check_release_contracts.sh
```

The release script checks version agreement, README sections, public examples,
pinned PlatformIO configuration and CI commands. Run the aggregate suite with
`sh test/run_host_checks.sh` to verify a deterministic source ZIP: the extracted
copy has no executable bits and runs the native, sanitizer, concurrency, example
and release checks. `GCC_CXX` and `CLANG_CXX` select genuine GNU GCC and Clang;
Apple's `g++` alias cannot substitute for the GNU GCC gate.

For a proposed release, supply the target version, planned publication date,
proposed tag and a release-notes file whose first line is its title. Run the
metadata mutations against a disposable copy with the same arguments.

```bash
bash test/check_release_contracts.sh --release VERSION DATE TAG NOTES_FILE
python3 -B test/test_release_metadata.py VERSION DATE TAG NOTES_FILE
```

These checks verify metadata. Final-commit CI and the applicable hardware and
publication gates are tracked separately in the release checklist.

Build both release archives from a full Git commit SHA with
`python3 -B test/package_release.py COMMIT OUTPUT_DIRECTORY`. Run it with the
release toolchain, including PlatformIO Core and both host compilers. The command
exports committed public blobs, runs the strict source ZIP check, and validates
the actual PlatformIO archive against the exported bytes. It reports the archive
digests and inventories. PlatformIO's library package excludes the test tree,
which is included in the source ZIP.

## Target compile

GitHub Actions contains PlatformIO compile gates for the ESP32-S3 reference board and every public ESP32-S3 example using `pio ci --project-conf platformio.ini --lib=.`.

## Evidence boundary

No host test proves:

- MCPWM fault latency on real silicon;
- BTS7960/IBT-2 truth tables;
- EN timing at the bridge pins;
- dead-time correctness at the power stage;
- regenerative current;
- hard-brake or dither current;
- thermal behavior;
- safe occupied-vehicle stopping distance.

Those remain physical validation gates.

## Repository validation

Documentation and style rules are checked during release validation rather than duplicated in repository-specific test code. Repository tests remain focused on library-specific behaviour, compatibility, release metadata, and examples.
