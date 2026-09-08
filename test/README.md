# ESP32_MCPWM tests

The test folder separates host-verifiable software behavior from target and physical validation.

## Native suite

Run:

```bash
bash test/run_native_tests.sh
```

The suite builds as C++11 with strict warnings. It uses deterministic Arduino, GPIO, MCPWM and `esp_timer` mocks and covers setup/containment, physical output ordering, custom-driver interface truthfulness, explicit drive/coast/brake behavior, literal changed semantics, lifecycle failures, software faults, MCPWM hardware-fault configuration, dither failure containment, capture, readback and commissioning API gating.

The same script also runs a synchronized ISR/task concurrency stress case.

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
