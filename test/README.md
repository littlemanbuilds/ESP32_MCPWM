# ESP32_MCPWM tests

The test folder separates host-verifiable software behavior from target and physical validation.

## Native suite

Run:

```bash
./test/run_native_tests.sh
```

The suite builds as C++11 with strict warnings. It uses deterministic Arduino, GPIO, MCPWM and `esp_timer` mocks and covers setup/containment, physical output ordering, custom-driver interface truthfulness, explicit drive/coast/brake behavior, literal changed semantics, lifecycle failures, software faults, MCPWM hardware-fault configuration, dither failure containment, capture, readback and commissioning API gating.

The same script also runs a synchronized ISR/task concurrency stress case.

## Sanitizers

```bash
./test/run_sanitizers.sh
./test/run_tsan.sh
```

AddressSanitizer/UndefinedBehaviorSanitizer exercise the deterministic suite. ThreadSanitizer exercises the shared ISR/task status behavior with a mutex-backed host equivalent of the ESP32 critical section.

## Public examples

```bash
./test/check_examples_host.sh
```

This is a strict host syntax gate only. It does not emulate MCPWM silicon.

## Release checks

```bash
./test/check_release_contracts.sh
```

The style gate audits all public source/example files and proves representative
failures are rejected with deterministic negative probes. The release script
also runs that gate, then checks version agreement, standard README sections,
required examples, pinned toolchain contracts, strict Git-aware candidate
inventory and protected/private-content rejection. Ignored local developer
artifacts do not fail this check.

## Target compile

GitHub Actions contains PlatformIO compile gates for verified ESP32-S3/original-ESP32 boards and public ESP32-S3 examples.

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

## LMB presentation conformance

LMB presentation rules are checked by the installed fleet conformance harness rather than duplicated in this repository. Repository tests remain focused on library-specific behaviour, compatibility, release metadata, and examples.
