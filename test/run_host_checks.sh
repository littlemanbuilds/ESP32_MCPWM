#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
for CXX in g++ clang++; do command -v "$CXX" >/dev/null 2>&1 && CXX="$CXX" "$ROOT/test/run_native_tests.sh"; done
CXX=g++ "$ROOT/test/run_sanitizers.sh"
"$ROOT/test/run_tsan.sh"
for CXX in g++ clang++; do command -v "$CXX" >/dev/null 2>&1 && CXX="$CXX" "$ROOT/test/check_examples_host.sh"; done
"$ROOT/test/check_release_contracts.sh"
echo "ESP32_MCPWM host checks: PASS"
