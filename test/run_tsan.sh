#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/test/build"
CXX="${CXX:-c++}"
mkdir -p "$BUILD"
"$CXX" -std=c++11 -O1 -g -fsanitize=thread -fno-omit-frame-pointer -pthread \
  -I"$ROOT/test/mocks" -I"$ROOT/src" \
  "$ROOT/src/HBridgeMotor.cpp" "$ROOT/test/mocks/mock_hal.cpp" \
  "$ROOT/test/native/test_concurrency.cpp" -o "$BUILD/test_mcpwm_tsan"
TSAN_OPTIONS=halt_on_error=1 "$BUILD/test_mcpwm_tsan"
