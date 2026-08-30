#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/test/build"
CXX="${CXX:-c++}"
mkdir -p "$BUILD"
"$CXX" -std=c++11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror \
  -I"$ROOT/test/mocks" -I"$ROOT/src" \
  "$ROOT/src/HBridgeMotor.cpp" "$ROOT/test/mocks/mock_hal.cpp" \
  "$ROOT/test/native/test_startup_state.cpp" -o "$BUILD/test_mcpwm"
"$BUILD/test_mcpwm"

"$CXX" -std=c++11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror \
  -DESP32_MCPWM_ENABLE_COMMISSIONING_API=1 \
  -I"$ROOT/test/mocks" -I"$ROOT/src" \
  "$ROOT/src/HBridgeMotor.cpp" "$ROOT/test/mocks/mock_hal.cpp" \
  "$ROOT/test/native/test_commissioning.cpp" -o "$BUILD/test_commissioning"
"$BUILD/test_commissioning"

"$CXX" -std=c++11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror \
  -I"$ROOT/test/mocks" -I"$ROOT/src" \
  "$ROOT/test/native/test_interface.cpp" -o "$BUILD/test_interface"
"$BUILD/test_interface"

"$CXX" -std=c++11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror \
  -pthread -I"$ROOT/test/mocks" -I"$ROOT/src" \
  "$ROOT/src/HBridgeMotor.cpp" "$ROOT/test/mocks/mock_hal.cpp" \
  "$ROOT/test/native/test_concurrency.cpp" -o "$BUILD/test_concurrency"
"$BUILD/test_concurrency"
