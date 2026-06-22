#!/usr/bin/env sh
# MIT License
# Copyright (c) 2026 Little Man Builds

set -eu

CXX="${CXX:-c++}"
OUTPUT="${TMPDIR:-/tmp}/esp32_mcpwm_native_tests"

"${CXX}" -std=c++17 -Wall -Wextra -Werror -Wno-unused-parameter \
    -I test/mocks -I src \
    src/HBridgeMotor.cpp test/native/test_startup_state.cpp \
    -o "${OUTPUT}"
"${OUTPUT}"
