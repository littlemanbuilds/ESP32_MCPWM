#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ "$#" -eq 0 ]; then
    exec python3 "$ROOT/test/check_distribution.py"
fi
[ "$#" -eq 1 ] && [ "$1" = --unpacked ] || { echo "Invalid host-check arguments" >&2; exit 1; }
GCC_CXX=${GCC_CXX:-g++}
CLANG_CXX=${CLANG_CXX:-clang++}
"$GCC_CXX" -dM -E -x c++ /dev/null | awk '/__GNUC__/ { gcc=1 } /__clang__/ { clang=1 } END { exit !(gcc && !clang) }' || {
    echo "Set GCC_CXX to genuine GNU g++; Apple's g++ alias is Clang." >&2
    exit 1
}
"$CLANG_CXX" -dM -E -x c++ /dev/null | awk '/__clang__/ { clang=1 } END { exit !clang }'
for CXX in "$GCC_CXX" "$CLANG_CXX"; do
    CXX="$CXX" bash "$ROOT/test/run_native_tests.sh"
    CXX="$CXX" bash "$ROOT/test/check_examples_host.sh"
done
CXX="$CLANG_CXX" bash "$ROOT/test/run_sanitizers.sh"
CXX="$CLANG_CXX" bash "$ROOT/test/run_tsan.sh"
bash "$ROOT/test/check_release_contracts.sh"
python3 -B "$ROOT/test/test_release_packaging.py"
echo "ESP32_MCPWM host checks: PASS"
