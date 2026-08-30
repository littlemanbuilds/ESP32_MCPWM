#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CXX="${CXX:-c++}"
for sketch in "$ROOT"/examples/*/*.ino; do
  echo "[syntax] $(basename "$sketch")"
  "$CXX" -x c++ -std=c++11 -fsyntax-only \
    -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror \
    -DARDUINO=10819 -I"$ROOT/test/host_stubs" -I"$ROOT/test/mocks" -I"$ROOT/src" "$sketch"
done
