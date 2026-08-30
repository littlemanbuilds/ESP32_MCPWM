#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
fail() { echo "FAIL: $*" >&2; exit 1; }

version_h="$(sed -n 's/^#define ESP32_MCPWM_VERSION "\([^"]*\)"/\1/p' "$ROOT/src/ESP32_MCPWM.h")"
version_props="$(sed -n 's/^version=//p' "$ROOT/library.properties")"
version_json="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["version"])' "$ROOT/library.json")"
[[ "$version_h" == "$version_props" && "$version_h" == "$version_json" ]] || fail "version metadata is inconsistent"
grep -Eq '^license=MIT$' "$ROOT/library.properties" || fail "library.properties must declare MIT"

for file in .clang-format .gitignore CHANGELOG.md LICENSE README.md RELEASE_CHECKLIST.md library.json library.properties platformio.ini src/ESP32_MCPWM.h test/run_host_checks.sh; do
  [[ -f "$ROOT/$file" ]] || fail "missing required file: $file"
done

for heading in \
  '## Contents' \
  '## Installation' \
  '## Supported targets' \
  '## Beginner path' \
  '## API reference' \
  '## Examples' \
  '## Testing and validation' \
  '## Deliberate limitations' \
  '## Repository structure' \
  '## Version history' \
  '## License'; do
  grep -Fq "$heading" "$ROOT/README.md" || fail "README missing: $heading"
done

for token in MotorOperationResult 'disableOutputs()' MotorHardwareFaultConfig 'FaultAction::DisableOutputs' ESP32_MCPWM_ENABLE_COMMISSIONING_API MotorHardwareReadback ThreadSanitizer; do
  grep -Fq "$token" "$ROOT/README.md" || fail "README missing established contract: $token"
done

for f in "$ROOT"/src/*.h "$ROOT"/src/*.cpp "$ROOT"/examples/*/*.ino; do
  grep -Fq '@file' "$f" || fail "missing @file Doxygen header: $f"
  grep -Fq '@author Little Man Builds (Darren Osborne)' "$f" || fail "missing LMB author header: $f"
done

[[ "$(find "$ROOT/examples" -mindepth 2 -maxdepth 2 -name '*.ino' | wc -l | tr -d ' ')" == "6" ]] || fail "expected six public examples"
[[ ! -e "$ROOT/platformio.ci.ini" ]] || fail "legacy platformio.ci.ini must not ship"

grep -Fq '55.03.38/platform-espressif32.zip' "$ROOT/platformio.ini" || fail "ESP32 platform must be explicitly pinned"
grep -Eq '^src_dir = test/compile_smoke$' "$ROOT/platformio.ini" || fail "PlatformIO run must use the target compile-smoke sketch"
grep -Eq '^lib_dir = \.\.$' "$ROOT/platformio.ini" || fail "PlatformIO run must discover this repository as a library"
[[ "$(grep -c '^\[env:esp32-s3-devkitc-1\]$' "$ROOT/platformio.ini")" == "1" ]] || fail "ESP32-S3 reference environment missing"
! grep -Fq '[env:esp32dev]' "$ROOT/platformio.ini" || fail "redundant classic ESP32 environment should not be in the routine matrix"
grep -Fq 'platformio==6.1.19' "$ROOT/.github/workflows/ci.yml" || fail "CI must pin PlatformIO Core 6.1.19"

if find "$ROOT/src" "$ROOT/examples" "$ROOT/test" -type f \( -name '*.o' -o -name '*.elf' -o -name '*.bin' -o -name '*.map' -o -name '*.zip' \) -print -quit | grep -q .; then
  fail "build/archive artefact found in public product tree"
fi

echo "PASS: ESP32_MCPWM release contracts"
