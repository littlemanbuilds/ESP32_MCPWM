#!/usr/bin/env python3
"""Build a deterministic source ZIP and run host checks without executable bits."""

import argparse
import hashlib
from pathlib import Path
import subprocess
import tempfile
import zipfile


SOURCE_FILES = (
    ".clang-format",
    ".github/workflows/ci.yml",
    ".gitignore",
    "CHANGELOG.md",
    "LICENSE",
    "README.md",
    "RELEASE_CHECKLIST.md",
    "examples/01_BasicMotorControl/01_BasicMotorControl.ino",
    "examples/02_FreewheelAndDitherBrake/02_FreewheelAndDitherBrake.ino",
    "examples/03_FaultInputSafety/03_FaultInputSafety.ino",
    "examples/04_RuntimeFrequencyChange/04_RuntimeFrequencyChange.ino",
    "examples/05_CaptureInput/05_CaptureInput.ino",
    "examples/06_TwoMotors/06_TwoMotors.ino",
    "keywords.txt",
    "library.json",
    "library.properties",
    "platformio.ini",
    "src/ESP32_MCPWM.h",
    "src/HBridgeMotor.cpp",
    "src/HBridgeMotor.h",
    "src/IMotorDriver.h",
    "test/README.md",
    "test/check_distribution.py",
    "test/check_examples_host.sh",
    "test/check_release_contracts.sh",
    "test/check_release_metadata.py",
    "test/compile_smoke/compile_smoke.ino",
    "test/hardware/shutdown/build.py",
    "test/hardware/shutdown/check_trace.py",
    "test/hardware/shutdown/main.cpp",
    "test/hardware/shutdown/qualify.py",
    "test/hardware/shutdown/trace.h",
    "test/host_stubs/Arduino.h",
    "test/mocks/Arduino.h",
    "test/mocks/driver/gpio.h",
    "test/mocks/driver/mcpwm.h",
    "test/mocks/esp_rom_gpio.h",
    "test/mocks/esp_timer.h",
    "test/mocks/freertos/portmacro.h",
    "test/mocks/mock_hal.cpp",
    "test/mocks/mock_hal.h",
    "test/mocks/soc/gpio_sig_map.h",
    "test/mocks/soc/mcpwm_reg.h",
    "test/mocks/soc/soc.h",
    "test/mocks/soc/soc_caps.h",
    "test/native/test_commissioning.cpp",
    "test/native/test_concurrency.cpp",
    "test/native/test_interface.cpp",
    "test/native/test_startup_state.cpp",
    "test/package_release.py",
    "test/run_host_checks.sh",
    "test/run_native_tests.sh",
    "test/run_sanitizers.sh",
    "test/run_tsan.sh",
    "test/test_release_metadata.py",
    "test/test_release_packaging.py",
)


def public_files(root):
    """Return the explicit public GitHub source inventory."""
    return [root / name for name in SOURCE_FILES]


def distribution_files():
    """Return positive projections of the reviewed GitHub source inventory."""
    return {
        "github": SOURCE_FILES,
        "platformio": tuple(name for name in SOURCE_FILES if not name.startswith("test/")),
        "arduino": tuple(name for name in SOURCE_FILES if not name.startswith(".")),
    }


def main():
    """Verify the public source package and return its unpacked host-check status."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="save the verified source ZIP")
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    files = public_files(root)
    with tempfile.TemporaryDirectory(prefix="esp32-mcpwm-package-") as temporary:
        temporary = Path(temporary)
        archive = temporary / "ESP32_MCPWM.zip"
        with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_STORED) as package:
            for path in files:
                if path.is_symlink() or not path.is_file():
                    raise RuntimeError(f"Missing or symlinked package file: {path}")
                entry = zipfile.ZipInfo("ESP32_MCPWM/" + path.relative_to(root).as_posix())
                entry.create_system = 3
                entry.external_attr = 0o100644 << 16
                package.writestr(entry, path.read_bytes())
        with zipfile.ZipFile(archive) as package:
            package.extractall(temporary)
        unpacked = temporary / "ESP32_MCPWM"
        for path in files:
            copy = unpacked / path.relative_to(root)
            copy.chmod(0o644)
            if hashlib.sha256(copy.read_bytes()).digest() != hashlib.sha256(path.read_bytes()).digest():
                raise RuntimeError(f"Archive content mismatch: {path}")
        result = subprocess.run(["sh", "test/run_host_checks.sh", "--unpacked"], cwd=unpacked, check=False)
        if result.returncode == 0:
            if arguments.output:
                with arguments.output.open("xb") as output:
                    output.write(archive.read_bytes())
            print("PASS: deterministic source ZIP and unpacked host checks", flush=True)
        return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
