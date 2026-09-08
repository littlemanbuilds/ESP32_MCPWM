#!/usr/bin/env python3
"""Build a deterministic source ZIP and run host checks without executable bits."""

import hashlib
from pathlib import Path
import subprocess
import tempfile
import zipfile


def main():
    """Verify the public source package and return its unpacked host-check status."""
    root = Path(__file__).resolve().parents[1]
    files = [root / name for name in (
        ".clang-format", ".gitignore", "CHANGELOG.md", "LICENSE", "README.md",
        "RELEASE_CHECKLIST.md", "keywords.txt", "library.json", "library.properties",
        "platformio.ini", ".github/workflows/ci.yml",
    )]
    for directory in ("src", "examples", "test"):
        for path in (root / directory).rglob("*"):
            relative = path.relative_to(root / directory)
            if any(part.startswith(".") for part in relative.parts) or relative.parts[0] == "build":
                continue
            if path.is_dir():
                continue
            if path.suffix not in (".h", ".cpp", ".ino", ".sh", ".py", ".md"):
                raise RuntimeError(f"Unexpected source-package file: {path.relative_to(root)}")
            files.append(path)
    files.sort()
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
            print("PASS: deterministic source ZIP and unpacked host checks", flush=True)
        return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
