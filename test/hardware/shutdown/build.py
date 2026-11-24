"""Build the isolated fixture and optional example sketches with installed PlatformIO."""
import base64
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile


PROFILES = {
    "legacy": "espressif32@6.12.0",
    "current": "https://github.com/pioarduino/platform-espressif32/releases/download/55.03.38/platform-espressif32.zip",
}
REGIONS = {
    "bootloader.bin": 0,
    "partitions.bin": 0x8000,
    "boot_app0.bin": 0xE000,
    "firmware.bin": 0x10000,
}


def sha256(content):
    return hashlib.sha256(content).hexdigest()


def run_build(root, scratch, profile, platformio, legacy_packages, sketches, emit):
    core = Path(os.environ["PLATFORMIO_CORE_DIR"])
    env = os.environ.copy()
    build = scratch / f"{profile}-build"
    env["PLATFORMIO_BUILD_DIR"] = str(build)
    if profile == "current":
        (core / "penv").symlink_to(Path(platformio).parent.parent, target_is_directory=True)
        env["PYTHONPATH"] = str((core / "packages/tool-esptoolpy").resolve())
        # pioarduino updates its board-local SCons script during checkprogsize.
        # Expose the installed SDK read-only and give only that script a scratch copy.
        sdk = core / "packages/framework-arduinoespressif32-libs"
        installed = sdk.resolve(strict=True)
        sdk.unlink()
        sdk.mkdir()
        for entry in installed.iterdir():
            if entry.name != "esp32s3":
                (sdk / entry.name).symlink_to(entry, target_is_directory=entry.is_dir())
            else:
                local = sdk / entry.name
                local.mkdir()
                for part in entry.iterdir():
                    if part.name == "pioarduino-build.py":
                        shutil.copyfile(part, local / part.name)
                    else:
                        (local / part.name).symlink_to(part, target_is_directory=part.is_dir())
    if profile == "legacy":
        for package in ("framework-arduinoespressif32", "tool-esptoolpy"):
            selected = core / "packages" / package
            if selected.is_symlink():
                selected.unlink()
            elif selected.exists():
                raise RuntimeError(f"Expected a disposable package link at {selected}")
            selected.symlink_to(legacy_packages / package, target_is_directory=True)

    project = scratch / profile
    project.mkdir()
    local_library = project / "lib" / "ESP32_MCPWM"
    local_library.mkdir(parents=True)
    (local_library / "src").symlink_to(root / "src", target_is_directory=True)
    shutil.copyfile(root / "library.json", local_library / "library.json")
    (project / "platformio.ini").write_text(f"""[env:shutdown]
platform = {PROFILES[profile]}
board = esp32-s3-devkitc-1
framework = arduino
board_upload.flash_size = 16MB
board_build.partitions = default_16MB.csv
build_src_flags = -Wformat=2 -Werror=format
""")
    sources = project / "src"
    sources.mkdir()
    fixture = root / "test/hardware/shutdown/main.cpp"
    shutil.copyfile(fixture.with_name("trace.h"), sources / "trace.h")
    inputs = [fixture, *sketches]
    for source in inputs:
        destination = sources / source.name
        shutil.copyfile(source, destination)
        subprocess.run([platformio, "run", "-d", project], check=True, env=env)
        destination.unlink()
        if source == fixture:
            output = build / "shutdown"
            framework = (core / "packages/framework-arduinoespressif32").resolve()
            boot_app = framework / "tools/partitions/boot_app0.bin"
            contents = {
                name: (boot_app if name == "boot_app0.bin" else output / name).read_bytes()
                for name in REGIONS
            }
            elf = (output / "firmware.elf").read_bytes()
            embedded = contents["firmware.bin"][0xB0:0xD0].hex()
            if embedded != sha256(elf):
                raise RuntimeError(f"Image ELF digest {embedded} disagrees with linked ELF {sha256(elf)}")
            manifest = {
                "profile": profile,
                "elf_sha256": embedded,
                "regions": {
                    name: {"offset": offset, "size": len(contents[name]), "sha256": sha256(contents[name])}
                    for name, offset in REGIONS.items()
                },
            }
            print("FIRMWARE " + json.dumps(manifest), flush=True)
            if emit:
                buffer = io.BytesIO()
                with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
                    for name, content in contents.items():
                        archive.writestr(name, content)
                    archive.writestr("firmware.elf", elf)
                    archive.writestr("manifest.json", json.dumps(manifest, indent=2) + "\n")
                print("IMAGE_TRANSFER " + profile + " " + base64.b64encode(buffer.getvalue()).decode(), flush=True)


if __name__ == "__main__":
    source, scratch, profile, pio, legacy, *flags = sys.argv[1:]
    root = Path(source)
    sketches = ([root / "test/compile_smoke/compile_smoke.ino", *sorted((root / "examples").glob("*/*.ino"))]
                if "--examples" in flags else [])
    run_build(root, Path(scratch), profile, pio, Path(legacy), sketches, "--emit" in flags)
