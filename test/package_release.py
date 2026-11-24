#!/usr/bin/env python3
"""Build release references from the bytes of an exact Git commit."""

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import zipfile

from check_distribution import distribution_files


def git(repository, *args):
    """Read Git objects without updating the index."""
    return subprocess.check_output(
        ["git", "--no-optional-locks", "-C", repository, *args]
    )


def export_commit(repository, commit, destination):
    """Export regular Git blobs and reject files outside the public inventory."""
    if not re.fullmatch(r"[0-9a-f]{40}|[0-9a-f]{64}", commit):
        raise ValueError("Supply the full release commit SHA")
    if git(repository, "rev-parse", "--verify", f"{commit}^{{commit}}").decode().strip() != commit:
        raise ValueError("The release SHA must identify a commit")

    entries = {}
    for entry in git(repository, "ls-tree", "-rz", commit).split(b"\0")[:-1]:
        metadata, name = entry.decode("utf-8").split("\t", 1)
        mode, kind, blob = metadata.split()
        parts = PurePosixPath(name).parts
        if (kind != "blob" or mode not in ("100644", "100755") or
                not parts or parts[0] == "/" or ".." in parts or
                any(piece in ("AGENTS.md", ".lmb", ".agents", ".codex", ".pio") for piece in parts)):
            raise ValueError(f"Non-public or unsupported release entry: {name}")
        entries[name] = (mode, blob)
    files = set(entries)
    expected = set(distribution_files()["github"])
    if files != expected:
        raise ValueError(f"Release tree differs from public inventory: extra={sorted(files - expected)}, "
                         f"missing={sorted(expected - files)}")
    for name, (mode, blob) in entries.items():
        path = destination / name
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as output:
            output.write(git(repository, "cat-file", "blob", blob))
        path.chmod(0o755 if mode == "100755" else 0o644)
    return files


def inspect_registry(archive, source):
    """Compare the actual PlatformIO tarball to committed public bytes."""
    expected = set(distribution_files()["platformio"])
    actual = set()
    with tarfile.open(archive, "r:gz") as package:
        for member in package:
            name = member.name
            if not member.isfile() or name not in expected or name in actual:
                raise ValueError(f"Unexpected PlatformIO archive member: {name}")
            with package.extractfile(member) as entry:
                if entry.read() != (source / name).read_bytes():
                    raise ValueError(f"PlatformIO archive bytes differ: {name}")
            actual.add(name)
    if actual != expected:
        raise ValueError(f"Missing PlatformIO archive members: {sorted(expected - actual)}")
    return actual


def main():
    """Package the named commit and print digests and inventories."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("commit")
    parser.add_argument("output", type=Path, help="new directory for the verified archives")
    parser.add_argument("--repository", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    repository = args.repository.resolve()

    with tempfile.TemporaryDirectory(prefix="esp32-mcpwm-release-") as temp:
        scratch = Path(temp)
        source = scratch / "source"
        files = export_commit(repository, args.commit, source)
        manifest = json.loads((source / "library.json").read_text())
        version = manifest["version"]
        if not re.fullmatch(r"\d+\.\d+\.\d+", version):
            raise ValueError("Unexpected release version")
        if manifest.get("export") != {"include": list(distribution_files()["platformio"])}:
            raise ValueError("PlatformIO export rules differ from the positive inventory")
        source_zip = scratch / f"ESP32_MCPWM-{version}.zip"
        registry_tar = scratch / f"ESP32_MCPWM-{version}.tar.gz"

        subprocess.run([sys.executable, "-B", source / "test/check_distribution.py", "--output", source_zip],
                       check=True)
        subprocess.run([os.environ.get("PIO", "pio"), "pkg", "pack", source, "--output", registry_tar],
                       check=True)
        registry_files = inspect_registry(registry_tar, source)
        archives = [(source_zip, files), (registry_tar, registry_files)]
        inventories = distribution_files()
        for channel, archive_format in (("github", "zip"), ("github", "tar.gz"), ("arduino", "zip")):
            reference = scratch / f"{channel}-ESP32_MCPWM-{version}.{archive_format}"
            inventory = inventories[channel]
            reference.write_bytes(git(repository, "archive", f"--format={archive_format}",
                                      f"--prefix=ESP32_MCPWM-{version}/", args.commit, "--", *inventory))
            if channel == "arduino":
                with zipfile.ZipFile(reference, "a") as package:
                    package.comment = b""
            archives.append((reference, inventory))

        result = {"commit": args.commit, "version": version, "archives": {}}
        args.output.mkdir()
        for archive, inventory in archives:
            shutil.copyfile(archive, args.output / archive.name)
            result["archives"][archive.name] = {
                "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
                "files": sorted(inventory),
            }
        print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
