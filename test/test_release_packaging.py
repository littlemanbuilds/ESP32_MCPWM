"""Check that release packaging follows committed public bytes."""

import io
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

from check_distribution import distribution_files, public_files
from package_release import export_commit, git, inspect_registry


def commit(repository):
    """Commit a disposable fixture with an isolated Git identity."""
    subprocess.run(["git", "-C", repository, "-c", "user.name=Packaging test",
                    "-c", "user.email=packaging@example.invalid", "commit", "-qm", "Fixture"],
                   check=True)
    return git(repository, "rev-parse", "HEAD").decode().strip()


def rejected(action, diagnostic):
    """Require rejection for the expected reason."""
    try:
        action()
    except ValueError as error:
        if diagnostic in str(error):
            return
        raise
    raise AssertionError(f"Accepted invalid packaging input: {diagnostic}")


def write_registry(archive, exported, members):
    """Build a disposable tar fixture with optional injected member contents."""
    with tarfile.open(archive, "w:gz") as package:
        for name, data in sorted(members.items()):
            if data is None:
                data = (exported / name).read_bytes()
            entry = tarfile.TarInfo(name)
            entry.size = len(data)
            package.addfile(entry, io.BytesIO(data))


def main():
    root = Path(__file__).resolve().parents[1]
    expected = distribution_files()
    manifest = json.loads((root / "library.json").read_text())
    assert manifest["export"] == {"include": list(expected["platformio"])}
    with tempfile.TemporaryDirectory(prefix="esp32-mcpwm-packaging-test-") as temporary:
        scratch = Path(temporary)
        repository = scratch / "repository"
        for original in public_files(root):
            copy = repository / original.relative_to(root)
            copy.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(original, copy)
        git(repository, "init", "-q")
        git(repository, "add", "-f", ".")
        public_commit = commit(repository)

        instructions = repository / "AGENTS.md"
        instructions.write_text("Local packaging regression marker\n")
        git(repository, "add", "-f", "AGENTS.md")
        (repository / ".lmb").mkdir()
        (repository / ".lmb/record.txt").write_text("Untracked packaging regression marker\n")
        (repository / "examples/.DS_Store").write_text("Ignored packaging regression marker\n")
        sentinel = "src/private_notes.h"
        (repository / sentinel).write_text("// Synthetic private source marker\n")
        (repository / "README.md").write_text("Working tree differs\n")

        exported = scratch / "exported"
        files = export_commit(repository, public_commit, exported)
        assert (exported / "README.md").read_bytes() == (root / "README.md").read_bytes()
        assert {"CHANGELOG.md", "RELEASE_CHECKLIST.md", "test/README.md"} <= files
        assert files == set(expected["github"])
        assert {"AGENTS.md", ".lmb/record.txt", "examples/.DS_Store", sentinel}.isdisjoint(files)

        private_commit = commit(repository)
        rejected(lambda: export_commit(repository, private_commit, scratch / "rejected"),
                 "Non-public or unsupported release entry: AGENTS.md")
        git(repository, "rm", "--cached", "-q", "AGENTS.md")
        git(repository, "add", "-f", ".lmb/record.txt")
        other_commit = commit(repository)
        rejected(lambda: export_commit(repository, other_commit, scratch / "rejected-record"),
                 "Non-public or unsupported release entry: .lmb/record.txt")
        git(repository, "rm", "--cached", "-q", ".lmb/record.txt")
        git(repository, "add", "--", sentinel)
        ordinary_path_commit = commit(repository)
        rejected(lambda: export_commit(repository, ordinary_path_commit, scratch / "rejected-source"),
                 "extra=['src/private_notes.h']")
        git(repository, "rm", "--cached", "-q", sentinel, "LICENSE")
        missing_file_commit = commit(repository)
        rejected(lambda: export_commit(repository, missing_file_commit, scratch / "rejected-missing"),
                 "missing=['LICENSE']")

        registry = scratch / "registry.tar.gz"
        members = dict.fromkeys(expected["platformio"])
        write_registry(registry, exported, members)
        assert inspect_registry(registry, exported) == set(members)
        write_registry(registry, exported, {**members, "AGENTS.md": instructions.read_bytes()})
        rejected(lambda: inspect_registry(registry, exported),
                 "Unexpected PlatformIO archive member: AGENTS.md")
        write_registry(registry, exported, {**members, sentinel: b"Synthetic private marker"})
        rejected(lambda: inspect_registry(registry, exported),
                 "Unexpected PlatformIO archive member: src/private_notes.h")
        write_registry(registry, exported, {**members, "README.md": b"Changed contents"})
        rejected(lambda: inspect_registry(registry, exported), "PlatformIO archive bytes differ: README.md")
        del members["LICENSE"]
        write_registry(registry, exported, members)
        rejected(lambda: inspect_registry(registry, exported), "Missing PlatformIO archive members: ['LICENSE']")

    print("PASS: exact Git inventory, staged and untracked isolation, and archive inventory and byte checks")


if __name__ == "__main__":
    main()
