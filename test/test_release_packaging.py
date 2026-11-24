"""Check that release packaging follows committed public bytes."""

import io
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

from check_distribution import public_files
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


def main():
    root = Path(__file__).resolve().parents[1]
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
        (repository / "README.md").write_text("Working tree differs\n")

        exported = scratch / "exported"
        files = export_commit(repository, public_commit, exported)
        assert (exported / "README.md").read_bytes() == (root / "README.md").read_bytes()
        assert {"CHANGELOG.md", "RELEASE_CHECKLIST.md", "test/README.md"} <= files
        assert {"AGENTS.md", ".lmb/record.txt"}.isdisjoint(files)

        private_commit = commit(repository)
        rejected(lambda: export_commit(repository, private_commit, scratch / "rejected"),
                 "Non-public or unsupported release entry: AGENTS.md")
        git(repository, "rm", "--cached", "-q", "AGENTS.md")
        git(repository, "add", "-f", ".lmb/record.txt")
        other_commit = commit(repository)
        rejected(lambda: export_commit(repository, other_commit, scratch / "rejected-record"),
                 "Non-public or unsupported release entry: .lmb/record.txt")

        registry = scratch / "registry.tar.gz"
        with tarfile.open(registry, "w:gz") as archive:
            for name in sorted(files):
                if not name.startswith("test/"):
                    archive.add(exported / name, arcname=name)
        assert "README.md" in inspect_registry(registry, exported, files)
        with tarfile.open(registry, "w:gz") as archive:
            entry = tarfile.TarInfo("AGENTS.md")
            data = instructions.read_bytes()
            entry.size = len(data)
            archive.addfile(entry, io.BytesIO(data))
        rejected(lambda: inspect_registry(registry, exported, files),
                 "Unexpected PlatformIO archive member: AGENTS.md")

    print("PASS: exact committed export, staged and untracked isolation, committed-control and archive negatives")


if __name__ == "__main__":
    main()
