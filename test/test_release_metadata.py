#!/usr/bin/env python3
"""Exercise release-readiness failures on disposable copies of public metadata."""

from datetime import date, timedelta
from pathlib import Path
import re
import shutil
import sys
import tempfile

from check_release_metadata import read, verify


FILES = ('src/ESP32_MCPWM.h', 'library.properties', 'library.json', 'README.md',
         'CHANGELOG.md', '.github/workflows/ci.yml')


def run(root, release):
    verify(root, release)
    target, planned, _, _ = release
    major, minor, patch = (int(field) for field in target.split('.'))
    wrong = f'{major}.{minor}.{patch + 1}'
    wrong_date = (date.fromisoformat(planned) + timedelta(days=1)).isoformat()
    versions = re.findall(r'^## \[([0-9.]+)\]', read(root, 'CHANGELOG.md'), re.MULTILINE)
    if len(versions) < 2 or versions[0] != target:
        raise AssertionError('Expected a target and preceding release version')
    previous = versions[1]
    heading = f'## [{target}]'
    mutations = (
        ('README.md', f'```text\n{target}\n```', f'```text\n{previous}\n```', 'README current version is stale'),
        ('src/ESP32_MCPWM.h', f'VERSION_MINOR {minor}', f'VERSION_MINOR {minor + 1}', 'Numeric version macros disagree'),
        ('library.properties', f'version={target}', f'version={wrong}', 'Package versions disagree'),
        ('library.json', f'"version": "{target}"', f'"version": "{wrong}"', 'Package versions disagree'),
        ('CHANGELOG.md', heading, f'## [{wrong}]', 'Missing or duplicate target changelog section'),
        ('CHANGELOG.md', heading, f'{heading}\n\n{heading}', 'Missing or duplicate target changelog section'),
        ('CHANGELOG.md', f'{heading} - {planned}', heading, 'Target changelog date disagrees'),
        ('CHANGELOG.md', f'{heading} - {planned}', f'{heading} - {wrong_date}', 'Target changelog date disagrees'),
        ('CHANGELOG.md', '## [Unreleased]\n', '## [Unreleased]\n\n- Pending change\n', 'Unreleased must be empty'),
        ('CHANGELOG.md', '## [2.0.0] - 2026-08-30', '## [2.0.0] - 2026-08-31', 'Published v2.0.0 and earlier'),
        ('README.md', f'upgrading from v{previous} to v{target}', f'upgrading from v{target} to v{previous}',
         'README migration source'),
        ('.github/workflows/ci.yml', 'python-version: "3.11"', 'python-version: "3.x"', 'must select Python 3.11'),
    )
    with tempfile.TemporaryDirectory(prefix='esp32-release-metadata-') as temp:
        copy = Path(temp)
        for file in FILES:
            dest = copy / file
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(root / file, dest)
        verify(copy, release)
        for file, original, defective, diagnostic in mutations:
            before = read(copy, file)
            if original not in before:
                raise AssertionError(f'Mutation did not match {file}: {original}')
            (copy / file).write_text(before.replace(original, defective, -1 if file.endswith("ci.yml") else 1), encoding='utf-8')
            if diagnostic == 'Unreleased must be empty':
                verify(copy)  # Development accepts pending changes.
            try:
                verify(copy, release)
            except ValueError as error:
                if diagnostic not in str(error):
                    raise AssertionError(f'Wrong rejection for {file}: {error}') from error
            else:
                raise AssertionError(f'Accepted defective {file}: {diagnostic}')
            finally:
                (copy / file).write_text(before, encoding='utf-8')
        verify(copy, release)
        try:
            verify(copy, (release[0], release[1], f'v{wrong}', release[3]))
        except ValueError as error:
            if 'Proposed tag' not in str(error):
                raise
        else:
            raise AssertionError('Accepted mismatched tag')
        bad_notes = copy / 'wrong-notes.md'
        bad_notes.write_text(release[3].read_text(encoding='utf-8').replace(f'v{target} — ', f'v{wrong} — ', 1), encoding='utf-8')
        try:
            verify(copy, (*release[:3], bad_notes))
        except ValueError as error:
            if 'Release note title' not in str(error):
                raise
        else:
            raise AssertionError('Accepted mismatched release note title')
    print(f'PASS: {len(mutations) + 2} release metadata negatives; development and release checks remain distinct')


if __name__ == '__main__':
    if len(sys.argv) != 5:
        raise SystemExit('Usage: test_release_metadata.py VERSION DATE TAG NOTES')
    run(Path(__file__).resolve().parents[1], (*sys.argv[1:4], Path(sys.argv[4]).resolve()))
