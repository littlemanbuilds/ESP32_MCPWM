#!/usr/bin/env python3
"""Check development metadata and the stricter proposed-release contract."""

import argparse
from datetime import date
import hashlib
import json
from pathlib import Path
import re
import sys


HISTORY_SHA256 = '820cb3ba97b7a79d86b135fce4a6f1767d853c3088614949eff8e314a70fc36e'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def one(pattern, text, message):
    matches = re.findall(pattern, text, re.MULTILINE)
    require(len(matches) == 1, message)
    return matches[0]


def read(root, path):
    return (root / path).read_text(encoding='utf-8')


def verify(root, release=None):
    header = read(root, 'src/ESP32_MCPWM.h')
    version = one(r'^#define ESP32_MCPWM_VERSION "(\d+\.\d+\.\d+)"$', header, 'Missing or duplicate version macro')
    numbers = [one(rf'^#define ESP32_MCPWM_VERSION_{field} (\d+)$', header, f'Missing or duplicate {field} macro')
               for field in ('MAJOR', 'MINOR', 'PATCH')]
    require('.'.join(numbers) == version, 'Numeric version macros disagree')
    properties = one(r'^version=(\S+)$', read(root, 'library.properties'), 'Missing or duplicate library.properties version')
    manifest = json.loads(read(root, 'library.json'))
    require(properties == version == manifest.get('version'), 'Package versions disagree')

    workflow = read(root, '.github/workflows/ci.yml')
    for job in ('platformio', 'examples'):
        section = one(rf'^  {job}:\n((?:\n|    [^\n]*\n)*)', workflow, f'Missing or duplicate CI job {job}')
        require(re.findall(r'^          python-version: (.+)$', section, re.MULTILINE) == ['"3.11"'],
                f'{job} must select Python 3.11')
    if release is None:
        return

    target, planned, tag, notes = release
    require(target == version and tag == f'v{version}', 'Proposed tag or release version disagrees with the package')
    require(date.fromisoformat(planned).isoformat() == planned, 'Release date must be ISO YYYY-MM-DD')
    checklist = read(root, 'RELEASE_CHECKLIST.md')
    require(one(r'^Target release \*\*([^*]+)\*\*\. Proposed publication \*\*([^*]+)\*\*\.', checklist,
                'Missing or duplicate checklist target') == (target, planned),
            'Checklist version or publication date disagrees')
    readme = read(root, 'README.md')
    current = one(r'^Current version:\n\n```text\n([^\n]+)\n```$', readme, 'Missing or duplicate README current version')
    require(current == version, 'README current version is stale')

    changelog = (root / 'CHANGELOG.md').read_bytes()
    text = changelog.decode('utf-8')
    require(one(r'^## \[Unreleased\][^\n]*\n([\s\S]*?)(?=^## |\Z)', text.replace('\r\n', '\n'),
                'Missing or duplicate Unreleased section') == '\n', 'Unreleased must be empty for release')
    heading = one(rf'^## \[{re.escape(version)}\](.*)$', text, 'Missing or duplicate target changelog section')
    require(heading == f' - {planned}', 'Target changelog date disagrees with release metadata')
    require(re.findall(r'^## \[([^\]]+)\]', text, re.MULTILINE)[:2] == ['Unreleased', version],
            'Target version must follow Unreleased')
    marker = b'## [2.0.0] - 2026-08-30\n'
    require(changelog.count(marker) == 1 and hashlib.sha256(marker + changelog.split(marker, 1)[1]).hexdigest() == HISTORY_SHA256,
            'Published v2.0.0 and earlier changelog entries changed')
    source = re.search(rf'^## \[{re.escape(version)}\].*?^## \[(\d+\.\d+\.\d+)\]', text, re.MULTILINE | re.DOTALL)
    require(source is not None, 'Previous release section is missing')
    migration = f'When upgrading from v{source[1]} to v{version},'
    require(readme.count(migration) == 1 and f'upgrading from v{version}' not in readme,
            'README migration source or destination is incorrect')
    title, _, body = notes.read_text(encoding='utf-8').partition('\n')
    require(title.startswith(f'{tag} — ') and title[len(tag) + 3:].strip(), 'Release note title or version disagrees')
    require(f'ESP32_MCPWM {tag} ' in body and f'upgrading from v{source[1]}' in body and 'CHANGELOG.md' in body,
            'Release note version, migration or changelog reference is missing')
    sections = re.findall(r'^### (.+)$', body, re.MULTILINE)
    require(len(sections) == 3 and sections[0] == 'Highlights'
            and sections[1] in ('Compatibility', 'Breaking Changes and Migration') and sections[2] == 'Validation',
            'Release notes need Highlights, Compatibility and Validation')
    highlights = body.split('### Highlights\n', 1)[1].split('### ', 1)[0]
    require(4 <= len(re.findall(r'^- ', highlights, re.MULTILINE)) <= 7, 'Release notes need four to seven highlights')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', nargs=4, metavar=('VERSION', 'DATE', 'TAG', 'NOTES'))
    arguments = parser.parse_args()
    release = arguments.release
    if release:
        release[-1] = Path(release[-1])
    try:
        verify(Path(__file__).resolve().parents[1], release)
    except (ValueError, OSError) as error:
        print(f'FAIL: {error}', file=sys.stderr)
        return 1
    print('PASS: release metadata' if release else 'PASS: development metadata')
    return 0


if __name__ == '__main__':
    sys.exit(main())
