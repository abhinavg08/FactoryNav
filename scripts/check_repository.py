"""Check publication files, Markdown targets, evidence claims and reachable history.

Uses only Python's standard library. It never prints matched credential values.
Run from any directory: python scripts/check_repository.py [--history]
"""
import argparse
from collections import Counter
from html.parser import HTMLParser
import io
import json
from pathlib import Path
import re
import struct
import subprocess
from urllib.parse import unquote, urlsplit
import zipfile

ROOT = Path(__file__).resolve().parents[1]
VIDEOS = {'.mp4', '.mkv', '.mov', '.webm', '.avi', '.m4v', '.gifv'}
SECRETS = {
    'GitHub credential': re.compile(rb'(?:github_pat_[A-Za-z0-9_]{40,}|gh[pousr]_[A-Za-z0-9]{30,})'),
    'AWS access key': re.compile(rb'(?:AKIA|ASIA)[A-Z0-9]{16}'),
    'private key': re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----'),
    'Slack credential': re.compile(rb'xox[baprs]-[A-Za-z0-9-]{20,}'),
    'Google API key': re.compile(rb'AIza[0-9A-Za-z_-]{35}'),
    'OpenAI credential': re.compile(rb'sk-(?:proj-|svcacct-)?[A-Za-z0-9_-]{40,}'),
    'credential in URL': re.compile(rb'https?://[^\s/@:]+:[^\s/@]{8,}@'),
    'Google Drive link': re.compile(rb'https?://(?:drive\.google\.com|docs\.google\.com/(?:file|uc))', re.I),
}


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT)


class ImageParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.targets = []

    def handle_starttag(self, tag, attrs):
        attributes = dict(attrs)
        if tag == 'img' and attributes.get('src'):
            self.targets.append(attributes['src'])
        if tag == 'a' and attributes.get('href'):
            self.targets.append(attributes['href'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--history', action='store_true')
    args = parser.parse_args()
    failures = []
    files = {p.decode() for p in git('ls-files', '-z').split(b'\0') if p}
    files |= {p.decode() for p in git('ls-files', '--others', '--exclude-standard', '-z').split(b'\0') if p}

    def scan(name, data):
        for label, pattern in SECRETS.items():
            if pattern.search(data):
                failures.append({'file': name, 'issue': label})
        if data.startswith(b'PK\x03\x04'):
            with zipfile.ZipFile(io.BytesIO(data)) as archive:
                for entry in archive.infolist():
                    if entry.is_dir():
                        continue
                    if Path(entry.filename).suffix.lower() in VIDEOS:
                        failures.append({'file': name, 'issue': 'video in archive'})
                    scan(name + ':' + entry.filename, archive.read(entry))

    for name in sorted(files):
        path = ROOT / name
        if not path.is_file():
            failures.append({'file': name, 'issue': 'tracked file missing'})
            continue
        if path.suffix.lower() in VIDEOS:
            failures.append({'file': name, 'issue': 'video selected for publication'})
        if path.name == '.env' or (path.name.startswith('.env.') and path.name != '.env.example'):
            failures.append({'file': name, 'issue': 'local environment secrets file'})
        if path.stat().st_size > 50 * 1024 * 1024:
            failures.append({'file': name, 'issue': 'file exceeds 50 MiB'})
        data = path.read_bytes()
        scan(name, data)
        if path.suffix.lower() != '.md':
            continue
        text = data.decode('utf-8')
        content = re.sub(r'^```[^\n]*\n.*?^```\s*$', '', text, flags=re.M | re.S)
        targets = re.findall(r'!?\[[^\]\n]*\]\(([^)\n]+)\)', content)
        images = ImageParser()
        images.feed(content)
        targets += images.targets
        for target in targets:
            target = target.strip().split(' "', 1)[0].strip('<>')
            parsed = urlsplit(target)
            if parsed.scheme or parsed.netloc or not parsed.path:
                continue
            resolved = (path.parent / unquote(parsed.path)).resolve()
            if not resolved.is_relative_to(ROOT) or not resolved.exists():
                failures.append({'file': name, 'issue': 'broken local link', 'target': target})

    image_dimensions = {}
    for name in ('media/normal_factory_map.png', 'media/dynamic_factory_map.png'):
        data = (ROOT / name).read_bytes()
        if data[:8] != b'\x89PNG\r\n\x1a\n':
            failures.append({'file': name, 'issue': 'map is not a real PNG'})
        else:
            image_dimensions[name] = struct.unpack('>II', data[16:24])
    readme = (ROOT / 'README.md').read_text(encoding='utf-8')
    table = readme.split('<table>', 1)[1].split('</table>', 1)[0]
    if not all(name in table for name in image_dimensions):
        failures.append({'file': 'README.md', 'issue': 'both maps must share the first HTML table'})

    runs = [json.loads(path.read_text()) for path in (ROOT / 'docs/dynamic_test_runs/validation').glob('*/host_result.json')]
    advanced = [row for row in runs if row['controller_profile'] == 'dynamic']
    accepted = [row for row in advanced if row['success']]
    counts = Counter(row['scenario'] for row in accepted)
    if len(advanced) != 17 or len(accepted) != 17 or counts['head_on'] != 3 or counts['full'] != 3:
        failures.append({'file': 'docs/dynamic_test_runs/validation', 'issue': 'headline trial counts do not match evidence'})
    if any(row['contact_events'] for row in accepted):
        failures.append({'file': 'docs/dynamic_test_runs/validation', 'issue': 'accepted trial reports contacts'})

    history_blobs = 0
    if args.history:
        objects = git('rev-list', '--objects', '--all').splitlines()
        process = subprocess.Popen(['git', 'cat-file', '--batch'], cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        try:
            for entry in objects:
                oid, _, raw_name = entry.partition(b' ')
                process.stdin.write(oid + b'\n')
                process.stdin.flush()
                header = process.stdout.readline().split()
                size = int(header[2])
                data = process.stdout.read(size)
                process.stdout.read(1)
                if header[1] == b'blob':
                    history_blobs += 1
                    name = raw_name.decode('utf-8', errors='replace')
                    if Path(name).suffix.lower() in VIDEOS:
                        failures.append({'file': 'history:' + name, 'issue': 'video in reachable history'})
                    scan('history:' + name, data)
        finally:
            process.stdin.close()
            process.wait(timeout=10)

    report = {'publication_files': len(files), 'history_blobs_scanned': history_blobs,
              'maps': image_dimensions, 'advanced_accepted': len(accepted),
              'head_on_accepted': counts['head_on'], 'full_missions_accepted': counts['full'],
              'accepted_reported_contacts': sum(row['contact_events'] for row in accepted),
              'failures': failures}
    print(json.dumps(report, indent=2))
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(main())
