"""Cache pinned perception wheels from official PyPI with SHA256 verification."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import time
from urllib.request import Request, urlopen

DESTINATION = Path(__file__).resolve().parents[1] / '.cache/wheels'


def fetch(url, start, end, total):
    request = Request(url, headers={'Range': f'bytes={start}-{end}'})
    with urlopen(request, timeout=60) as response:
        if response.status != 206 or response.headers.get('Content-Range') != f'bytes {start}-{end}/{total}':
            raise RuntimeError('Server did not return the requested byte range')
        data = response.read()
    if len(data) != end - start + 1:
        raise RuntimeError('Incomplete wheel byte range')
    return data


def main():
    DESTINATION.mkdir(parents=True, exist_ok=True)
    lock_path = DESTINATION.parents[1] / 'setup/perception_wheels.lock.json'
    existing = json.loads(lock_path.read_text()) if lock_path.exists() else {}
    manifest = {}
    for package, version in [
        ('opencv-python', '4.11.0.86'), ('polars-runtime-32', '2.1.0'),
        ('ultralytics', '8.4.191'), ('polars', '2.1.0'),
        ('nvidia-ml-py', '13.615.71'), ('ultralytics-thop', '2.2.3'),
        ('ultralytics-platform', '0.1.100'), ('httpx', '0.28.1'),
        ('httpcore', '1.0.9'), ('anyio', '4.15.1'), ('h11', '0.16.0'),
        ('typing-extensions', '4.16.0'), ('tzdata', '2026.5')
    ]:
        known = existing.get(package)
        if known and known['version'] == version:
            cached = DESTINATION / known['filename']
            if cached.exists() and hashlib.sha256(cached.read_bytes()).hexdigest() == known['sha256']:
                manifest[package] = known
                print(f'Cached and verified: {cached.name}', flush=True)
                continue
        with urlopen(f'https://pypi.org/pypi/{package}/{version}/json', timeout=30) as response:
            metadata = json.load(response)
        wheel = next(item for item in metadata['urls'] if item['filename'].endswith('.whl') and (
            ('manylinux' in item['filename'] and 'x86_64' in item['filename'])
            or item['filename'].endswith('-none-any.whl')))
        if known and known['version'] == version and wheel['digests']['sha256'] != known['sha256']:
            raise RuntimeError(f'PyPI checksum changed for pinned package {package}')
        target = DESTINATION / wheel['filename']
        manifest[package] = {'version': version, 'filename': wheel['filename'],
                             'sha256': wheel['digests']['sha256'], 'url': wheel['url']}
        if target.exists() and hashlib.sha256(target.read_bytes()).hexdigest() == wheel['digests']['sha256']:
            print(f'Cached and verified: {target.name}', flush=True)
            continue
        began = time.monotonic()
        size = wheel['size']
        chunk_size = 1024 * 1024
        ranges = [(offset, min(offset + chunk_size, size) - 1) for offset in range(0, size, chunk_size)]
        with ThreadPoolExecutor(max_workers=16) as pool:
            chunks = list(pool.map(lambda pair: fetch(wheel['url'], *pair, size), ranges))
        data = b''.join(chunks)
        if hashlib.sha256(data).hexdigest() != wheel['digests']['sha256']:
            raise RuntimeError('Wheel SHA256 does not match official PyPI metadata')
        target.write_bytes(data)
        print(json.dumps({'file': target.name, 'bytes': len(data), 'seconds': round(time.monotonic() - began, 2),
                          'sha256': wheel['digests']['sha256']}), flush=True)
    lock_path.write_text(
        json.dumps(manifest, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
