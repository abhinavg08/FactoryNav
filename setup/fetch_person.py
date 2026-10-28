"""Fetch the attributed OSRF standing-person model into the local asset cache."""
import hashlib
import json
from pathlib import Path
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
MODEL_ROOT = ROOT / '.cache' / 'models'
API = 'https://api.github.com/repos/osrf/gazebo_models'


def read_url(url):
    request = Request(url, headers={'User-Agent': 'FactoryNav-AI-setup'})
    with urlopen(request, timeout=40) as response:
        return response.read()


def main():
    lock_path = ROOT / 'setup' / 'person_assets.lock.json'
    lock = json.loads(lock_path.read_text()) if lock_path.exists() else None
    if lock and all((MODEL_ROOT / name).is_file() and
                    hashlib.sha256((MODEL_ROOT / name).read_bytes()).hexdigest() == checksum
                    for name, checksum in lock['files'].items()) and (
                        MODEL_ROOT / 'person_standing/OSRF-LICENSE.txt').is_file():
        print(json.dumps({'cached_and_verified': True, 'revision': lock['revision'],
                          'model_directory': str(MODEL_ROOT / 'person_standing')}))
        return
    revision = lock['revision'] if lock else json.loads(read_url(f'{API}/commits/master'))['sha']
    manifest = {'repository': 'https://github.com/osrf/gazebo_models',
                'revision': revision, 'license': 'CC-BY-3.0',
                'author': 'Marina Kollmitz', 'files': {}}

    def fetch_directory(repository_path):
        for item in json.loads(read_url(f'{API}/contents/{repository_path}?ref={revision}')):
            if item['type'] == 'dir':
                fetch_directory(item['path'])
            elif item['type'] == 'file':
                relative = Path(item['path'])
                destination = (MODEL_ROOT / relative).resolve()
                if not destination.is_relative_to((MODEL_ROOT / 'person_standing').resolve()):
                    raise RuntimeError('Unexpected model asset path')
                destination.parent.mkdir(parents=True, exist_ok=True)
                data = read_url(item['download_url'])
                checksum = hashlib.sha256(data).hexdigest()
                if lock and lock['files'].get(item['path']) != checksum:
                    raise RuntimeError(f'Asset checksum changed: {item["path"]}')
                destination.write_bytes(data)
                manifest['files'][item['path']] = checksum

    fetch_directory('person_standing')
    license_path = MODEL_ROOT / 'person_standing' / 'OSRF-LICENSE.txt'
    license_path.write_bytes(read_url(f'https://raw.githubusercontent.com/osrf/gazebo_models/{revision}/LICENSE'))
    lock_path.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'model_directory': str(license_path.parent), **manifest}, indent=2))


if __name__ == '__main__':
    main()
