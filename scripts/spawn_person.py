#!/usr/bin/env python3
"""Spawn the attributed textured person for real Gazebo-camera detection tests."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=-0.45)
    parser.add_argument('--yaw', type=float, default=-1.5708)
    args = parser.parse_args()
    model_path = Path(__file__).resolve().parents[1] / '.cache/models/person_standing/model.sdf'
    if not model_path.exists():
        raise SystemExit('Person asset missing. Run python3 setup/fetch_person.py first.')
    tree = ET.parse(model_path)
    model = tree.getroot().find('model')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{args.x} {args.y} 0 0 0 {args.yaw}'
    request = f'sdf: {json.dumps(ET.tostring(tree.getroot(), encoding="unicode"))} name: "factory_person" allow_renaming: false'
    completed = subprocess.run(['gz', 'service', '-s', '/world/default/create',
        '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', request], capture_output=True, text=True, timeout=12)
    success = completed.returncode == 0 and bool(re.search(r'data:\s*true', completed.stdout))
    print(json.dumps({'success': success, 'x': args.x, 'y': args.y,
                      'response': completed.stdout.strip(), 'stderr': completed.stderr.strip()}))
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
