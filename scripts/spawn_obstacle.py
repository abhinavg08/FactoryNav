#!/usr/bin/env python3
"""Insert a real, collision-enabled blockage into the factory's central aisle."""
import argparse
import json
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='default')
    parser.add_argument('--name', default='unexpected_obstacle')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    args = parser.parse_args()
    # Height intersects the robot's real simulated LiDAR plane. Static collision
    # geometry prevents the robot from pushing the blockage out of its path.
    sdf = f'''<sdf version="1.9"><model name="blockage"><static>true</static>
      <pose>{args.x} {args.y} 0.5 0 0 0</pose><link name="body">
      <collision name="collision"><geometry><box><size>0.7 2.2 1.0</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>0.7 2.2 1.0</size></box></geometry>
      <material><ambient>0.95 0.15 0.08 1</ambient><diffuse>0.95 0.15 0.08 1</diffuse></material>
      </visual></link></model></sdf>'''
    request = f'sdf: {json.dumps(sdf)} name: {json.dumps(args.name)} allow_renaming: false'
    command = ['gz', 'service', '-s', f'/world/{args.world}/create',
               '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
               '--timeout', '5000', '--req', request]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=12, check=False)
    success = completed.returncode == 0 and bool(re.search(r'data:\s*true', completed.stdout))
    print(json.dumps({'success': success, 'name': args.name, 'x': args.x, 'y': args.y,
                      'size_m': [0.7, 2.2, 1.0], 'response': completed.stdout.strip(),
                      'stderr': completed.stderr.strip()}))
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
