"""Audit a recorded single-human camera trial using evaluation-only poses."""
import argparse
import json
import math
from pathlib import Path


def verify(directory):
    trajectory = [json.loads(s) for s in (directory / 'trajectory.jsonl').read_text().splitlines()]
    metrics = [json.loads(s) for s in (directory / 'perception.jsonl').read_text().splitlines()]
    result = json.loads((directory / 'result.json').read_text())
    names = {name for sample in trajectory for name in sample['agents']}
    if len(names) != 1 or not next(iter(names)).startswith('human'):
        raise ValueError('This audit requires exactly one human and no other traffic actor')
    name = next(iter(names))

    def motion(stamp):
        index = min(range(len(trajectory)), key=lambda i: abs(trajectory[i]['sim_s'] - stamp))
        sample = trajectory[index]
        prior = trajectory[max(0, index - 5)]
        dt = sample['sim_s'] - prior['sim_s']
        if abs(sample['sim_s'] - stamp) > .2 or dt < .3:
            return None
        pose = sample['agents'][name]
        speed = math.dist(pose[:2], prior['agents'][name][:2]) / dt
        dx, dy = pose[0] - sample['robot'][0], pose[1] - sample['robot'][1]
        bearing = math.remainder(math.atan2(dy, dx) - sample['robot'][2], 2 * math.pi)
        return {'speed_m_s': speed, 'bearing_rad': bearing,
                'in_camera_frustum': abs(bearing) < .65 and math.hypot(dx, dy) < 12}

    frames = []
    for metric in metrics:
        boxes = [d for d in metric.get('detections', []) if d['class'] == 'person']
        stamp = metric.get('source_stamp_s')
        if not boxes or stamp is None:
            continue
        observed = motion(stamp)
        if observed and observed['speed_m_s'] > .05 and observed['in_camera_frustum']:
            frames.append({'source_stamp_s': stamp, **observed, 'detections': boxes})
    stamps = sorted({f['source_stamp_s'] for f in frames})
    widths = [d['xyxy'][2] - d['xyxy'][0] for f in frames for d in f['detections']]
    best_file = directory / 'best_detection.json'
    best = json.loads(best_file.read_text()) if best_file.exists() else None
    best_motion = motion(best['source_stamp_s']) if best else None
    passed = (len(stamps) >= 3 and stamps[-1] - stamps[0] > 1
              and result['ros_person_detection_messages'] >= 3
              and max(widths) - min(widths) > 1)
    report = {'success': passed, 'actor': name,
              'distinct_person_frames_while_actor_moving_in_frustum': len(stamps),
              'source_time_span_s': stamps[-1] - stamps[0] if stamps else 0,
              'bbox_width_range_px': [min(widths), max(widths)] if widths else None,
              'ros_person_arrays': result['ros_person_detection_messages'],
              'best_image_source_stamp_s': best['source_stamp_s'] if best else None,
              'best_image_actor_motion': best_motion,
              'frames': frames,
              'note': 'Offline singleton-scene evidence, not an identity tracker or metric depth from YOLO. Gazebo truth never feeds navigation.'}
    (directory / 'single_person_validation.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    report = verify(parser.parse_args().directory)
    print(json.dumps({k: v for k, v in report.items() if k != 'frames'}, indent=2))
    raise SystemExit(0 if report['success'] else 1)
