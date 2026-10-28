"""Offline comparison of sensor forecasts with evaluation-only Gazebo truth."""
import argparse
import bisect
import json
import math
from pathlib import Path
import statistics


def evaluate(directory):
    trajectory=[json.loads(line) for line in (directory/'trajectory.jsonl').read_text().splitlines()]
    predictions=[json.loads(line) for line in (directory/'prediction_observations.jsonl').read_text().splitlines()]
    times=[s['sim_s'] for s in trajectory]; errors=[]; future_errors=[]; active=0; associated=0; without_yaw=0
    def nearest(stamp):
        index=bisect.bisect_left(times,stamp)
        return min(range(max(0,index-1),min(len(times),index+1)),key=lambda i:abs(times[i]-stamp))
    for message in predictions:
        if not times: break
        index=nearest(message['source_stamp_s']); sample=trajectory[index]
        if 'odom_yaw' not in sample: without_yaw+=1; continue
        previous=trajectory[max(0,index-5)]; dt=sample['sim_s']-previous['sim_s']
        if dt<.3 or not sample['agents'] or sample['odom'] is None: continue
        angle=sample['robot'][2]-sample['odom_yaw']; c=math.cos(angle); s=math.sin(angle)
        for track in message['tracks']:
            if not track['forecast_active']: continue
            active+=1; px,py=track['observed_center_odom']; dx=px-sample['odom'][0]; dy=py-sample['odom'][1]
            world=(sample['robot'][0]+c*dx-s*dy,sample['robot'][1]+s*dx+c*dy)
            name=min(sample['agents'],key=lambda n:math.dist(world,sample['agents'][n][:2]))
            if math.dist(world,sample['agents'][name][:2])>.9 or name not in previous['agents']: continue
            associated+=1; pose=sample['agents'][name]; old=previous['agents'][name]
            actual=((pose[0]-old[0])/dt,(pose[1]-old[1])/dt)
            vx,vy=track['velocity_odom_m_s']; estimated=(c*vx-s*vy,s*vx+c*vy)
            errors.append(math.dist(actual,estimated))
            future=trajectory[nearest(sample['sim_s']+2)]
            if abs(future['sim_s']-sample['sim_s']-2)>.2 or name not in future['agents']: continue
            # Compare displacements so visible-surface centroid bias is not mistaken for forecast error.
            displacement=(future['agents'][name][0]-pose[0],future['agents'][name][1]-pose[1])
            future_errors.append(math.dist(displacement,(2*estimated[0],2*estimated[1])))
    result={'active_track_observations':active,'associated_with_truth_within_0_9m':associated,
            'samples_without_odom_yaw':without_yaw,
            'median_velocity_vector_error_m_s':statistics.median(errors) if errors else None,
            'p95_velocity_vector_error_m_s':sorted(errors)[int(.95*(len(errors)-1))] if errors else None,
            'median_2s_displacement_error_m':statistics.median(future_errors) if future_errors else None,
            'p95_2s_displacement_error_m':sorted(future_errors)[int(.95*(len(future_errors)-1))] if future_errors else None,
            'note':'Offline truth association for evaluation only. Includes reversals and acceleration; not semantic tracking or intent prediction.'}
    (directory/'forecast_evaluation.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(); p.add_argument('--root',type=Path,default=Path('docs/dynamic_test_runs/validation')); args=p.parse_args()
    for file in sorted(args.root.glob('*/prediction_observations.jsonl')):
        result=evaluate(file.parent); print(file.parent.name,json.dumps(result))


if __name__=='__main__': main()
