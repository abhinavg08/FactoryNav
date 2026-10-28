"""Summarize saved measured runs; no synthesized trajectories or outcomes."""
import argparse
import csv
import json
from pathlib import Path
import statistics


def main():
    p=argparse.ArgumentParser(); p.add_argument('--root',type=Path,default=Path('/workspace/docs/dynamic_test_runs')); p.add_argument('--output',type=Path,default=Path('/workspace/docs')); args=p.parse_args()
    records=[]
    for path in sorted(args.root.rglob('result.json')):
        row=json.loads(path.read_text())
        if 'controller_profile' not in row: continue
        row['evidence']=path.relative_to(args.root).as_posix(); row['goal_sim_s']=sum(g['simulation_s'] for g in row.get('goals',[])); records.append(row)
    fields=['evidence','scenario','controller_profile','seed','success','navigation_success','safety_pass','goal_sim_s','path_length_m','minimum_estimated_footprint_clearance_m','minimum_agent_footprint_clearance_m','contact_events','stopped_sim_s','slowing_sim_s','near_agent_stopped_sim_s','near_agent_progress_m','longest_near_agent_stop_s','recovery_count','angular_command_sign_changes_over_0_25rad_s','continuous_passing_evidence','person_detection_frames','maximum_person_confidence','median_inference_ms']
    args.output.mkdir(parents=True,exist_ok=True)
    with (args.output/'dynamic_results.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore'); writer.writeheader(); writer.writerows(records)
    (args.output/'dynamic_results.json').write_text(json.dumps(records,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    media=args.output.parent/'media'; media.mkdir(exist_ok=True)
    candidates=[r for r in records if r['scenario']=='head_on' and r['controller_profile']=='dynamic' and r['success'] and any(s['lethal_costmap_samples']>0 for s in r.get('sensor_validation',{}).values())]
    if candidates:
        chosen=candidates[-1]; directory=args.root/chosen['evidence']; trajectory=[json.loads(s) for s in directory.with_name('trajectory.jsonl').read_text().splitlines()]
        fig,axes=plt.subplots(2,1,figsize=(11,7),gridspec_kw={'height_ratios':[1,1.2]})
        axes[0].plot([s['robot'][0] for s in trajectory],[s['robot'][1] for s in trajectory],label='Robot measured trajectory')
        for name in trajectory[-1]['agents']:
            samples=[s for s in trajectory if name in s['agents']]
            axes[0].plot([s['agents'][name][0] for s in samples],[s['agents'][name][1] for s in samples],label=f'{name} measured trajectory')
        axes[0].set(xlabel='World x (m)',ylabel='World y (m)',title=f'Actual head-on passing — {chosen["evidence"]}'); axes[0].legend(); axes[0].grid(alpha=.3)
        start=trajectory[0]['sim_s']; times=[s['sim_s']-start for s in trajectory]
        clearances=[min((v for k,v in s['clearances'].items() if k in s['agents']),default=float('nan')) for s in trajectory]
        axes[1].plot(times,clearances,label='Footprint clearance (m)'); axes[1].axhline(0,color='red',label='Geometric contact boundary')
        axes[1].set(xlabel='Elapsed simulation time (s)',ylabel='Clearance (m)',ylim=(-.1,3)); axes[1].legend(); axes[1].grid(alpha=.3)
        fig.tight_layout(); fig.savefig(media/'human_avoidance_trajectory.png',dpi=150); plt.close(fig)
    print(f'Exported {len(records)} measured runs.')


if __name__=='__main__': main()
