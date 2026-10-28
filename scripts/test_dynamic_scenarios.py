"""Fresh-container, reproducible real ROS/Gazebo experiments (host Python)."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import threading
import time
import hashlib
import shutil
import shlex

ROOT=Path(__file__).resolve().parents[1]
STARTS={'head_on':(-7,0,0),'crossing':(-5,0,0),'alongside':(-7,0,0),'cart':(-5,0,0),'cart_shared':(-7,0,0),'blocked':(-9.925,-.7,1.57079632679),'contact_probe':(-7,0,0),'full':(-8,-6.5,0),'static':(-8,-6.5,0)}


def run_case(scenario,profile,seed,root,perception=False,gui=False,keep=False):
    out=root/f'{profile}_{scenario}_{seed}'
    if out.exists(): raise RuntimeError(f'Refusing to overwrite evidence: {out}')
    out.mkdir(parents=True)
    sources=['factorynav_bringup/config/dynamic_nav2.yaml','factorynav_bringup/config/mppi_dynamic.yaml','factorynav_bringup/config/dynamic_scene.json','factorynav_bringup/launch/dynamic_factory.launch.py','factorynav_dynamic/factorynav_dynamic/traffic.py','factorynav_dynamic/factorynav_dynamic/models.py','factorynav_dynamic/factorynav_dynamic/benchmark.py','factorynav_dynamic/factorynav_dynamic/forecast.py']
    (out/'source_hashes.json').write_text(json.dumps({s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources},indent=2)+'\n')
    for source in ('factorynav_dynamic/factorynav_dynamic/lidar_geometry.py','factorynav_mppi_critics/src/dynamic_obstacle_critic.cpp','factorynav_mppi_critics/include/factorynav_mppi_critics/temporal_cost.hpp'):
        hashes=json.loads((out/'source_hashes.json').read_text()); hashes[source]=hashlib.sha256((ROOT/source).read_bytes()).hexdigest()
        (out/'source_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    env=dict(os.environ); x,y,yaw=STARTS[scenario]
    env.update(FACTORYNAV_PROFILE=profile,FACTORYNAV_START_X=str(x),FACTORYNAV_START_Y=str(y),FACTORYNAV_START_YAW=str(yaw),FACTORYNAV_GUI=str(gui).lower(),FACTORYNAV_PERCEPTION=str(perception).lower(),FACTORYNAV_TRAFFIC='none')
    compose=['docker','compose','-f','compose.yaml']
    if perception: compose+=['-f','compose.perception.yaml']
    compose+=['-f','compose.dynamic.yaml']
    print(f'Starting {profile}/{scenario}/seed={seed}',flush=True)
    case_wall_start=time.time()
    startup=subprocess.run(compose+['up','-d','--no-build','--force-recreate'],cwd=ROOT,env=env,capture_output=True,text=True)
    (out/'startup.log').write_text(startup.stdout+startup.stderr)
    if startup.returncode: raise RuntimeError(startup.stderr)
    samples=[]; done=threading.Event()
    def resources():
        while not done.is_set():
            stats=subprocess.run(['docker','stats','--no-stream','--format','{{json .}}','cele-simulation-1'],capture_output=True,text=True)
            try: samples.append({'wall_time':time.time(),**json.loads(stats.stdout)})
            except (ValueError,TypeError): pass
            done.wait(5)
    sampler=threading.Thread(target=resources,daemon=True); sampler.start()
    relative=out.relative_to(ROOT).as_posix()
    command=f'source /opt/ros/jazzy/setup.bash; export PYTHONPATH=/workspace/factorynav_dynamic:/workspace/scripts:$PYTHONPATH; python3 -u -m factorynav_dynamic.benchmark --scenario {scenario} --profile {profile} --seed {seed} --output {shlex.quote("/workspace/"+relative)}'
    started=time.monotonic()
    try:
        with (out/'execution.log').open('w') as log:
            result=subprocess.run(['docker','compose','exec','-T','simulation','bash','-lc',command],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1000)
        code=result.returncode
    finally:
        done.set(); sampler.join(timeout=15)
        (out/'resources.json').write_text(json.dumps(samples,indent=2)+'\n')
        logs=subprocess.run(['docker','compose','logs','--no-color','simulation'],cwd=ROOT,capture_output=True,text=True)
        (out/'simulation.log').write_text(logs.stdout+logs.stderr)
        effective=ROOT/'log/dynamic/effective_nav2.yaml'
        if effective.exists(): shutil.copy2(effective,out/'effective_nav2.yaml')
        forecast_log=ROOT/'log/dynamic/forecast.log'
        if forecast_log.exists(): shutil.copy2(forecast_log,out/'forecast.log')
        if perception:
            for file in (ROOT/'log/dynamic/perception').glob('*'):
                if file.is_file() and file.stat().st_mtime>=case_wall_start: shutil.copy2(file,out/file.name)
            yolo=ROOT/'log/dynamic/yolo.log'
            if yolo.exists(): shutil.copy2(yolo,out/'yolo.log')
    record=json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else {'success':False,'error':(out/'execution.log').read_text()[-3000:]}
    record['host_elapsed_s']=time.monotonic()-started
    (out/'host_result.json').write_text(json.dumps(record,indent=2)+'\n')
    summary={k:record.get(k) for k in ('scenario','controller_profile','success','error','minimum_agent_footprint_clearance_m','contact_events','continuous_passing_evidence','near_agent_stopped_sim_s','sensor_validation','person_detection_frames')}
    summary['failed_acceptance_checks']=[k for k,v in record.get('acceptance_checks',{}).items() if not v]
    print(json.dumps(summary),flush=True)
    if not keep: subprocess.run(['docker','compose','stop','simulation'],cwd=ROOT,stdout=subprocess.DEVNULL,check=True)
    return code==0 and record['success']


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--scenarios',nargs='+',choices=list(STARTS),default=['head_on','crossing','alongside','cart','full'])
    parser.add_argument('--profiles',nargs='+',choices=['baseline','dynamic'],default=['dynamic']); parser.add_argument('--repeat',type=int,default=3)
    parser.add_argument('--start-seed',type=int,default=0)
    parser.add_argument('--final-suite',action='store_true',help='Five scenarios x3, YOLO on full missions, extra safety cases, and original-profile head-on comparison')
    parser.add_argument('--fail-fast',action='store_true')
    parser.add_argument('--resume',action='store_true',help='Retain completed trials only when their recorded source hashes still match')
    parser.add_argument('--output',type=Path,required=True); parser.add_argument('--perception',action='store_true'); parser.add_argument('--gui',action='store_true'); parser.add_argument('--keep',action='store_true')
    args=parser.parse_args(); root=(ROOT/args.output).resolve(); root.relative_to(ROOT); outcomes=[]
    if args.final_suite:
        jobs=[(scenario,'dynamic',seed,scenario=='full') for scenario in ('head_on','crossing','alongside','cart','full') for seed in range(3)]
        jobs += [(scenario,'dynamic',0,False) for scenario in ('blocked','cart_shared')]
        jobs += [('head_on','baseline',seed,False) for seed in range(3)]
    else:
        jobs=[(scenario,profile,seed,args.perception) for profile in args.profiles for scenario in args.scenarios for seed in range(args.start_seed,args.start_seed+args.repeat)]
    for scenario,profile,seed,perception in jobs:
        completed=root/f'{profile}_{scenario}_{seed}'
        if args.resume and (completed/'host_result.json').exists():
            hashes=json.loads((completed/'source_hashes.json').read_text())
            changed=[name for name,digest in hashes.items() if not (ROOT/name).exists() or hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest]
            if changed: raise RuntimeError(f'Cannot resume changed sources for {completed.name}: {changed}; use a new output directory')
            record=json.loads((completed/'host_result.json').read_text())
            outcomes.append(bool(record['success']))
            print(f'Retained completed {completed.name}: success={record["success"]}',flush=True)
            continue
        outcomes.append(run_case(scenario,profile,seed,root,perception,args.gui,args.keep))
        if args.fail_fast and not outcomes[-1]: break
    print(f'{sum(outcomes)}/{len(outcomes)} passed',flush=True)
    return 0 if all(outcomes) else 1


if __name__=='__main__': raise SystemExit(main())
