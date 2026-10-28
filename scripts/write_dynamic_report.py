"""Write Markdown reports directly from saved experiment results."""
import json
from pathlib import Path
import re
import statistics
import shutil

ROOT=Path(__file__).resolve().parents[1]
RUNS=ROOT/'docs/dynamic_test_runs/validation'


def read_runs():
    result=[]
    for path in sorted(RUNS.glob('*/host_result.json')):
        row=json.loads(path.read_text()); row['directory']=path.parent; result.append(row)
    return result


def mean(rows,key): return statistics.mean(r[key] for r in rows)
def duration(row): return sum(g['simulation_s'] for g in row['goals'])
def rtf(row): return duration(row)/sum(g['wall_s'] for g in row['goals']) if row['goals'] else 0
def fmt(value): return '—' if value is None else f'{value:.3f}'


def resources(rows):
    samples=[s for r in rows for s in json.loads((r['directory']/'resources.json').read_text())]
    cpu=[float(s['CPUPerc'].rstrip('%')) for s in samples]
    memory=[]
    for s in samples:
        match=re.match(r'([\d.]+)([KMG]?i?B)',s['MemUsage'])
        if match:
            value,unit=match.groups(); memory.append(float(value)*{'B':1/1048576,'kB':1/1024,'KB':1/1024,'KiB':1/1024,'MB':1,'MiB':1,'GB':1024,'GiB':1024}.get(unit,1))
    return statistics.median(cpu) if cpu else 0,max(memory,default=0)


def main():
    rows=read_runs(); dynamic=[r for r in rows if r['controller_profile']=='dynamic']
    lines=['# Measured dynamic navigation results','',
        'These results come from real Gazebo physics, rendered sensors, and Nav2 actions on the local Windows/Docker host. No trajectory was prescribed to the robot. Each final trial starts in a fresh container. Head-on/alongside variants change actor offset; crossing/cart variants change start delay. Full missions repeat the same independent traffic routes. MPPI sampling is stochastic. This is a small simulation benchmark, not a real-world safety validation.','',
        'Final evidence is in [`dynamic_test_runs/validation`](dynamic_test_runs/validation). Every directory contains action results, full trajectories, contacts, LiDAR/costmap observations, effective parameters, source hashes and resource samples. `success` requires goal success, no non-floor robot contact or sampled footprint overlap, at least 0.08 m physical clearance, moving actors observed by LiDAR, local costmap marking, and the applicable passing/wait behavior. Individual acceptance checks are saved.','',
        'The [sealed benchmark source bundle](dynamic_test_runs/validation/benchmark_sources.zip) preserves all recorded source bytes and line endings across Git/platform normalization. The recorded launch hash also matches [`benchmark_launch_snapshot.py`](dynamic_test_runs/validation/benchmark_launch_snapshot.py). After benchmarking, the launch received GUI-only changes: an explicit Gazebo window size and RViz zoom to fit the map. Controller parameters, navigation code, traffic and geometry remain the benchmarked versions. The final GUI is checked separately.','',
        '| Scenario | Successful / trials | Mean goal time (sim s) | Min actor clearance (m) | Max nearby stopped time (s) | Contacts | Max recoveries |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for scenario in ('head_on','crossing','alongside','cart','full'):
        selected=[r for r in dynamic if r['scenario']==scenario]
        if not selected: continue
        lines.append(f'| {scenario} | {sum(r["success"] for r in selected)}/{len(selected)} | {statistics.mean(duration(r) for r in selected):.2f} | {min(r["minimum_agent_footprint_clearance_m"] for r in selected):.3f} | {max(r["near_agent_stopped_sim_s"] for r in selected):.2f} | {sum(r["contact_events"] for r in selected)} | {max(r["recovery_count"] for r in selected)} |')
    head=[r for r in dynamic if r['scenario']=='head_on']
    lines+=['',f'Continuous head-on passing classification: **{sum(r["continuous_passing_evidence"] for r in head)}/{len(head)}**. Classification requires a change in longitudinal order, lateral motion, measured progress near the actor, no long nearby stop, and no contact/overlap. The [measurement definitions](dynamic_factory_architecture.md) give thresholds.',
        '',f'Advanced trials meeting the sampled 0.08 m clearance margin: **{sum(r.get("clearance_margin_0_08m_pass",False) for r in dynamic)}/{len(dynamic)}**. All timing, stopped time, and minimum-distance values are measured, including terminal goal settling.','',
        '![Measured head-on trajectories and clearance](../media/human_avoidance_trajectory.png)','',
        '## Sensors, contacts and re-entry','',
        'The isolated [`physical_v1/dynamic_contact_probe_0`](dynamic_test_runs/physical_v1/dynamic_contact_probe_0/result.json) deliberately drives a human into a stationary robot. Gazebo reports the human/camera collision pair and the robot is displaced 1.77 m. This confirms physical interaction and contact transport; it is excluded from navigation success rates. The first static run predates the corrected contact topic and uses complete-run geometric clearance instead.','',
        '| Scenario | LiDAR samples matching actor boundary | Local lethal-mark samples | Old-location clearing observations |',
        '|---|---:|---:|---:|']
    for scenario in ('head_on','crossing','alongside','cart','full'):
        selected=[r for r in dynamic if r['scenario']==scenario]
        counts=[s for r in selected for s in r['sensor_validation'].values()]
        if counts: lines.append(f'| {scenario} | {sum(s["lidar_observed_samples"] for s in counts)} | {sum(s["lethal_costmap_samples"] for s in counts)} | {sum(s["clearing_observations"] for s in counts)} |')
    reentries=sum(i.get('reentered_vacated_agent_footprint') is not None for r in dynamic for i in r['interactions'].values())
    lines+=['',f'The saved trajectories identify **{reentries} actor/run pairs** where the robot later enters a previously occupied actor footprint after the actor has departed. Marking and clearing are measured from ROS costmap messages, never injected from ground truth. An actor outside the rolling grid or behind an occlusion may have zero local marking/clearing observations; consult its individual log.','',
        'Additional final-profile safety scenarios:','',
        '| Scenario | Accepted | Sim time (s) | Min actor clearance (m) | Nearby stopped time (s) | Contacts | Recoveries |',
        '|---|---|---:|---:|---:|---:|---:|']
    for r in dynamic:
        if r['scenario'] in ('blocked','cart_shared'):
            lines.append(f'| {r["scenario"]} | {r["success"]} | {duration(r):.2f} | {r["minimum_agent_footprint_clearance_m"]:.3f} | {r["near_agent_stopped_sim_s"]:.2f} | {r["contact_events"]} | {r["recovery_count"]} |')
    lines+=['','The blocked case uses the 0.95 m dead-end bay, where passing beside a human is geometrically impossible. Its acceptance requires a measured wait followed by progress. `cart_shared` tests an approaching cart in the open central corridor. Recovery feedback is reported even when a goal eventually succeeds.','',
        '## Sensor motion estimates','',
        'The following offline evaluation associates active sensor tracks with the nearest Gazebo actor within 0.9 m. Truth is used only for evaluation. Error columns show the range of per-trial medians; p95 is the largest trial p95. Two-second errors compare displacement, excluding initial center-estimation bias. Unassociated tracks are counted rather than silently treated as accurate.','',
        '| Scenario | Associated / active estimates | Median velocity error range (m/s) | Worst p95 velocity error (m/s) | Median 2 s displacement error range (m) | Worst p95 2 s error (m) |',
        '|---|---:|---:|---:|---:|---:|']
    for scenario in ('head_on','crossing','alongside','cart','full','blocked','cart_shared'):
        evaluations=[]
        for r in dynamic:
            file=r['directory']/'forecast_evaluation.json'
            if r['scenario']==scenario and file.exists(): evaluations.append(json.loads(file.read_text()))
        if not evaluations: continue
        def span(key):
            values=[e[key] for e in evaluations if e.get(key) is not None]
            return f'{min(values):.3f}–{max(values):.3f}' if values else '—'
        def maximum(key): return fmt(max((e[key] for e in evaluations if e.get(key) is not None),default=None))
        lines.append(f'| {scenario} | {sum(e["associated_with_truth_within_0_9m"] for e in evaluations)}/{sum(e["active_track_observations"] for e in evaluations)} | {span("median_velocity_vector_error_m_s")} | {maximum("p95_velocity_vector_error_m_s")} | {span("median_2s_displacement_error_m")} | {maximum("p95_2s_displacement_error_m")} |')
    lines+=['','Reversals, acceleration, occlusion and partial surfaces limit these constant-velocity estimates. The declared 1.0 × 0.7 m rectangular size prior supplies an occluded cart dimension, never its position, velocity or route. It must be changed and revalidated for different equipment.','',
        '## Multi-station mission and YOLO','']
    full=[r for r in dynamic if r['scenario']=='full']
    if full:
        image_caption='The image above is an actual annotated frame from [`dynamic_full_0`](dynamic_test_runs/validation/dynamic_full_0/best_detection.json), source simulation timestamp 13.8 s. The crossing worker is briefly turning at this instant. The table measures moving-human visibility over the complete mission rather than claiming motion from a single still image.'
        singleton=ROOT/'docs/dynamic_test_runs/gui_head_on/dynamic_head_on_0'
        audit_file=singleton/'single_person_validation.json'
        if audit_file.exists():
            audit=json.loads(audit_file.read_text()); motion=audit.get('best_image_actor_motion')
            if audit['success'] and motion and motion['speed_m_s']>.05 and motion['in_camera_frustum']:
                shutil.copy2(singleton/'annotated_person.png',ROOT/'media/dynamic_yolo26_person.png')
                image_caption=f'This actual annotated frame is from the separate [single-human GUI camera trial](dynamic_test_runs/gui_head_on/dynamic_head_on_0/single_person_validation.json), source timestamp {audit["best_image_source_stamp_s"]:.3f} s. The only human is moving at {motion["speed_m_s"]:.3f} m/s at that timestamp. The offline audit finds {audit["distinct_person_frames_while_actor_moving_in_frustum"]} distinct positive moving-human frames spanning {audit["source_time_span_s"]:.2f} s, with changing bounding boxes and actual ROS person arrays. This extra GUI trial is excluded from the 17-run main matrix.'
        lines+=['The mission is Loading → Assembly → Quality inspection → Delivery. Both humans and the cart remain independently active throughout all three navigation goals. Agent travel distances and timestamps in every result/trajectory verify continued motion.','',
            '| Run | Goal statuses | Sim time (s) | ROS person arrays | Person frames with moving human in camera frustum | Best confidence | Median inference (ms) |',
            '|---|---|---:|---:|---:|---:|---:|']
        for r in full:
            lines.append(f'| {r["seed"]} | {", ".join(str(g["status"]) for g in r["goals"])} | {duration(r):.2f} | {r.get("ros_person_detection_messages",0)} | {r.get("person_frames_with_moving_human_in_camera_frustum",0)} | {fmt(r["maximum_person_confidence"])} | {fmt(r["median_inference_ms"])} |')
        lines+=['','Action status 4 means SUCCEEDED. The moving-human count combines actual person detections with ground-truth motion and camera-frustum checks at source-image timestamps; it is not identity tracking or depth estimation. Inference uses CPU. `/perception/detections` carries actual `vision_msgs/Detection2DArray` messages and `/perception/annotated` carries images. YOLO does not command the robot.','',
            '![Actual live factory YOLO output](../media/dynamic_yolo26_person.png)',
            image_caption,'']
    if dynamic:
        cpu,memory=resources(dynamic)
        lines += ['## Runtime','',f'For the headless final matrix, Docker CPU median across sampled startup/navigation intervals is {cpu:.1f}% (100% is one logical core). Peak container memory: {memory:.0f} MiB. Navigation action real-time factor ranges from {min(rtf(r) for r in dynamic):.3f} to {max(rtf(r) for r in dynamic):.3f}. Software rendering is enabled; CUDA inference was not used. Raw resource samples are retained rather than presenting these small runs as hardware performance guarantees.','']
    gui_rows=[]
    for relative in ('gui_head_on/dynamic_head_on_0','gui_live_full','gui_layout_final'):
        file=ROOT/'docs/dynamic_test_runs'/relative/'result.json'
        if not file.exists(): continue
        r=json.loads(file.read_text())
        gui_rows.append(f'| [{relative}](dynamic_test_runs/{relative}/result.json) | {r["success"]} | {duration(r):.2f} | {r["minimum_agent_footprint_clearance_m"]:.3f} | {r["contact_events"]} | {r["recovery_count"]} | {fmt(r["median_inference_ms"])} |')
    if gui_rows:
        lines+=['## Additional GUI checks','',
            '| Trial | Accepted | Sim time (s) | Min actor clearance (m) | Contacts | Recoveries | Median inference (ms) |',
            '|---|---|---:|---:|---:|---:|---:|',*gui_rows,'',
            'These additional trials are excluded from the 17-run headless matrix and its resource summary. `gui_live_full` enables MPPI candidate visualization; `gui_layout_final` verifies the corrected default layout with debug visualization disabled. Software GUI rendering increases inference latency. [Recorded topic connections](dynamic_gui_topics.txt) confirm the MarkerArray publisher and RViz subscriber, plus the annotated camera subscription. [Source verification](dynamic_source_verification.json) checks all 20 benchmark source sets against the preserved launch and current navigation/traffic code, and confirms identical Nav2 parameters for the final GUI and full benchmark.','']
    lines+=['## Preserved baseline and development failures','',
        '- Original factory regression: SUCCEEDED, 25.903 s; original unexpected-obstacle detour: SUCCEEDED, 45.203 s. Their evidence is retained as `regression_factory.json` and `regression_dynamic.json`.',
        '- The complex static mission reached all six destinations after the Loading start: 70.540 m travel, 246.753 sim s across goals, zero recoveries, 0.238 m minimum sampled static footprint clearance.',
        '- An initial local-layer configuration omitted the per-scan maximum obstacle height, leaving the local grid empty. The explicit 2 m height filter fixes it. Earlier `physical_v1` and `comparison_v2` dynamic runs are development evidence, not final local-layer validation.',
        '- The first crossing pilot contacted a wheel and later aborted. Its full evidence remains in `scenario_pilots/dynamic_crossing_0`. The path-angle critic previously looked only four path points ahead; increasing that to 20 points and its weight to 4 removed the observed full-turn trap in the successful retests. Repeated final outcomes are listed above.',
        '- Repeated tuning exposed smaller margins and contacts in some variants. Increasing padding alone did not solve it; spatially projected future occupancy and a rotation shim also failed stress cases. These development records remain under `margin_v2`, `margin_v4`, `horizon_v6`, `forecast_v8`, `shim_v9`, and later prototype directories. The [tuning history](dynamic_tuning_history.md) records the sequence.',
        '- A repeated cart trial exposed unstable rectangle yaw and false lateral velocity from adjacent-ray noise. The whole-cluster boundary-residual orientation fit fixes that observed failure; a noisy-corner unit test and a fresh complete matrix accompany it. The earlier matrix remains in `pre_robust_box_v15`.',
        '- The final design keeps scan-only costmaps and loads a native time-indexed MPPI critic. It scores simultaneous robot/obstacle positions, adds a terminal continued-motion risk cost using relative endpoint velocity, and discourages excessive angular travel outside the final metre of the goal. The final table includes every completed repeated outcome, including any failure.',
        '- The headless upstream xacro startup race and contact-topic placement were diagnosed and corrected. See the implementation checklist and retained diagnostics.','',
        '## Demo and limits','',
        'The map, trajectory plot and camera frames accompany the measurements. The [105-second recording plan](demo_script.md) uses labeled clips from separate experiments. The [command reference](dynamic_commands.md) covers launch, optional perception, actor activation, missions, tests and cleanup.','',
        'Limitations: standing meshes translate without a walking gait; traffic uses upright rigid proxies and predetermined routes; no human intent prediction, semantic navigation control, tracking identity, or validated CUDA inference is implemented. Safety measurements sample conservative planar geometry at 10 Hz plus physics contacts. The cart trials include a tight 0.087 m margin and a 7 s nearby wait; continuous zero-stop behavior is demonstrated specifically by the head-on trials. Other layouts, equipment dimensions, higher speeds and real robots require further validation.','']
    regression_lines=[]
    for scenario in ('factory','dynamic'):
        file=ROOT/f'docs/dynamic_test_runs/regression_{scenario}_final.json'
        if file.exists():
            regression=json.loads(file.read_text())
            regression_lines.append(f'| Original {scenario} | {regression["success"]} | {regression["completion_wall_time_s"]:.3f} | Nav2 action status {regression["action_status"]} |')
    file=ROOT/'docs/dynamic_test_runs/regression_perception_final.json'
    if file.exists():
        regression=json.loads(file.read_text())
        regression_lines.append(f'| Original YOLO camera | {regression["success"]} | {regression["observation_wall_time_s"]:.2f} | {regression["distinct_person_frames"]} person frames; {regression["matched_image_detection_annotation_stamps"]} matching stamps; calibrated CameraInfo and optical TF |')
    if regression_lines:
        index=lines.index('## Demo and limits')
        lines[index:index]=['## Post-change original-mode regressions','',
            '| Check | Passed | Wall time (s) | Evidence |','|---|---|---:|---|',*regression_lines,'',
            'Raw results are the `regression_*_final.json` files in `dynamic_test_runs/`. These reruns use the original world and launch after the advanced implementation changes.','']
    if (ROOT/'media/dynamic_factory_gui.jpg').exists():
        index=lines.index('## Demo and limits')+2
        lines[index:index]=['![Actual Gazebo and RViz desktop](../media/dynamic_factory_gui.jpg)','']
    (ROOT/'docs/dynamic_navigation_results.md').write_text('\n'.join(lines),encoding='utf-8')
    comparison=['# Original MPPI settings versus advanced MPPI tuning','',
        'The preserved factory already used `nav2_mppi_controller::MPPIController`. This compares its original MPPI settings with the advanced profile on the same expanded world, speed limits, physical footprint, 10 Hz LiDAR, traffic routes and starting poses. Shared scenario overrides also use a 0.35 m / 20 s progress checker and full costmap publication. The original factory launch remains unchanged. It is an integrated profile comparison: costmap size/rates, MPPI tuning and the native temporal critic change. The LiDAR tracking node runs in both profiles for observation; only the advanced profile loads its native critic. This does not isolate one parameter or compare DWB against MPPI.','',
        '| Profile | Acceptance / trials | Goals succeeded | Continuous passes | Mean sim time (s) | Mean path (m) | Min human clearance (m) | Contacts |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for profile in ('baseline','dynamic'):
        selected=[r for r in rows if r['scenario']=='head_on' and r['controller_profile']==profile]
        if not selected: continue
        comparison.append(f'| {profile} | {sum(r["success"] for r in selected)}/{len(selected)} | {sum(r["navigation_success"] for r in selected)} | {sum(r["continuous_passing_evidence"] for r in selected)} | {statistics.mean(duration(r) for r in selected):.2f} | {mean(selected,"path_length_m"):.3f} | {min(r["minimum_agent_footprint_clearance_m"] for r in selected):.3f} | {sum(r["contact_events"] for r in selected)} |')
    comparison+=['',
        '| Profile | Mean total stopped (s) | Mean slowing (s) | Max nearby stopped (s) | Max recoveries | Mean yaw-command sign changes | Median CPU | Mean RTF |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for profile in ('baseline','dynamic'):
        selected=[r for r in rows if r['scenario']=='head_on' and r['controller_profile']==profile]
        if not selected: continue
        cpu,_=resources(selected)
        comparison.append(f'| {profile} | {mean(selected,"stopped_sim_s"):.2f} | {mean(selected,"slowing_sim_s"):.2f} | {max(r["near_agent_stopped_sim_s"] for r in selected):.2f} | {max(r["recovery_count"] for r in selected)} | {mean(selected,"angular_command_sign_changes_over_0_25rad_s"):.1f} | {cpu:.1f}% | {statistics.mean(rtf(r) for r in selected):.3f} |')
    comparison+=['','Slowing uses measured speed 0.02–0.24 m/s; stopped uses <0.02 m/s. Yaw-command sign changes count only commands with magnitude above 0.25 rad/s. They include normal steering reversals and are a repeatable motion proxy, not a count of diagnosed oscillation failures. CPU samples include startup and navigation; 100% is one logical core.','']
    baseline=[r for r in rows if r['scenario']=='head_on' and r['controller_profile']=='baseline']
    if baseline and head:
        baseline_cpu,_=resources(baseline); dynamic_cpu,_=resources(head)
        comparison+=['',f'The selected advanced profile has worst head-on clearance {min(r["minimum_agent_footprint_clearance_m"] for r in head):.3f} m, versus {min(r["minimum_agent_footprint_clearance_m"] for r in baseline):.3f} m for the original settings. Mean goal time is {statistics.mean(duration(r) for r in head):.2f} versus {statistics.mean(duration(r) for r in baseline):.2f} sim s; sampled median CPU is {dynamic_cpu:.1f}% versus {baseline_cpu:.1f}%. Selection weighs the measured clearance and acceptance outcomes together with this time/compute tradeoff.','']
    comparison+=['','The advanced profile is the default for dynamic mode; the original factory and advanced `-ControllerProfile baseline` remain selectable. Inspect the clearance and contact outcomes together with completion time. A completed goal with a very small positive gap is weaker evidence than a larger margin. Three variants are a modest test, not a statistical guarantee.','',
        '| Parameter | Original settings | Advanced |','|---|---|---|',
        '| Controller / model | MPPI / DiffDrive | MPPI / DiffDrive |',
        '| Control frequency, step | 20 Hz, 0.05 s | 20 Hz, 0.05 s |',
        '| Horizon, batch | 56 steps (2.8 s), 2000 | 96 steps (4.8 s), 2000 |',
        '| Angular sampling deviation | 0.4 rad/s | 0.8 rad/s |',
        '| Forward/reverse/yaw limits | 0.35 / −0.15 m/s, 1.2 rad/s | Same |',
        '| Linear / angular acceleration | 3 / 3.5 | 1.0 forward, 1.2 braking / 2.0 |',
        '| Cost critic footprint | false, stride 2 | true, stride 1 |',
        '| Path alignment weight | 14 | 5 |',
        '| Path angle weight / lookahead | 2 / 4 points | 4 / 20 points |',
        '| Local costmap | 3 × 3 m, voxel, 5 Hz | 6 × 6 m, obstacle layer, 10 Hz |',
        '| Inflation radius | 0.7 m | 0.65 m |',
        '| Costmap footprint padding | 0.01 m | 0.05 m |',
        '| Time-indexed obstacle critic | None | LiDAR-derived velocity and extent, same-time trajectory costs |',
        '| Terminal continued-motion risk | None | Soft endpoint risk cost, additional 8 s linear-motion heuristic |','',
        'The wider rolling grid gives the 4.8 s candidate horizon usable observed space. Footprint-aware cost evaluation rejects rectangular-body conflicts. Lower path alignment weight permits lateral departures. The native critic compares candidate robot positions with obstacle estimates at matching future times; it does not fill costmaps with future points. Its terminal cost uses relative endpoint velocity to discourage stopping on an approaching obstacle\'s continued-motion line while allowing safe continued motion through a crossing. An angular-travel cost discourages full turns and is relaxed within 1 m of the goal for final corrections. The tested profile uses MPPI directly, without the experimental rotation shim. Inflation and Collision Monitor remain enabled. The scan source explicitly allows heights 0–2 m, marks returns and clears with raytracing. Per-run `effective_nav2.yaml` records all inherited and overridden parameters.','',
        'The native plugin builds against the installed Jazzy headers. Its CTest checks same-time collision risk, a collision-free lateral candidate, rectangle distances, crossing endpoint motion, and the finite-horizon stopped-endpoint trap. Eight Python geometry tests check physical clearance, scan-circle fitting, declared box-size priors and noisy-corner orientation. Real simulation trials provide the navigation evidence. Constant-velocity estimates are limited by partial visibility, acceleration and reversals; the terminal lookahead is a conservative risk heuristic, not validated intent prediction.','',
        'References: [Nav2 Jazzy MPPI configuration](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/), [Collision Monitor](https://docs.nav2.org/jazzy/tutorials/general_tutorials/using_collision_monitor/using_collision_monitor/), [Gazebo Harmonic VelocityControl implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/velocity_control/VelocityControl.cc).','']
    (ROOT/'docs/controller_comparison.md').write_text('\n'.join(comparison),encoding='utf-8')
    print(f'Reported {len(rows)} final-run records.')


if __name__=='__main__': main()
