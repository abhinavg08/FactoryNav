# Four-stage presentation script

Run the stages in order. These are instructions for you to execute; this guide
does not start or change the running simulation by itself.

| Stage | Scene | LiDAR | Nav2 / MPPI | YOLO |
|---|---|---|---|---|
| 1. Simple static | Original factory, fixed obstacles | Yes | Yes | Off |
| 2. Simple unexpected blockage | Original factory, a new stationary obstacle appears during navigation | Yes | Yes | Off |
| 3. Complex static | Larger factory, fixed equipment, no moving traffic | Yes | Yes, advanced profile | Off |
| 4. Complex dynamic | Larger factory, two moving humans and a moving cart | Yes | Yes, advanced profile | On |

In stage 2, “dynamic” refers to the environment changing when an obstacle is
inserted. The inserted box remains stationary. Stage 4 demonstrates continuously
moving traffic. Both maps already use MPPI; the advanced profile adds tuning
and a native time-indexed obstacle critic driven by LiDAR motion estimates.

## Prepare once

Complete [setup](setup.md) first. Keep Docker Desktop running. From the parent of your cloned repository, open PowerShell and enter:

```powershell
cd factorynav
```

Open the existing preview:
http://localhost:6080/vnc.html?autoconnect=1&resize=scale

Each command below recreates this project's simulation and sends goals
automatically. The preview disconnects during replacement: refresh/reconnect
when the new desktop is ready. Do not send manual Nav2 goals during a test.
Start recording before launching each stage and keep its original footage.

Gazebo is the physical simulation view on the left. RViz on the right shows
localization, sensor returns, costmaps, paths and navigation feedback. Show
actual goal completion using `Feedback: reached` or the successful result.

Opening narration:

> “This is FactoryNav, a TurtleBot3 factory navigation demonstration using ROS 2, Gazebo, Nav2, MPPI and LiDAR.”

## 1. Simple factory with static obstacles

```powershell
.\scripts\run_tests.ps1 -Scenario factory -Repeat 1
```

This is the original factory. It starts at Loading `(-4.5, 0)` and sends one
goal to Workstation B `(4.5, 0)`. The open central aisle permits a nearly
straight route. Workstation A is not part of this run.

Say:

> “The robot uses a pre-built map and LiDAR to reach Workstation B. Nav2 plans the route, while MPPI controls forward speed and turning. The clear central aisle produces a nearly straight path.”

Highlight:

- Green Loading pad and blue Workstation B pad.
- Estimated robot pose, LiDAR returns and the planned route in RViz.
- Actual robot motion in Gazebo and successful arrival.

The coloured floor pads mark station locations. Physical racks, workbenches
and pallets are obstacles. This run demonstrates localization, path following
and goal completion; it does not demonstrate a multi-station mission.

Visual reference: `../media/normal_factory_map.png`.

## 2. Simple factory with an unexpected blockage

```powershell
.\scripts\run_tests.ps1 -Scenario dynamic -Repeat 1
```

This still uses the original factory. The same mission starts, then a red
collision-enabled box is inserted after approximately 1 metre of observed
robot travel. The box remains stationary after insertion.

Say:

> “Now an unexpected blockage appears after navigation begins. LiDAR detects the occupied space, and the costmap updates. Nav2 replans around the racks, and the robot continues toward the same destination.”

Highlight:

- The new red box appearing in Gazebo.
- New occupied cells in the costmap and the changed planned route.
- The longer detour around the rack ends and arrival at the original goal.

LiDAR and Nav2 handle this test. YOLO is disabled. Describe this as an
unexpected-obstacle or replanning test, rather than a moving-person test.

## 3. Complex factory with static obstacles

```powershell
.\.conda\python.exe scripts/test_dynamic_scenarios.py --scenarios static --repeat 1 --gui --keep --output ("docs/dynamic_test_runs/demo_complex_static_" + (Get-Date -Format yyyyMMdd_HHmmss_fff))
```

No humans or cart are spawned. The robot follows station goals in this order:

Loading → Assembly → CNC → Quality → Storage → Packing → Delivery.

Say:

> “The larger factory introduces seven station locations, intersections and alternative corridors. With traffic disabled, the mission script sends station goals in sequence. Nav2 and MPPI navigate between them around the fixed equipment.”

Highlight:

- The larger layout and alternative routes around the racks.
- Multiple turns and arrivals at different station locations.
- Completion of all six navigation goals after the Loading start.

This is navigation between station goal points. Physical cargo pickup or
manipulation is not implemented. YOLO is disabled for this stage.

Visual reference: `../media/dynamic_factory_map.png`.

## 4. Complex factory with moving traffic and YOLO

```powershell
.\.conda\python.exe scripts/test_dynamic_scenarios.py --scenarios full --repeat 1 --gui --perception --keep --output ("docs/dynamic_test_runs/demo_complex_dynamic_" + (Get-Date -Format yyyyMMdd_HHmmss_fff))
```

Both humans and the cart move independently during the mission:

Loading → Assembly → Quality → Delivery.

Say:

> “Now two humans and an industrial cart move independently through the factory. LiDAR observes their changing positions. The advanced controller uses short-term motion estimates to evaluate candidate robot trajectories and adjust speed or direction. YOLO separately labels people visible to the camera.”

Highlight:

- All three traffic agents moving in Gazebo while the robot navigates.
- Changing LiDAR returns and costmap occupancy as traffic moves.
- Actual path or speed adjustments where an interaction occurs.
- Person boxes and confidence in the annotated camera stream when a human is visible.
- Three completed station goals and the saved result.

The RGB camera faces forward, so person detections appear only when a person
is in view and detected. LiDAR provides surrounding geometry. YOLO detections
do not feed the navigation controller. MPPI predicts candidate robot motion;
the custom critic uses limited constant-velocity obstacle estimates, without
human intent prediction. Slowing or waiting is an acceptable fallback when
space is unsafe; describe the behaviour actually visible in the run.

The test owns its traffic controllers and stops them when the mission ends.
`--keep` leaves the simulation open for inspection after completion.

Closing narration:

> “Each test saves its actual navigation result. The advanced runs also record trajectories, contacts, clearance and stopped time, so I can evaluate the behaviour beyond simply reaching a goal.”

## Timing and presentation

The complete live runs take several minutes. For a 110-second video, use
clearly labelled excerpts:

| Video time | Content |
|---|---|
| 0:00–0:10 | Introduction and factory overview |
| 0:10–0:25 | Simple static run |
| 0:25–0:45 | Simple map: unexpected blockage and detour |
| 0:45–1:10 | Complex static mission excerpts |
| 1:10–1:40 | Complex dynamic mission and live YOLO |
| 1:40–1:50 | Actual results and closing statement |

Label separate experiments, cuts and any playback speed changes. Keep normal
speed around obstacle interactions. Show YOLO during stage 4 only when an
actual detection is present. A dedicated `head_on` experiment is available
in `demo_script.md` if you want an additional focused passing clip; the full
mission does not guarantee that exact encounter.

Each advanced command generates its own timestamp, including milliseconds,
so it also works in a new PowerShell window from the root of your cloned `factorynav` directory.
Repeating a command preserves previous evidence in its original directory.
The advanced runner refuses to overwrite an existing evidence directory.
To stop this project's simulation afterward:

```powershell
docker compose down
```
