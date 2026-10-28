# Test results

Date: 2026-10-28, Asia/Kolkata.

| Check | Status | Actual evidence |
| --- | --- | --- |
| Conda creation | PASS | `D:\cele\.conda`, Python 3.12.16 |
| PowerShell activation | PASS | `sys.executable` resolves to project `.conda\python.exe` |
| NumPy/PyYAML imports and operations | PASS | NumPy 1.26.4, PyYAML 6.0.3 |
| Python dependency consistency | PASS | `pip check`: no broken requirements |
| Windows GPU | PASS | RTX 4060, 8188 MiB, driver 560.94 |
| WSL GPU | PASS | `nvidia-smi` succeeds inside Ubuntu |
| Ubuntu 24.04 container | PASS | Official ROS image reports Ubuntu 24.04.5, Python 3.12.3 |
| Container GPU visibility | PASS | NVIDIA runtime exposes GPU and `/dev/dxg` |
| WSLg mount from Docker | FAIL | Ubuntu distro mount service socket unavailable |
| ROS/Nav2/Gazebo installation | PASS | Jazzy, Nav2 1.3.13, Gazebo Sim 8.15.0 |
| Gazebo + RViz GUI | PASS | Both visually inspected through noVNC |
| Real sensor topics and TF | PASS | /scan, /odom, /clock, map→base_link, odom→base_link |
| Baseline navigation goal | PASS | 12.902 s, NavigateToPose status 4, error code 0 |
| Factory world and correct map | PASS | Generated from the same collision geometry; mission reached goal |
| Unexpected obstacle/alternate route | PASS ×2 | Spawn after ~1 m travel, costmap marking, detour beyond rack ends, goal success |
| YOLO disabled | PASS | All four successful core goal tests ran without YOLO |
| ROS package build | PASS | Both packages built with colcon in 9.44 s |
| Live camera + YOLO26 | PASS | 28 person frames, valid CameraInfo/optical TF, 20 matching annotation/detection/raw stamps |
| Navigation with YOLO running | PASS | Person present, 29.752 s, 9.336 m, action SUCCEEDED |
| Semantic safety control | NOT IMPLEMENTED | Optional feature |

GPU visibility is not a CUDA tensor computation test, nor evidence of
hardware-accelerated Gazebo rendering. No navigation measurements or perception
results have been claimed without execution.

## Recorded navigation runs

| File in `test_runs/` | Outcome | Wall time | Odometry path | Minimum observed LiDAR range |
| --- | --- | ---: | ---: | ---: |
| `baseline_goal.json` | Failed precheck before sending a goal; lifecycle startup race | — | — | — |
| `baseline_goal_retry.json` | PASS | 12.902 s | 3.474 m | 0.363 m |
| `factory_baseline_goal.json` | PASS | 25.953 s | 8.928 m | 0.918 m |
| `factory_dynamic_goal_1.json` | PASS | 44.551 s | 14.537 m | 0.806 m |
| `factory_dynamic_goal_2.json` | PASS | 45.351 s | 14.625 m | 0.811 m |
| `factory_launcher_retest.json` | PASS | 25.952 s | 8.922 m | 0.932 m |
| `factory_with_perception_goal.json` | PASS | 29.752 s | 9.336 m | 0.545 m |

The two dynamic runs spawned a 0.7×2.2×1.0 m collision box after 1.011 m and
1.005 m of observed motion. Both detected lethal costmap cells in a region
free in the static map. Global plans reached |y|=4.0 m; observed odometry reached
|y|=4.041 m and 4.033 m, beyond the rack ends at |y|=3.2 m. Both goals returned
status SUCCEEDED and Nav2 error code 0. This establishes an actual alternative
route, rather than only a pause in front of the blockage.

The baseline precheck was corrected to wait for all required lifecycle nodes
to become active. The original failure is retained. All six listed navigation
goals completed successfully; the earlier precheck submitted no goal. The final
run included a textured person and active YOLO inference, and ended at yaw π.

Distances are odometry estimates. LiDAR range is sensor-to-return distance,
not footprint-edge clearance. The virtual desktop is Mesa llvmpipe with OpenGL
4.5; GPU rendering and CUDA inference are not established by these results.

## Live perception

`test_runs/perception_verification.json` records a 15-second live observation:
50 raw images, 28 detection messages with a person in distinct source frames,
and 20 exact timestamp matches across raw image, annotation, and detection.
CameraInfo intrinsics were valid at 640×480 and the base-to-optical transform
was available. The observed person score was 0.8538. Median measured inference
was 47.25 ms on CPU; 28 messages in 15 seconds is approximately 1.87 output Hz.
The detector's startup-inclusive internal rate was 1.569 Hz at that snapshot.

The cold first inference was discarded as stale, as recorded in `log/yolo.log`.
Camera-disconnect behavior has not been separately exercised. There is no
semantic safety controller, and no camera–LiDAR metric association claim.

The current detector's real publisher/subscriber graph is recorded in
`test_runs/yolo_node_info.txt`. It subscribes to camera/clock and publishes
detections, annotations, metrics, and standard ROS metadata; it does not publish
velocity. Navigation with perception running and the person present succeeded.
All prior baseline/dynamic tests ran without YOLO.

`media/yolo26_gazebo_person.png` is an actual model-generated annotation;
`media/gazebo_camera_person.png` is its corresponding raw simulated camera image.
The model is `yolo26n.pt`, downloaded from the official Ultralytics v8.4.0 asset
release. Its source and SHA256 are in `setup/yolo26_weights.lock.json`.
