# Troubleshooting

| Symptom | Investigation and remedy |
| --- | --- |
| Conda is not on PATH | Use a Conda PowerShell prompt or pass `-CondaExe` to `setup/create_env.ps1` and `setup/activate.ps1`. Discovery uses the shell, environment, PATH and common installation directories; no profile changes are made. |
| WSL is Ubuntu 26.04 | Use the supplied Ubuntu 24.04 container. Do not mix Noble apt repositories into Ubuntu 26.04 or replace its Python. |
| Docker engine unavailable | Run `docker desktop start`, wait for startup, then check `docker info`. Linux containers are required. |
| WSLg mount fails | The audit found a missing Ubuntu distro-service socket. The local virtual desktop avoids that mount; `log/opengl.txt` records its actual renderer. |
| Desktop disconnects on reset | Reconnect or reload noVNC after the replacement container starts. Inspect `log/novnc.log` and `log/x11vnc.log`. |
| Lifecycle nodes are initially inactive | The checker now waits for sensor data, localization, and active lifecycle states before sending a goal. |
| Goal fails | Inspect result JSON and `log/factory.log`; check `/scan`, `/odom`, `/clock`, TF, lifecycle, and free-space goal coordinates. |
| Duplicate obstacle/person | Spawn scripts reject duplicate names. Recreate this project's container for a clean world. |
| Missing perception environment | Build with `scripts/run_demo.ps1 -Perception -Build`; inspect `log/yolo.log`. The Windows Conda environment cannot load Linux ROS extensions. |
| No person detection | Inspect `log/camera_probe.png`, CameraInfo, optical TF, weights, and actual detection topics. A successful spawn alone is not recognition evidence. |
| CLI graph looks stale | Use `--no-daemon --spin-time 5` for node queries so discovery has time to complete. Direct action/sensor checks do not use the CLI daemon. |
| Slow or inconsistent wheel index | The optional-image helper downloads pinned wheels from official PyPI, verifies hashes, and installs the resulting local cache. |

The initial lifecycle precheck failure is preserved in
[`test_runs/baseline_goal.json`](test_runs/baseline_goal.json); it occurred before any navigation goal was
submitted. The corrected checker and successful retry are recorded separately.
Timeouts request goal cancellation. Use fresh project containers for repeatable
starting coordinates and avoid Gazebo's in-place time reset during tests.

## Advanced-mode diagnostics

| Symptom | Verified cause / remedy |
|---|---|
| Headless Gazebo fails while opening a temporary xacro world | The upstream launch raced temporary world creation. The advanced launch starts Gazebo directly on its generated SDF, then reuses the robot/Nav2 bringup. |
| Humans appear in LiDAR but not the local costmap | An omitted per-source maximum obstacle height rejected all returns. The advanced scan source explicitly allows 0–2 m. Verify actual marking in `sensor_observations.jsonl`. |
| Contact topic exists but no robot/actor collision is reported | Gazebo's contact topic must be configured inside `<contact>`. The isolated contact probe validates actual transport and physical robot displacement. |
| Close cart fit suddenly rotates and reports lateral motion | Adjacent-ray direction averaging was noisy. The selected fitter uses whole-cluster boundary residuals; keep the box-size priors matched to the equipment. Earlier failed trials are retained. |
| A scenario directory already exists | The runner intentionally refuses to overwrite evidence. Use a new name or `--resume` for completed runs with matching source hashes. Archive an interrupted run before rerunning it. |
| noVNC unavailable during tests | Headless trials run the sensor renderer without the desktop/websocket server. Start the advanced GUI launcher after the test suite finishes. |

Logs for advanced mode are under `log/dynamic/`; completed experiments copy
their own logs and effective parameters into `docs/dynamic_test_runs/`.
