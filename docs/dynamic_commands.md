# Advanced mode commands

Use PowerShell in the root of your cloned `factorynav` directory. The local `.conda` is for host tooling; ROS runs
inside the Docker image. The original README documents base/perception image
builds and the attributed model/weight caches. On a new checkout, populate the
person cache with `.\.conda\python.exe setup/fetch_person.py` before advanced
traffic runs. Use the existing perception setup for YOLO weights and wheels.

## Start, stop and choose features

```powershell
# Full desktop and independently moving traffic, with optional YOLO
.\scripts\run_demo.ps1 -DynamicFactory -Perception
# Same navigation without YOLO or camera processing
.\scripts\run_demo.ps1 -DynamicFactory
# Static advanced world, original MPPI settings, no GUI
.\scripts\run_demo.ps1 -DynamicFactory -Headless -Traffic none -ControllerProfile baseline
docker compose logs -f simulation
docker compose down
docker compose ps
```

Launchers use one Compose service and replace the previous configuration when
needed. Automated experiments always force recreation. A benchmark's `--keep`
option leaves that single world running for inspection. Do not run a second
benchmark or traffic controller against the same live actors.
An unchanged `run_demo.ps1` invocation starts/reuses the existing world. To
restart at Loading, use `docker compose down` and launch again, or use the
fresh-container test runner.

## Activate selected actors manually

Start with `-DynamicFactory -Traffic none`, then run the desired controller(s):

```powershell
# Two humans, no cart
docker compose exec -d simulation bash -lc 'source /opt/ros/jazzy/setup.bash; export PYTHONPATH=/workspace/factorynav_dynamic:$PYTHONPATH; python3 -m factorynav_dynamic.traffic --scenario full --agents human_a human_b'
# Industrial cart in the same full-world traffic configuration
docker compose exec -d simulation bash -lc 'source /opt/ros/jazzy/setup.bash; export PYTHONPATH=/workspace/factorynav_dynamic:$PYTHONPATH; python3 -m factorynav_dynamic.traffic --scenario full --agents cart'
```

The `configs` function in `factorynav_dynamic/traffic.py` centralizes speeds,
delays, loop flags, routes and cart dimensions. The head-on/crossing/alongside
benchmark seeds vary lateral position or start delay; MPPI itself remains
stochastic. Arbitrary new dimensions/routes require rerunning geometry and
navigation checks. The recorder reads cart dimensions from the same scenario
configuration as the physical model.
The LiDAR rectangle model separately declares `box_long_side` and
`box_short_side` (defaults 1.0 and 0.7 m). Match these priors to a changed cart
and rerun validation; they provide dimensions only, never actor poses or routes.

## Mission and fresh-world tests

```powershell
# In a GUI launched with its default full traffic
docker compose exec -T simulation bash -lc 'source /opt/ros/jazzy/setup.bash; export PYTHONPATH=/workspace/factorynav_dynamic:$PYTHONPATH; python3 -m factorynav_dynamic.benchmark --scenario full --external-traffic --output /workspace/log/dynamic/live_mission'
# Each default scenario, three attempts, YOLO disabled
.\.conda\python.exe scripts/test_dynamic_scenarios.py --output docs/dynamic_test_runs/my_suite
# One named scenario, with optional GUI/perception
.\.conda\python.exe scripts/test_dynamic_scenarios.py --scenarios head_on --repeat 1 --gui --perception --keep --output docs/dynamic_test_runs/my_head_on
# After that single-human camera trial completes, audit actual moving frames
.\.conda\python.exe scripts/verify_single_person_run.py docs/dynamic_test_runs/my_head_on/dynamic_head_on_0
# Original settings versus advanced tuning, matching traffic variants
.\.conda\python.exe scripts/test_dynamic_scenarios.py --scenarios head_on --profiles baseline dynamic --repeat 3 --output docs/dynamic_test_runs/my_comparison
```

Default scenarios: `head_on crossing alongside cart full`. Extra scenarios:
`static cart_shared blocked contact_probe`. The blocked case exercises the
0.95 m bay. The deliberate `contact_probe` calibrates sensors and must be
reported separately from navigation safety tests. Output directories are never
overwritten. Per-run `execution.log`, `simulation.log`, `result.json`, and
`progress.json` explain outcomes, including failures. `trajectory.jsonl` is
written on completion; `progress.json` updates during the run.

`--final-suite` runs three trials of each main scenario, enables YOLO for its
full missions, adds the two extra safety scenarios, and runs three original
profile head-on comparisons. The stricter 0.08 m physical-clearance check may
fail the original profile even when its Nav2 action succeeds; the result file
separates those outcomes. The CLI returns nonzero if any acceptance check fails.
Use `--resume` with the same output directory to retain completed trials only
when all recorded source hashes still match. Interrupted directories must be
archived first; completed evidence is never overwritten.

## Rebuild geometry, package and summaries

```powershell
docker compose exec -T simulation bash -lc 'source /opt/ros/jazzy/setup.bash; python3 /workspace/scripts/generate_dynamic_factory.py'
docker compose exec -T simulation bash -lc 'source /opt/ros/jazzy/setup.bash; colcon build --base-paths factorynav_dynamic --packages-select factorynav_dynamic --symlink-install'
docker compose exec -T simulation python3 /workspace/scripts/export_navigation_metrics.py --root /workspace/docs/dynamic_test_runs/validation --output /workspace/docs
$env:PYTHONPATH="$PWD\factorynav_dynamic"
.\.conda\python.exe -m unittest discover -s factorynav_dynamic/test -v
```

The benchmark can run from the source workspace as above. After a colcon build,
source `install/setup.bash` and use `ros2 run factorynav_dynamic benchmark` or
`ros2 run factorynav_dynamic traffic`. The supplied launch is intended for this
mounted `/workspace` layout, with the matching generated world and assets.

The advanced launcher automatically builds `factorynav_mppi_critics` in Release
mode if its native library is missing or older than its source, then sources
the workspace overlay. A fresh build takes about 20 seconds on the tested host.
Run its timing/terminal-risk tests with:

```powershell
docker compose exec -T simulation ctest --test-dir /workspace/build/factorynav_mppi_critics --output-on-failure
.\.conda\python.exe scripts/evaluate_forecasts.py --root docs/dynamic_test_runs/validation
.\.conda\python.exe scripts/write_dynamic_report.py
```

Original regressions remain `./scripts/run_tests.ps1 -Scenario factory` and
`-Scenario dynamic`. That legacy `dynamic` test inserts one unexpected static
blockage; the new named scenarios exercise continuously moving obstacles.
