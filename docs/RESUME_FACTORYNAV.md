# Historical implementation checkpoint

This file records the pre-publication implementation state. Use the current
[README](../README.md) and [setup guide](setup.md) for reproducible commands;
runtime descriptions below are historical observations, not current process status.

Implementation and validation are complete on `feature/dynamic-factory-navigation`.
The original baseline remains on `main` at `c342a41`. Host tooling uses the local
`D:\cele\.conda`; ROS runs in the preserved Ubuntu 24.04 Docker environment.
Use `git log -1` for the local implementation checkpoint. No GitHub push was made.

## Verified outcomes

- 17/17 advanced headless trials accepted: five principal scenarios repeated
  three times, plus blocked-bay and shared-cart checks. Zero contacts.
- 3/3 head-on passes with zero nearby stopped time; minimum actor gap 0.227 m.
- 3/3 full missions completed all three goals with two humans, cart and live YOLO.
- Original-profile comparison: all three goals reached, 1/3 strict acceptance.
  One conservative footprint overlap and one sub-0.08 m gap were rejected.
- Original factory, obstacle-detour and camera regressions passed after changes.
- Additional GUI head-on and two GUI full missions passed without contact.
  Final layout mission: 191.127 sim s, 0.276 m minimum actor gap, zero recoveries.
- Single-human camera audit: 32 moving-person frames spanning 17.8 s; changing
  boxes, actual ROS detections, and motion at the saved image timestamp.
- Eight Python geometry tests and native temporal-cost CTest passed. All four
  ROS packages built together; installed benchmark/forecast/traffic commands found.

See `dynamic_navigation_results.md`, `controller_comparison.md`, and
`dynamic_implementation_checklist.md`. Raw final matrix: `dynamic_test_runs/validation`.
Development failures are retained separately, including `pre_robust_box_v15`.

## Reproduce and inspect

```powershell
.\scripts\run_demo.ps1 -DynamicFactory -Perception
.\.conda\python.exe scripts/test_dynamic_scenarios.py --final-suite --output docs/dynamic_test_runs/new_reproduction
```

The final GUI is left running with independent traffic at
http://localhost:6080/vnc.html?autoconnect=1&resize=scale . Its last mission reached
Delivery. To reset at Loading, stop/remove this project's container with
`docker compose down`, then run the launcher again. To halt everything for this
project, use `docker compose down`. Other projects' containers are untouched.

The final GUI-only window sizing and RViz zoom were added after benchmarking.
The exact recorded launch is `dynamic_test_runs/validation/benchmark_launch_snapshot.py`.
The adjacent `benchmark_sources.zip` preserves every recorded source byte and
line ending, including files Git normalizes during checkout on other platforms.
`dynamic_source_verification.json` verifies all 20 recorded source sets and an
identical final-GUI/main-mission Nav2 parameter hash. No hashes were rewritten.
Use a new output directory for reproduction; the strict `--resume` guard will
correctly reject the older launch hash if pointed at the completed main matrix.

## Presentation and limits

Actual desktop: `../media/dynamic_factory_gui.jpg`. Actual moving-person camera
frame: `../media/dynamic_yolo26_person.png`. The 105-second OBS plan is in
`demo_script.md`; the tested container lacks ffmpeg/xwd/import. No MP4 exists.

Known limits include a tight 0.087 m cart gap, 7 s wait in one cart trial and
15 recovery feedback events in the blocked bay. Humans translate without gait
animation; rectangle fitting uses declared size priors. Forecasts have no
human-intent model. YOLO is observational, CPU inference is tested, and no
CUDA or real-robot safety claim is made.

No subagents, automations or scheduled continuation were created.
