# FactoryNav documentation and evidence

Start with the [repository overview](../README.md) and [portable setup guide](setup.md).

| Guide | Contents |
| :--- | :--- |
| [Advanced architecture](dynamic_factory_architecture.md) | Sensor/control boundaries, temporal critic and measurement definitions |
| [Advanced results](dynamic_navigation_results.md) | Final 17-trial matrix, GUI checks, tracking errors and limitations |
| [Controller comparison](controller_comparison.md) | Original MPPI settings versus the advanced profile |
| [Command reference](dynamic_commands.md) | Manual missions, independent traffic, fresh-world scenarios and cleanup |
| [Four-stage presentation guide](four_stage_demo_script.md) | Normal/complex factories with static/dynamic scenarios and accurate narration |
| [Original architecture](architecture.md) | Baseline ROS and perception pipeline |
| [Original test results](test_results.md) | Original factory, blockage and camera regressions |
| [Troubleshooting](troubleshooting.md) | Known setup, rendering, discovery and scenario issues |
| [Credits and licensing](credits.md) | Original code, upstream dependencies and attributed assets |

## Evidence organization

- **Final headless matrix:** [`dynamic_test_runs/validation/`](dynamic_test_runs/validation/). Includes 17 advanced trials and three original-profile comparisons; every completed outcome is retained.
- **Exact benchmark sources:** [`benchmark_sources.zip`](dynamic_test_runs/validation/benchmark_sources.zip) preserves byte-level source hashes across Git line-ending normalization. The [source verification report](dynamic_source_verification.json) records the later GUI-only changes.
- **Additional GUI validation:** [`gui_head_on/`](dynamic_test_runs/gui_head_on/), [`gui_live_full/`](dynamic_test_runs/gui_live_full/) and [`gui_layout_final/`](dynamic_test_runs/gui_layout_final/). These are separate from the headline matrix.
- **Original-mode regressions:** `regression_*_final.json` in [`dynamic_test_runs/`](dynamic_test_runs/) and the original [`test_runs/`](test_runs/).
- **Development failures and calibration:** retained under their original experiment directories. The [tuning history](dynamic_tuning_history.md) explains their purpose; deliberately colliding contact probes are excluded from success rates.

Recorded logs, parameters, timestamps, historical machine paths and source bundles are evidence and remain unchanged. New local experiments are ignored by default; promote a completed run deliberately rather than publishing every generated file. Presentation-only local reruns are not part of the published results.

## Local checks without a simulation

```powershell
.\.conda\python.exe scripts/check_repository.py --history
$env:PYTHONPATH = Join-Path $PWD 'factorynav_dynamic'
.\.conda\python.exe -m unittest discover -s factorynav_dynamic/test -v
```

The first command checks selected files and reachable history for recognized credential patterns, video files, Google Drive links, broken local Markdown targets, PNG maps and result-count consistency. It reports paths and issue categories without printing matched credentials. This is a targeted publication check, not a guarantee that arbitrary secrets can be detected.
