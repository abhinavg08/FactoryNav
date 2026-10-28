# Original MPPI settings versus advanced MPPI tuning

The preserved factory already used `nav2_mppi_controller::MPPIController`. This compares its original MPPI settings with the advanced profile on the same expanded world, speed limits, physical footprint, 10 Hz LiDAR, traffic routes and starting poses. Shared scenario overrides also use a 0.35 m / 20 s progress checker and full costmap publication. The original factory launch remains unchanged. It is an integrated profile comparison: costmap size/rates, MPPI tuning and the native temporal critic change. The LiDAR tracking node runs in both profiles for observation; only the advanced profile loads its native critic. This does not isolate one parameter or compare DWB against MPPI.

| Profile | Acceptance / trials | Goals succeeded | Continuous passes | Mean sim time (s) | Mean path (m) | Min human clearance (m) | Contacts |
|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | 1/3 | 3 | 2 | 43.27 | 14.524 | -0.034 | 0 |
| dynamic | 3/3 | 3 | 3 | 47.69 | 14.425 | 0.227 | 0 |

| Profile | Mean total stopped (s) | Mean slowing (s) | Max nearby stopped (s) | Max recoveries | Mean yaw-command sign changes | Median CPU | Mean RTF |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | 1.13 | 3.79 | 1.20 | 0 | 1.3 | 256.5% | 0.982 |
| dynamic | 0.83 | 9.49 | 0.00 | 0 | 0.3 | 272.9% | 0.984 |

Slowing uses measured speed 0.02–0.24 m/s; stopped uses <0.02 m/s. Yaw-command sign changes count only commands with magnitude above 0.25 rad/s. They include normal steering reversals and are a repeatable motion proxy, not a count of diagnosed oscillation failures. CPU samples include startup and navigation; 100% is one logical core.


The selected advanced profile has worst head-on clearance 0.227 m, versus -0.034 m for the original settings. Mean goal time is 47.69 versus 43.27 sim s; sampled median CPU is 272.9% versus 256.5%. Selection weighs the measured clearance and acceptance outcomes together with this time/compute tradeoff.


The advanced profile is the default for dynamic mode; the original factory and advanced `-ControllerProfile baseline` remain selectable. Inspect the clearance and contact outcomes together with completion time. A completed goal with a very small positive gap is weaker evidence than a larger margin. Three variants are a modest test, not a statistical guarantee.

| Parameter | Original settings | Advanced |
|---|---|---|
| Controller / model | MPPI / DiffDrive | MPPI / DiffDrive |
| Control frequency, step | 20 Hz, 0.05 s | 20 Hz, 0.05 s |
| Horizon, batch | 56 steps (2.8 s), 2000 | 96 steps (4.8 s), 2000 |
| Angular sampling deviation | 0.4 rad/s | 0.8 rad/s |
| Forward/reverse/yaw limits | 0.35 / −0.15 m/s, 1.2 rad/s | Same |
| Linear / angular acceleration | 3 / 3.5 | 1.0 forward, 1.2 braking / 2.0 |
| Cost critic footprint | false, stride 2 | true, stride 1 |
| Path alignment weight | 14 | 5 |
| Path angle weight / lookahead | 2 / 4 points | 4 / 20 points |
| Local costmap | 3 × 3 m, voxel, 5 Hz | 6 × 6 m, obstacle layer, 10 Hz |
| Inflation radius | 0.7 m | 0.65 m |
| Costmap footprint padding | 0.01 m | 0.05 m |
| Time-indexed obstacle critic | None | LiDAR-derived velocity and extent, same-time trajectory costs |
| Terminal continued-motion risk | None | Soft endpoint risk cost, additional 8 s linear-motion heuristic |

The wider rolling grid gives the 4.8 s candidate horizon usable observed space. Footprint-aware cost evaluation rejects rectangular-body conflicts. Lower path alignment weight permits lateral departures. The native critic compares candidate robot positions with obstacle estimates at matching future times; it does not fill costmaps with future points. Its terminal cost uses relative endpoint velocity to discourage stopping on an approaching obstacle's continued-motion line while allowing safe continued motion through a crossing. An angular-travel cost discourages full turns and is relaxed within 1 m of the goal for final corrections. The tested profile uses MPPI directly, without the experimental rotation shim. Inflation and Collision Monitor remain enabled. The scan source explicitly allows heights 0–2 m, marks returns and clears with raytracing. Per-run `effective_nav2.yaml` records all inherited and overridden parameters.

The native plugin builds against the installed Jazzy headers. Its CTest checks same-time collision risk, a collision-free lateral candidate, rectangle distances, crossing endpoint motion, and the finite-horizon stopped-endpoint trap. Eight Python geometry tests check physical clearance, scan-circle fitting, declared box-size priors and noisy-corner orientation. Real simulation trials provide the navigation evidence. Constant-velocity estimates are limited by partial visibility, acceleration and reversals; the terminal lookahead is a conservative risk heuristic, not validated intent prediction.

References: [Nav2 Jazzy MPPI configuration](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/), [Collision Monitor](https://docs.nav2.org/jazzy/tutorials/general_tutorials/using_collision_monitor/using_collision_monitor/), [Gazebo Harmonic VelocityControl implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/velocity_control/VelocityControl.cc).
