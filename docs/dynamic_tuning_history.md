# Retained engineering iterations

Every outcome below is a real simulation result. These development runs are
separate from final validation and include failures. Consult each directory's
effective parameters where present; older profiles are also saved alongside
the evidence. A successful Nav2 action alone is insufficient for acceptance.

| Evidence directory | Change / observation | Outcome |
|---|---|---|
| `static_first` | Expanded map, no agents | Six goals succeeded; 70.540 m; initial contact topics still required correction |
| `physical_v1` | Physical human contact calibration, first head-on run | Contact pair and robot displacement verified; continuous passing observed, but new local layer's scan height filter was invalid |
| `comparison_v2` | Original/tuned first comparison | Both completed; original gap only 0.020 m; dynamic local-layer validation still failed |
| `local_layer_fixed` | Explicit per-scan maximum height 2 m | Real local marking/clearing; head-on passing and blocked-bay stop/resume succeeded |
| `scenario_pilots` | Crossing, alongside, cart pilots | First crossing contacted a wheel and aborted; alongside and both cart cases succeeded |
| `path_angle_v2` | Path-angle lookahead 4 → 20 points, weight 2 → 4 | Crossing and head-on retests passed |
| `mission_pilot` | Both humans, cart and live YOLO | Full mission passed without contact; CPU YOLO confidence reached 0.884 |
| `margin_v2` | Repeated head-on/crossing trials | All six actions succeeded without contact; late crossing gap only 0.016 m; later batch interrupted to improve margin |
| `padding_v3` | 0.06 m planning padding | Crossing gap improved to 0.078 m; another head-on run slowed too much and passed with only 0.011 m |
| `padding_v4`, `margin_v4` | 0.10 m planning padding | One difficult crossing passed; repeated central head-on case contacted the human |
| `horizon_v5` | 96-step horizon, angular sampling deviation 0.8 | Head-on passed at 0.131 m; crossing gap 0.079 m narrowly failed the 0.08 m acceptance threshold |
| `horizon_v6` | 0.05 m planning padding | Three continuous head-on passes; first crossing contacted the human; two later startup trials invalidated by recorder metadata handling |
| `forecast_v7` | LiDAR-cluster forecast prototype | Recorder metadata error invalidated both trials; these are not navigation outcomes |
| `forecast_v8` | Recorder fixed; forecast added to MPPI | Forecast alone did not fix the full-turn crossing trap; failed run retained |
| `shim_v9` | Nav2 rotation shim around MPPI plus forecast | All three crossings passed; two head-on variants passed continuously; third became trapped, contacted human and aborted |
| `forecast_gate_v10` | Forecast only beyond 2 m; reject projected points within 0.7 m | Difficult head-on passed at 0.291 m without stopping; crossing gap 0.057 m failed margin |
| `directional_forecast_v11` | Gate near closing forecasts, retain transverse forecasts | Head-on passed at 0.282 m; crossing contacted the human twice |
| `temporal_critic_v12`, `temporal_v12_repeats` | Native same-time MPPI critic, terminal continued-motion cost and angular-travel cost | Two difficult pilots passed; repeated head-on seed 2 still contacted the human; three crossings and two alongside trials passed |
| `circle_model_v13` | Fit actual human scan arcs; use relative endpoint velocity in terminal risk | All three head-on and crossing variants passed; alongside passed with one slow goal correction; oversized circular cart bound prevented goal completion |
| `box_model_v14` | Rectangle-aware cart fit with declared size priors | Cart pilot passed at 0.440 m; head-on pilot passed at 0.266 m |
| `pre_robust_box_v15` | Relax angular cost within 1 m of goal | Three head-on, crossing and alongside variants passed; alongside recoveries eliminated. Cart seed 1 contacted twice; seed 2 missed the 0.08 m margin. Whole evidence retained, including completed mission pilots |
| `robust_box_v16_pilot` | Fit rectangle orientation from whole-cluster boundary residual instead of noisy adjacent rays | Previously failing cart seed 1 passed at 0.401 m with no contact; noisy-corner regression test passed |
| `validation` | Fresh matrix using the selected whole-cluster fit and native temporal critic | Final outcomes are generated in [navigation results](dynamic_navigation_results.md); this directory is kept separate from development pilots |

The recurring failure mode was a robot slowing or turning while an independent
agent continued into it. Collision Monitor can stop robot commands, but a stop
alone cannot stop an approaching human model. Physically validated contacts
and ground-truth geometry exposed this instead of allowing an apparently
successful action to hide it. Increasing footprint padding alone worsened one
interaction. Longer trajectory search helped head-on cases; heading correction
helped the crossing turn trap. Forecast gating could not solve the limitation
of placing time-collapsed future occupancy into an ordinary spatial costmap.
The selected implementation evaluates robot and obstacle positions at matching
times inside MPPI, leaving the raw-scan costmaps intact. Its terminal heuristic
uses relative candidate velocity so a stop in an approaching actor's line is
penalized without blocking safe continued crossing motion.

Repeated cart tests exposed another distinct problem: close corner returns
made an adjacent-ray orientation average jump by roughly 50 degrees. The
resulting fitted center shift appeared as false lateral velocity. The final
fitter minimizes boundary residual over the entire cluster and refines the
angle around its best one-degree sample. A noisy synthetic corner tests this
specific geometry failure; fresh physical trials assess its navigation effect.
The 1.0 × 0.7 m box prior is explicit and configurable, and must be recalibrated
for other equipment.

No agent controller reads robot pose or changes its route to guarantee a pass.
The original factory remains separate and available throughout development.
