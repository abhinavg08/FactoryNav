# FactoryNav AI 2.0 implementation checkpoints

Baseline: commit `c342a41`, preserved on `main`. Work branch:
`feature/dynamic-factory-navigation`.

- [x] Audit original source, evidence, Docker, controller, sensors, and velocity chain.
- [x] Original factory regression: goal succeeded in 25.903 seconds.
- [x] Original obstacle-detour regression: succeeded in 45.203 seconds.
- [x] Generate a separate complex world, map, and labeled overview.
- [x] Verify conservative footprint connectivity to every station and an alternative route.
- [x] Execute static navigation through all stations (six successful goals, 70.540 m).
- [x] Validate moving human geometry, actual LiDAR returns, and physical contacts.
- [x] Demonstrate continuous head-on passing: 3/3 main trials, zero nearby stopped time.
- [x] Compare original MPPI and advanced MPPI: 1/3 versus 3/3 stricter acceptance.
- [x] Exercise crossing, alongside, cart, shared-cart and blocked-bay stop/resume.
- [x] Complete three main full missions with both humans, cart and live YOLO.
- [x] Audit a single-human GUI run: 32 distinct moving-person detection frames over 17.8 s.
- [x] Accept all 17 advanced main trials; preserve both original-profile clearance failures.
- [x] Generate source-derived reports, CSV, trajectory plot and forecast evaluation.
- [x] Build all four ROS packages together and verify installed entry points.
- [x] Rerun original factory (25.902 s), detour (45.802 s) and camera checks (25 person frames).
- [x] Verify optional MPPI MarkerArray publisher and RViz subscriber.
- [x] Correct GUI window sizing; complete the final GUI mission with no contacts/recoveries.
- [x] Save actual desktop and moving-person camera evidence and a 105-second recording plan.
- [x] Prepare the documented implementation and evidence for its local Git checkpoint.

The final GUI mission reached all three goals in 191.127 sim s, with 0.276 m
minimum actor clearance. GUI-only launch changes follow the headless matrix;
the benchmark launch snapshot and source-integrity check preserve that
distinction. Final GUI and main full-mission Nav2 parameter hashes are identical.
No MP4 is claimed: the requested recording-plan fallback is supplied. No GitHub
publication was performed.

## Setup issues retained

The upstream simulation launch started Gazebo before its xacro world file was
created in a headless launch. The advanced launch now starts Gazebo directly on
the already generated SDF, while reusing Nav2's robot and navigation bringup.
The original launch remains unchanged. Failure logs are retained under
`log/dynamic/failures/` and the first diagnostic lines in `dynamic_test_runs/`.

The contact-system implementation requires its topic under `<contact>`, rather
than only the sensor's general topic element. This was corrected before the
dynamic-contact tests. The first static run therefore uses geometric clearance
as its complete-run proximity evidence; contact instrumentation is validated
separately before reporting contact counts in dynamic tests.
