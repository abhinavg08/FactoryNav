# Planned 90-second demo

The core navigation and camera-perception scenarios passed. This is the recording
plan; no demo video exists yet. Demo video link: pending recording/publication.

| Time | Planned view and narration |
| --- | --- |
| 0:00–0:15 | Keep Gazebo and RViz visible together. Identify the robot, factory layout, map, and LiDAR scan. |
| 0:15–0:25 | Send the Loading → Workstation B goal with automatic blockage insertion after 1 m. Show the initial path, motion, and red obstacle. |
| 0:25–1:05 | Show costmap marking, the detour around the rack ends, and successful arrival. Actual repeated goals took about 45 seconds. |
| 1:05–1:25 | Show a clearly labeled separate clean perception probe: the textured person, real annotated camera stream, and measured CPU inference. State that YOLO does not control velocity. |
| 1:25–1:30 | Show the recorded success/result table and repository. State the semantic-control and CUDA limitations. |

## Recording from Windows

1. Start the tested simulation and open the local noVNC desktop. Arrange Gazebo
   and RViz side by side, with their titles visible and the robot in view.
2. In an installed OBS Studio, create a Display Capture or Window Capture source
   for that desktop/browser. Choose 1920×1080 or 1600×900, 30 fps, and MKV output.
3. Run one rehearsal using the exact commands and starting state from the final
   verified setup. Restart the project simulation for a clean take. Start the
   world before recording, then use `send_goal.py --obstacle-after-m 1.0` for
   the mission rather than the reset-performing test launcher.
4. Start recording, perform the sequence, stop recording, and use OBS's Remux
   Recordings command to produce an MP4 if required by the submission.
5. Review the entire file for readable UI and genuine action-result evidence.
   Add a public link only after the user authorizes publishing.

The person probe uses a separate clean run so the person does not overlap the
central blockage. Label that cut clearly. Use concise captions and describe
YOLO as a perception layer. Do not substitute invented telemetry or imply that
YOLO caused the demonstrated LiDAR/Nav2 detour.
