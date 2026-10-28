# 105-second recording plan

No final MP4 is included. Record actual simulation footage with these commands
and edit clearly labeled clips. Never present separated experiments as one
continuous mission. Keep the final video under two minutes.

1. Start the GUI: `./scripts/run_demo.ps1 -DynamicFactory -Perception`.
   Open `http://localhost:6080/vnc.html?autoconnect=1&resize=scale`. Wait for
   Gazebo, RViz, both humans and the cart. The left/right desktop layout is
   configured by Gazebo's explicit window size and Openbox. RViz fits the
   whole map. If the noVNC page was already open, reload it. A real prepared
   view is saved as `media/dynamic_factory_gui.jpg`.
2. In OBS, create a 1600 × 900 scene. Add Window Capture for the browser or
   Codex window displaying noVNC, crop to the remote desktop, and fit to canvas.
   Use 30 fps, an H.264 encoder supported by the local installation, and MKV
   recording. Use OBS File → Remux Recordings to export MP4 after capture.
   A Windows application-window recording is also usable if OBS is unavailable.
3. Use the tested benchmark commands below to obtain the individual clips.
   Each test resets Gazebo; reload noVNC when the new desktop is ready. Start
   recording before launching the test, since actors move as soon as its
   sensor/localization checks pass. Keep unedited originals with test logs.

```powershell
# Separate head-on clip with real LiDAR control and YOLO observations
.\.conda\python.exe scripts/test_dynamic_scenarios.py --scenarios head_on --repeat 1 --gui --perception --keep --output docs/dynamic_test_runs/recording_head_on

# Separate crossing cart clip
.\.conda\python.exe scripts/test_dynamic_scenarios.py --scenarios cart --repeat 1 --gui --perception --keep --output docs/dynamic_test_runs/recording_cart

# Actual Loading → Assembly → Quality → Delivery mission; all agents remain active
.\.conda\python.exe scripts/test_dynamic_scenarios.py --scenarios full --repeat 1 --gui --perception --keep --output docs/dynamic_test_runs/recording_mission
```

Choose new output directory names for additional captures; the runner refuses
to overwrite previous evidence. GUI rendering may reduce simulation real-time
factor. Use simulation timestamps in the logs when explaining navigation time.

| Video time | Actual content | Caption |
|---|---|---|
| 0:00–0:15 | Wide factory view, labeled map, visibly moving humans/cart | ROS 2 Jazzy · Gazebo Harmonic · Nav2 |
| 0:15–0:40 | Head-on approach, lateral maneuver, separation and continued progress | Head-on test · real LiDAR and MPPI |
| 0:40–1:00 | Cart crossing the intended route, changing costmap and robot trajectory | Separate crossing-cart experiment |
| 1:00–1:30 | Selected portions of the full mission and live YOLO annotations | Mission excerpt · label any speed-up |
| 1:30–1:45 | Real action success and saved result table | Measured trials; include failures/limitations in linked report |

Show RViz's map, robot, global path, local costmap, and scan. Enable only the
displays needed for the clip. YOLO's annotated image is available on
`/perception/annotated`; add an RViz Image display with that topic if it is not
already visible. For a short MPPI visualization session, use
`./scripts/run_demo.ps1 -DynamicFactory -Perception -DebugTrajectories`.
Leave it disabled for benchmarks. This launcher configures the single world;
do not start a second ROS/Gazebo launch. Candidate robot trajectories are not
predictions of human intent.

For a larger camera close-up, enlarge or undock RViz's annotated-image panel,
or use the existing image viewer:

```powershell
docker compose exec -d simulation bash -lc 'source /opt/ros/jazzy/setup.bash; DISPLAY=:99 ros2 run rqt_image_view rqt_image_view /perception/annotated'
```

Keep the raw full-mission recording, then cut the close-up into the labeled
YOLO segment. The saved `media/dynamic_yolo26_person.png` is an actual still
from the verified single-human moving-camera trial, not a video clip.

The cyan point cloud, when visible, previews two seconds of constant-velocity
obstacle motion from LiDAR clusters. It is visual only; the MPPI critic uses
time-indexed motion estimates rather than adding future points to costmaps.
Label the preview separately from the actual scan and camera detections.

Use this narration only over the demonstrated behavior: “The robot transports
materials between manufacturing stations. LiDAR updates its costmaps as people
and carts move through the factory. Nav2 selects feasible trajectories, and
the robot continues toward its destination. YOLO26 provides a separate live
person-detection view.”

Finish with `docker compose down`. Inspect `docker compose ps` to ensure no
duplicate simulation remains.
