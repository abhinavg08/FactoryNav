# Visual provenance

Every gallery image comes from this project. No stock or fabricated simulation visuals are used.

| File | Source and interpretation |
| :--- | :--- |
| [normal_factory_map.png](normal_factory_map.png) | Labelled geometry from `factorynav_bringup/config/factory_scene.json`, rendered by `scripts/render_factory_overview.py`. Dashed arrow indicates the default goal direction, not a recorded trajectory. |
| [dynamic_factory_map.png](dynamic_factory_map.png) | Actual generated occupancy geometry and stations from `dynamic_scene.json`, produced by `scripts/generate_dynamic_factory.py`. Dashed routes are geometric alternatives. |
| [dynamic_factory_gui.jpg](dynamic_factory_gui.jpg) | Actual Gazebo/RViz desktop captured during an advanced navigation mission. [Capture metadata](dynamic_factory_gui.json). |
| [human_avoidance_trajectory.png](human_avoidance_trajectory.png) | Measured robot/human trajectories and conservative footprint clearance from final [`dynamic_head_on_2`](../docs/dynamic_test_runs/validation/dynamic_head_on_2/result.json). |
| [dynamic_yolo26_person.png](dynamic_yolo26_person.png) | Actual annotated robot-camera frame from the [single-human GUI trial audit](../docs/dynamic_test_runs/gui_head_on/dynamic_head_on_0/single_person_validation.json), timestamp 25.800 s; actor moving at approximately 0.221 m/s. |
| [gazebo_camera_person.png](gazebo_camera_person.png), [yolo26_gazebo_person.png](yolo26_gazebo_person.png) | Original camera input and actual YOLO annotation used in the baseline perception checks. |

The standing-person model visible in camera images is by Marina Kollmitz via OSRF's Gazebo model repository under CC-BY-3.0. See [credits](../docs/credits.md) and the [pinned asset manifest](../setup/person_assets.lock.json).

## Regenerate the normal map diagram

Use the already-built base image; no simulation starts:

```powershell
$factorynavRoot = (Get-Location).Path
docker run --rm --pull=never --mount "type=bind,source=$factorynavRoot,target=/workspace" --workdir /workspace --entrypoint python3 factorynav-ai:jazzy /workspace/scripts/render_factory_overview.py
```

The complex map generator also regenerates its world, occupancy map and geometry checks. Run it only when intentionally updating scene geometry; do not rewrite sealed benchmark evidence to match a new scene. Camera frames and desktop screenshots are recorded observations and cannot be regenerated without a new run.
