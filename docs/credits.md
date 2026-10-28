# Credits and references

FactoryNav is created and maintained by [Abhinav Reddy](https://github.com/abhinavg08).
The upstream acknowledgements below identify dependencies and assets.

This repository's original scripts and configuration use the root MIT license.
Installed dependencies retain their own licenses. No ownership of third-party
robot descriptions, simulator assets, maps, or algorithms is claimed.

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) provides the middleware,
  Python client library, transforms, lifecycle services, and actions.
- [Navigation2](https://github.com/ros-navigation/navigation2/tree/jazzy),
  Apache-2.0, supplies the baseline launch, localization, costmaps, planning,
  control, and recovery behavior.
- [Nav2 minimal simulation](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation)
  supplies the compatible TurtleBot3 simulation and associated robot assets.
- [ROBOTIS TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
  is the robot platform used by the upstream baseline.
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/) provides simulation
  and sensor generation, integrated with ROS by `ros_gz`.
- [Microsoft WSLg container guidance](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md)
  describes display and vGPU mounts used during the graphics compatibility audit.
- [Conda](https://docs.conda.io/projects/conda/en/stable/) and conda-forge
  provide the project-local Windows Python tooling environment.
- Xvfb, Openbox, x11vnc, noVNC, and websockify provide the local virtual desktop
  fallback when direct WSLg mounting is unavailable.
- [Ultralytics YOLO26](https://docs.ultralytics.com/models/yolo26/) supplies the
  optional pretrained detector. Ultralytics retains its upstream
  [AGPL-3.0 license](https://github.com/ultralytics/ultralytics/blob/main/LICENSE).
- The [standing-person model](https://github.com/osrf/gazebo_models/tree/master/person_standing)
  by Marina Kollmitz was created with MakeHuman and is distributed through OSRF's
  [CC-BY-3.0 model repository](https://creativecommons.org/licenses/by/3.0/).
  Meshes and textures are cached without modification;
  the original camera probe uses a static instance. Advanced traffic attaches
  the same mesh to a velocity-controlled, collision-enabled rigid human proxy;
  it translates and turns without an animated walking gait.
  `setup/person_assets.lock.json` pins source revision and asset checksums.

## Distribution boundaries

The root MIT license covers original FactoryNav code. It does not replace the
AGPL-3.0 terms applicable to the optional Ultralytics runtime or downloaded
models, nor the attribution requirements of the person model. Review those
upstream terms for redistribution and deployment. The repository excludes
downloaded weights, meshes, textures, wheels and container images. Rendered
person imagery is attributed above and in [visual provenance](../media/README.md).
The supplied fetch script retains an upstream license copy in the local cache.
See also [NOTICE](../NOTICE).
