# Submission answers

These reflect the executed navigation and live camera-perception implementation.

**1. How did you approach the problem?**
I audited the actual machine, established the official TurtleBot3/Nav2 baseline,
then generated a factory world and matching map from one geometry definition.
I inserted a real collision-enabled obstacle after motion began and verified
costmap marking, an alternate route, and successful arrival in repeated runs.

**2. What tools and frameworks did you use?**
ROS 2 Jazzy, Gazebo Harmonic, Navigation2, AMCL, ros_gz, RViz, TurtleBot3 Waffle,
Python/rclpy, colcon, Docker Desktop/WSL2, and a project-local Conda tooling
environment. The optional camera layer uses YOLO26, PyTorch, OpenCV/cv_bridge,
and vision_msgs. Actual Gazebo frames produced stable person detections, and
navigation also passed while inference was running.

**3. What challenges did you encounter?**
The installed WSL distribution was Ubuntu 26.04 rather than Jazzy/Harmonic's
Ubuntu 24.04 binary target. Docker could not mount that distribution's WSLg
display. An early checker also ran before lifecycle activation had completed.
The perception setup encountered slow wheel transfers, a package-index lookup
failure, and an inherited missing Python timezone dependency.

**4. How did you troubleshoot those challenges?**
I checked OS versions, package availability, GPU visibility, graphics mounts,
ROS topics, TF, lifecycle services, and action results. I preserved the existing
OS, used a compatible container with a documented virtual desktop, and corrected
the checker's activation wait before rerunning the baseline and factory tests.
I cached pinned wheels from official PyPI with SHA256 verification, fixed the
timezone dependency, kept NumPy compatible with cv_bridge, and validated live
image/detection timestamps and CPU inference timing.

**5. What did you learn?**
Reliable navigation depends on compatible runtime versions, consistent time,
correct transforms, maps that match collision geometry, and evidence from the
actual ROS graph. Separating optional perception from the command chain protects
a working LiDAR/Nav2 baseline.

**6. What would you improve with more time?**
I would expand the test matrix and trajectory recording, configure accelerated
graphics and CUDA inference, validate perception across viewpoints and lighting,
and then consider calibrated camera–LiDAR association and a supported semantic
safety arbiter. Physical robot validation would be a further step.
