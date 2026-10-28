FROM ros:jazzy-ros-base-noble@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb3-sim \
    ros-jazzy-ros-gz \
    ros-jazzy-rviz2 \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    python3-colcon-common-extensions \
    python3-numpy python3-yaml python3-pil python3-pip python3-venv \
    mesa-utils xvfb x11vnc novnc websockify openbox xterm \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=26 \
    ROS_LOG_DIR=/workspace/log/ros \
    QT_X11_NO_MITSHM=1
WORKDIR /workspace
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash", "/workspace/scripts/run_demo.sh", "--baseline", "--virtual-display"]
