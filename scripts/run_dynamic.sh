#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source /opt/ros/jazzy/setup.bash
critic_library=install/factorynav_mppi_critics/lib/libfactorynav_dynamic_critic.so
if [[ ! -f "$critic_library" || factorynav_mppi_critics/src/dynamic_obstacle_critic.cpp -nt "$critic_library" || factorynav_mppi_critics/include/factorynav_mppi_critics/temporal_cost.hpp -nt "$critic_library" ]]; then
  colcon --log-base log/colcon build --base-paths factorynav_mppi_critics --packages-select factorynav_mppi_critics --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
source install/setup.bash
export PYTHONPATH="/workspace/factorynav_dynamic:/workspace/scripts:${PYTHONPATH:-}"
mkdir -p log/dynamic
pids=()
cleanup() {
  trap - EXIT INT TERM
  for process in "${pids[@]}"; do kill -INT -- "-$process" 2>/dev/null || true; done
  sleep 1
  for process in "${pids[@]}"; do kill -TERM -- "-$process" 2>/dev/null || true; done
  for process in "${pids[@]}"; do wait "$process" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
headless=True; rviz=False
if [[ "${FACTORYNAV_GUI:-false}" == true ]]; then
  export DISPLAY=:99
  setsid Xvfb :99 -screen 0 1600x900x24 +extension GLX +render -noreset -nolisten tcp > log/dynamic/xvfb.log 2>&1 & pids+=("$!")
  for _ in {1..50}; do [[ -S /tmp/.X11-unix/X99 ]] && break; sleep .1; done
  python3 setup/desktop_layout.py
  setsid openbox --config-file /tmp/factorynav_openbox.xml > log/dynamic/openbox.log 2>&1 & pids+=("$!")
  setsid x11vnc -display :99 -rfbport 5900 -localhost -nopw -forever -shared > log/dynamic/vnc.log 2>&1 & pids+=("$!")
  setsid websockify --web /usr/share/novnc 6080 localhost:5900 > log/dynamic/novnc.log 2>&1 & pids+=("$!")
  headless=False; rviz=True
else
  # Gazebo GPU LiDAR still needs a renderer. Xvfb provides it without GUI load.
  export DISPLAY=:99
  setsid Xvfb :99 -screen 0 640x480x24 +extension GLX +render -noreset -nolisten tcp > log/dynamic/xvfb.log 2>&1 & pids+=("$!")
fi
for _ in {1..50}; do [[ -S /tmp/.X11-unix/X99 ]] && break; sleep .1; done
setsid ros2 launch factorynav_bringup/launch/dynamic_factory.launch.py \
  "headless:=$headless" "use_rviz:=$rviz" "enable_camera:=${FACTORYNAV_PERCEPTION:-false}" \
  "controller_profile:=${FACTORYNAV_PROFILE:-dynamic}" \
  "debug_trajectories:=${FACTORYNAV_DEBUG_TRAJECTORIES:-false}" \
  "x_pose:=${FACTORYNAV_START_X:--8.0}" "y_pose:=${FACTORYNAV_START_Y:--6.5}" "yaw:=${FACTORYNAV_START_YAW:-0.0}" \
  > >(tee log/dynamic/simulation.log) 2>&1 &
main_pid=$!; pids+=("$main_pid")
if [[ "${FACTORYNAV_FORECAST:-true}" == true ]]; then
  setsid python3 -m factorynav_dynamic.forecast > log/dynamic/forecast.log 2>&1 & pids+=("$!")
fi
if [[ "${FACTORYNAV_PERCEPTION:-false}" == true ]]; then
  if [[ ! -x /opt/factorynav-perception/bin/python ]]; then echo 'Use the perception image.' >&2; exit 1; fi
  setsid /opt/factorynav-perception/bin/python factorynav_perception/factorynav_perception/yolo_detector.py --ros-args -p output_dir:=/workspace/log/dynamic/perception > log/dynamic/yolo.log 2>&1 & pids+=("$!")
fi
if [[ "${FACTORYNAV_TRAFFIC:-none}" != none ]]; then
  setsid python3 -m factorynav_dynamic.traffic --scenario full --seed 0 > log/dynamic/traffic.log 2>&1 & pids+=("$!")
fi
wait "$main_pid"
