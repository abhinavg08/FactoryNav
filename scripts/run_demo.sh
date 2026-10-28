#!/usr/bin/env bash
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
virtual_display=false
headless=True
use_rviz=False
scenario=factory
perception=false
for argument in "$@"; do
  case "$argument" in
    --baseline) scenario=baseline ;;
    --factory) scenario=factory ;;
    --perception) perception=true ;;
    --gui) headless=False; use_rviz=True ;;
    --virtual-display) virtual_display=true; headless=False; use_rviz=True ;;
    *) echo "Unknown option: $argument" >&2; exit 2 ;;
  esac
done
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy is missing. Use the project Docker image or Ubuntu 24.04 setup.' >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
if $perception && [[ ! -x /opt/factorynav-perception/bin/python ]]; then
  echo 'Perception environment missing. Build setup/Dockerfile.perception and use compose.perception.yaml.' >&2
  exit 1
fi
if $perception && [[ "$scenario" != factory ]]; then
  echo '--perception is supported with --factory.' >&2
  exit 2
fi
if $perception; then
  export GZ_SIM_RESOURCE_PATH="$project_root/.cache/models:${GZ_SIM_RESOURCE_PATH:-}"
fi
mkdir -p "$project_root/log"
pids=()
launch_pid=''
perception_pid=''
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$perception_pid" ]]; then
    kill -INT -- "-$perception_pid" 2>/dev/null || true
    kill -TERM -- "-$perception_pid" 2>/dev/null || true
    wait "$perception_pid" 2>/dev/null || true
  fi
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep 0.2
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  for child_pid in "${pids[@]}"; do
    kill "$child_pid" 2>/dev/null || true
    wait "$child_pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if $virtual_display; then
  export DISPLAY=:99
  Xvfb "$DISPLAY" -screen 0 1600x900x24 +extension GLX +render -noreset -nolisten tcp > "$project_root/log/xvfb.log" 2>&1 &
  pids+=("$!")
  for _ in {1..50}; do
    [[ -S /tmp/.X11-unix/X99 ]] && break
    sleep 0.1
  done
  python3 "$project_root/setup/desktop_layout.py"
  openbox --config-file /tmp/factorynav_openbox.xml > "$project_root/log/openbox.log" 2>&1 & pids+=("$!")
  x11vnc -display "$DISPLAY" -rfbport 5900 -localhost -nopw -forever -shared > "$project_root/log/x11vnc.log" 2>&1 & pids+=("$!")
  websockify --web /usr/share/novnc 6080 localhost:5900 > "$project_root/log/novnc.log" 2>&1 & pids+=("$!")
  glxinfo -B > "$project_root/log/opengl.txt" 2>&1 || true
  echo 'Virtual desktop: http://localhost:6080/vnc.html?autoconnect=1&resize=scale'
fi
if [[ "$scenario" == factory ]]; then
  launch_args=("$project_root/factorynav_bringup/launch/factory.launch.py")
  launch_args+=("enable_camera:=$perception")
else
  launch_args=(nav2_bringup tb3_simulation_launch.py)
fi
echo "Starting Nav2 TurtleBot3 $scenario (headless=$headless, rviz=$use_rviz)"
setsid ros2 launch "${launch_args[@]}" \
  "headless:=$headless" "use_rviz:=$use_rviz" \
  > >(tee "$project_root/log/$scenario.log") 2>&1 &
launch_pid=$!
if $perception; then
  setsid /opt/factorynav-perception/bin/python \
    "$project_root/factorynav_perception/factorynav_perception/yolo_detector.py" \
    > "$project_root/log/yolo.log" 2>&1 &
  perception_pid=$!
fi
wait "$launch_pid"
