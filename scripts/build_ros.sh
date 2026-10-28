#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source /opt/ros/jazzy/setup.bash
python_executable=/usr/bin/python3
if [[ -x /opt/factorynav-perception/bin/python ]]; then
  python_executable=/opt/factorynav-perception/bin/python
fi
"$python_executable" -m colcon --log-base log/colcon build \
  --base-paths factorynav_bringup factorynav_perception factorynav_dynamic factorynav_mppi_critics --cmake-args -DCMAKE_BUILD_TYPE=Release
