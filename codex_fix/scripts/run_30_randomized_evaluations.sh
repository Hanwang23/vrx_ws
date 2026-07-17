#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/han/Ai_ws/Study/vrx_ws/install/setup.bash
set -u
export GZ_VERSION=garden

cd /home/han/Ai_ws/Study/vrx_ws
colcon build --merge-install --base-paths codex_fix \
  --packages-select codex_usv_controller
source install/setup.bash

exec ros2 run codex_usv_controller run_evaluation \
  --trials 30 \
  --launch-file random_buoy_course.launch.py \
  --base-seed "${BASE_SEED:-1000}" \
  --timeout 600 \
  --max-mean-error 3.0 \
  --max-waypoint-error 3.5 \
  "$@"
