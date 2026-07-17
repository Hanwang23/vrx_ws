#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source /home/han/Ai_ws/Study/vrx_ws/install/setup.bash
set -euo pipefail

export GZ_VERSION=garden
cd /home/han/Ai_ws/Study/vrx_ws

extra_args=()
if [[ "${FORCE_CLEAN_STALE:-False}" == "True" ]]; then
  extra_args+=(--force-clean-stale)
fi

ros2 run codex_usv_controller run_evaluation \
  --trials "${TRIALS:-3}" \
  --launch-file multi_waypoint_course.launch.py \
  --expected-waypoint-count 8 \
  --timeout "${WATCHDOG_TIMEOUT:-1500}" \
  --max-wall-time "${MAX_WALL_TIME:-1200}" \
  --max-mean-error 2.5 \
  --max-waypoint-error 3.0 \
  --max-alignment-yaw-rate 15.0 \
  --max-alignment-command-yaw-rate 6.0 \
  --max-continuous-rotation 75.0 \
  --max-waypoint-duration 260.0 \
  "${extra_args[@]}"
