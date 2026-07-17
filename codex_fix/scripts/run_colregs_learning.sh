#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${WORKSPACE}"
source /opt/ros/humble/setup.bash
export GZ_VERSION="${GZ_VERSION:-garden}"

colcon build --merge-install --base-paths codex_fix \
  --packages-select codex_usv_controller
source install/setup.bash
set -u

exec ros2 run codex_usv_controller run_evaluation \
  --trials 1 \
  --launch-file colregs_learning.launch.py \
  --timeout 600 \
  --max-mean-error 3.0 \
  --max-waypoint-error 3.5 \
  --min-colregs-active-samples 20 \
  --min-dynamic-track-count 2 \
  "$@"
