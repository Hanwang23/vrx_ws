#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${WORKSPACE}"
source /opt/ros/humble/setup.bash
export GZ_VERSION="${GZ_VERSION:-garden}"
colcon build --merge-install --base-paths han_usv_controller \
  --packages-select han_usv_controller
source install/setup.bash
set -u

exec ros2 run han_usv_controller run_evaluation --trials 3 "$@"
