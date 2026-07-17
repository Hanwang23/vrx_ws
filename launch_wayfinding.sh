#!/usr/bin/env bash
# ============================================================
#  VRX Wayfinding 自主导航启动脚本
#  用法: bash launch_wayfinding.sh
# ============================================================

set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-wayfinding_task}"
HEADLESS="${VRX_HEADLESS:-False}"
SIM_PID=""

cleanup() {
    if [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" 2>/dev/null; then
        echo "正在关闭仿真..."
        kill -INT "${SIM_PID}" 2>/dev/null || true
        wait "${SIM_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "=========================================="
echo "  VRX Wayfinding 自主导航"
echo "=========================================="

# ---- 环境设置 ----
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
export GZ_VERSION=garden
set -u

echo "[1/3] 启动 Gazebo 仿真 (${WORLD})..."
ros2 launch vrx_gz competition.launch.py world:="${WORLD}" headless:="${HEADLESS}" &
SIM_PID=$!

echo "[2/3] 等待 GPS、IMU、推进器和任务话题..."
READY=0
for i in $(seq 1 60); do
    TOPICS="$(ros2 topic list 2>/dev/null || true)"
    if grep -qx '/wamv/sensors/gps/gps/fix' <<<"${TOPICS}" && \
       grep -qx '/wamv/sensors/imu/imu/data' <<<"${TOPICS}" && \
       grep -qx '/wamv/thrusters/left/thrust' <<<"${TOPICS}" && \
       grep -qx '/vrx/wayfinding/waypoints' <<<"${TOPICS}" && \
       timeout 3 ros2 topic echo --once /wamv/sensors/gps/gps/fix \
           >/dev/null 2>&1 && \
       timeout 3 ros2 topic echo --once /vrx/task/info \
           >/dev/null 2>&1; then
        READY=1
        break
    fi
    if ! kill -0 "${SIM_PID}" 2>/dev/null; then
        echo "仿真进程提前退出。"
        exit 1
    fi
    printf '\r  等待中... %d/60 秒' "${i}"
    sleep 1
done
echo ""

if [[ "${READY}" -ne 1 ]]; then
    echo "60 秒内未等到完整的 ROS 话题。"
    exit 1
fi

echo "[3/3] 启动自主控制器..."
echo "=========================================="
python3 "${WORKSPACE_DIR}/autonomous_controller.py"
echo "=========================================="
echo "完成。"
