终端 1-启动仿真
export GZ_VERSION=garden
source /opt/ros/humble/setup.bash
source ~/Ai_ws/Study/vrx_ws/install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

终端 2 — 运行控制器（二选一）：
source ~/Ai_ws/Study/vrx_ws/install/setup.bash

# 选项 A：键盘遥操（WASD控制，最安全）
python3 ~/Ai_ws/Study/vrx_ws/virtual_joystick.py

# 选项 B：自动前进测试
python3 ~/Ai_ws/Study/vrx_ws/auto_pilot.py

四、验证命令速查表(看实时数据把--once去掉)

# 看所有话题（应有 36 个）
ros2 topic list | wc -l

# 看 GPS 数据（应显示悉尼坐标 -33.72°, 150.67°）
ros2 topic echo /wamv/sensors/gps/gps/fix --once

# 看 IMU 数据
ros2 topic echo /wamv/sensors/imu/imu/data --once

# 检查话题发布者（应有 1 个 publisher）
ros2 topic info /wamv/sensors/imu/imu/data

# 检查 TF 帧
ros2 topic echo /tf_static --once | grep child_frame_id

# 手动发推进器命令测试
ros2 topic pub --once /wamv/thrusters/left/thrust std_msgs/msg/Float64 "data: 500.0"

自定义世界模型，在同一个终端：
export GZ_SIM_RESOURCE_PATH=/home/han/Ai_ws/Study/vrx_ws/src/example_vrx_package:$GZ_SIM_RESOURCE_PATH

export GZ_VERSION=garden
source /opt/ros/humble/setup.bash
source ~/Ai_ws/Study/vrx_ws/install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta_custom
