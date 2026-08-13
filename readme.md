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


7.16
所有修改都在 [han_usv_controller](/home/han/Ai_ws/Study/vrx_ws/han_usv_controller) 中，根目录旧控制器没有改动。
本次改进
新增 [无限学习世界 (line 634)](/home/han/Ai_ws/Study/vrx_ws/han_usv_controller/worlds/wayfinding_task.sdf:634)，任务时长改为 100 个仿真年，默认不再受 320 秒限制。
保留 Dubins + ILOS + PID 控制链。
新增避障 episode 检测和事件触发 Dubins 重规划：雷达连续清空 0.5 秒才认为避障结束。
偏离旧路径超过 8 米才重规划。
5 秒冷却，防止频繁改路。
距目标小于两个转弯半径时禁止重规划。
障碍重新出现会取消尚未激活的规划。

RViz 增加 PATH REV、DEV，可直观看到路径偏离和路径版本变化。
Gazebo、RViz、点云配置、控制器现在由同一个 launch 启动。
timed_competition 和 competition_mode 分开控制。
完整说明见 [README.md](/home/han/Ai_ws/Study/vrx_ws/han_usv_controller/README.md)。


带 16 个额外浮标的学习场景
ros2 launch han_usv_controller buoy_course.launch.py

不需要额外浮标时：
ros2 launch han_usv_controller simulation.launch.py

恢复官方 300 秒竞赛倒计时：
ros2 launch han_usv_controller simulation.launch.py \
  timed_competition:=True
加入
在 RViz 中：
青色线：Dubins/ILOS 参考路径。
DEV：船到路径投影点的距离。
PATH REV：路径版本，重规划时会增加。
红球：确认的障碍物。
黄色箭头：局部避障航向。



7.17-10.23
推荐运行
最推荐从官方 Wayfinding 学习任务开始。下面一条命令会同时打开 Gazebo、RViz 和控制器：
ros2 launch han_usv_controller simulation.launch.py
默认学习世界没有约 320 秒的竞赛倒计时，可以慢慢观察。
其他实验入口：
# 增加 16 个浮标
ros2 launch han_usv_controller buoy_course.launch.py

# 强制测试 State Lattice 绕行
ros2 launch han_usv_controller lattice_stress.launch.py

# 两艘动态目标船和 COLREGs
ros2 launch han_usv_controller colregs_learning.launch.py

# 随机浮标布局
ros2 launch han_usv_controller \
  random_buoy_course.launch.py \
  scenario_seed:=1000
电脑性能不足时：
ros2 launch han_usv_controller simulation.launch.py \
  headless:=True rviz:=False
观察控制状态：
ros2 topic echo /autonomous_usv/status

打开 Gazebo、RViz，并运行 8 航点控制器：
ros2 launch han_usv_controller multi_waypoint_course.launch.py




1. 基础场景
ros2 launch han_usv_controller simulation.launch.py
启动：
Gazebo
RViz
WAM-V
EKF
自主控制器
原始 3 个 Wayfinding 航点
适合先确认船能否正常完成基础航点。这里主要观察 Dubins 路径、ILOS 跟踪和最终艏向对准，不额外制造复杂障碍。
2. 随机浮标场
ros2 launch han_usv_controller random_buoy_course.launch.py scenario_seed:=1000
增加 16 个随机浮标，包括红绿浮标门和橙色障碍物。
scenario_seed:=1000 是随机种子：
相同种子会得到相同浮标布局。
改为 1001、1002 会生成其他布局。
适合检查算法是不是只对某一组固定障碍有效。
3. 固定浮标场
ros2 launch han_usv_controller buoy_course.launch.py
增加固定位置的：
6 对红绿浮标门。
4 个橙色障碍浮标。
合计 16 个浮标。
适合学习 RViz 中的点云、浮标候选、占据栅格、局部避障方向和实际轨迹。因为每次布局相同，方便修改参数前后对比。
4. State Lattice 压力场
ros2 launch han_usv_controller lattice_stress.launch.py
在固定浮标场基础上增加一组障碍带，故意阻挡简单的解析 Dubins 路径，迫使控制器使用：
占据栅格
 -> 障碍膨胀
 -> State Lattice / Hybrid A*
 -> 碰撞检查路径
 -> ILOS 跟踪
主要观察状态中的：
guidance_mode: lattice_ilos
lattice_expanded_states
lattice_planning_time_ms
lattice_fallback
正常情况应看到 lattice_expanded_states > 0，并且 lattice_fallback=false。
5. 动态船舶与 COLREGs
ros2 launch han_usv_controller colregs_learning.launch.py
启动航道浮标和两艘移动目标船，用于测试：
动态目标多目标跟踪。
目标速度估计。
CPA 最近会遇距离。
TCPA 最近会遇时间。
对遇、横越和追越分类。
COLREGs 右转避让与主动降速。
重点状态字段：
dynamic_track_count
colregs_active
colregs_encounter
colregs_action
colregs_tcpa_s
colregs_dcpa_m
推荐学习顺序
simulation
  -> multi_waypoint_course
  -> buoy_course
  -> random_buoy_course
  -> lattice_stress
  -> colregs_learning