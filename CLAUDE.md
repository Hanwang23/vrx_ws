# VRX 仿真项目 — CLAUDE.md

> 本文档由 Claude Code 自动审查生成（2026-07-10）
> 覆盖环境概况、架构分析、已知问题、修复方案、教学大纲

---

## 1. 项目概况

| 项目 | 说明 |
|------|------|
| 工作空间 | `/home/han/Ai_ws/Study/vrx_ws/` |
| ROS 2 版本 | **Humble Hawksbill** (Ubuntu 22.04) |
| Gazebo 版本 | **Garden (gz-sim 7.9.0)** |
| 桥接包 | `ros_gz` 0.244.25（已升级，兼容 gz-sim7） |
| 源码 | `src/vrx-humble/` — 5 个包全部编译成功，零错误零警告 |
| 仿真世界 | `sydney_regatta` — 悉尼港湾，含海浪、浮标、码头、射击目标 |
| 载体 | WAM-V 双体无人艇，H 型推进器布局（2 个尾部推进器） |

### 已安装的 5 个 ROS 包

| 包名 | 作用 |
|------|------|
| `vrx_gz` | Gazebo 插件（评分、波浪、浮力等）+ Python 启动/桥接库 |
| `vrx_ros` | ROS 节点（pose_tf_broadcaster, optical_frame_publisher, monitor_sim） |
| `vrx_gazebo` | Gazebo 模型资产 |
| `wamv_description` | WAM-V URDF 模型 |
| `wamv_gazebo` | WAM-V Gazebo 模板和 xacro |

### WAM-V 传感器载荷（vrx_sensors_enabled:=true）

| 传感器 | 话题（ROS 侧） | 方向 |
|--------|-----------------|------|
| 前左摄像头 | `/wamv/sensors/cameras/front_left_camera/image_raw` | GZ→ROS |
| 前右摄像头 | `/wamv/sensors/cameras/front_right_camera/image_raw` | GZ→ROS |
| 右侧摄像头 | `/wamv/sensors/cameras/middle_right_camera/image_raw` | GZ→ROS |
| 16 线激光雷达 | `/wamv/sensors/lidars/lidar_wamv/scan` + `/points` | GZ→ROS |
| IMU | `/wamv/sensors/imu/imu/data` | GZ→ROS |
| GPS | `/wamv/sensors/gps/gps/fix` | GZ→ROS |
| 声学水听器 | `/wamv/sensors/acoustics/receiver/range_bearing` | GZ→ROS |
| 球发射器 | `/wamv/shooters/ball_shooter/fire` | ROS→GZ |
| 左推进器 | `/wamv/thrusters/left/thrust` + `/pos` | ROS→GZ |
| 右推进器 | `/wamv/thrusters/right/thrust` + `/pos` | ROS→GZ |

---

## 2. 架构对比：官方 Launch vs vrx_start.sh

### 官方 competition.launch.py 做的事

```
competition.launch.py
├── 启动 Gazebo Sim (gz-sim7) + 世界文件
├── monitor_sim.py（监控 Gazebo 生命周期，自动关 ROS）
├── xacro 生成 URDF → gz sdf -p 转 SDF
├── ros_gz_sim/create 生成 WAM-V 模型
├── ros_gz_bridge (节点A: 竞赛桥)
│   ├── /clock (仿真时钟)
│   ├── /vrx/task/info (任务状态/分数)
│   ├── /vrx/debug/wind/* (风速/风向)
│   └── 任务特定话题（停驻目标、航路点等）
├── ros_gz_bridge (节点B: 传感器/载荷桥) ← 自动从 SDF 发析
│   ├── 所有摄像头 image_raw + camera_info
│   ├── 激光雷达 scan + points
│   ├── IMU / GPS / 声学水听器
│   ├── 推进器命令（ROS→GZ）
│   ├── pose / pose_static / joint_states
│   └── 碰撞检测 / 里程计
├── pose_tf_broadcaster（传感器 TF 广播）
└── robot_state_publisher（URDF → TF 树）
```

### vrx_start.sh 做的事

```
vrx_start.sh
├── 启动 Gazebo Sim (直接调用 gz sim)
├── xacro 生成 SDF（没有 gz sdf -p 转换）
├── gz service 直接生成 WAM-V
├── gz topic 发送 /vrx/release 解锁
└── ros_gz_bridge (仅 5 个话题，全部 ROS→GZ)
    ├── /wamv/thrusters/left/thrust
    ├── /wamv/thrusters/right/thrust
    ├── /wamv/thrusters/left/pos
    ├── /wamv/thrusters/right/pos
    └── /wamv/shooters/ball_shooter/fire
```

### 缺失的关键功能

| 缺失项 | 影响 |
|--------|------|
| **所有传感器桥接**（摄像头/激光雷达/IMU/GPS） | 自主航行无法获取任何感知数据 |
| **`/clock` 桥接** | ROS 用系统时钟而非仿真时钟，TF 查找失败 |
| **pose / joint_states 桥接** | 无法获取位姿和关节状态 |
| **robot_state_publisher** | 没有 TF 树，传感器坐标系断裂 |
| **pose_tf_broadcaster** | 传感器位姿 TF 缺失 |
| **monitor_sim.py** | Gazebo 退出后 ROS 节点变成孤儿进程 |
| **任务状态桥接** | 不知道任务何时开始/结束、当前分数 |
| **声学水听器桥接** | 声学跟踪任务无法完成 |

**总结：vrx_start.sh 只能手动开船，完全无法做自主任务。**

---

## 3. 控制器脚本分析

### 四个脚本对比

| 脚本 | 用途 | 推力值 | 安全? | 接口 |
|------|------|--------|-------|------|
| `virtual_joystick.py` | 键盘遥操（WASD） | 1500 | ✅ | ROS 2 |
| `auto_pilot.py` | 自动前进 30 秒 | 1500 | ✅ | ROS 2 |
| `thruster_test.py` | 推进器功能测试 | 3000 | ⚠️ 超限 | ROS 2 |
| `direct_controller.py` | 键盘遥操（绕过 ROS） | 3000 | ⚠️ 超限 | gz topic |

### 推力安全范围

- **最大推力**：~2353 N（H 型配置）
- **安全范围**：0 ~ 2000（推荐 1500）
- **超限值 3000**：会被插件截断到 2353，但说明作者不了解限值

### 关键警告

> ⛔ **任何两个脚本同时运行都会冲突！**
> - 两个 ROS 脚本：50Hz 交替发命令，船会抖动
> - direct_controller + ROS 脚本：GZ transport 和 ROS bridge 同时写推进器，行为随机
> - **永远只运行一个控制程序**

### 所有脚本的共同缺陷

**100% 开环控制** — 没有任何脚本订阅传感器话题。无反馈 = 无法做：
- 航点跟踪（需要 GPS + IMU）
- 避障（需要激光雷达）
- 视觉感知（需要摄像头）
- 闭环 PID 控制（需要任何状态反馈）

---

## 4. 环境健康检查结果

| 检查项 | 状态 |
|--------|------|
| ROS 2 Humble + Gazebo Garden 版本匹配 | ✅ 正确 |
| 5 个包全部编译成功（零错误零警告） | ✅ 正确 |
| 环境变量（AMENT_PREFIX_PATH, GZ_SIM_SYSTEM_PLUGIN_PATH） | ✅ 已设置 |
| Python 依赖（sdformat13, rclpy, tf2_ros 等） | ✅ 全部可用 |
| 22 个评分/物理插件 .so 库 | ✅ 已编译安装 |
| xacro, robot_state_publisher, joy 等系统包 | ✅ 已安装 |
| 多工作空间共存（fishbot_slam_nav_ws） | ✅ 无冲突 |

---

## 5. 关键修复：ros_gz 传输层兼容性（2026-07-10）

### 问题

apt 安装的 `ros-humble-ros-gz-sim` (0.244.25) 链接 `ignition-transport11`（Fortress），
但 Gazebo 7.9.0 使用 `gz-transport12`（Garden），导致 `ros_gz_sim/create` 无法连接
Gazebo 服务器，WAM-V 无法生成，所有传感器桥接无法启动。

**症状**：`ros2 topic list` 只显示 11 个话题（无传感器），`create` 节点反复输出
`Requesting list of world names`。

### 修复

在 `vrx_ws/src/` 中从源码编译 `ros_gz`（`humble` 分支），设置 `GZ_VERSION=garden`：

```bash
cd ~/Ai_ws/Study/vrx_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b humble
cd ~/Ai_ws/Study/vrx_ws
export GZ_VERSION=garden
source /opt/ros/humble/setup.bash
colcon build --merge-install --cmake-args "-DBUILD_TESTING=OFF"
```

**验证修复**：`ldd install/lib/ros_gz_sim/create | grep gz-transport` 应显示
`libgz-transport12.so`（而非 `libignition-transport11.so`）。

### 三轮交叉验证结果（2026-07-10）

| 轮次 | 话题数 | 传感器 | TF | GPS | 推进器 | 船体移动 |
|------|--------|--------|-----|-----|--------|----------|
| 第1轮 | 36 ✅ | 全部就绪 ✅ | 正常 ✅ | 悉尼坐标 ✅ | 命令发送 ✅ | — |
| 第2轮 | 36 ✅ | 全部就绪 ✅ | 10+帧 ✅ | 悉尼坐标 ✅ | 命令发送 ✅ | — |
| 第3轮 | 36 ✅ | 全部就绪 ✅ | 正常 ✅ | Δ15m/10s ✅ | 全链路 ✅ | +15m北 +10m东 ✅ |

---

## 6. 修复方案（更新）

### 方案 A：直接使用官方 Launch（推荐，已验证）

```bash
# 终端 1：启动完整仿真（含所有桥接、TF、任务系统）
export GZ_VERSION=garden
source /opt/ros/humble/setup.bash
source ~/Ai_ws/Study/vrx_ws/install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

> ⚠️ **必须设置 `GZ_VERSION=garden`**，否则 ros_gz 会使用错误的传输层。

这会自动处理所有桥接、TF 广播、传感器数据。然后运行控制器：

```bash
# 终端 2：键盘遥操（推荐）
export GZ_VERSION=garden
source ~/Ai_ws/Study/vrx_ws/install/setup.bash
python3 /home/han/Ai_ws/Study/vrx_ws/virtual_joystick.py

# 或自动前进测试
python3 /home/han/Ai_ws/Study/vrx_ws/auto_pilot.py
```

### 方案 B：改进 vrx_start.sh

如果坚持用 vrx_start.sh（例如只需要手动控制），至少需要添加以下桥接：

```bash
# 在现有的 5 个推进器桥之后，添加传感器桥
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/model/wamv/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
  '/world/sydney_regatta/model/wamv/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' \
  '/wamv/sensors/imu/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU' \
  '/wamv/sensors/gps/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat' \
  '/wamv/sensors/lidars/lidar_wamv/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/wamv/sensors/lidars/lidar_wamv/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' \
  '/wamv/sensors/cameras/front_left_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/wamv/sensors/cameras/front_right_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/wamv/sensors/cameras/middle_right_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/vrx/task/info@ros_gz_interfaces/msg/ParamVec[gz.msgs.Param' \
  --ros-args -p use_sim_time:=true -p expand_gz_topic_names:=true &

# 启动 robot_state_publisher（发布 TF 树）
ros2 run robot_state_publisher robot_state_publisher \
  /home/han/Ai_ws/Study/vrx_ws/install/share/wamv_gazebo/urdf/wamv_gazebo.urdf.xacro \
  --ros-args -p use_sim_time:=true &

# 启动 pose_tf_broadcaster（传感器位姿 TF）
ros2 run vrx_ros pose_tf_broadcaster --ros-args -p use_sim_time:=true &
```

### 方案 C：编写自主控制器（长期目标）

需要新建一个 ROS 2 节点，包含：

```python
# 伪代码框架
class AutonomousUSV(Node):
    def __init__(self):
        # 订阅传感器
        self.imu_sub = Subscriber(Imu, '/wamv/sensors/imu/imu/data')
        self.gps_sub = Subscriber(NavSatFix, '/wamv/sensors/gps/gps/fix')
        self.lidar_sub = Subscriber(LaserScan, '/wamv/sensors/lidars/lidar_wamv/scan')

        # 发布推进器命令
        self.left_pub = Publisher(Float64, '/wamv/thrusters/left/thrust')
        self.right_pub = Publisher(Float64, '/wamv/thrusters/right/thrust')

        # PID 控制器
        self.heading_pid = PID(kp=..., ki=..., kd=...)
        self.speed_pid = PID(kp=..., ki=..., kd=...)

        # 航点管理
        self.waypoints = [(lat, lon), ...]
        self.current_wp_idx = 0
```

---

## 6. 明早教学大纲

### 第一课：环境和启动（15 分钟）

1. **理解三层架构**
   - Gazebo（物理仿真 + 渲染）
   - ros_gz_bridge（ROS ↔ Gazebo 通信桥梁）
   - ROS 2 节点（你的控制代码）

2. **正确启动流程**
   ```bash
   # 终端 1
   source /opt/ros/humble/setup.bash
   source ~/Ai_ws/Study/vrx_ws/install/setup.bash
   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
   ```

3. **验证一切正常**
   ```bash
   # 终端 2
   ros2 topic list          # 应该看到 40+ 个话题
   ros2 topic echo /wamv/sensors/gps/gps/fix  # 验证 GPS 数据
   ```

### 第二课：手动控制（15 分钟）

1. **运行 virtual_joystick.py**
   - 只用这一个脚本，不要同时运行其他控制器
   - WASD 控制，空格停止，Q 退出

2. **理解差速驱动**
   - 前进：左右推力相同
   - 转弯：左右推力不同（差速）
   - 推力安全范围：0 ~ 2000

3. **在 Gazebo GUI 中观察**
   - WAM-V 的运动
   - 浮标的漂浮
   - 海浪效果

### 第三课：话题和数据流（15 分钟）

1. **查看传感器数据**
   ```bash
   ros2 topic hz /wamv/sensors/imu/imu/data        # IMU 频率
   ros2 topic hz /wamv/sensors/gps/gps/fix          # GPS 频率
   ros2 topic hz /wamv/sensors/lidars/lidar_wamv/scan  # 激光雷达频率
   ros2 topic echo /wamv/sensors/imu/imu/data -n 1  # 看一条 IMU 数据
   ```

2. **理解话题方向**
   - `]` = ROS → Gazebo（执行器命令）
   - `[` = Gazebo → ROS（传感器数据）

3. **RQT 可视化**
   ```bash
   rqt_graph   # 看节点和话题连接图
   rqt_plot /wamv/sensors/imu/imu/data/angular_velocity/z  # IMU 陀螺仪实时曲线
   ```

### 第四课：问题排查（10 分钟）

1. **船不动？**
   - 检查是否多个控制器同时运行
   - 检查推力值是否在合理范围
   - `ros2 topic info /wamv/thrusters/left/thrust` 看发布者数量

2. **没有传感器数据？**
   - 确认用 `competition.launch.py` 而非 `vrx_start.sh`
   - `ros2 topic list | grep wamv` 确认话题存在
   - `ros2 topic hz /wamv/sensors/imu/imu/data` 确认有数据

3. **TF 错误？**
   - 确认 `robot_state_publisher` 在运行
   - `ros2 run tf2_tools view_frames` 生成 TF 树 PDF

---

## 7. 文件结构速查

```
vrx_ws/
├── vrx_start.sh              # 手动启动脚本（仅推进器控制）
├── virtual_joystick.py       # 键盘遥操（推荐，安全推力值）
├── auto_pilot.py             # 自动前进 30 秒测试
├── direct_controller.py      # 绕过 ROS 的键盘控制（推力超限）
├── thruster_test.py          # 推进器功能测试（推力超限）
├── readme.md                 # 安装笔记（不完整）
├── src/vrx-humble/           # 源码
│   ├── vrx_gz/               # 核心 Gazebo 包
│   │   ├── launch/           # launch 文件
│   │   ├── worlds/           # 世界文件
│   │   └── src/vrx_gz/       # bridge.py, bridges.py, payload_bridges.py
│   ├── vrx_ros/              # ROS 辅助节点
│   ├── vrx_urdf/             # URDF 相关
│   ├── vrx_gazebo/           # 模型资产
│   └── wamv_gazebo/          # WAM-V Gazebo 模板
└── install/                  # 编译安装目录
    └── share/                # 已安装的资源文件
```

---

## 8. 可用的世界文件

| 世界 | 文件名 | 任务类型 |
|------|--------|----------|
| 悉尼港湾（默认） | `sydney_regatta.sdf` | 自由航行/练习 |
| 停驻任务 | `stationkeeping_task.sdf` | 保持在目标位置 |
| 航路跟踪 | `follow_path_task.sdf` | 沿路径航行 |
| 导航任务 | `navigation_task.sdf` | 综合导航 |
| 航路寻找 | `wayfinding_task.sdf` | 按航路点航行 |
| 野生动物 | `wildlife_task.sdf` | 识别和报告动物 |
| 感知任务 | `perception_task.sdf` | 视觉感知 |
| 扫描对接 | `scan_dock_deliver_task.sdf` | 扫描-对接-交付 |
| 声学感知 | `acoustic_perception_task.sdf` | 声学信号处理 |
| 声学跟踪 | `acoustic_tracking_task.sdf` | 声学定位跟踪 |
| 体操竞技 | `gymkhana_task.sdf` | 综合竞技 |

---

## 9. 常用命令速查

```bash
# ===== 环境设置 =====
source /opt/ros/humble/setup.bash
source ~/Ai_ws/Study/vrx_ws/install/setup.bash

# ===== 启动仿真 =====
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

# ===== 话题调试 =====
ros2 topic list                          # 列出所有话题
ros2 topic list | grep wamv              # 只看 WAM-V 相关
ros2 topic hz /wamv/sensors/imu/imu/data # 检查频率
ros2 topic echo /wamv/sensors/gps/gps/fix -n 1  # 看一条数据
ros2 topic info /wamv/thrusters/left/thrust      # 查看发布者/订阅者

# ===== 节点调试 =====
ros2 node list                   # 列出所有节点
ros2 node info /wamv/bridge_node # 查看节点详情

# ===== TF 调试 =====
ros2 run tf2_tools view_frames   # 生成 TF 树 PDF
ros2 run tf2_ros tf2_echo base_link front_left_camera_link  # 查看具体 TF

# ===== Gazebo 调试 =====
gz topic -l                      # 列出 Gazebo 话题
gz topic -t /world/sydney_regatta/stats -n 1  # 仿真状态
gz service -l                    # 列出 Gazebo 服务

# ===== 进程管理 =====
pkill -f gz sim                  # 杀掉 Gazebo
pkill -f ros_gz_bridge           # 杀掉桥接
ps aux | grep -E "gz|ros"        # 查看相关进程
```

---

*文档生成工具：Claude Code | 审查日期：2026-07-10*
