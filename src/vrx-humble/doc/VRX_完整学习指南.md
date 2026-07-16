# VRX 项目完整学习指南

> **项目地址：** [https://github.com/osrf/vrx/tree/humble](https://github.com/osrf/vrx/tree/humble)
> **官方 Wiki：** [https://github.com/osrf/vrx/wiki](https://github.com/osrf/vrx/wiki)
> **适用版本：** ROS 2 Humble + Gazebo Garden/Gz Sim 7
> **预计学习时间：** 3-6 个月（每天 2-3 小时）

---

## 目录

1. [项目概述](#1-项目概述)
2. [学习路线图](#2-学习路线图)
3. [第一阶段：环境搭建（1-2周）](#3-第一阶段环境搭建1-2周)
4. [第二阶段：基础操作（2-3周）](#4-第二阶段基础操作2-3周)
5. [第三阶段：WAM-V 定制（2-3周）](#5-第三阶段wam-v-定制2-3周)
6. [第四阶段：环境与物理（2-3周）](#6-第四阶段环境与物理2-3周)
7. [第五阶段：竞赛任务（4-6周）](#7-第五阶段竞赛任务4-6周)
8. [第六阶段：自主控制（4-8周）](#8-第六阶段自主控制4-8周)
9. [第七阶段：高级优化（持续）](#9-第七阶段高级优化持续)
10. [项目源码深度解析](#10-项目源码深度解析)
11. [学习资源汇总](#11-学习资源汇总)
12. [常见问题解答](#12-常见问题解答)

---

## 1. 项目概述

### 1.1 什么是 VRX？

VRX（Virtual RobotX）是一个用于**无人水面艇（USV）自主性**设计、开发和评估的仿真平台。它基于：
- **ROS 2**（Robot Operating System 2）
- **Gazebo Sim**（机器人仿真器）
- **SDF**（Simulation Description Format）

### 1.2 项目目标

VRX 旨在：
- 提供真实的海洋环境仿真
- 支持 WAM-V（Wave Adaptive Modular Vessel）载具
- 实现多种竞赛任务（定点保持、寻路、感知等）
- 促进海事机器人研究和教育

### 1.3 核心组件

```
VRX 项目
├── vrx_gazebo/          # 主仿真包（世界、模型、评分插件）
├── usv_gazebo_plugins/  # USV 物理插件（推力、风、浮力）
├── wave_gazebo_plugins/ # 波浪物理插件
├── wamv_description/    # WAM-V 机器人描述（URDF/XACRO）
├── wamv_gazebo/         # WAM-V Gazebo 生成
└── vrx_gazebo_python/   # Python 工具
```

---

## 2. 学习路线图

### 2.1 学习阶段总览

| 阶段 | 内容 | 预计时间 | 难度 |
|------|------|----------|------|
| 第一阶段 | 环境搭建 | 1-2 周 | ⭐⭐ |
| 第二阶段 | 基础操作 | 2-3 周 | ⭐⭐ |
| 第三阶段 | WAM-V 定制 | 2-3 周 | ⭐⭐⭐ |
| 第四阶段 | 环境与物理 | 2-3 周 | ⭐⭐⭐ |
| 第五阶段 | 竞赛任务 | 4-6 周 | ⭐⭐⭐⭐ |
| 第六阶段 | 自主控制 | 4-8 周 | ⭐⭐⭐⭐⭐ |
| 第七阶段 | 高级优化 | 持续 | ⭐⭐⭐⭐⭐ |

### 2.2 前置知识要求

**必备知识：**
- Linux 基础命令行操作
- Python 或 C++ 编程基础
- 基本的机器人学概念

**推荐知识：**
- ROS 2 基础（话题、服务、节点）
- 3D 几何和坐标变换
- 控制理论基础（PID 控制）

---

## 3. 第一阶段：环境搭建（1-2周）

### 3.1 目标
- 成功安装 VRX 环境
- 能够运行第一个仿真世界
- 理解基本的项目结构

### 3.2 学习步骤

#### 步骤 1：系统准备（2-3小时）

**硬件要求：**
- CPU：现代多核处理器（Intel i5 或同等）
- 内存：8GB 以上（推荐 16GB）
- 显卡：Nvidia 显卡（推荐 RTX 3060 或更高）
- 存储：50GB 可用空间

**软件要求：**
```bash
# Ubuntu 22.04 LTS
lsb_release -a

# 检查显卡驱动
nvidia-smi
```

#### 步骤 2：安装 ROS 2 Humble（2-3小时）

```bash
# 设置 sources.list
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 设置环境
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 步骤 3：安装 Gazebo Sim（1-2小时）

```bash
# 添加 Gazebo 仓库
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update

# 安装 Gazebo Sim 7
sudo apt-get install gz-sim7
```

#### 步骤 4：克隆并编译 VRX（1-2小时）

```bash
# 创建工作空间
mkdir -p ~/vrx_ws/src
cd ~/vrx_ws/src

# 克隆 VRX 仓库
git clone https://github.com/osrf/vrx.git -b humble
cd vrx

# 安装依赖
cd ~/vrx_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install

# 设置环境
echo "source ~/vrx_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 步骤 5：测试安装（30分钟）

```bash
# 运行空世界测试
ros2 launch vrx_gazebo vrx_empty.launch.py
```

**成功标准：**
- Gazebo 窗口正常打开
- 能看到海洋环境
- 没有错误信息

### 3.3 学习资源

- [VRX 安装教程](https://github.com/osrf/vrx/wiki/getting_started_tutorial)
- [ROS 2 Humble 安装指南](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo Sim 安装指南](https://gazebosim.org/docs/garden/install_ubuntu)

### 3.4 阶段检查点

- [ ] ROS 2 Humble 安装成功
- [ ] Gazebo Sim 7 安装成功
- [ ] VRX 工作空间编译成功
- [ ] 能够运行空世界仿真

---

## 4. 第二阶段：基础操作（2-3周）

### 4.1 目标
- 学会驾驶 WAM-V
- 理解传感器数据
- 掌握 RViz 可视化

### 4.2 学习步骤

#### 步骤 1：启动完整环境（1小时）

```bash
# 启动悉尼赛艇中心世界
ros2 launch vrx_gazebo vrx.launch.py world:=sydneyregatta
```

**观察内容：**
- WAM-V 载具外观
- 海洋环境（波浪、天空）
- 周围的浮标和障碍物

#### 步骤 2：手动驾驶 WAM-V（3-5小时）

```bash
# 启动手柄控制
ros2 launch vrx_gazebo usv_joy.launch.py
```

**手柄控制说明：**
- `L1` 按钮：紧急停止开关（必须持续按住）
- 左摇杆：左推进器控制
- 右摇杆：右推进器控制
- `A` 按钮：球发射器

**练习任务：**
1. 直线前进和后退
2. 原地旋转
3. 8 字形行驶
4. 靠近浮标并保持距离

#### 步骤 3：理解传感器数据（2-3小时）

**主要传感器话题：**
```bash
# 查看所有话题
ros2 topic list

# GPS 数据
ros2 topic echo /wamv/sensors/gps/navsat/navsat

# IMU 数据
ros2 topic echo /wamv/sensors/imu/imu

# 激光雷达数据
ros2 topic echo /wamv/sensors/lidar/scan

# 相机数据
ros2 topic echo /wamv/sensors/camera/front_left_camera/image
```

#### 步骤 4：使用 RViz 可视化（2-3小时）

```bash
# 启动 RViz
ros2 launch vrx_gazebo rviz.launch.py
```

**RViz 配置：**
1. 添加 RobotModel 显示 WAM-V
2. 添加 LaserScan 显示激光雷达
3. 添加 Image 显示相机画面
4. 添加 Map 显示占用栅格地图
5. 添加 TF 显示坐标变换

#### 步骤 5：查看任务状态（1小时）

```bash
# 查看任务信息
ros2 topic echo /vrx/task/info
```

**任务状态字段：**
- `state`：任务状态（ready/running/finished）
- `score`：当前得分
- `time`：剩余时间

### 4.3 学习资源

- [驾驶 WAM-V 教程](https://github.com/osrf/vrx/wiki/teleop_tutorial)
- [推进器铰接教程](https://github.com/osrf/vrx/wiki/thruster_articulation_tutorial)
- [添加赛道元素教程](https://github.com/osrf/vrx/wiki/Adding-course-elements_tutorial)
- [RViz 可视化教程](https://github.com/osrf/vrx/wiki/rviz_tutorial)

### 4.4 阶段检查点

- [ ] 能够手动驾驶 WAM-V 完成基本动作
- [ ] 理解各传感器数据的含义
- [ ] 能够在 RViz 中可视化传感器数据
- [ ] 理解任务状态和评分机制

---

## 5. 第三阶段：WAM-V 定制（2-3周）

### 5.1 目标
- 学会自定义 WAM-V 配置
- 理解 URDF/XACRO 结构
- 掌握传感器和推进器配置

### 5.2 学习步骤

#### 步骤 1：理解 WAM-V 结构（3-4小时）

**目录结构：**
```
wamv_description/
├── urdf/
│   ├── wamv_base.urdf.xacro          # 基础船体
│   ├── thrusters/                     # 推进器 XACRO 宏
│   ├── sensors/                       # 传感器 XACRO 宏
│   │   ├── lidar/
│   │   ├── camera/
│   │   ├── gps/
│   │   └── imu/
│   └── components/                    # 结构组件
├── meshes/                            # 3D 网格文件
└── CMakeLists.txt
```

#### 步骤 2：使用默认配置（1-2小时）

```bash
# 使用默认配置启动
ros2 launch vrx_gazebo vrx.launch.py world:=sydneyregatta
```

**默认配置包含：**
- 3 个摄像头（前左、前右、后方）
- 1 个激光雷达
- 1 个 GPS
- 1 个 IMU
- 2 个推进器（H 型配置）

#### 步骤 3：创建空 WAM-V（2-3小时）

```bash
# 创建工作目录
mkdir -p ~/my_wamv
cd ~/my_wamv

# 创建空配置文件
touch empty_thruster_config.yaml
touch empty_component_config.yaml

# 生成空 WAM-V
ros2 launch vrx_gazebo generate_wamv.launch.py \
  component_yaml:=`pwd`/empty_component_config.yaml \
  thruster_yaml:=`pwd`/empty_thruster_config.yaml \
  wamv_target:=`pwd`/wamv_target.urdf \
  wamv_locked:=False

# 使用空 WAM-V 启动
ros2 launch vrx_gazebo vrx.launch.py world:=sydneyregatta urdf:=`pwd`/wamv_target.urdf
```

#### 步骤 4：自定义推进器配置（3-4小时）

**推进器配置文件示例（`thruster_config.yaml`）：**
```yaml
engine:
  - prefix: "left"
    position: "-2.373776 1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
  - prefix: "right"
    position: "-2.373776 -1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
  - prefix: "middle"
    position: "0 0 0.318237"
    orientation: "0.0 0.0 0.0"
```

**配置说明：**
- `prefix`：推进器名称前缀
- `position`：相对于 base_link 的位置 (x, y, z)
- `orientation`：相对于 base_link 的姿态 (roll, pitch, yaw)

#### 步骤 5：自定义传感器配置（3-4小时）

**传感器配置文件示例（`component_config.yaml`）：**
```yaml
wamv_camera:
    - name: front_left_camera
      visualize: False
      x: 0.75
      y: 0.1
      z: 1.5
      R: 0.0
      P: ${radians(15)}
      Y: 0.0
      post_Y: 0.0
    - name: front_right_camera
      visualize: False
      x: 0.75
      y: -0.1
      z: 1.5
      R: 0.0
      P: ${radians(15)}
      Y: 0.0
      post_Y: 0.0

wamv_gps:
    - name: gps_wamv
      x: -0.85
      y: 0.0
      z: 1.3
      R: 0.0
      P: 0.0
      Y: 0.0

wamv_imu:
    - name: imu_wamv
      x: 0.3
      y: -0.2
      z: 1.3
      R: 0.0
      P: 0.0
      Y: 0.0

lidar:
    - name: lidar_wamv
      type: 16_beam
      x: 0.7
      y: 0.0
      z: 1.8
      R: 0.0
      P: ${radians(8)}
      Y: 0.0
      post_Y: 0.0

wamv_ball_shooter:
    - name: ball_shooter
      x: 0.55
      y: -0.3
      z: 1.3
      pitch: ${radians(-20)}
      yaw: 0.0
```

#### 步骤 6：生成自定义 WAM-V（1-2小时）

```bash
# 使用自定义配置生成 WAM-V
ros2 launch vrx_gazebo generate_wamv.launch.py \
  component_yaml:=`pwd`/component_config.yaml \
  thruster_yaml:=`pwd`/thruster_config.yaml \
  wamv_target:=`pwd`/wamv_target.urdf \
  wamv_locked:=False

# 测试自定义 WAM-V
ros2 launch vrx_gazebo vrx.launch.py world:=sydneyregatta urdf:=`pwd`/wamv_target.urdf
```

### 5.3 学习资源

- [默认 WAM-V 配置教程](https://github.com/osrf/vrx/wiki/default_wamv_tutorial)
- [创建空 WAM-V 教程](https://github.com/osrf/vrx/wiki/empty_wamv_tutorial)
- [generate_wamv.launch.py 教程](https://github.com/osrf/vrx/wiki/generate_wamv_tutorial)
- [自定义推进器教程](https://github.com/osrf/vrx/wiki/custom_thrusters_tutorial)
- [自定义组件教程](https://github.com/osrf/vrx/wiki/custom_components_tutorial)

### 5.4 阶段检查点

- [ ] 理解 WAM-V 的 URDF/XACRO 结构
- [ ] 能够创建空的 WAM-V 配置
- [ ] 能够自定义推进器配置
- [ ] 能够自定义传感器配置
- [ ] 能够生成并测试自定义 WAM-V

---

## 6. 第四阶段：环境与物理（2-3周）

### 6.1 目标
- 理解水动力学模型
- 学会调整环境参数
- 掌握波浪、风、雾的配置

### 6.2 学习步骤

#### 步骤 1：理解水动力学（4-5小时）

**关键插件：**
- `SimpleHydrodynamics`：简化的水动力学模型
- `Surface`：浮力模型

**参数说明：**
```xml
<plugin filename="libSimpleHydrodynamics.so" name="vrx::SimpleHydrodynamics">
  <link_name>wamv/base_link</link_name>
  <!-- 附加质量 -->
  <xDotU>0.0</xDotU>
  <yDotV>0.0</yDotV>
  <nDotR>0.0</nDotR>
  <!-- 线性和二次阻力 -->
  <xU>100.0</xU>
  <xUU>150.0</xUU>
  <yV>100.0</yV>
  <yVV>100.0</yVV>
  <zW>500.0</zW>
  <kP>300.0</kP>
  <kPP>600.0</kPP>
  <mQ>900.0</mQ>
  <mQQ>900.0</mQQ>
  <nR>800.0</nR>
  <nRR>800.0</nRR>
</plugin>
```

**参数含义：**
- `xU`、`xUU`：纵荡方向的线性和非线性阻力
- `yV`、`yVV`：横荡方向的线性和非线性阻力
- `nR`、`nRR`：偏航方向的线性和非线性阻力

#### 步骤 2：调整风参数（2-3小时）

**风插件配置：**
```xml
<plugin filename="libUSVWind.so" name="vrx::USVWind">
  <wind_obj>
    <name>wamv</name>
    <link_name>wamv/base_link</link_name>
    <coeff_vector>.5 .5 .33</coeff_vector>
  </wind_obj>
  <!-- 风 -->
  <wind_direction>240</wind_direction>
  <wind_mean_velocity>0.0</wind_mean_velocity>
  <var_wind_gain_constants>0</var_wind_gain_constants>
  <var_wind_time_constants>2</var_wind_time_constants>
  <random_seed>10</random_seed>
  <update_rate>10</update_rate>
</plugin>
```

**参数说明：**
- `wind_direction`：风向（度）
- `wind_mean_velocity`：平均风速（m/s）
- `coeff_vector`：风阻系数

#### 步骤 3：调整波浪参数（2-3小时）

**波浪场配置：**
```xml
<plugin filename="libPublisherPlugin.so" name="vrx::PublisherPlugin">
  <message type="gz.msgs.Param" topic="/vrx/wavefield/parameters" every="2.0">
    params {
      key: "direction"
      value {
        type: DOUBLE
        double_value: 0.0
      }
    }
    params {
      key: "gain"
      value {
        type: DOUBLE
        double_value: 0.3
      }
    }
    params {
      key: "period"
      value {
        type: DOUBLE
        double_value: 6.0
      }
    }
    params {
      key: "steepness"
      value {
        type: DOUBLE
        double_value: 0.0
      }
    }
  </message>
</plugin>
```

**参数说明：**
- `direction`：波浪方向（度）
- `gain`：波浪增益（0-1）
- `period`：波浪周期（秒）
- `steepness`：波浪陡度

#### 步骤 4：调整雾参数（1-2小时）

**雾配置：**
```xml
<scene>
  <fog>
    <type>linear</type>
    <start>10</start>
    <end>100</end>
    <density>0.5</density>
    <color>0.5 0.5 0.5</color>
  </fog>
</scene>
```

#### 步骤 5：调整环境光（1-2小时）

**环境光配置：**
```xml
<scene>
  <ambient>0.5 0.5 0.5 1.0</ambient>
  <background>0.7 0.7 0.7 1.0</background>
  <shadows>true</shadows>
</scene>
```

### 6.3 学习资源

- [水动力学参数教程](https://github.com/osrf/vrx/wiki/hydrodynamic_params_tutorial)
- [自定义环境因素教程](https://github.com/osrf/vrx/wiki/env_params_tutorial)
- [调整风参数教程](https://github.com/osrf/vrx/wiki/wind_params_tutorial)
- [调整波浪参数教程](https://github.com/osrf/vrx/wiki/wave_params_tutorial)
- [调整雾参数教程](https://github.com/osrf/vrx/wiki/fog_params_tutorial)
- [调整环境光教程](https://github.com/osrf/vrx/wiki/ambient_params_tutorial)

### 6.4 阶段检查点

- [ ] 理解水动力学模型的基本原理
- [ ] 能够调整风参数并观察效果
- [ ] 能够调整波浪参数并观察效果
- [ ] 能够调整雾和环境光参数
- [ ] 理解环境参数对载具行为的影响

---

## 7. 第五阶段：竞赛任务（4-6周）

### 7.1 目标
- 掌握所有 8 个竞赛任务
- 理解评分机制
- 能够手动完成基本任务

### 7.2 竞赛任务详解

#### 任务 1：定点保持（Stationkeeping）（3-4天）

**任务描述：**
导航到目标位姿并保持位置。

**评分标准：**
- 位置误差越小，得分越高
- 姿态误差越小，得分越高
- 时间越短，得分越高

**启动命令：**
```bash
ros2 launch vrx_gazebo vrx.launch.py world:=stationkeeping_task
```

**关键话题：**
```bash
# 任务状态
ros2 topic echo /vrx/task/info

# 目标位姿
ros2 topic echo /vrx/stationkeeping/goal
```

**实现思路：**
1. 订阅 GPS 获取当前位置
2. 订阅 IMU 获取当前姿态
3. 计算与目标的误差
4. 使用 PID 控制器调整推力

#### 任务 2：寻路（Wayfinding）（3-4天）

**任务描述：**
按顺序访问一系列航点（浮标）。

**评分标准：**
- 访问的航点数量
- 访问顺序的正确性
- 完成时间

**启动命令：**
```bash
ros2 launch vrx_gazebo vrx.launch.py world:=wayfinding_task
```

**关键话题：**
```bash
# 航点列表
ros2 topic echo /vrx/wayfinding/waypoints

# 当前航点
ros2 topic echo /vrx/wayfinding/current_waypoint
```

**实现思路：**
1. 获取航点列表
2. 计算到当前航点的方向
3. 控制载具朝向航点行驶
4. 到达后切换到下一个航点

#### 任务 3：感知（Perception）（4-5天）

**任务描述：**
识别和分类环境中的浮标/形状。

**评分标准：**
- 识别的正确性
- 分类的准确性
- 完成时间

**启动命令：**
```bash
ros2 launch vrx_gazebo vrx.launch.py world:=perception_task
```

**关键话题：**
```bash
# 相机图像
ros2 topic echo /wamv/sensors/camera/front_left_camera/image

# 感知结果
ros2 topic echo /vrx/perception/results
```

**实现思路：**
1. 订阅相机图像
2. 使用图像处理识别浮标
3. 分类浮标颜色和形状
4. 发布识别结果

#### 任务 4：声学感知（Acoustic Perception）（3-4天）

**任务描述：**
检测水下声学信标并报告位置。

**评分标准：**
- 位置估计的准确性
- 完成时间

**启动命令：**
```bash
ros2 launch vrx_gazebo vrx.launch.py world:=acoustic_perception_task
```

**关键话题：**
```bash
# 声学数据
ros2 topic echo /wamv/sensors/acoustics/receiver/range_bearing
```

#### 任务 5：野生动物遭遇与避让（Wildlife）（4-5天）

**任务描述：**
跟踪一组移动的动物并根据动物类型规划行动。

**评分标准：**
- 跟踪的准确性
- 避碰的成功率
- 行动的适当性

**启动命令：**
```bash
ros2 launch vrx_gazebo vrx.launch.py world:=wildlife_task
```

#### 任务 6：沿路径行驶（Follow the Path）（3-4天）

**任务描述：**
沿着由浮标标记的通道行驶，避免碰撞。

**评分标准：**
- 路径跟踪的准确性
- 碰撞次数
- 完成时间

**启动命令：**
```bash
ros2 launch vrx_gazebo vrx.launch.py world:=navigation_task
```

#### 任务 7：声学跟踪（Acoustic Tracking）（4-5天）

**任务描述：**
跟踪移动的水下声学信标，同时避开障碍物。

**评分标准：**
- 跟踪距离
- 碰撞次数
- 完成时间

**启动命令：**
```bash
ros2 launch vrx_gazebo vrx.launch.py world:=acoustic_tracking_task
```

#### 任务 8：扫描、对接与交付（Scan, Dock, Deliver）（5-7天）

**任务描述：**
检测对接口，执行对接操作，交付载荷。

**评分标准：**
- 扫描码的正确识别
- 对接的成功率
- 载荷交付的完成度

**启动命令：**
```bash
ros2 launch vrx_gazebo vrx.launch.py world:=scan_dock_deliver_task
```

### 7.3 通用 ROS 话题

```bash
# 任务状态
/vrx/task/info

# 任务状态字段
- state: ready/running/finished
- score: 当前得分
- time: 剩余时间
- num_collisions: 碰撞次数
```

### 7.4 学习资源

- [VRX 竞赛参与指南](https://github.com/osrf/vrx/wiki/vrx_2023-participation_overview)
- [任务教程](https://github.com/osrf/vrx/wiki/vrx_2023-task_tutorials)
- [定点保持任务](https://github.com/osrf/vrx/wiki/vrx_2023-stationkeeping_task)
- [寻路任务](https://github.com/osrf/vrx/wiki/vrx_2023-wayfinding_task)
- [感知任务](https://github.com/osrf/vrx/wiki/vrx_2023-perception_task)

### 7.5 阶段检查点

- [ ] 能够手动完成定点保持任务
- [ ] 能够手动完成寻路任务
- [ ] 理解所有任务的评分机制
- [ ] 能够监控任务状态和得分
- [ ] 能够在不同环境条件下运行任务

---

## 8. 第六阶段：自主控制（4-8周）

### 8.1 目标
- 开发自主控制算法
- 实现自动驾驶
- 集成感知和规划

### 8.2 学习步骤

#### 步骤 1：创建 ROS 2 包（2-3小时）

```bash
# 创建新的 ROS 2 包
cd ~/vrx_ws/src
ros2 pkg create --build-type ament_python my_vrx_controller

# 编辑 package.xml
cd my_vrx_controller
```

**package.xml 示例：**
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_vrx_controller</name>
  <version>0.0.1</version>
  <description>My VRX autonomous controller</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### 步骤 2：实现基础控制器（4-6小时）

**Python 节点示例：**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
import math

class StationkeepingController(Node):
    def __init__(self):
        super().__init__('stationkeeping_controller')
        
        # 订阅器
        self.gps_sub = self.create_subscription(
            NavSatFix, '/wamv/sensors/gps/navsat/navsat', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/wamv/sensors/imu/imu', self.imu_callback, 10)
        
        # 发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/wamv/cmd_vel', 10)
        
        # 目标位置
        self.target_lat = -33.724
        self.target_lon = 150.675
        
        # 当前位置
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_yaw = 0.0
        
        # PID 参数
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.1
        self.prev_error = 0.0
        self.integral = 0.0
        
        # 定时器
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Stationkeeping controller started')
    
    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
    
    def imu_callback(self, msg):
        # 从四元数计算偏航角
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def control_loop(self):
        # 计算距离误差
        lat_error = self.target_lat - self.current_lat
        lon_error = self.target_lon - self.current_lon
        distance = math.sqrt(lat_error**2 + lon_error**2) * 111000  # 转换为米
        
        # 计算角度误差
        target_angle = math.atan2(lon_error, lat_error)
        angle_error = target_angle - self.current_yaw
        
        # 归一化角度误差
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # PID 控制
        self.integral += distance
        derivative = distance - self.prev_error
        self.prev_error = distance
        
        # 发布控制命令
        cmd = Twist()
        if distance > 1.0:  # 距离阈值
            cmd.linear.x = self.kp * distance + self.ki * self.integral + self.kd * derivative
            cmd.angular.z = self.kp * angle_error
        
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Distance: {distance:.2f}m, Angle error: {math.degrees(angle_error):.2f}°')

def main(args=None):
    rclpy.init(args=args)
    node = StationkeepingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 步骤 3：实现寻路控制器（4-6小时）

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
import math

class WayfindingController(Node):
    def __init__(self):
        super().__init__('wayfinding_controller')
        
        # 订阅器
        self.gps_sub = self.create_subscription(
            NavSatFix, '/wamv/sensors/gps/navsat/navsat', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/wamv/sensors/imu/imu', self.imu_callback, 10)
        
        # 发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/wamv/cmd_vel', 10)
        
        # 航点列表
        self.waypoints = [
            (-33.724, 150.675),
            (-33.725, 150.676),
            (-33.726, 150.677),
        ]
        self.current_waypoint_idx = 0
        
        # 当前位置
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_yaw = 0.0
        
        # PID 参数
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.1
        self.prev_error = 0.0
        self.integral = 0.0
        
        # 定时器
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Wayfinding controller started')
    
    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
    
    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def control_loop(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!')
            return
        
        # 获取当前目标航点
        target_lat, target_lon = self.waypoints[self.current_waypoint_idx]
        
        # 计算距离误差
        lat_error = target_lat - self.current_lat
        lon_error = target_lon - self.current_lon
        distance = math.sqrt(lat_error**2 + lon_error**2) * 111000
        
        # 检查是否到达航点
        if distance < 2.0:  # 2米阈值
            self.current_waypoint_idx += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx}')
            return
        
        # 计算角度误差
        target_angle = math.atan2(lon_error, lat_error)
        angle_error = target_angle - self.current_yaw
        
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # PID 控制
        self.integral += distance
        derivative = distance - self.prev_error
        self.prev_error = distance
        
        cmd = Twist()
        cmd.linear.x = self.kp * distance + self.ki * self.integral + self.kd * derivative
        cmd.angular.z = self.kp * angle_error
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WayfindingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 步骤 4：实现感知模块（6-8小时）

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BuoyDetector(Node):
    def __init__(self):
        super().__init__('buoy_detector')
        
        self.bridge = CvBridge()
        
        # 订阅相机图像
        self.image_sub = self.create_subscription(
            Image, '/wamv/sensors/camera/front_left_camera/image', 
            self.image_callback, 10)
        
        # 发布检测结果
        self.detection_pub = self.create_publisher(
            Image, '/buoy_detection/image', 10)
        
        self.get_logger().info('Buoy detector started')
    
    def image_callback(self, msg):
        # 转换图像格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # 转换到 HSV 颜色空间
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 检测红色浮标
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        
        # 检测绿色浮标
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        
        # 查找轮廓
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 绘制检测结果
        for contour in contours_red:
            if cv2.contourArea(contour) > 100:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(cv_image, 'Red', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        for contour in contours_green:
            if cv2.contourArea(contour) > 100:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(cv_image, 'Green', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 发布结果
        result_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        self.detection_pub.publish(result_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BuoyDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 步骤 5：集成 Nav2 导航（8-12小时）

```bash
# 安装 Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# 创建 Nav2 配置文件
mkdir -p ~/vrx_ws/src/my_vrx_controller/config
```

**Nav2 配置示例：**
```yaml
# nav2_params.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_goal_updater_bt_node

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_dwb_controller/DWBLocalGoal"
      min_vel_x: 0.0
      max_vel_x: 0.5
      min_vel_theta: -1.0
      max_vel_theta: 1.0
```

### 8.3 学习资源

- [ROS 2 教程](https://docs.ros.org/en/humble/Tutorials.html)
- [Nav2 文档](https://navigation.ros.org/)
- [Gazebo 插件开发](https://gazebosim.org/api/sim/8/createsystemplugins.html)

### 8.4 阶段检查点

- [ ] 能够创建 ROS 2 包
- [ ] 能够实现基础的定点保持控制器
- [ ] 能够实现寻路控制器
- [ ] 能够实现基本的感知模块
- [ ] 能够集成 Nav2 进行自主导航

---

## 9. 第七阶段：高级优化（持续）

### 9.1 目标
- 优化控制算法
- 提高竞赛成绩
- 探索高级功能

### 9.2 优化方向

#### 方向 1：控制算法优化（持续）

**PID 参数调优：**
```python
# 使用 Ziegler-Nichols 方法
# 1. 将 Ki 和 Kd 设为 0
# 2. 逐渐增加 Kp 直到系统振荡
# 3. 记录临界增益 Ku 和振荡周期 Tu
# 4. 计算 PID 参数：
#    Kp = 0.6 * Ku
#    Ki = 2 * Kp / Tu
#    Kd = Kp * Tu / 8
```

**模型预测控制（MPC）：**
- 使用 MPC 替代 PID
- 预测未来状态
- 优化控制输入

#### 方向 2：感知优化（持续）

**深度学习目标检测：**
```python
# 使用 YOLO 或 SSD
import torch
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
results = model(cv_image)
```

**语义分割：**
- 识别水面、天空、障碍物
- 使用 U-Net 或 DeepLab

#### 方向 3：路径规划优化（持续）

**A* 算法：**
```python
import heapq

def a_star(grid, start, goal):
    # 实现 A* 算法
    pass
```

**RRT（快速随机树）：**
```python
def rrt(start, goal, obstacles):
    # 实现 RRT 算法
    pass
```

#### 方向 4：仿真优化（持续）

**参数辨识：**
- 识别真实的水动力学参数
- 使用系统辨识方法

**硬件在环仿真：**
- 连接真实传感器
- 测试实际控制器

### 9.3 学习资源

- [控制理论教程](https://en.wikipedia.org/wiki/Control_theory)
- [ROS 2 高级教程](https://docs.ros.org/en/humble/Tutorials/Advanced.html)
- [Gazebo 高级功能](https://gazebosim.org/api/sim/8/)

### 9.4 阶段检查点

- [ ] 能够优化 PID 参数
- [ ] 能够实现 MPC 控制器
- [ ] 能够使用深度学习进行目标检测
- [ ] 能够实现高级路径规划算法
- [ ] 能够进行参数辨识和系统优化

---

## 10. 项目源码深度解析

### 10.1 核心源码结构

```
vrx/
├── vrx_gazebo/
│   ├── src/
│   │   ├── scoring_plugin/
│   │   │   ├── ScoringPlugin.cc      # 评分基类
│   │   │   └── ScoringPlugin.hh
│   │   ├── scoring_plugins/
│   │   │   ├── StationKeepingScoringPlugin.cc
│   │   │   ├── WayfindingScoringPlugin.cc
│   │   │   └── ...
│   │   └── plugins/
│   │       └── ...
│   ├── launch/
│   ├── worlds/
│   └── models/
├── usv_gazebo_plugins/
│   ├── src/
│   │   ├── usv_gazebo_thrust_plugin.cc
│   │   ├── usv_gazebo_wind_plugin.cc
│   │   └── ...
│   └── include/
└── wamv_description/
    ├── urdf/
    └── meshes/
```

### 10.2 关键源码解析

#### ScoringPlugin.cc（评分基类）

**主要功能：**
- 管理任务计时器（ready/running/finished）
- 累计得分
- 发布 ROS 话题

**关键方法：**
```cpp
class ScoringPlugin : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
    // 配置插件
    void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm, EventManager &_eventMgr) override;
    
    // 预更新循环
    void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;
    
protected:
    // 任务状态
    enum class TaskState { READY, RUNNING, FINISHED };
    
    // 得分
    double score_ = 0.0;
    
    // 计时器
    double ready_time_ = 0.0;
    double running_time_ = 0.0;
};
```

#### usv_gazebo_thrust_plugin.cc（推力插件）

**主要功能：**
- 模拟推力器产生的力和力矩
- 支持差动驱动
- 支持推力器角度控制

**关键参数：**
```xml
<plugin filename="libusv_gazebo_thrust_plugin.so" name="usv_gazebo_thrust_plugin">
  <left_propeller_link>wamv/left_propeller_link</left_propeller_link>
  <right_propeller_link>wamv/right_propeller_link</right_propeller_link>
  <left_engine_link>wamv/left_engine_link</left_engine_link>
  <right_engine_link>wamv/right_engine_link</right_engine_link>
  <thrust_coefficient>0.004422</thrust_coefficient>
  <fluid_density>1000</fluid_density>
  <propeller_diameter>0.2</propeller_diameter>
</plugin>
```

### 10.3 编译和调试

```bash
# 编译特定包
colcon build --packages-select vrx_gazebo

# 编译并测试
colcon build --symlink-install
colcon test --packages-select vrx_gazebo

# 查看编译输出
colcon build --event-handlers console_direct+
```

---

## 11. 学习资源汇总

### 11.1 官方文档

- [VRX 官方文档](https://vrx.readthedocs.io)
- [VRX GitHub Wiki](https://github.com/osrf/vrx/wiki)
- [ROS 2 官方文档](https://docs.ros.org/en/humble/)
- [Gazebo Sim 文档](https://gazebosim.org/docs/)

### 11.2 教程和课程

- [ROS 2 教程](https://docs.ros.org/en/humble/Tutorials.html)
- [Gazebo 教程](https://gazebosim.org/tutorials)
- [Nav2 教程](https://navigation.ros.org/tutorials/)

### 11.3 论文和报告

- [VRX 论文](https://ieeexplore.ieee.org/document/8962724)
- [RobotX 竞赛](https://robotx.org/)

### 11.4 社区资源

- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)
- [Gazebo 社区](https://community.gazebosim.org/)

---

## 12. 常见问题解答

### 12.1 安装问题

**Q: 编译时找不到依赖包怎么办？**
```bash
# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 如果还不行，手动安装
sudo apt install ros-humble-<package_name>
```

**Q: Gazebo 启动时崩溃怎么办？**
```bash
# 检查显卡驱动
nvidia-smi

# 设置环境变量
export MESA_GL_VERSION_OVERRIDE=3.3
```

### 12.2 运行问题

**Q: WAM-V 不动怎么办？**
```bash
# 检查推进器话题
ros2 topic list | grep thrust

# 手动发布测试命令
ros2 topic pub /wamv/thrusters/left/thrust std_msgs/Float64 "data: 100.0"
```

**Q: 传感器数据没有怎么办？**
```bash
# 检查传感器话题
ros2 topic list | grep sensor

# 查看话题信息
ros2 topic info /wamv/sensors/gps/navsat/navsat
```

### 12.3 开发问题

**Q: 如何添加自定义插件？**
1. 创建插件源码
2. 在 CMakeLists.txt 中添加编译规则
3. 在 SDF 文件中引用插件

**Q: 如何调试 ROS 2 节点？**
```bash
# 查看节点列表
ros2 node list

# 查看节点信息
ros2 node info /my_node

# 查看话题数据
ros2 topic echo /my_topic
```

---

## 附录：学习计划模板

### 每周学习计划

| 日期 | 内容 | 时间 | 完成情况 |
|------|------|------|----------|
| 周一 | | | |
| 周二 | | | |
| 周三 | | | |
| 周四 | | | |
| 周五 | | | |
| 周六 | | | |
| 周日 | | | |

### 月度学习计划

| 月份 | 阶段 | 目标 | 完成情况 |
|------|------|------|----------|
| 第 1 月 | | | |
| 第 2 月 | | | |
| 第 3 月 | | | |
| 第 4 月 | | | |
| 第 5 月 | | | |
| 第 6 月 | | | |

---

**文档版本：** v1.0
**最后更新：** 2026-07-09
**作者：** AI Assistant
