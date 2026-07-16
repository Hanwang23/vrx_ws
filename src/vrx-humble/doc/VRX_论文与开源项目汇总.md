# VRX 相关学术论文与开源项目汇总

> **整理时间：** 2026-07-09
> **基于项目：** VRX (Virtual RobotX) - [https://github.com/osrf/vrx](https://github.com/osrf/vrx)

---

## 目录

1. [核心论文](#1-核心论文)
2. [相关学术论文](#2-相关学术论文)
3. [开源仿真项目](#3-开源仿真项目)
4. [开源导航与控制项目](#4-开源导航与控制项目)
5. [开源感知与视觉项目](#5-开源感知与视觉项目)
6. [竞赛相关资源](#6-竞赛相关资源)
7. [学习资源](#7-学习资源)

---

## 1. 核心论文

### 1.1 VRX 项目奠基论文

| 论文标题 | 作者 | 发表时间 | 下载链接 |
|---------|------|---------|---------|
| **Toward Maritime Robotic Simulation in Gazebo** | Brian Bingham, Carlos Aguero, Michael McCarrin 等 | 2019 | [IEEE Xplore](https://ieeexplore.ieee.org/document/8962724) |

**摘要：**
本文介绍了在 Gazebo 仿真器中开发海事机器人仿真能力的工作。重点是为无人水面艇（USV）和其他海事机器人实现逼真的海洋环境仿真。

**引用格式：**
```bibtex
@InProceedings{bingham19toward,
  Title    = {Toward Maritime Robotic Simulation in Gazebo},
  Author   = {Brian Bingham and Carlos Aguero and Michael McCarrin and Joseph Klamo and Joshua Malia and Kevin Allen and Tyler Lum and Marshall Rawson and Rumman Waqar},
  Booktitle = {Proceedings of MTS/IEEE OCEANS Conference},
  Year     = {2019},
  Address  = {Seattle, WA},
  Month    = {October}
}
```

---

## 2. 相关学术论文

### 2.1 深度强化学习与 USV 导航

| 论文标题 | 作者 | 发表时间 | 下载链接 |
|---------|------|---------|---------|
| **Deep-Reinforcement-Learning-Based Motion Control for Unmanned Surface Vehicles with Environmental Disturbances** | - | 2023 | [IEEE Xplore](https://ieeexplore.ieee.org/document/10318284) |
| **Dynamic Obstacle Avoidance for USVs Using Cross-Domain Deep Reinforcement Learning and Neural Network Model Predictive Controller** | - | 2023 | [MDPI Sensors](https://www.mdpi.com/1424-8220/23/7/3572) |

**研究方向：**
- 基于 DQN、PPO、SAC、TD3 的 USV 运动控制
- 环境扰动下的自主导航
- 动态避障算法

### 2.2 无人机与水面艇协同

| 论文标题 | 作者 | 发表时间 | 下载链接 |
|---------|------|---------|---------|
| **Vision-Guided UAV Landing on a Swaying Ocean Platform in Simulation** | - | 2023 | [IEEE Xplore](https://ieeexplore.ieee.org/document/10249476) |

**研究方向：**
- UAV 在摇晃海洋平台上的视觉引导着陆
- 多域机器人协同操作

### 2.3 COLREGS 合规导航

| 论文标题 | 作者 | 发表时间 | 下载链接 |
|---------|------|---------|---------|
| **COLREG-Compliant Simulation Environment for Verifying USV Motion Planning Algorithms** | - | 2023 | [IEEE Xplore](https://ieeexplore.ieee.org/document/10244676) |
| **Multi-domain inspection of offshore wind farms using an autonomous surface vehicle** | - | 2021 | [Springer](https://link.springer.com/article/10.1007/s42452-021-04451-5) |

**研究方向：**
- COLREGS 国际海上避碰规则
- 多船避碰仿真
- 海上风电场巡检

### 2.4 其他相关论文

| 论文标题 | 关键词 | 搜索建议 |
|---------|--------|---------|
| Deep Reinforcement Learning for Autonomous Navigation of Unmanned Surface Vehicles: A Comprehensive Review | DRL、USV、综述 | Google Scholar 搜索 |
| Multi-objective Path Planning for USVs in Complex Waters | 路径规划、多目标优化 | IEEE Xplore 搜索 |
| Simulation-to-real Transfer for USV Navigation | Sim2Real、迁移学习 | arXiv 搜索 |
| Multi-USV Cooperative Navigation using MARL | 多智能体、强化学习 | Google Scholar 搜索 |

**推荐搜索网站：**
- [Google Scholar](https://scholar.google.com)
- [IEEE Xplore](https://ieeexplore.ieee.org)
- [arXiv](https://arxiv.org)
- [ResearchGate](https://www.researchgate.net)

---

## 3. 开源仿真项目

### 3.1 VRX (Virtual RobotX) - 官方项目

| 项目 | 链接 | 描述 |
|------|------|------|
| **VRX 主仓库** | [https://github.com/osrf/vrx](https://github.com/osrf/vrx) | VRX 仿真环境主仓库 |
| **VRX Humble 分支** | [https://github.com/osrf/vrx/tree/humble](https://github.com/osrf/vrx/tree/humble) | ROS 2 Humble 版本 |
| **VRX Wiki** | [https://github.com/osrf/vrx/wiki](https://github.com/osrf/vrx/wiki) | 官方文档和教程 |
| **VRX Docker** | [https://github.com/osrf/vrx-docker](https://github.com/osrf/vrx-docker) | Docker 竞赛环境 |

**特点：**
- 官方 RobotX 竞赛仿真平台
- 基于 Gazebo Sim 和 ROS 2
- 包含 8 个竞赛任务
- 支持 WAM-V 载具

### 3.2 UUV Simulator - 水下机器人仿真

| 项目 | 链接 | 描述 |
|------|------|------|
| **UUV Simulator** | [https://github.com/uuvsimulator/uuv_simulator](https://github.com/uuvsimulator/uuv_simulator) | 水下/水面机器人仿真框架 |

**特点：**
- Gazebo/ROS 集成
- 支持 UUV 和 USV 仿真
- 包含水动力学模型
- 支持多种传感器（声纳、DVL、IMU 等）
- 多机器人协同仿真

**主要组件：**
```
uuv_simulator/
├── uuv_descriptions/    # 载具模型（URDF/Xacro）
├── uuv_gazebo_plugins/  # Gazebo 物理插件
├── uuv_control/         # 控制器
├── uuv_world_plugins/   # 环境插件（水流、波浪）
└── uuv_sensor_plugins/  # 传感器插件
```

### 3.3 MultiVessel_Simulation - 多船避碰仿真

| 项目 | 链接 | 描述 |
|------|------|------|
| **MultiVessel_Simulation** | [https://github.com/FieldRoboticsLab/MultiVessel_Simulation](https://github.com/FieldRoboticsLab/MultiVessel_Simulation) | COLREGS 合规的多船避碰仿真 |

**特点：**
- 多船相遇场景仿真
- COLREGS 规则实现
- 避碰算法测试平台

### 3.4 基于 VRX 的二次开发项目（ROS 2 Humble）

#### 3.4.1 MARUS - 海洋自主机器人仿真器

| 项目 | 链接 | 描述 |
|------|------|------|
| **MARUS** | [https://github.com/marus-project/marus](https://github.com/marus-project/marus) | 海洋自主机器人仿真器 |

**特点：**
- 基于 ROS 2 和 Gazebo（Ignition/Harmonic）
- 支持 AUV、USV 等多种海洋机器人
- 提供水动力学仿真插件
- 支持传感器仿真（声纳、相机、IMU 等）

**适用场景：**
- 海洋机器人算法开发
- 水下/水面任务仿真
- 多机器人协同测试

#### 3.4.2 Stonefish - 海洋机器人仿真库

| 项目 | 链接 | 描述 |
|------|------|------|
| **Stonefish** | [https://github.com/PatrykCieslak/stonefish](https://github.com/PatrykCieslak/stonefish) | C++ 海洋机器人仿真库 |
| **Stonefish ROS2** | [https://github.com/PatrykCieslak/stonefish_ros2](https://github.com/PatrykCieslak/stonefish_ros2) | ROS 2 绑定 |

**特点：**
- 高保真水动力学仿真
- 支持 ROV、AUV 仿真
- ROS/ROS 2 集成
- 传感器仿真（声纳、相机、IMU 等）
- 逼真的海洋环境渲染

**适用场景：**
- 水下机器人研究
- 自主水下航行器算法测试
- 海洋环境感知

#### 3.4.3 ArduPilot + ROS 2 集成

| 项目 | 链接 | 描述 |
|------|------|------|
| **ArduPilot** | [https://github.com/ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot) | 开源自动驾驶仪 |
| **ArduPilot Gazebo** | [https://github.com/ArduPilot/ardupilot_gz](https://github.com/ArduPilot/ardupilot_gz) | Gazebo 集成 |
| **ArduPilot ROS** | [https://github.com/ArduPilot/ardupilot_ros](https://github.com/ArduPilot/ardupilot_ros) | ROS 2 绑定 |

**特点：**
- 支持多种载具类型（飞机、汽车、船只）
- ArduRover 用于 USV 控制
- SITL（软件在环）仿真
- ROS 2 集成
- 成熟的自动驾驶算法

**适用场景：**
- USV 自动驾驶开发
- 路径规划和导航
- 硬件在环测试

#### 3.4.4 基于 Nav2 的 USV 导航

| 项目 | 链接 | 描述 |
|------|------|------|
| **Nav2** | [https://github.com/ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) | ROS 2 导航框架 |
| **Nav2 文档** | [https://navigation.ros.org/](https://navigation.ros.org/) | 官方文档 |

**USV 适配要点：**
- 修改代价地图以适应水面环境
- 调整路径规划器以考虑洋流
- 适配控制器以处理水面动力学
- 集成 GPS 和 IMU 传感器

**适用场景：**
- 自主水面艇导航
- 航点跟踪
- 避障导航

#### 3.4.5 VRX 竞赛团队解决方案

| 项目 | 链接 | 描述 |
|------|------|------|
| **VRX 2023 解决方案示例** | [GitHub 搜索 "vrx 2023 solution"](https://github.com/search?q=vrx+2023+solution) | 竞赛团队开源代码 |
| **VRX 团队代码** | [GitHub 搜索 "vrx team autonomous"](https://github.com/search?q=vrx+team+autonomous) | 各团队自主控制代码 |

**典型实现：**
- PID 控制器实现
- 航点跟踪算法
- 浮标检测和分类
- 任务状态机

**学习价值：**
- 了解竞赛级解决方案
- 学习最佳实践
- 获取灵感和参考

#### 3.4.6 其他基于 VRX 的项目

| 项目 | 链接 | 描述 |
|------|------|------|
| **VRX Nav2 集成** | [GitHub 搜索 "vrx nav2"](https://github.com/search?q=vrx+nav2) | VRX 与 Nav2 集成示例 |
| **VRX 感知模块** | [GitHub 搜索 "vrx perception"](https://github.com/search?q=vrx+perception) | VRX 感知算法实现 |
| **VRX 控制器** | [GitHub 搜索 "vrx controller"](https://github.com/search?q=vrx+controller) | VRX 自定义控制器 |

**推荐搜索关键词：**
```
vrx ROS2 humble autonomous
vrx stationkeeping controller
vrx wayfinding navigation
vrx perception buoy detection
WAM-V autonomous control
```

### 3.5 其他仿真项目

| 项目 | 链接 | 描述 |
|------|------|------|
| **VORTEX-SIM** | [https://github.com/NTNU-Autonomous-Marine-Operations-Lab](https://github.com/NTNU-Autonomous-Marine-Operations-Lab) | NTNU 海洋机器人仿真框架 |
| **MBZIRC** | [GitHub 搜索 "MBZIRC"](https://github.com/search?q=MBZIRC) | Mohammed Bin Zayed 国际机器人挑战赛仿真 |
| **boat_simulator** | [GitHub 搜索 "boat simulator ROS"](https://github.com/search?q=boat+simulator+ROS) | 轻量级船艇仿真框架 |

---

## 4. 开源导航与控制项目

### 4.1 ROS 2 导航栈

| 项目 | 链接 | 描述 |
|------|------|------|
| **Nav2** | [https://github.com/ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) | ROS 2 导航框架 |
| **Navigation2 文档** | [https://navigation.ros.org/](https://navigation.ros.org/) | 官方文档 |

**特点：**
- 路径规划（NavFn、Smac、Theta*）
- 行为树控制
- 代价地图
- 恢复行为

### 4.2 自动驾驶算法

| 项目 | 链接 | 描述 |
|------|------|------|
| **Autoware** | [https://github.com/autowarefoundation/autoware](https://github.com/autowarefoundation/autoware) | 自动驾驶框架（可适配 USV） |
| **Autoware 文档** | [https://autoware.org/](https://autoware.org/) | 官方文档 |

### 4.3 路径规划算法

| 算法 | 链接 | 描述 |
|------|------|------|
| **A* 算法** | [Wikipedia](https://en.wikipedia.org/wiki/A*_search_algorithm) | 经典路径规划算法 |
| **RRT/RRT*** | [GitHub 搜索 "RRT ROS"](https://github.com/search?q=RRT+ROS) | 快速随机树算法 |
| **D* Lite** | [Wikipedia](https://en.wikipedia.org/wiki/D*) | 动态路径规划算法 |

---

## 5. 开源感知与视觉项目

### 5.1 目标检测

| 项目 | 链接 | 描述 |
|------|------|------|
| **YOLOv8** | [https://github.com/ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) | 实时目标检测 |
| **Detectron2** | [https://github.com/facebookresearch/detectron2](https://github.com/facebookresearch/detectron2) | Facebook 目标检测框架 |
| **MMDetection** | [https://github.com/open-mmlab/mmdetection](https://github.com/open-mmlab/mmdetection) | OpenMMLab 目标检测工具箱 |

### 5.2 语义分割

| 项目 | 链接 | 描述 |
|------|------|------|
| **Segment Anything** | [https://github.com/facebookresearch/segment-anything](https://github.com/facebookresearch/segment-anything) | Meta 通用分割模型 |
| **DeepLabV3** | [GitHub 搜索 "DeepLabV3 PyTorch"](https://github.com/search?q=DeepLabV3+PyTorch) | 语义分割模型 |

### 5.3 ROS 2 视觉包

| 项目 | 链接 | 描述 |
|------|------|------|
| **vision_opencv** | [https://github.com/ros-perception/vision_opencv](https://github.com/ros-perception/vision_opencv) | ROS 2 OpenCV 集成 |
| **image_transport_plugins** | [https://github.com/ros-perception/image_transport_plugins](https://github.com/ros-perception/image_transport_plugins) | 图像传输插件 |

---

## 6. 竞赛相关资源

### 6.1 RobotX 竞赛

| 资源 | 链接 | 描述 |
|------|------|------|
| **RobotX 官网** | [https://robotx.org/](https://robotx.org/) | RobotX 竞赛官方网站 |
| **VRX 竞赛** | [https://robotx.org/virtual-robotx/](https://robotx.org/virtual-robotx/) | 虚拟 RobotX 竞赛 |
| **RoboBoat 竞赛** | [https://robonation.org/programs/roboboat/](https://robonation.org/programs/roboboat/) | RoboBoat 竞赛 |

### 6.2 竞赛任务

| 任务 | 描述 | 相关论文 |
|------|------|---------|
| **定点保持** | 导航到目标位姿并保持位置 | PID 控制、MPC 控制 |
| **寻路** | 按顺序访问航点 | 路径规划算法 |
| **感知** | 识别和分类浮标 | 目标检测、图像分类 |
| **避碰** | 避免与障碍物碰撞 | COLREGS、DRL |

---

## 7. 学习资源

### 7.1 ROS 2 学习

| 资源 | 链接 | 描述 |
|------|------|------|
| **ROS 2 官方教程** | [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html) | ROS 2 Humble 官方教程 |
| **The Construct** | [https://www.theconstructsim.com/](https://www.theconstructsim.com/) | ROS 在线学习平台 |
| **ROS Discourse** | [https://discourse.ros.org/](https://discourse.ros.org/) | ROS 社区论坛 |

### 7.2 Gazebo 学习

| 资源 | 链接 | 描述 |
|------|------|------|
| **Gazebo 官方教程** | [https://gazebosim.org/tutorials](https://gazebosim.org/tutorials) | Gazebo 官方教程 |
| **Gazebo 社区** | [https://community.gazebosim.org/](https://community.gazebosim.org/) | Gazebo 社区论坛 |

### 7.3 控制理论

| 资源 | 链接 | 描述 |
|------|------|------|
| **Control Tutorials for MATLAB** | [https://ctms.engin.umich.edu/](https://ctms.engin.umich.edu/) | 控制理论教程 |
| **Underactuated Robotics** | [https://underactuated.mit.edu/](https://underactuated.mit.edu/) | MIT 欠驱动机器人课程 |

### 7.4 深度学习

| 资源 | 链接 | 描述 |
|------|------|------|
| **Deep Learning Specialization** | [https://www.coursera.org/specializations/deep-learning](https://www.coursera.org/specializations/deep-learning) | Coursera 深度学习课程 |
| **PyTorch Tutorials** | [https://pytorch.org/tutorials/](https://pytorch.org/tutorials/) | PyTorch 官方教程 |
| **Reinforcement Learning** | [https://www.deepmind.com/learning-resources](https://www.deepmind.com/learning-resources) | DeepMind 强化学习资源 |

---

## 附录：GitHub 搜索建议

### 搜索关键词

```
# USV 相关
USV simulation ROS Gazebo
unmanned surface vehicle autonomous navigation
autonomous boat simulation

# 海洋机器人
marine robot simulation
ocean robotics Gazebo
maritime autonomous systems

# 竞赛相关
RobotX simulation
VRX Virtual RobotX
autonomous boat competition

# 算法相关
path planning ROS2
obstacle avoidance USV
COLREGS navigation
```

### 推荐 GitHub 组织

| 组织 | 链接 | 描述 |
|------|------|------|
| **osrf** | [https://github.com/osrf](https://github.com/osrf) | Open Source Robotics Foundation |
| **ros-navigation** | [https://github.com/ros-navigation](https://github.com/ros-navigation) | ROS 导航栈 |
| **uuvsimulator** | [https://github.com/uuvsimulator](https://github.com/uuvsimulator) | UUV 仿真器 |
| **FieldRoboticsLab** | [https://github.com/FieldRoboticsLab](https://github.com/FieldRoboticsLab) | 野外机器人实验室 |

---

**文档版本：** v1.0
**最后更新：** 2026-07-09
**整理者：** AI Assistant
