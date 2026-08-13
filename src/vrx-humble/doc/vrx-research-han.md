# VRX / Virtual RobotX 论文与开源项目检索清单

> 生成时间：2026-07-09  
> 主题：VRX / Virtual RobotX / WAM-V / USV Gazebo / Maritime RobotX 相关论文、下载链接、GitHub 开源项目链接。  
> 说明：Google Scholar 对自动化访问限制较多，因此本清单同时参考 IEEE Xplore、MDPI、Springer、arXiv、SCITEPRESS、Crossref/OpenAlex/Semantic Scholar 与 GitHub 检索结果。IEEE 论文部分可能需要机构权限；开放获取论文优先给出 PDF 直链。

---

## 1. VRX 核心论文与官方项目

VRX 官方仓库是 Virtual RobotX 仿真环境源码与文档主页，主要用于无人水面艇 USV 在海洋环境中的 Gazebo / ROS 仿真。官方推荐新用户使用 Gazebo Harmonic 与 ROS 2 Jazzy，并要求引用 OCEANS 2019 论文 *Toward Maritime Robotic Simulation in Gazebo*。

| 类型 | 名称 | 链接 |
|---|---|---|
| 核心论文 | **Toward Maritime Robotic Simulation in Gazebo**, OCEANS 2019 | [IEEE DOI](https://doi.org/10.23919/OCEANS40490.2019.8962724) / [IEEE Xplore](https://ieeexplore.ieee.org/document/8962724) / [ResearchGate PDF 页面](https://www.researchgate.net/publication/338729474_Toward_Maritime_Robotic_Simulation_in_Gazebo) |
| 官方源码 | **osrf/vrx** | [GitHub](https://github.com/osrf/vrx) / [ZIP 下载：jazzy 分支](https://github.com/osrf/vrx/archive/refs/heads/jazzy.zip) |

---

## 2. 基于 / 使用 / 引用 VRX 的论文清单

| 方向 | 论文 | 年份 | 下载 / DOI | 相关开源 |
|---|---:|---:|---|---|
| VRX 核心仿真 | **Toward Maritime Robotic Simulation in Gazebo** | 2019 | [IEEE DOI](https://doi.org/10.23919/OCEANS40490.2019.8962724) / [IEEE Xplore](https://ieeexplore.ieee.org/document/8962724) / [ResearchGate PDF 页面](https://www.researchgate.net/publication/338729474_Toward_Maritime_Robotic_Simulation_in_Gazebo) | [osrf/vrx](https://github.com/osrf/vrx) |
| 强化学习 / 运动控制 | **DRL-Based Motion Control for Unmanned Surface Vehicles with Environmental Disturbances** | 2023 | [IEEE DOI](https://doi.org/10.1109/ICUS58632.2023.10318284) / [IEEE Xplore](https://ieeexplore.ieee.org/document/10318284) | 暂未找到官方代码 |
| 强化学习 / 动态避障 | **Dynamic Obstacle Avoidance for USVs Using Cross-Domain Deep Reinforcement Learning and Neural Network Model Predictive Controller** | 2023 | [MDPI 页面](https://www.mdpi.com/1424-8220/23/7/3572) / [PDF](https://www.mdpi.com/1424-8220/23/7/3572/pdf) / [DOI](https://doi.org/10.3390/s23073572) | 暂未找到官方代码 |
| UAV-USV 协同 / 降落 | **Vision-Guided UAV Landing on a Swaying Ocean Platform in Simulation** | 2023 | [IEEE DOI](https://doi.org/10.1109/RCAR58764.2023.10249476) / [IEEE Xplore](https://ieeexplore.ieee.org/document/10249476) | 暂未找到官方代码 |
| COLREG / 多船仿真 / 路径规划验证 | **COLREG-Compliant Simulation Environment for Verifying USV Motion Planning Algorithms** | 2023 | [IEEE DOI](https://doi.org/10.1109/OCEANSLimerick52467.2023.10244676) / [IEEE Xplore](https://ieeexplore.ieee.org/document/10244676) | [FieldRoboticsLab/MultiVessel_Simulation](https://github.com/FieldRoboticsLab/MultiVessel_Simulation) |
| 海上风电场巡检 / ASV 应用 | **Multi-domain inspection of offshore wind farms using an autonomous surface vehicle** | 2021 | [Springer 页面](https://doi.org/10.1007/s42452-021-04451-5) / [PDF](https://link.springer.com/content/pdf/10.1007/s42452-021-04451-5.pdf) | 暂未找到官方代码 |
| WAM-V 自主导航 / GPMP | **A Fully-Autonomous Framework of Unmanned Surface Vehicles in Maritime Environments Using Gaussian Process Motion Planning** | 2022 arXiv / 2023 IEEE JOE | [arXiv 页面](https://arxiv.org/abs/2204.10826) / [PDF](https://arxiv.org/pdf/2204.10826) / [IEEE DOI](https://doi.org/10.1109/JOE.2022.3194165) | [jiaweimeng/wam-v-autopilot](https://github.com/jiaweimeng/wam-v-autopilot) |
| 3-DOF 模型辨识 / 控制 | **Identifying Kinetic Model Parameters and Implementing 3-DOF Control for a Dual-Thruster USV: A Case Study Using the VRX Simulation Environment** | 2024 | [SCITEPRESS DOI](https://doi.org/10.5220/0013010600003822) / [PDF](https://www.scitepress.org/Papers/2024/130106/130106.pdf) | 暂未找到官方代码 |
| VRX + Simulink / 运动控制 | **Construction of Simulation System for USV Motion Control and Design of Multi-Mode Controllers Based on VRX and Simulink** | 2025 | [MDPI 页面](https://www.mdpi.com/2076-3417/15/8/4213) / [PDF](https://www.mdpi.com/2076-3417/15/8/4213/pdf) / [DOI](https://doi.org/10.3390/app15084213) | 暂未找到官方代码 |
| VRX 扩展平台 / IAcquaBot | **Open-Access Simulation Platform and Motion Control Design for a Surface Robotic Vehicle in the VRX Environment** | 2025 | [MDPI DOI](https://doi.org/10.3390/robotics14100147) / [PDF](https://www.mdpi.com/2218-6581/14/10/147/pdf) | [BraJavSa/iacquabotsim](https://github.com/BraJavSa/iacquabotsim) |
| 路径保持 / APF | **PK-APF: Path-Keeping Algorithm for USVs Based on Artificial Potential Field** | 2022 | [MDPI 页面](https://www.mdpi.com/2076-3417/12/16/8201) / [PDF](https://www.mdpi.com/2076-3417/12/16/8201/pdf) / [DOI](https://doi.org/10.3390/app12168201) | 暂未找到官方代码 |
| 自主靠泊 / Gazebo USV | **Model Reference Adaptive Control-Based Autonomous Berthing of an Unmanned Surface Vehicle under Environmental Disturbance** | 2022 | [MDPI 页面](https://www.mdpi.com/2075-1702/10/4/244) / [PDF](https://www.mdpi.com/2075-1702/10/4/244/pdf) / [DOI](https://doi.org/10.3390/machines10040244) | 不是严格 VRX 官方项目，但与 Gazebo USV / 靠泊控制高度相关 |
| 强化学习奖励函数 / 碰撞避障 | **Navigating the Trade-Offs: A Quantitative Analysis of Reinforcement Learning Reward Functions for Autonomous Maritime Collision Avoidance** | 2025 | [MDPI DOI](https://doi.org/10.3390/jmse13122233) / [PDF](https://www.mdpi.com/2077-1312/13/12/2233/pdf) | 暂未找到官方代码 |

---

## 3. GitHub 上值得下载的 VRX / USV 开源项目

| 项目 | 作用 | 源码 | ZIP 下载 |
|---|---|---|---|
| **osrf/vrx** | 官方 VRX；ROS 2 Jazzy + Gazebo Harmonic 推荐入口 | [GitHub](https://github.com/osrf/vrx) | [jazzy.zip](https://github.com/osrf/vrx/archive/refs/heads/jazzy.zip) |
| **david-dorf/wamv_gz** | 简化版 WAM-V，ROS 2 Jazzy + Gazebo Harmonic，基于 VRX | [GitHub](https://github.com/david-dorf/wamv_gz) | [main.zip](https://github.com/david-dorf/wamv_gz/archive/refs/heads/main.zip) |
| **Tinker-Twins/SINGABOAT-VRX** | 2022 Virtual RobotX 参赛方案，含任务管理、控制、规划、感知 | [GitHub](https://github.com/Tinker-Twins/SINGABOAT-VRX) | [main.zip](https://github.com/Tinker-Twins/SINGABOAT-VRX/archive/refs/heads/main.zip) |
| **USYD-RowBot/usyd_vrx** | University of Sydney RowBot VRX 仓库 | [GitHub](https://github.com/USYD-RowBot/usyd_vrx) | [master.zip](https://github.com/USYD-RowBot/usyd_vrx/archive/refs/heads/master.zip) |
| **FieldRoboticsLab/MultiVessel_Simulation** | COLREG 多船仿真、WAM-V 多船场景、路径规划验证 | [GitHub](https://github.com/FieldRoboticsLab/MultiVessel_Simulation) | [main.zip](https://github.com/FieldRoboticsLab/MultiVessel_Simulation/archive/refs/heads/main.zip) |
| **jhlenes/usv_simulator** | ROS Melodic + Gazebo 9 USV 模拟器，从旧版 osrf/vrx fork | [GitHub](https://github.com/jhlenes/usv_simulator) | [master.zip](https://github.com/jhlenes/usv_simulator/archive/refs/heads/master.zip) |
| **srmainwaring/asv_wave_sim** | Gazebo 波浪与水面船舶仿真插件，可补充 VRX 海况建模 | [GitHub](https://github.com/srmainwaring/asv_wave_sim) | [master.zip](https://github.com/srmainwaring/asv_wave_sim/archive/refs/heads/master.zip) |
| **jiaweimeng/wam-v-autopilot** | WAM-V autopilot，基于 osrf/vrx 的 WAM-V 20 Gazebo 环境 | [GitHub](https://github.com/jiaweimeng/wam-v-autopilot) | [main.zip](https://github.com/jiaweimeng/wam-v-autopilot/archive/refs/heads/main.zip) |
| **BraJavSa/iacquabotsim** | IAcquaBot Sim；VRX fork，扩展 USV、传感器和控制器 | [GitHub](https://github.com/BraJavSa/iacquabotsim) | [main.zip](https://github.com/BraJavSa/iacquabotsim/archive/refs/heads/main.zip) |
| **kavindagehan/asv-autonomous-docking-and-path-tracking** | MATLAB/Simulink + ROS 2 + Gazebo/VRX 的 WAM-V 路径跟踪与靠泊 GNC 框架 | [GitHub](https://github.com/kavindagehan/asv-autonomous-docking-and-path-tracking) | [main.zip](https://github.com/kavindagehan/asv-autonomous-docking-and-path-tracking/archive/refs/heads/main.zip) |
| **ingeniarius-ltd/aquatic_simulator** | 基于 uuv_simulator 和 usv_vrx 的多机器人水域仿真测试床 | [GitHub](https://github.com/ingeniarius-ltd/aquatic_simulator) | [main.zip](https://github.com/ingeniarius-ltd/aquatic_simulator/archive/refs/heads/main.zip) |
| **OUXT-Polaris/ros_ship_packages** | ROS 船舶 / USV 仿真包，非纯 VRX 但对 USV Gazebo 研究有参考价值 | [GitHub](https://github.com/OUXT-Polaris/ros_ship_packages) | [master.zip](https://github.com/OUXT-Polaris/ros_ship_packages/archive/refs/heads/master.zip) |
| **OUXT-Polaris/vrx_packages** | Virtual Maritime RobotX Challenge 相关 ROS package | [GitHub](https://github.com/OUXT-Polaris/vrx_packages) | [master.zip](https://github.com/OUXT-Polaris/vrx_packages/archive/refs/heads/master.zip) |
| **wangzhao9562/vrx_nav_test** | WAM-V 在 ROS 环境下的建图与导航测试 | [GitHub](https://github.com/wangzhao9562/vrx_nav_test) | [master.zip](https://github.com/wangzhao9562/vrx_nav_test/archive/refs/heads/master.zip) |

---

## 4. 推荐下载 / 克隆顺序

### 4.1 官方 VRX

```bash
git clone https://github.com/osrf/vrx.git -b jazzy
```

### 4.2 轻量 WAM-V / ROS 2 Jazzy / Gazebo Harmonic

```bash
git clone https://github.com/david-dorf/wamv_gz.git
```

### 4.3 多船 / COLREG / 路径规划验证

```bash
git clone https://github.com/FieldRoboticsLab/MultiVessel_Simulation.git
```

### 4.4 WAM-V 自主导航与 GPMP

```bash
git clone https://github.com/jiaweimeng/wam-v-autopilot.git
```

### 4.5 Simulink / 路径跟踪 / 靠泊控制

```bash
git clone https://github.com/kavindagehan/asv-autonomous-docking-and-path-tracking.git
```

### 4.6 VRX 竞赛完整方案参考

```bash
git clone https://github.com/Tinker-Twins/SINGABOAT-VRX.git
```

### 4.7 波浪与水面船舶插件

```bash
git clone https://github.com/srmainwaring/asv_wave_sim.git
```

---

## 5. 后续研究关键词

可以继续用以下关键词在 Google Scholar / Semantic Scholar / IEEE / GitHub 中扩展检索：

- `Virtual RobotX`
- `VRX simulation USV`
- `WAM-V Gazebo ROS`
- `Maritime RobotX simulation`
- `USV COLREG Gazebo`
- `USV reinforcement learning VRX`
- `WAM-V autonomous docking`
- `USV motion control Simulink VRX`
- `surface vessel simulation Gazebo`
- `marine robotics Gazebo Harmonic ROS 2`

---

## 6. 备注

1. IEEE 论文通常无法直接公开 PDF 下载，需要学校 / 机构账号或作者预印本。
2. MDPI、Springer Open、arXiv、SCITEPRESS 中部分论文可直接下载 PDF。
3. GitHub ZIP 链接对应默认分支；如果项目后续默认分支变化，建议优先使用 GitHub 页面中的绿色 `Code` 按钮下载。
4. 对 VRX 新环境开发，建议优先使用 `osrf/vrx` 的 `jazzy` 分支；旧 ROS Melodic / Gazebo 9 项目更适合复现实验，不一定适合新项目。
