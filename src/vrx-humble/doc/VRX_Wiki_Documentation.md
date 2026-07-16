# VRX Wiki Documentation
# VRX Wiki 文档

> **Source / 来源:** [https://github.com/osrf/vrx/wiki](https://github.com/osrf/vrx/wiki)
>
> This document is a compiled version of the VRX project wiki.
> The VRX Simulation Environment is an extensible framework dedicated to
> facilitating the design, development, and evaluation of
> uncrewed surface vessel (USV) autonomy.
>
> 本文档是 VRX 项目 Wiki 的编译版本。
> VRX 仿真环境是一个用于无人水面艇 (USV) 自主性设计、开发和评估的可扩展框架。

> **Note / 注意:** As of Release 3.0, the VRX Simulation Environment runs
> with **Gazebo Sim Harmonic** and **ROS 2 Jazzy** by default.
> This is the recommended starting point for new users.
>
> **注意：** 自 3.0 版本起，VRX 仿真环境默认使用 **Gazebo Sim Harmonic** 和 **ROS 2 Jazzy**。
> 这是新用户的推荐起点。

---

## Table of Contents / 目录

1. [VRX Wiki 首页](#vrx-wiki-home) — 介绍
2. [教程概览](#tutorials-overview) — 教程概览
3. [VRX 入门指南](#getting-started-with-vrx) — 入门指南
   - [选择安装方式](#choose-installation-method) — 选择安装方式
   - [准备系统环境](#prepare-your-system) — 准备系统环境
   - [Docker 容器准备](#alternative-prepare-your-docker-container) — Docker 容器准备
   - [安装 VRX](#installing-vrx) — 安装 VRX
   - [系统要求](#system-requirements) — 系统要求
4. [运行 VRX](#running-vrx) — 运行 VRX
   - [熟悉 VRX 环境](#getting-around-the-vrx-environment) — 熟悉 VRX 环境
5. [驾驶 WAM-V](#driving-the-wam-v) — 驾驶 WAM-V
   - [推进器铰接](#thruster-articulation) — 推进器铰接
6. [添加赛道元素](#adding-course-elements) — 添加赛道元素
   - [使用声学定位器](#using-the-acoustic-pinger) — 使用声学定位器
7. [RViz 可视化](#visualizing-with-rviz) — RViz 可视化
8. [自定义 WAM-V（初级）](#customizing-the-wam-v-beginner) — 自定义 WAM-V (初级)
   - [默认 WAM-V 配置](#using-the-default-wam-v-configuration) — 默认 WAM-V 配置
   - [创建空 WAM-V](#creating-an-empty-wam-v) — 创建空 WAM-V
   - [generate_wamv.launch.py](#generatewamvlaunchpy) — generate_wamv.launch.py
   - [自定义推进器配置](#customizing-the-thruster-configuration) — 自定义推进器配置
   - [自定义 WAM-V 组件](#customizing-wam-v-components) — 自定义 WAM-V 组件
9. [自定义 WAM-V（中级）](#customizing-the-wam-v-intermediate) — 自定义 WAM-V (中级)
   - [水动力参数](#hydrodynamic-parameters) — 水动力参数
   - [自定义环境因素](#customizing-environmental-factors) — 自定义环境因素
   - [调整风参数](#adjusting-wind-parameters) — 调整风参数
   - [调整波浪参数](#adjusting-wave-parameters) — 调整波浪参数
   - [调整雾参数](#adjusting-fog-parameters) — 调整雾参数
   - [调整环境光](#adjusting-ambient-light) — 调整环境光
10. [RoboBoat 教程概览](#roboboat-tutorials-overview) — RoboBoat 教程概览
   - [运行 RoboBoat 示例世界](#running-the-roboboat-example-world) — 运行 RoboBoat 示例世界
   - [遥控 RoboBoat 载具](#teleoperate-your-roboboat-vehicle) — 遥控 RoboBoat 车辆
   - [自定义 RoboBoat 载具](#customize-your-roboboat-vehicle) — 自定义 RoboBoat 车辆
11. [2023 VRX 竞赛参与指南](#vrx-competition-2023-how-to-participate) — 2023 VRX 竞赛参与指南
   - [第一阶段：Hello World](#phase-1-hello-world) — 第一阶段: Hello World
   - [第二阶段：彩排](#phase-2-dress-rehearsal) — 第二阶段: 彩排
   - [第二阶段练习世界](#phase-2-practice-worlds) — 第二阶段练习世界
   - [第三阶段：VRX 挑战赛](#phase-3-vrx-challenge) — 第三阶段: VRX 挑战赛
   - [提交流程](#submission-process) — 提交流程
   - [WAMV 合规性](#wamv-compliance) — WAMV 合规性
   - [验证](#validation) — 验证
   - [测试](#testing) — 测试
   - [如何练习](#how-to-practice) — 如何练习
      - [任务 1：定点保持](#task-1-stationkeeping)
      - [任务 2：寻路](#task-2-wayfinding)
      - [任务 3：感知](#task-3-perception)
      - [任务 4：声学感知](#task-4-acoustic-perception)
      - [任务 5：野生动物遭遇与避让](#task-5-wildlife-encounter-and-avoid)
      - [任务 6：沿路径行驶](#task-6-follow-the-path)
      - [任务 7：声学跟踪](#task-7-acoustic-tracking)
      - [任务 8：扫描、对接与交付](#task-8-scan-dock-and-deliver)
12. [使用 Docker 打包提交](#package-your-submission-with-docker) — Docker 打包提交
   - [VRX 参赛者 Docker 入门](#docker-orientation-for-vrx-competitors) — 参赛者 Docker 指南
   - [创建参赛者镜像](#creating-a-competitor-image) — 创建参赛镜像
   - [Docker 准备工作](#docker-preparation) — Docker 准备工作
   - [使用 docker commit 的交互式流程](#interactive-process-using-docker-commit) — 交互式 docker commit 流程
   - [使用 Dockerfile 的脚本化流程](#scripted-process-using-a-dockerfile) — Dockerfile 脚本化流程
   - [最小工作示例](#minimal-working-examples) — 最小工作示例
   - [试运行故障排除](#troubleshooting-trial-runs) — 试运行故障排除
13. [技术文档](#technical-documentation) — 技术文档
   - [平台概述](#platform-overview) — 平台概述
   - [坐标系约定](#frame-conventions) — 坐标系约定
   - [浮力插件文档](#buoyancy-plugin-documentation) — 浮力插件文档
   - [波浪场生成](#wavefield-generation) — 波场生成
   - [波浪场包络](#wavefield-envelope) — 波场包络
   - [Docker 开发](#docker-development) — Docker 开发
14. [VRX Classic（旧版）](#vrx-classic-legacy) — VRX 经典版 (旧版)
   - [VRX Classic 教程](#vrx-classic-tutorials) — VRX 经典版教程
15. [故障排除](#troubleshooting) — 故障排除
16. [贡献者](#contributors) — 贡献者

---


==============================================================================

# 第 1 章：VRX Wiki 首页
# 第 1 章: 介绍
==============================================================================

# 欢迎访问 VRX Wiki

本 Wiki 为 VRX 仿真环境提供技术文档和教程，以及 VRX 环境支持的虚拟活动和竞赛。

## VRX 仿真环境

多功能 VRX 仿真环境是一个可扩展框架，致力于促进无人水面艇 (USV) 自主性的设计、开发和评估。

## [教程](https://github.com/osrf/vrx/wiki/tutorials)

如何使用 VRX 仿真环境。

## 竞赛

VRX 最初是为了满足各类竞赛的需求而设计的，例如：

* [2023 Virtual RobotX (VRX) Competition](https://github.com/osrf/vrx/wiki/vrx_2023-participation_overview)
* [2022 Virtual RobotX (VRX) Competition](https://robotx.org/programs/vrx-competition-2022/)
* 2020 Virtual Ocean Robotics Challenge (VORC)
* [2019 Virtual RobotX (VRX) Competition](https://robotx.org/programs/2019-virtual-robotx-competition/)
* [2019 RobotX Interactive Forum Hackathon (PDF)](https://robonation.org/app/uploads/sites/2/2019/09/2019-RobotX-Interactive-Forum-2019-Program.pdf)

鉴于与 [RobotBoat](https://robonation.org/programs/roboboat/) 的相似性，团队已通过引入世界和模型来扩展环境，专门支持开发 [RoboBoat 解决方案](https://github.com/osrf/vrx/wiki/tutorials#roboboat) 的团队。

## 研究

VRX 仿真环境已被海事机器人研发社区广泛采用，不断演进以适应感知、学习和控制方面的进步，同时探索 USV 能力的新应用。有关我们 OCEANS 论文的更多详情，请参阅[引用](https://ieeexplore.ieee.org/document/8962724/citations?tabFilter=papers#citations)。

* [Deep-Reinforcement-Learning-Based Motion Control for Unmanned Surface Vehicles with Environmental Disturbances](https://ieeexplore.ieee.org/document/10318284)
* [Dynamic Obstacle Avoidance for USVs Using Cross-Domain Deep Reinforcement Learning and Neural Network Model Predictive Controller](https://www.mdpi.com/1424-8220/23/7/3572)
* [Vision-Guided UAV Landing on a Swaying Ocean Platform in Simulation](https://ieeexplore.ieee.org/document/10249476)
* [COLREG-Compliant Simulation Environment for Verifying USV Motion Planning Algorithms](https://ieeexplore.ieee.org/document/10244676) with corresponding [source code](https://github.com/FieldRoboticsLab/MultiVessel_Simulation)
* [Multi-domain inspection of offshore wind farms using an autonomous surface vehicle](https://link.springer.com/article/10.1007/s42452-021-04451-5)

自 3.0 版本起，VRX 仿真环境默认使用 Gazebo Sim Harmonic 和 ROS 2 Jazzy。这是新用户的推荐起点，以下链接的教程均假设使用此配置。

## VRX Classic

希望保持与 Gazebo Classic 和 ROS 1 兼容性的用户，可以通过构建此仓库的 `gazebo_classic` 分支来实现。

* [点击此处访问 VRX Classic 的 Wiki 页面。](https://github.com/osrf/vrx/wiki/VRX-Classic-Home)
* **重要提示**：我们已于 2023 年春季将 `gazebo_classic` 分支从官方支持分支转变为社区支持分支。

## 如何引用

如果您在工作中使用了 VRX 仿真，请引用我们的总结论文 "Toward Maritime Robotic Simulation in Gazebo"。

```
@InProceedings{bingham19toward,
  Title                    = {Toward Maritime Robotic Simulation in Gazebo},
  Author                   = {Brian Bingham and Carlos Aguero and Michael McCarrin and Joseph Klamo and Joshua Malia and Kevin Allen and Tyler Lum and Marshall Rawson and Rumman Waqar},
  Booktitle                = {Proceedings of MTS/IEEE OCEANS Conference},
  Year                     = {2019},
  Address                  = {Seattle, WA},
  Month                    = {October}
}
```


==============================================================================

# 第 2 章：教程概览
# 第 2 章: 教程概览
==============================================================================

我们建议按照顺序学习教程，因为每个教程都建立在前一个的基础上。完成这些教程后，您将熟悉 Gazebo、ROS 以及如何使用 VRX。

## VRX 仿真环境介绍

### [VRX 入门指南](https://github.com/osrf/vrx/wiki/getting_started_tutorial)

如何安装和运行 VRX 环境。

* [概览](https://github.com/osrf/vrx/wiki/getting_started_tutorial)
* [选择安装方式](https://github.com/osrf/vrx/wiki/installation_method_tutorial)
* [准备系统环境](https://github.com/osrf/vrx/wiki/preparing_system_tutorial)
* [Docker 容器准备](https://github.com/osrf/vrx/wiki/docker_install_tutorial)
* [安装 VRX](https://github.com/osrf/vrx/wiki/installation_tutorial)
* [运行 VRX](https://github.com/osrf/vrx/wiki/running_vrx_tutorial)

### [熟悉 VRX 环境](https://github.com/osrf/vrx/wiki/getting_around_tutorial)

一些与 VRX 交互的简单教程。

* [概览](https://github.com/osrf/vrx/wiki/getting_around_tutorial)
* [驾驶 WAM-V](https://github.com/osrf/vrx/wiki/teleop_tutorial)
* [推进器铰接](https://github.com/osrf/vrx/wiki/thruster_articulation_tutorial)\*
* [添加赛道元素](https://github.com/osrf/vrx/wiki/Adding-course-elements_tutorial)
* [使用声学定位器](https://github.com/osrf/vrx/wiki/Acoustic-pinger_tutorial)
* [RViz 可视化](https://github.com/osrf/vrx/wiki/rviz_tutorial)\*

### [自定义 WAM-V（初级）](https://github.com/osrf/vrx/wiki/customizing_wamv_beginner_tutorial)

如何创建自定义的推进器和组件配置。

* [概览](https://github.com/osrf/vrx/wiki/customizing_wamv_beginner_tutorial)
* [默认 WAM-V 配置](https://github.com/osrf/vrx/wiki/default_wamv_tutorial)
* [创建"空" WAM-V](https://github.com/osrf/vrx/wiki/empty_wamv_tutorial)
* [`generate_wamv.launch.py`](https://github.com/osrf/vrx/wiki/generate_wamv_tutorial)
* [自定义推进器配置](https://github.com/osrf/vrx/wiki/custom_thrusters_tutorial)
* [自定义 WAM-V 组件](https://github.com/osrf/vrx/wiki/custom_components_tutorial)

### [自定义 WAM-V（中级）](https://github.com/osrf/vrx/wiki/wamv_params_tutorial)

如何指定自定义动力学和推进特性（建设中）

* [WAM-V 水动力和推进特性](https://github.com/osrf/vrx/wiki/wamv_params_tutorial)

### [自定义环境因素](https://github.com/osrf/vrx/wiki/env_params_tutorial)

如何指定风和波浪行为、雾和环境光。

* [概览](https://github.com/osrf/vrx/wiki/env_params_tutorial)
* [调整风参数](https://github.com/osrf/vrx/wiki/wind_params_tutorial)
* [调整波浪参数](https://github.com/osrf/vrx/wiki/wave_params_tutorial)
* [调整雾参数](https://github.com/osrf/vrx/wiki/fog_params_tutorial)
* [调整环境光](https://github.com/osrf/vrx/wiki/ambient_params_tutorial)

## RoboBoat 教程

### [RoboBoat 概览](https://github.com/osrf/vrx/wiki/tutorials_roboboat)

如果您是 RoboBoat 用户，如何使用 VRX。

* [概览](https://github.com/osrf/vrx/wiki/tutorials_roboboat)
* [运行 RoboBoat 示例世界](https://github.com/osrf/vrx/wiki/running_roboboat_tutorial)
* [遥控您的载具](https://github.com/osrf/vrx/wiki/teleop_roboboat_tutorial)
* [自定义您的载具](https://github.com/osrf/vrx/wiki/roboboat_customizing_tutorial)

## VRX 竞赛 2023

### [如何参与](https://github.com/osrf/vrx/wiki/vrx_2023-participation_overview)

VRX 竞赛各阶段的说明和提交指南。

* [概览](https://github.com/osrf/vrx/wiki/vrx_2023-participation_overview)
* [第一阶段：Hello World](https://github.com/osrf/vrx/wiki/vrx_2023-phase1_hello_world)
* [第二阶段：彩排](https://github.com/osrf/vrx/wiki/vrx_2023-phase2_dress_rehearsal)
  + [第二阶段练习世界](https://github.com/osrf/vrx/wiki/vrx_2023-phase2_practice_worlds)
* [第三阶段：VRX 挑战赛](https://github.com/osrf/vrx/wiki/vrx_2023-phase3_challenge)
* [提交流程](https://github.com/osrf/vrx/wiki/vrx_2023-submission_process)
* [WAMV 合规性](https://github.com/osrf/vrx/wiki/wamv_compliance)
* [验证](https://github.com/osrf/vrx/wiki/vrx_2023-validation)
* [测试](https://github.com/osrf/vrx/wiki/vrx_2023-testing)

### [如何练习](https://github.com/osrf/vrx/wiki/vrx_2023-task_tutorials)

VRX 2023 竞赛任务分解。

* [概览](https://github.com/osrf/vrx/wiki/vrx_2023-task_tutorials)
* [任务 1：定点保持](https://github.com/osrf/vrx/wiki/vrx_2023-stationkeeping_task)
* [任务 2：寻路](https://github.com/osrf/vrx/wiki/vrx_2023-wayfinding_task)
* [任务 3：感知](https://github.com/osrf/vrx/wiki/vrx_2023-perception_task)
* [任务 4：声学感知](https://github.com/osrf/vrx/wiki/vrx_2023-acoustic_perception_task)
* [任务 5：野生动物遭遇与避让](https://github.com/osrf/vrx/wiki/vrx_2023-wildlife_task)
* [任务 6：沿路径行驶](https://github.com/osrf/vrx/wiki/vrx_2023-follow_the_path_task)
* [任务 7：声学跟踪](https://github.com/osrf/vrx/wiki/vrx_2023-acoustic_tracking_task)
* [任务 8：扫描、对接与交付](https://github.com/osrf/vrx/wiki/vrx_2023-scan_dock_deliver_task)

### [使用 Docker 打包提交](https://github.com/osrf/vrx/wiki/tutorials-vrx_competitor_image)

要参加 VRX 竞赛，团队必须创建一个运行其虚拟自主系统的 Docker 镜像。以下内容解释了如何创建、交互和调试参赛者镜像。

#### [VRX 参赛者 Docker 入门](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_orientation)

Docker 镜像和容器的简要说明，以及它们与 VRX 竞赛的关系。

#### [创建参赛者镜像](https://github.com/osrf/vrx/wiki/tutorials-vrx_make_competitor_image)

本系列教程解释了如何创建和交互参赛者镜像。

* [镜像创建概述](https://github.com/osrf/vrx/wiki/tutorials-vrx_make_competitor_image)
* [准备工作](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_setup)
* [使用 `docker commit` 的交互式流程](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_interactive)
* [使用 `Dockerfile` 的脚本化流程](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_scripted)
* [最小工作示例](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_mwes)

#### [试运行故障排除](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_troubleshooting)

本系列教程解释了如何测试和调试参赛者镜像。

* [故障排除概述](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_troubleshooting)
* [故障排除前提条件](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_before_trouble)
* [获取基本调试信息](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_debug_info)
* [检查正在运行的容器](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_examine)
* [手动运行您的容器](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_manual_run)
* [手动运行试运行](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_manual_trial)

## 过往 VRX 活动教程

### [VRX 竞赛 2022](https://github.com/osrf/vrx/wiki/vrx_2022-task_tutorials)

VRX 2022 任务的存档教程。

### [RobotX 交互论坛 2019](https://github.com/osrf/vrx/wiki/rxi_2019-overview)

2019 年在新加坡举办的黑客马拉松挑战赛概述和演练。

### [VRX 竞赛 2019](https://github.com/osrf/vrx/wiki/vrx_2019-task_tutorials)

VRX 2019 任务的存档教程。


==============================================================================

# 第 3 章：VRX 入门指南
# 第 3 章: 入门指南
==============================================================================

# 序列概览：

本教程序列介绍了 VRX 仿真环境的安装和运行步骤。这是所有其他教程的前提。开始前请确保查看[系统要求](https://github.com/osrf/vrx/wiki/system_requirements)。

## 目录

本序列包含以下教程：

* [选择安装方式](https://github.com/osrf/vrx/wiki/installation_method_tutorial)
* [准备系统环境](https://github.com/osrf/vrx/wiki/preparing_system_tutorial)
* [Docker 容器准备](https://github.com/osrf/vrx/wiki/docker_install_tutorial)
* [安装 VRX](https://github.com/osrf/vrx/wiki/installation_tutorial)
* [运行 VRX](https://github.com/osrf/vrx/wiki/running_vrx_tutorial)

| Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [选择安装方式](https://github.com/osrf/vrx/wiki/installation_method_tutorial) |
| --- | --- |


---


## 选择安装方式

# 选择安装方式。

要构建 VRX 软件，您需要一个安装了必要依赖项的开发环境（包括 ROS、Gazebo 和一些实用工具）。有两种方式可以实现：

### 选项 A：[主机安装（默认）](https://github.com/osrf/vrx/wiki/preparing_system_tutorial)

将您的主机设置为开发环境。

* 只要您能够安装[系统要求](https://github.com/osrf/vrx/wiki/system_requirements)中列出的特定版本的 Ubuntu/ROS/Gazebo，这就是最简单的配置。
* 这是默认方法。

### 选项 B：[VRX Docker 容器安装](https://github.com/osrf/vrx/wiki/docker_install_tutorial)

设置一个包含必要依赖项的 Docker 容器，并在容器内运行 VRX。

* 这需要先安装 Docker，然后将依赖项安装到 Docker 镜像中。
* 该镜像功能类似于轻量级虚拟机，允许您构建和运行 VRX。
* 此选项在概念上稍微复杂一些，但优势在于不会（大部分）影响您主机的配置。
* 如果您使用同一主机进行多个使用不同软件环境（例如不同 ROS 版本）的开发项目，或者只是不想在主机系统上安装软件包，请选择此选项。

| Back: [序列概览](https://github.com/osrf/vrx/wiki/getting_started_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next (default): [准备系统环境](https://github.com/osrf/vrx/wiki/preparing_system_tutorial) |
| --- | --- | --- |
|  |  | Next (docker): [准备 Docker 容器](https://github.com/osrf/vrx/wiki/docker_install_tutorial) |


---


## 准备系统环境

# 准备您的主机

本指南介绍如何在主机上安装构建和运行 VRX 所需的依赖项。

## 步骤 1：安装 ROS 2 Humble 和 Gazebo Garden。

按照提供的安装说明操作，完成后返回此处：

* [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)
* [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu)

## 步骤 2：安装额外依赖：

```
sudo apt install python3-sdformat14 ros-jazzy-xacro ros-jazzy-ros-gz-interfaces
```

| Back: [选择安装方式](https://github.com/osrf/vrx/wiki/installation_method_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [安装 VRX](https://github.com/osrf/vrx/wiki/installation_tutorial) |
| --- | --- | --- |


---


## Docker 容器准备

# 使用 Docker 容器安装 VRX

您不必直接在主机上安装 ROS 和其他依赖项，可以使用 Docker 容器来创建构建和运行 VRX 仿真平台所需的环境。

## 步骤 1：[安装依赖项](https://github.com/Field-Robotics-Lab/dockwater/wiki/Install-Dependencies)

按照链接页面中 dockwater Wiki 的说明准备系统以构建镜像。本教程将指导您安装以下工具：

* Docker：容器管理工具
* Nvidia-toolkit：Nvidia 的软件，用于从 Docker 镜像启用 GPU 支持。
* Rocker：Docker 包装器，帮助构建和运行 Docker 镜像，使其正确配置以适应您的本地硬件。

## 步骤 2：运行容器

假设使用默认配置，您应该可以使用以下命令运行：

```
rocker --pull --devices /dev/input/js0 --x11 --nvidia --user --home ghcr.io/osrf/vrx-devel:latest /bin/bash
```

Rocker 命令将自定义您的镜像。默认包含以下自定义项：

* 镜像将配置为与本地主机相同的用户名和 `uid`，并将挂载您的主目录，以便您可以使用本地主机文件系统中的文件。
* 它还将连接到您的 Nvidia GPU 和 X 窗口，以便您可以使用加速图形运行 Gazebo。
* 它将允许使用游戏手柄（如果存在）
* 有关可用的自定义项和选项的完整列表，请参阅 `rocker --help`

镜像自定义完成后，脚本将启动一个容器，您可以通过 bash shell 与其交互。

* 如果成功，您将拥有一个在容器内运行 bash shell 的终端（它可能看起来与您启动的终端非常相似，但提示符会更改为反映容器 ID）。
* 通过在此容器中工作，您将能够按照与基于主机的安装方法相同的说明安装和运行 VRX。

## 在 Docker 容器中工作的注意事项：

* 请注意，您对容器所做的更改默认不是持久的。相反，容器提供了一个运行时环境，负责处理从本地主目录挂载的代码编译和运行所需的依赖项。
* 如果您使用 Docker 为 VRX 竞赛准备提交内容，请参阅 [VRX Docker 镜像创建教程序列](https://github.com/osrf/vrx/wiki/tutorials-vrx_make_competitor_image)
* 我们强烈建议阅读 [Docker 概述](https://docs.docker.com/get-started/overview/) 以熟悉 Docker 的概念和架构。

| Back: [选择安装方式](https://github.com/osrf/vrx/wiki/installation_method_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [安装 VRX](https://github.com/osrf/vrx/wiki/installation_tutorial) |
| --- | --- | --- |


---


## 安装 VRX

设置好开发环境后，以下步骤将下载并构建 VRX：

1. 创建一个 colcon 工作空间并克隆 vrx 仓库

   ```
   mkdir -p ~/vrx_ws/src
   cd ~/vrx_ws/src
   git clone https://github.com/osrf/vrx.git
   ```
2. 源化您的 ROS 2 安装。

   ```
   source /opt/ros/jazzy/setup.bash
   ```
3. 构建工作空间

   ```
   cd ~/vrx_ws
   colcon build --merge-install
   ```

## 设置环境

现在您已经构建了仿真环境，在使用之前需要源化设置脚本。从工作空间根目录运行：

```
. install/setup.bash
```

请注意，通常在使用 ROS 和 ROS 2 工作空间时，您始终需要执行此步骤。忘记源化环境是新用户中最常见的错误之一。如果您是 ROS 2 工作空间的新手，您可能会觉得[这个教程](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)很有帮助。

| Back: [准备系统环境](https://github.com/osrf/vrx/wiki/preparing_system_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [运行 VRX](https://github.com/osrf/vrx/wiki/running_vrx_tutorial) |
| --- | --- | --- |


---


## 系统要求

以下要求适用于 VRX 2.0.0 及以上版本（用于 VRX 2023 竞赛）。有关旧版本，请参阅 [VRX Classic 的要求](https://github.com/osrf/vrx/wiki/system_requirements_classic)。

## 推荐系统配置

### 硬件

为了运行 VRX，我们推荐以下最低硬件配置：

* 现代多核 CPU，例如 Intel Core i5
* 8 GB 内存
* Nvidia 显卡，例如 Nvidia RTX 3060

虽然其他硬件配置也可能可行，但我们仅对满足上述规格的系统提供官方支持。

该系统可以在没有专用 GPU 的情况下运行，但有 GPU 访问权限时 Gazebo 仿真将运行得更快（应能实时运行）。没有 GPU 时，仿真可能运行得明显慢于实时。

### 软件

[系统设置教程](https://github.com/osrf/vrx/wiki/tutorials)提供了直接在主机计算机上或使用 Docker 容器安装此环境的说明。

VRX 软件支持以下软件环境：

* Ubuntu Desktop 24.04 Noble (64-bit)
* Gazebo Sim 8.0.0+ (Gazebo Harmonic)
* ROS 2 Jazzy

### 外设

我们还建议准备一个游戏手柄，以便在仿真世界中驾驶 WAM-V 时进行测试。在示例中，我们使用 Logitech F310（[walmart](https://www.walmart.com/ip/Logitech-F310-GamePad/16419686)）。


==============================================================================

# 第 4 章：运行 VRX
# 第 4 章: 运行 VRX
==============================================================================

本页面提供了环境的描述，并演示了如何在此世界中生成 USV。还假设您已经完成了[安装教程](https://github.com/osrf/vrx/wiki/installation_tutorial)，并熟悉运行 Gazebo 仿真器和 ROS 开发工具的基础知识。

## 悉尼赛艇环境

整个悉尼国际赛艇中心，2023 年 RobotX 竞赛场地。

![VRX](https://github.com/osrf/vrx/raw/main/images/sydney_regatta_gzsim.png)

## 生成位置

由于环境大小，这可能需要一些时间。首次启动此世界时，它还将从 [Fuel 上的 vrx 集合](https://app.ignitionrobotics.org/OpenRobotics/fuel/collections/vrx) 下载 3D 模型

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

| Back: [安装 VRX](https://github.com/osrf/vrx/wiki/installation_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) |
| --- | --- |


---


## 熟悉 VRX 环境

# 序列概览：

本序列通过一些简单教程介绍 VRX 仿真，包括与默认环境中包含的载具、赛道和传感器的交互。

## 目录

本序列包含以下教程：

* [驾驶 WAM-V](https://github.com/osrf/vrx/wiki/teleop_tutorial)
* [推进器铰接](https://github.com/osrf/vrx/wiki/thruster_articulation_tutorial)\*
* [添加赛道元素](https://github.com/osrf/vrx/wiki/Adding-course-elements_tutorial)
* [使用声学定位器](https://github.com/osrf/vrx/wiki/Acoustic-pinger_tutorial)
* [RViz 可视化](https://github.com/osrf/vrx/wiki/rviz_tutorial)\*

| Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [驾驶 WAM-V](https://github.com/osrf/vrx/wiki/teleop_tutorial) |
| --- | --- |


==============================================================================

# 第 5 章：驾驶 WAM-V
# 第 5 章: 驾驶 WAM-V
==============================================================================

# WAM-V 遥操作

本指南介绍如何使用游戏手柄手动控制（遥操作）WAM-V。

## 启动仿真

我们首先使用以下命令启动仿真：

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

这将启动一个包含 WAM-V 的环境，两个铰接式后推进器，由以下 ROS 2 话题控制：

```
/wamv/thrusters/left/pos
/wamv/thrusters/left/thrust
/wamv/thrusters/right/pos
/wamv/thrusters/right/thrust
```

**注意**：需要进行一些自定义才能支持其他推进器配置的遥操作。

安装以下依赖项：

```
sudo apt install ros-jazzy-joy-teleop
```

要使用游戏手柄启用遥操作，请运行：

```
ros2 launch vrx_gz usv_joy_teleop.py
```

游戏手柄直接驱动左右推进器 - 类似于[差速驱动轮式机器人](https://en.wikipedia.org/wiki/Differential_wheeled_robot)。

* `L1` 按钮需要始终按住，同时配合以下命令（死人开关），
* 左摇杆上下轴（轴 1）映射到左推进器前进/后退，
* 右摇杆上下轴（轴 3）映射到右推进器前进/后退，
* 左摇杆左右轴（轴 2）映射到左推进器顺时针/逆时针旋转，
* 右摇杆左右轴（轴 4）映射到右推进器顺时针/逆时针旋转。

因此，将两个摇杆同时向前推应该会使 WAM-V 向前行驶。

您可以使用 `A` 按钮激活球发射器。

### 日志itech F310 注意事项：

如果您使用默认配置：

* 确保 Mode 指示灯未亮。
* 控制器底部的开关必须设置为 "D"，而不是 "X"。

| Back: [序列概览](https://github.com/osrf/vrx/wiki/getting_around_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [推进器铰接](https://github.com/osrf/vrx/wiki/thruster_articulation_tutorial) |
| --- | --- | --- |


---


## 推进器铰接

## 描述

VRX 仿真器支持旋转推进器以改变推力方向的功能。此功能默认启用。

## 使用 `rqt` 控制推进器铰接

`rqt` 工具提供了一个方便的图形界面，用于可视化和实验 ROS2 话题。我们将使用发布者功能来探索推进器铰接参数。要设置它，首先启动示例世界：

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

这将启动一个包含使用 `AFT` 推进器配置的 WAM-V 的环境。使用 Gazebo 界面，将相机定位到与水面齐平的位置，以便两个推进器都可见：

![WAM-V AFT Thruster Default Configuration](https://github.com/osrf/vrx/wiki/images/wamv_h_thruster_default.png)
接下来，在另一个终端中运行

```
ros2 topic list | grep thrust
```

以查看所有推进器相关话题的列表。确认每个推进器都有一个推进器位置话题和一个推力命令话题，并记下完整的话题名称。

从终端启动 rqt：

```
rqt
```

这将打开一个类似于下图所示的新窗口。点击 *Plugins* 下拉菜单，滚动到 *Topics*，然后点击 *Message Publisher*。您应该在窗口中看到一个新的发布功能部分。选择上面列出的四个推进器角度/命令话题，然后按右侧的加号添加它们。点击箭头展开每个话题，以在 "expression" 字段中显示当前值：

![rqt Publisher Thruster Topics](https://github.com/osrf/vrx/wiki/images/rqt_pub_thruster_topics_2023.png)

### 发布推力角度和命令

要查看向推进器话题发布各种值的效果，双击话题的 expression 值并输入新的期望值。然后点击话题旁边的复选标记开始发布。

#### 示例：改变角度和推力

* 将 `thrusters/left/pos` 和 `thrusters/right/pos` 更改为 1.57，以查看左右推进器逆时针（正方向）旋转 90 度（pi/2 弧度），如下所示。

![rqt_publisher Rotated Counter Clockwise](https://github.com/osrf/vrx/wiki/images/rqt_pub_clockwise_spin_2023.png)

* 现在将 `thrusters/left/thrust` 和 `thrusters/right/thrust` 话题设置为 2，使 WAM-V 顺时针旋转。

#### 示例：超过最大角度

* 将所有值重置为 0，以关闭推进器并将其返回原始位置。
* 现在将左右 `thrust` 值改回 2，并将左右推进器角度更改为 -10。
* 我们已将最大推进器角度设置为 $\pm \pi$（$\pm$ 3.14），因此 `-10` 将被裁剪为 $-\pi$ 或 `-3.14`。
* 这将导致左右推进器顺时针（负方向）旋转 -180 度（$-\pi$ 弧度），如下所示。

![rqt Publisher Angle Clip Demo](https://github.com/osrf/vrx/wiki/images/rqt_pub_pos_clip.png)

WAM-V 现在应该向后移动。

## 遥操作教程控制推进器铰接

* 使用游戏手柄控制也支持对推进器角度的有限控制。有关更多信息，请阅读[遥操作教程](https://github.com/osrf/vrx/wiki/teleop_tutorial)。

| Back: [驾驶 WAM-V](https://github.com/osrf/vrx/wiki/teleop_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [添加赛道元素](https://github.com/osrf/vrx/wiki/Adding-course-elements_tutorial) |
| --- | --- | --- |


==============================================================================

# 第 6 章：添加赛道元素
# 第 6 章: 添加赛道元素
==============================================================================

# 概述

VRX 有一组基本的世界，包含水域、天空、海岸线和一些固定位置的 RobotX 元素。创建您自己的静态赛道甚至向运行中的仿真添加新元素都很简单。

## 前提条件

本指南假设您已经按照[设置说明](https://github.com/osrf/vrx/wiki/installation_tutorial)安装了 ROS 2 和 vrx 软件包。

---

## 创建世界文件

世界文件定义了 Gazebo 启动时的初始环境，包括光照、天空、地面和模型。让我们复制一个示例作为世界文件的起点：

```
$ mkdir example_vrx_package
$ cd example_vrx_package/
$ cp <YOUR_VRX_INSTALLATION>/src/vrx/vrx_gz/worlds/sydney_regatta.sdf sydney_regatta_custom.sdf
```

请注意这是一个 **.sdf** 文件。如果您不熟悉 SDF 文件，应该先阅读[这个](http://sdformat.org/tutorials?tut=spec_world&cat=specification&)教程。

让我们浏览此文件并进行一些更改。用您喜欢的编辑器打开 `sydney_regatta_custom`。首先，注意  标签内的以下两行：

```
<include>
  <pose> 0 0 0.2 0 0 0 </pose>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/sydney_regatta</uri>
</include>
```

此代码块设置了一个空的悉尼赛艇环境（仅水域、天空和海岸线），从 [Fuel](https://app.gazebosim.org/) 下载。

文件的其余部分只是添加不同的环境元素，如波浪和风。

让我们在世界中添加一个浮标作为避障物。将以下行放在  标签之前：

```
<include>
  <name>mb_round_buoy_orange_custom</name>
  <pose>-520 180 0 0 1.57 0</pose>
  <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/mb_round_buoy_orange</uri>

  <plugin name="vrx::PolyhedraBuoyancyDrag"
          filename="libPolyhedraBuoyancyDrag.so">
    <fluid_density>1000</fluid_density>
    <fluid_level>0.0</fluid_level>
    <linear_drag>25.0</linear_drag>
    <angular_drag>2.0</angular_drag>
    <buoyancy name="collision_outer">
      <link_name>link</link_name>
      <pose>0 0 0 0 0 0</pose>
      <geometry>
        <sphere>
          <radius>0.25</radius>
      </sphere>
      </geometry>
    </buoyancy>
    <wavefield>
      <size>1000 1000</size>
      <cell_count>50 50></cell_count>
      <wave>
        <model>PMS</model>
        <period>5.0</period>
        <number>3</number>
        <scale>1.1</scale>
        <gain>0.3</gain>
        <direction>1 0</direction>
        <angle>0.4</angle>
        <tau>2.0</tau>
        <amplitude>0.0</amplitude>
        <steepness>0.0</steepness>
      </wave>
    </wavefield>
  </plugin>
</include>
```

### 使用自定义世界运行 vrx

现在您已经创建了一个新的世界文件，可以使用此世界再次启动仿真：

首先，更新 `GZ_SIM_RESOURCE_PATH` 环境变量，让 Gazebo 知道有一个包含世界文件的新目录：

```
export GZ_SIM_RESOURCE_PATH=/home/caguero/example_vrx_package:$GZ_SIM_RESOURCE_PATH
```

接下来，使用自定义世界参数运行仿真：

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta_custom
```

如果一切顺利，您应该看到一个新的橙色浮标被添加到 Gazebo 中。

| Back: [推进器铰接](https://github.com/osrf/vrx/wiki/thruster_articulation_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [使用声学定位器](https://github.com/osrf/vrx/wiki/Acoustic-pinger_tutorial) |
| --- | --- | --- |


---


## 使用声学定位器

# 声学信标

一些 RobotX 挑战赛任务使用水下声学信标。例如，RobotX "进出口门" 任务要求团队检测活动信标并通过信标所在的门。

## 读取声学信标消息

VRX WAM-V 配备了检测水下声学信标的传感器。该传感器使用 ParamVec [消息定义](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_interfaces/msg/ParamVec.msg) 定期向 ROS 2 话题 `/wamv/pingers/pinger/range_bearing` 发布数据。

您可以使用 `ros2 topic echo` 命令读取此传感器的输出，如下所示：

* 启动 VRX 仿真环境：

```
ros2 launch vrx_gz competition.launch.py world:=gymkhana_task
```

* 显示声学传感器发布的数据：

```
ros2 topic echo /wamv/pingers/pinger/range_bearing
```

您应该看到类似以下的消息流：

```
---
header:
  stamp:
    sec: 5
    nanosec: 0
  frame_id: wamv/pinger
params:
- name: elevation
  value:
    type: 3
    bool_value: false
    integer_value: 0
    double_value: 0.007911467873143386
    string_value: ''
    byte_array_value: []
    bool_array_value: []
    integer_array_value: []
    double_array_value: []
    string_array_value: []
- name: bearing
  value:
    type: 3
    bool_value: false
    integer_value: 0
    double_value: 0.22815154700097853
    string_value: ''
    byte_array_value: []
    bool_array_value: []
    integer_array_value: []
    double_array_value: []
    string_array_value: []
- name: range
  value:
    type: 3
    bool_value: false
    integer_value: 0
    double_value: 139.4759424138178
    string_value: ''
    byte_array_value: []
    bool_array_value: []
    integer_array_value: []
    double_array_value: []
    string_array_value: []
---
```

**注意**：

* 传感器为您提供距离（range）和两个角度（bearing 和 elevation）。
* 您从传感器获得的值包含一些噪声。

## 更改定位器位置

话题 `/wamv/pingers/pinger/set_pinger_position` 允许您更改信标位置。您需要发布一个 Vector3 消息。[这里](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3.html)是消息定义。

例如，要将定位器位置更改为原点，您可以运行：

```
ros2 topic pub --once /wamv/pingers/pinger/set_pinger_position geometry_msgs/msg/Vector3 "x: 0
y: 0
z: 0"
```

在回显定位器数据的同时尝试此命令，看看距离值如何剧烈变化。

| Back: [添加赛道元素](https://github.com/osrf/vrx/wiki/Adding-course-elements_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [Visualizing with Rviz](https://github.com/osrf/vrx/wiki/rviz_tutorial) |
| --- | --- | --- |


==============================================================================

# 第 7 章：RViz 可视化
# 第 7 章: RViz 可视化
==============================================================================

# 概览

[RViz](http://wiki.ros.org/rviz) 是用于可视化消息的标准 ROS 工具。本教程介绍如何运行 RViz2 来显示您使用 Gazebo 仿真的 WAM-V 和传感器。RViz2 的文档尚未在 wiki 上更新，因此如需更多详情，您可能有兴趣查看[代码库](https://github.com/ros2/rviz/tree/jazzy)

## 启动 Gazebo

运行仿真：

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

保持此仿真在本教程的其余部分运行。

## 运行 RViz

要打开为 WAM-V 配置的 RViz，请打开一个新终端并运行：

```
ros2 launch vrx_gazebo rviz.launch.py
```

RViz 应该会打开并显示 WAM-V 和相机！尝试[四处驾驶](https://github.com/osrf/vrx/wiki/teleop_tutorial)以在 RViz 中查看不同的传感器测量值。

![rviz_video](https://github.com/osrf/vrx/wiki/images/rviz_thruster_move.gif)

## 自定义 RViz

上面的示例启动文件使用预配置的话题集运行 RViz。您可以在 RViz GUI 中添加、删除和编辑话题等，以更好地可视化您的机器人。

| Back: [使用声学定位器](https://github.com/osrf/vrx/wiki/Acoustic-pinger_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) |
| --- | --- |


==============================================================================

# 第 8 章：自定义 WAM-V（初级）
# 第 8 章: 自定义 WAM-V (初级)
==============================================================================

# 概览

本教程序列的目的是演示如何创建自定义 WAM-V 推进器和组件配置。这涉及编写用户生成的推进器 YAML 文件和用户生成的组件 YAML 文件，然后运行脚本生成具有指定推进器和组件的自定义 WAM-V URDF 文件。然后可以将此 WAM-V URDF 文件作为参数传递给 VRX 仿真 `launch` 文件。

本序列中的所有教程都假设您已经按照 installation_tutorial 教程中描述的方式构建并源化了 VRX 工作空间。

## 目录

* [默认 WAM-V 配置](https://github.com/osrf/vrx/wiki/default_wamv_tutorial)
* [创建"空" WAM-V](https://github.com/osrf/vrx/wiki/empty_wamv_tutorial)
* [`generate_wamv.launch.py`](https://github.com/osrf/vrx/wiki/generate_wamv_tutorial)
* [自定义推进器配置](https://github.com/osrf/vrx/wiki/custom_thrusters_tutorial)
* [自定义 WAM-V 组件](https://github.com/osrf/vrx/wiki/custom_components_tutorial)

| Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [默认 WAM-V 配置](https://github.com/osrf/vrx/wiki/default_wamv_tutorial) |
| --- | --- |


---


## 默认 WAM-V 配置

在开始自定义 WAM-V 之前，让我们先检查一下默认配置。默认 WAM-V 使用 H 推进器配置，配备三个摄像头、一个激光雷达、一个球发射器、一个 GPS 和一个 IMU。

要使用此配置启动示例世界，请运行

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

这应该会启动悉尼赛艇世界并生成一个 WAM-V，其组件和推进器与下图所示非常相似：

![default_wamv_2023](https://private-user-images.githubusercontent.com/8611855/238059911-6cece4a7-e943-4301-86fd-d7e2d092ab01.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM1MjgxNTcsIm5iZiI6MTc4MzUyNzg1NywicGF0aCI6Ii84NjExODU1LzIzODA1OTkxMS02Y2VjZTRhNy1lOTQzLTQzMDEtODZmZC1kN2UyZDA5MmFiMDEucG5nP1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI2MDcwOCUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNjA3MDhUMTYyNDE3WiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9NGFkMzBjZWJhNWIwMDNhYWQ1Mzc2MTQ4ZTk5M2Q0NDhiODhhZDZkZTg1NjQ2NWFmYmNkZjUwZmNmMTdkYTYzNyZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QmcmVzcG9uc2UtY29udGVudC10eXBlPWltYWdlJTJGcG5nIn0.8LlfzeBv0UwSFxvKkS5M0gm-HBd21217RvZKmq6hTYM)

| Back: [序列概览](https://github.com/osrf/vrx/wiki/customizing_wamv_beginner_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [Creating an "Empty" WAM-V](https://github.com/osrf/vrx/wiki/empty_wamv_tutorial) |
| --- | --- | --- |


---


## 创建空 WAM-V

# 生成空的 WAM-V

现在让我们用一个"空"的 WAM-V 替换默认配置，我们可以向其中添加组件和推进器。

* 为您的自定义 WAM-V 创建一个目录，例如：

  ```
  mkdir ~/my_wamv
  cd ~/my_wamv
  ```
* 现在创建两个空白配置文件，一个用于推进器，一个用于组件：

  ```
  touch empty_thruster_config.yaml
  touch empty_component_config.yaml
  ```
* 使用 `generate_wamv.launch.py` 为您的 WAM-V 创建新的 `urdf`：

  ```
  ros2 launch vrx_gazebo generate_wamv.launch.py component_yaml:=`pwd`/empty_component_config.yaml thruster_yaml:=`pwd`/empty_thruster_config.yaml wamv_target:=`pwd`/wamv_target.urdf wamv_locked:=False
  ```
* 在终端中看到以下确认消息且无错误：

```
[INFO] [launch]: All log files can be found below /home/caguero/.ros/log/2022-12-22-21-53-36-523165-cold-3060856
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [generate_wamv.py-1]: process started with pid [3060857]
[generate_wamv.py-1] [INFO] [1671742416.733867948] [configure_wamv]:
[generate_wamv.py-1] Using /home/caguero/my_wamv/empty_thruster_config.yaml as the thruster configuration yaml file
[generate_wamv.py-1]
[generate_wamv.py-1] [INFO] [1671742416.734094421] [configure_wamv]:
[generate_wamv.py-1] Trying to open /home/caguero/my_wamv/empty_thruster_config.xacro
[generate_wamv.py-1]
[generate_wamv.py-1] [INFO] [1671742416.737537713] [configure_wamv]:
[generate_wamv.py-1] Using /home/caguero/my_wamv/empty_component_config.yaml as the component configuration yaml file
[generate_wamv.py-1]
[generate_wamv.py-1] WAM-V urdf file sucessfully generated. File location: /home/caguero/my_wamv/wamv_target.urdf
[INFO] [generate_wamv.py-1]: process has finished cleanly [pid 3060857]
```

* 使用您的 WAM-V 启动示例世界：

  ```
  ros2 launch vrx_gz competition.launch.py world:=sydney_regatta urdf:=`pwd`/wamv_target.urdf
  ```
* 您现在应该看到您的 WAM-V 没有推进器或组件：

![empty_wamv_2023](https://private-user-images.githubusercontent.com/8611855/238059990-1ca8dd05-38f1-4fa0-aa4d-969f8515391a.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM1MjgxNjksIm5iZiI6MTc4MzUyNzg2OSwicGF0aCI6Ii84NjExODU1LzIzODA1OTk5MC0xY2E4ZGQwNS0zOGYxLTRmYTAtYWE0ZC05NjlmODUxNTM5MWEucG5nP1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI2MDcwOCUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNjA3MDhUMTYyNDI5WiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9NzUzNTE4YTA0NjYzODQ0YThhN2RkZGIwYmQzNzEyYTMwYzdhNjA1ZGM4NmM1ODY4YmQ0ZGU4MjE5YzNkNWE5ZiZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QmcmVzcG9uc2UtY29udGVudC10eXBlPWltYWdlJTJGcG5nIn0.1h_Hr0vBM4n8IbqpuoJlYf27ck3nLWSDRdxnpPd5vlo)

| Back: [默认 WAM-V 配置](https://github.com/osrf/vrx/wiki/default_wamv_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [`generate_wamv.launch.py`](https://github.com/osrf/vrx/wiki/generate_wamv_tutorial) |
| --- | --- | --- |


---


## generate_wamv.launch.py / generate_wamv.launch.py

在上一个教程中，我们使用以下冗长的命令创建了一个空的 WAM-V：

```
ros2 launch vrx_gazebo generate_wamv.launch.py component_yaml:=`pwd`/empty_component_config.yaml thruster_yaml:=`pwd`/empty_thruster_config.yaml wamv_target:=`pwd`/wamv_target.urdf wamv_locked:=False
```

此命令启动一个名为 `generate_wamv.launch.py` 的脚本，它读取您创建的配置文件并为您的 WAM-V 生成一个 `urdf` 文件。生成 `urdf` 后，您可以将其传递给 `competition.launch.py` 以在仿真中使用。

请注意，当调用 `competition.launch.py` 或任何其他仿真启动文件时，如果给定了 `urdf` 参数，它将使用该文件作为 WAM-V 配置。如果未给定 `urdf` 参数，则将使用启动文件中给出的默认配置。

## `generate_wamv.launch.py` 参数说明：

`generate_wamv.launch.py` 脚本接受以下参数：

* `thruster_yaml`：输入，推进器 YAML 配置文件的完整路径。如果未给定，使用[默认推进器 yaml](https://github.com/osrf/vrx/blob/main/vrx_urdf/vrx_gazebo/config/wamv_config/example_thruster_config.yaml)\*
* `component_yaml`：输入，组件 YAML 配置文件的完整路径。如果未给定，使用[默认组件 yaml](https://github.com/osrf/vrx/blob/main/vrx_urdf/vrx_gazebo/config/wamv_config/example_component_config.yaml)\*
* `wamv_target`：输出，将生成的 WAM-V URDF 的完整路径
* `wamv_locked`：输入，布尔值，决定生成的 WAM-V 是否应锁定在原位。此功能仅用于 VRX 竞赛，普通使用应设置为 `False`。

| Back: [Creating an "Empty" WAM-V](https://github.com/osrf/vrx/wiki/empty_wamv_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [自定义推进器配置](https://github.com/osrf/vrx/wiki/custom_thrusters_tutorial) |
| --- | --- | --- |


---


## 自定义推进器配置

现在让我们为 WAM-V 添加自定义推进器配置。

* 在您创建的 `my_wamv` 目录中，使用文本编辑器创建一个名为 `example_thruster_config.yaml` 的文件，并添加以下内容：

```
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

此文件指定了三个推进器（左、右和中），排列为 "T" 推进器配置。稍后，您可以编辑此文件以进一步自定义您的 WAM-V。

* 运行脚本以使用这些新指定的推进器和组件生成 WAM-V 的 URDF。

```
ros2 launch vrx_gazebo generate_wamv.launch.py component_yaml:=`pwd`/empty_component_config.yaml thruster_yaml:=`pwd`/example_thruster_config.yaml wamv_target:=`pwd`/wamv_target.urdf wamv_locked:=False
```

* WAM-V 现在应该有三个推进器：

![custom_thrusters_2023](https://private-user-images.githubusercontent.com/8611855/238060121-a8a84404-0055-4f13-bf34-69dc808b52d9.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM1MjgxNzQsIm5iZiI6MTc4MzUyNzg3NCwicGF0aCI6Ii84NjExODU1LzIzODA2MDEyMS1hOGE4NDQwNC0wMDU1LTRmMTMtYmYzNC02OWRjODA4YjUyZDkucG5nP1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI2MDcwOCUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNjA3MDhUMTYyNDM0WiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9N2E0OGUwYzVlNTYwYjY5ZGU4OWRmMjc0MTRjOWY5Yzk4MDhkYjdlMjA0YTIwMzhiOTgzNTE5NGFmZjIzYWViMyZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QmcmVzcG9uc2UtY29udGVudC10eXBlPWltYWdlJTJGcG5nIn0.7vdJdSuBZ-X6I-m1LOqoQPHtJBrMth0xjM-lTZVdwno)

| Back: [`generate_wamv.launch.py`](https://github.com/osrf/vrx/wiki/generate_wamv_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [自定义 WAM-V 组件](https://github.com/osrf/vrx/wiki/custom_components_tutorial) |
| --- | --- | --- |


---


## 自定义 WAM-V 组件

# 自定义组件配置

最后，让我们指定一个自定义组件配置。

* 在您创建的 `my_wamv` 目录中，使用文本编辑器创建一个名为 `example_component_config.yaml` 的文件，并添加以下内容：

```
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
    - name: far_left_camera
      visualize: False
      x: 0.75
      y: 0.3
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
wamv_pinger:
    - sensor_name: receiver
      position: 1.0 0 -1.0
```

此文件添加了与默认 WAM-V 组件配置相同的所有组件，但在载具上的排列不同（例如，球发射器现在在平台的另一侧）。稍后，您可以编辑此文件以进一步自定义您的 WAM-V。

* 运行 `generate_wamv.launch.py` 脚本以使用这些组件生成 WAM-V 的 URDF。

```
ros2 launch vrx_gazebo generate_wamv.launch.py component_yaml:=`pwd`/example_component_config.yaml thruster_yaml:=`pwd`/example_thruster_config.yaml wamv_target:=`pwd`/wamv_target.urdf wamv_locked:=False
```

* WAM-V 现在应该有按上述位置排列的组件。从上一个教程中，它还将有 3 个推进器：

![custom_compoent_2023](https://private-user-images.githubusercontent.com/8611855/238060292-eb7875ec-4545-4d07-a1db-53bdbbecd014.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM1MjgxNzksIm5iZiI6MTc4MzUyNzg3OSwicGF0aCI6Ii84NjExODU1LzIzODA2MDI5Mi1lYjc4NzVlYy00NTQ1LTRkMDctYTFkYi01M2JkYmJlY2QwMTQucG5nP1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI2MDcwOCUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNjA3MDhUMTYyNDM5WiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9MGQ1NWUyODU2MDJmNGZlYmNkMzIyOWJhZmQzYmYxZGE5YjZlMTM3NDE3NjExNWFjZDEzZDJhNWVjNjE4MWEzMSZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QmcmVzcG9uc2UtY29udGVudC10eXBlPWltYWdlJTJGcG5nIn0.wOy4wZ-4OeAHIfdvIOPJcvV9gYu8nVJ3TqEtzTujMlY)

| Back: [自定义推进器配置](https://github.com/osrf/vrx/wiki/custom_thrusters_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) |
| --- | --- |


==============================================================================

# 第 9 章：自定义 WAM-V（中级）
# 第 9 章: 自定义 WAM-V (中级)
==============================================================================

# 概览

* WAM-V 的行为由一组 [Gazebo 插件](https://gazebosim.org/api/sim/8/createsystemplugins.html) 决定。
* 与水动力学和推进特性相关的参数在两个 `xacro` 文件中设置。
* 水动力参数（如阻力和附加质量）在 `wamv_gazebo_dynamics_plugin.xacro` 中设置
* 推进特性（如力限制和线性/非线性映射）在 `wamv_gazebo_thruster_config.xacro` 中设置

有关推导许多数值的详细信息，以及更深入的理论探讨，请参阅此 FAU 出版物 <https://doi.org/10.1016/j.oceaneng.2016.09.037>

## 目录

* [调整水动力参数](https://github.com/osrf/vrx/wiki/hydrodynamic_params_tutorial)
* 调整推进特性


---


## 水动力参数

# 调整水动力模型

* 我们为默认 USV 使用的水动力模型在[简单水动力插件](https://github.com/osrf/vrx/blob/main/vrx_gz/src/SimpleHydrodynamics.hh)中实现。
* 此插件的系数表征了 WAM-V 上的线性和非线性阻力。
* 我们为默认 USV 使用的浮力模型通过我们的 [Surface 插件](https://github.com/osrf/vrx/blob/main/vrx_gz/src/Surface.hh) 实现。
* 这模拟了物体在流体表面的浮力。
* 我们将 Surface 插件的实例附加到每个船体，并将 Simple Hydrodynamics 插件的实例附加到整艘船（`wamv_gazebo_dynamics_plugin.xacro`）。

## 步骤

### 步骤 1：修改参数

对于 Surface 插件：

```
<plugin filename="libSurface.so" name="vrx::Surface">
 <link_name>${namespace}/base_link</link_name>
 <hull_length>4.9</hull_length>
 <hull_radius>0.213</hull_radius>
 <fluid_level>0</fluid_level>
 <points>
    <point>0.6 1.03 0</point>
    <point>-1.4 1.03 0</point>
  </points>
  <wavefield>
    <topic>/vrx/wavefield/parameters</topic>
  </wavefield>
</plugin>
```

这是单体船的附加插件，其中船体被简化为由 `hull_length` 和 `hull_radius` 定义的圆柱体。

对于 WAM-V，尝试将 `hull_length` 减半。

值得注意的是，插件默认假设船体为圆柱形，但可以覆盖此行为。

### 步骤 2：重新构建

重新编译您的工作空间：

```
cd <VRX_WS>
GZ_VERSION=harmonic
colcon build --merge-install
```

### 步骤 3：测试

启动仿真：

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

放大您的 WAM-V。如果您只调整了一个船体的船体长度，WAM-V 应该看起来不平衡。如果您调整了两个船体的长度，WAM-V 应该在水中坐得更低。

### 步骤 4：

将修改后的 WAM-V Surface 插件变量返回到原始值。

对于 Simple Hydrodynamics 插件

```
<plugin
  filename="libSimpleHydrodynamics.so"
  name="vrx::SimpleHydrodynamics">
  <link_name>${namespace}/base_link</link_name>
  <!-- Added mass -->
  <xDotU>0.0</xDotU>
  <yDotV>0.0</yDotV>
  <nDotR>0.0</nDotR>
  <!-- Linear and quadratic drag -->
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

`xU` 和 `xUU` 分别是纵荡方向的线性和非线性阻力项。将每个变量的值增加到能在仿真中看到明显变化的程度（乘以 2 似乎是一个好的开始）。然后，按照步骤 2 和 3 重新构建。

WAM-V 的驾驶特性应该有明显变化。如果您按建议增加了值，WAM-V 在纵荡方向上应该看起来更慢。

有关每个系数影响的更深入探讨，此 FAU 出版物 <https://doi.org/10.1016/j.oceaneng.2016.09.037> 提供了很好的概述。


---


## 自定义环境因素

# 概览

* VRX 环境的行为由一组 [Gazebo 插件](https://gazebosim.org/api/plugin/2/introduction.html) 生成。
* 许多决定环境条件的参数（如波浪、风等）可以通过使用 SDF 配置文件来更改。
* 更改这些参数允许用户在各种条件下测试他们的解决方案。
* 有关参数值的说明，请参阅[操作理论](https://github.com/osrf/vrx/wiki/vrx_theory)。

本序列给出了说明如何更改风、波浪、雾和环境光参数的示例。

* 讨论的所有参数都存储在单独的世界文件中，位于 `vrx_gz/worlds`。
* 在整个序列中，我们将使用 `sydney_regatta` 世界作为示例。

## 目录

* [调整风参数](https://github.com/osrf/vrx/wiki/wind_params_tutorial)
* [调整波浪参数](https://github.com/osrf/vrx/wiki/wave_params_tutorial)
* [调整雾参数](https://github.com/osrf/vrx/wiki/fog_params_tutorial)
* [调整环境光](https://github.com/osrf/vrx/wiki/ambient_params_tutorial)

| Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [调整风参数](https://github.com/osrf/vrx/wiki/wind_params_tutorial) |
| --- | --- |


---


## 调整风参数

# 调整风

* 风可以用方向和平均速度来表征。
* 在 VRX 的当前实现中，风仅影响 WAM-V。
* 用于 WAM-V 的风阻系数基于 FAU 报告（Station-keeping control of an unmanned surface vehicle exposed to current and wind disturbances）[https://doi.org/10.1016/j.oceaneng.2016.09.037]。

## 步骤

### 步骤 1：修改参数

打开 `vrx/vrx_gz/worlds/sydney_regatta.sdf` 并编辑风插件元素中给出的值：

```
<!-- Load the plugin for the wind -->
<plugin
  filename="libUSVWind.so"
  name="vrx::USVWind">
  <wind_obj>
    <name>wamv</name>
    <link_name>wamv/base_link</link_name>
    <coeff_vector>.5 .5 .33</coeff_vector>
  </wind_obj>
  <!-- Wind -->
  <wind_direction>240</wind_direction>
  <!-- in degrees -->
  <wind_mean_velocity>0.0</wind_mean_velocity>
  <var_wind_gain_constants>0</var_wind_gain_constants>
  <var_wind_time_constants>2</var_wind_time_constants>
  <random_seed>10</random_seed>
  <!-- set to zero/empty to randomize -->
  <update_rate>10</update_rate>
  <topic_wind_speed>/vrx/debug/wind/speed</topic_wind_speed>
  <topic_wind_direction>/vrx/debug/wind/direction</topic_wind_direction>
 </plugin>
```

例如，尝试将 `wind_mean_velocity` 更改为 5（m/s），将 `wind_direction` 更改为 180（度）。

### 步骤 2：重新构建

重新编译您的工作空间：

```
cd <VRX_WS>
GZ_VERSION=harmonic colcon build --merge-install
```

### 步骤 3：测试

启动仿真：

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

在另一个终端中，源化工作空间并回显风向话题以确认更改：

```
. install/setup.bash
ros2 topic echo /vrx/debug/wind/direction
```

要查看风速效果，按 CTRL+C 终止回显命令并运行：

```
ros2 topic echo /vrx/debug/wind/speed
```

您应该看到一系列风速值在 5 m/s 的平均值附近变化。

| Back: [环境参数概述](https://github.com/osrf/vrx/wiki/env_params_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [调整波浪参数](https://github.com/osrf/vrx/wiki/wave_params_tutorial) |
| --- | --- | --- |


---


## 调整波浪参数

# 调整波浪

* 波浪场的参数由附加到世界的发布者插件广播到 `/vrx/wavefield/parameters` 话题。
* 波浪场插件的实例（从此话题读取）必须附加到受波浪场影响的所有对象。

## 步骤

### 步骤 1：修改参数

* 打开 `vrx/vrx_gz/worlds/sydney_regatta.sdf` 并编辑发布者插件消息中列出的方向、增益、周期和陡度值：

  ```
      <!-- The wave field -->
      <plugin filename="libPublisherPlugin.so" name="vrx::PublisherPlugin">
        <message type="gz.msgs.Param" topic="/vrx/wavefield/parameters"
                 every="2.0">
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

例如，尝试将增益更改为 0.8。

### 步骤 2：重新构建

重新编译您的工作空间：

```
cd <VRX_WS>
colcon build --merge-install
```

### 步骤 3：测试

启动仿真：

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

您应该观察到您的 WAM-V 由于风和更大的波浪冲击 WAM-V 而缓慢移动。

| Back: [调整风参数](https://github.com/osrf/vrx/wiki/wind_params_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [调整雾参数](https://github.com/osrf/vrx/wiki/fog_params_tutorial) |
| --- | --- | --- |


---


## 调整雾参数

### *注意：Gazebo Sim 中尚未实现雾效果*

*VRX 的雾效果依赖于 Gazebo Sim 对 sdformat 规范中描述的雾元素的实现。此功能在 Gazebo Classic 中可用，但尚未移植到 Gazebo Harmonic。请注意，在此移植完成之前，下面教程中概述的步骤不会产生雾效果。*

# 调整雾参数

* 雾可以作为 `<scene>` 的元素指定。详情请参阅 [sdformat 规范](http://sdformat.org/spec?ver=1.10&elem=scene#scene_fog)。

## 步骤

### 步骤 1：修改参数

打开 `vrx/vrx_gz/worlds/sydney_regatta.sdf` 并向场景添加雾：

```
    <scene>
      <sky></sky>
      <grid>false</grid>
      <ambient>1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8</background>
      <fog>
        <type>linear</type>
        <color>1 1 1 1</color>
        <start>1</start>
        <end>100</end>
        <density>1</density>
      </fog>

    </scene>
```

### 步骤 2：重新构建

重新编译您的工作空间：

```
cd <VRX_WS>
colcon build --merge-install
```

### 步骤 3：测试

启动仿真：

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

场景现在应该有雾了。

| Back: [调整波浪参数](https://github.com/osrf/vrx/wiki/wave_params_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [调整环境光](https://github.com/osrf/vrx/wiki/ambient_params_tutorial) |
| --- | --- | --- |


---


## 调整环境光

* The color of ambient light can be set in the `scene` element of individual world files, found in `vrx_gz/worlds`.
* The color is stored as a vector of RBG values between 0 and 1.
* In this tutorial we will use the `sydney_regatta` world as an example.

## 步骤

### 步骤 1：修改参数

打开 `vrx/vrx_gz/worlds/sydney_regatta.sdf` 并编辑指定环境光颜色的 RBG 值：

```
    <scene>
      <sky></sky>
      <grid>false</grid>
      <ambient>1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8</background>
    </scene>
```

例如，尝试将所有值更改为 0 以关闭环境光：`<ambient>0.0 0.0 0.0</ambient>`

#### 注意：

注意不要将上述参数与 `<ambient_light>` 参数混淆，后者目前在 VRX 中未使用。

### 步骤 2：重新构建

重新编译您的工作空间：

```
cd <VRX_WS>
colcon build --merge-install
```

### 步骤 3：测试

启动仿真：

```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

您应该观察到由于没有环境光，物体阴影变得更加锐利。

#### 修改前：

![ambient111](https://private-user-images.githubusercontent.com/8611855/257351928-74bd655f-2376-418c-a007-bdc4f1e935d1.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM1MjgyMDMsIm5iZiI6MTc4MzUyNzkwMywicGF0aCI6Ii84NjExODU1LzI1NzM1MTkyOC03NGJkNjU1Zi0yMzc2LTQxOGMtYTAwNy1iZGM0ZjFlOTM1ZDEucG5nP1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI2MDcwOCUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNjA3MDhUMTYyNTAzWiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9MDUzNjgwNDExOGJlZTIyZDZjNzEzMDEyYjk3N2Y3ZTk2YzhjZmI4ZjEzZmU4NWQyYmFlNzM3MDIzOGE4MmIzNCZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QmcmVzcG9uc2UtY29udGVudC10eXBlPWltYWdlJTJGcG5nIn0.ud6kuUw0mZcHX-GZKtP_54cohTl48rQwfnahla0rCb4)

此截图显示了 `<ambient>1.0 1.0 1.0</ambient>` 时树木的阴影。

#### 修改后：

![ambient000](https://private-user-images.githubusercontent.com/8611855/257351949-b257dbfa-a443-4e9a-ae8b-8753273ba558.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM1MjgyMDMsIm5iZiI6MTc4MzUyNzkwMywicGF0aCI6Ii84NjExODU1LzI1NzM1MTk0OS1iMjU3ZGJmYS1hNDQzLTRlOWEtYWU4Yi04NzUzMjczYmE1NTgucG5nP1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI2MDcwOCUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNjA3MDhUMTYyNTAzWiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9OTM1Y2Q0NjYyYzI2ZTM4NDIxYmMyMTFmMmVjMDFmMDA4ZTZkOTEyZmZkYjMxYWFhMzE4OGIyYzdlZGVmYTJlOCZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QmcmVzcG9uc2UtY29udGVudC10eXBlPWltYWdlJTJGcG5nIn0.7chr1i8I9mTk0EY_8BYDBuAI472wYVSURbub9uoE8Hc)

此截图显示了 `<ambient>0.0 0.0 0.0</ambient>` 时树木的阴影。

| Back: [调整雾参数](https://github.com/osrf/vrx/wiki/fog_params_tutorial) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) |
| --- | --- |


==============================================================================

# 第 10 章：RoboBoat 教程概览
# 第 10 章: RoboBoat 教程概览
==============================================================================

我们建议按照顺序学习教程，因为每个教程都建立在前一个的基础上。完成这些教程后，您将熟悉如何在 VRX 环境中使用您的 RoboBoat 模型。

## 目录

* [运行 RoboBoat 示例世界](https://github.com/osrf/vrx/wiki/running_roboboat_tutorial)
* [遥控您的载具](https://github.com/osrf/vrx/wiki/teleop_roboboat_tutorial)
* [自定义您的载具](https://github.com/osrf/vrx/wiki/roboboat_customizing_tutorial)


---


## 运行 RoboBoat 示例世界

本页面提供了环境的描述，并演示了如何在此世界中生成 USV。还假设您已经完成了[安装教程](https://github.com/osrf/vrx/wiki/installation_tutorial)，并熟悉运行 Gazebo 仿真器和 ROS 开发工具的基础知识。

## Nathan Benderson 公园环境

RoboBoat 竞赛场地是美国佛罗里达州萨拉索塔县的 Nathan Benderson 公园：

![Nathan Benderson Park](https://github.com/osrf/vrx/wiki/images/nbpark.png)

## 运行空世界

由于环境大小，这可能需要一些时间。首次启动此世界时，它还将从 [Fuel 上的 vrx 集合](https://app.ignitionrobotics.org/OpenRobotics/fuel/collections/vrx) 下载 3D 模型

```
ros2 launch vrx_gz vrx_environment.launch.py world:=nbpark
```

## 添加载具

要打开包含示例 RoboBoat 载具（或您自己的自定义载具）的世界文件，您需要将其添加到世界中。打开位于 ~/vrx_ws/src/vrx/vrx_gz/worlds 的 nbpark.sdf 文件，并添加以下代码块：

```
<!-- RoboBoat 01 -->
<include>
  <name>roboboat01</name>
  <pose>-185 1088 0 0 0 0</pose>
  <uri>roboboat01</uri>
</include>
```

启动修改后的世界：

```
ros2 launch vrx_gz vrx_environment.launch.py world:=nbpark
```

| Back: [安装 VRX](https://github.com/osrf/vrx/wiki/installation_tutorial) | Top: [Roboboat 教程](https://github.com/osrf/vrx/wiki/tutorials_roboboat) |
| --- | --- |


---


## 遥控 RoboBoat 车辆

本页面提供了如何驾驶生成的 USV 的描述。假设您已经完成了关于[安装](https://github.com/osrf/vrx/wiki/installation_tutorial)和[生成](https://github.com/osrf/vrx/wiki/running_roboboat_tutorial)的前述教程。还假设您对 Gazebo 有基本的了解。

## 推进器设置

我们在上一个教程中生成的船的执行器由 Gazebo Sim Thruster 类定义。虽然有许多方法可以控制航向，但有两个推进器，我们可以简单地采用差动推力。

## 启动遥操作节点

### 启动 ROS/Gazebo 桥接器

要打开包含示例 RoboBoat 载具（或您自己的自定义载具）的世界文件，您需要将其添加到世界中。打开位于 ~/vrx_ws/src/vrx/vrx_gz/worlds 的 `nbpark.sdf` 文件，并添加以下代码块：

```
<!-- RoboBoat 01 -->
<include>
  <name>roboboat01</name>
  <pose>-185 1088 0 0 0 0</pose>
  <uri>roboboat01</uri>
</include>
```

然后，启动环境：

```
ros2 launch vrx_gz vrx_environment.launch.py world:=nbpark
```

要从 ROS2 向 Gazebo Sim 发送消息，我们必须使用桥接器，指定我们需要发送的话题和消息。
对于左推进器：

```
ros2 run ros_gz_bridge parameter_bridge /model/roboboat01/joint/left_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double
```

对于右推进器：

```
ros2 run ros_gz_bridge parameter_bridge /model/roboboat01/joint/right_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double
```

#### 测试桥接器

您可以通过向一个或两个推进器发布话题来测试桥接器是否正常工作：

```
ros2 topic pub /model/roboboat01/joint/left_engine_propeller_joint/cmd_thrust std_msgs/msg/Float64 "data: 150"
```

使用此命令您应该看到您的船在转圈。

### 游戏手柄遥操作

#### 配置调整

在两个桥接命令运行的情况下，在另一个终端启动

```
ros2 launch vrx_gz usv_joy_teleop.py teleop_config:=/home/caguero/vrx2023_ws/src/vrx/vrx_gz/config/roboboat.yaml
```

游戏手柄直接驱动左右推进器 - 类似于[差速驱动轮式机器人](https://en.wikipedia.org/wiki/Differential_wheeled_robot)。

* `L1` 按钮需要始终按住，同时配合以下命令（死人开关），
* 左摇杆上下轴（轴 1）映射到左推进器前进/后退，
* 右摇杆上下轴（轴 3）映射到右推进器前进/后退

因此，将两个摇杆同时向前推应该会使船向前行驶。


---


## 自定义 RoboBoat 车辆

# 概述

本教程的目的是演示如何修改提供的 RoboBoat 参考模型之一。

# 创建自定义工作空间

创建一个工作空间来存储您全新的模型，命名为 `my_roboboat`。

```
mkdir -p ~/gazebo_maritime/models/my_roboboat && cd ~/gazebo_maritime/models/my_roboboat
```

让我们下载此模型的模板：

```
curl https://raw.githubusercontent.com/wiki/osrf/vrx/files/my_roboboat/model.config -o model.config
curl https://raw.githubusercontent.com/wiki/osrf/vrx/files/my_roboboat/model.sdf -o model.sdf
mkdir meshes
curl https://raw.githubusercontent.com/wiki/osrf/vrx/files/my_roboboat/meshes/gps.dae -o meshes/gps.dae
curl https://raw.githubusercontent.com/wiki/osrf/vrx/files/my_roboboat/meshes/gps.png -o meshes/gps.png
```

目前，这与 VRX 中可用的 RoboBoat01 模型非常相似。让我们测试它。用您喜欢的编辑器打开 VRX 工作空间中的 `<YOUR_VRX_WORKSPACE/src/vrx/vrx_gz/worlds/npark.sdf` 文件。然后在 `<world>` 标签内添加您的自定义 roboboat 模型：

```
<!-- My custom roboboat -->
<include>
  <name>my_roboboat</name>
  <pose>-175 1120 0 0 0 3.14</pose>
  <uri>my_roboboat</uri>
</include>
```

转到您的 VRX 工作空间，重新编译，并运行 Gazebo：

```
export GZ_SIM_RESOURCE_PATH=:$HOME/gazebo_maritime/models:$GZ_SIM_RESOURCE_PATH
cd ~/vrx_2023
colcon build --merge-install
ros2 launch vrx_gz vrx_environment.launch.py world:=nbpark
```

您应该看到您的模型在 Nathan Benderson 公园中成功加载。

![Screenshot from 2023-12-21 21-30-00](https://private-user-images.githubusercontent.com/1440739/292312862-c063e306-9716-4aeb-8f0e-4636af8fa738.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM1MjgyMTksIm5iZiI6MTc4MzUyNzkxOSwicGF0aCI6Ii8xNDQwNzM5LzI5MjMxMjg2Mi1jMDYzZTMwNi05NzE2LTRhZWItOGYwZS00NjM2YWY4ZmE3MzgucG5nP1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI2MDcwOCUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNjA3MDhUMTYyNTE5WiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9ODE3MWE4NjIzZjI2NjcxNzNmMzc4Y2M5YzQwYzI2YjY4ZjAxOTExZTAwMjc5OTgxNTJhMmQ4MjY3MzMyZmFjOCZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QmcmVzcG9uc2UtY29udGVudC10eXBlPWltYWdlJTJGcG5nIn0.8oJLzsPHHEMkh8Ri99sILppsrLb6IxJETjxG93goRlE)

# 添加传感器

让我们为您的自定义模型添加 GPS 传感器。打开 `~/gazebo_maritime/models/my_roboboat/model.sdf` 并取消注释以下代码块：

```
<!-- Uncomment to add GPS sensor -->
<visual name="gps_visual">
  <pose>0.3 0.0 0.25 0 0 0</pose>
  <geometry>
     <mesh>
      <uri>file://my_roboboat/meshes/gps.dae</uri>
    </mesh>
  </geometry>
</visual>
<sensor name="navsat" type="navsat">
  <pose>0.3 0.0 0.25 0 0 0</pose>
  <always_on>1</always_on>
  <update_rate>1</update_rate>
</sensor>
```

请注意我们添加了一个传感器和一个视觉元素。视觉元素只是在我们添加传感器的同一位置添加一个天线以增强视觉效果。传感器类型是 `navsat`，以 1 Hz 生成新的测量值，位于 `base_link` 的 `<pose>0.3 0.0 0.25 0 0 0</pose>` 处。

让我们在 Gazebo 中测试我们的修改：

```
ros2 launch vrx_gz vrx_environment.launch.py world:=nbpark
```

![Screenshot from 2023-12-21 21-39-41](https://private-user-images.githubusercontent.com/1440739/292314538-dbbf9930-aae7-492c-ba43-8a850f6046bc.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM1MjgyMTksIm5iZiI6MTc4MzUyNzkxOSwicGF0aCI6Ii8xNDQwNzM5LzI5MjMxNDUzOC1kYmJmOTkzMC1hYWU3LTQ5MmMtYmE0My04YTg1MGY2MDQ2YmMucG5nP1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI2MDcwOCUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNjA3MDhUMTYyNTE5WiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9YmY5ODA1NmE5Y2ZlYTgwZTgyZTczNjgzNjc3NWYzMDJhY2VkZmNlMzZlZWVmNTFjNjJiNzQzYzE3M2Q5YTcyNiZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QmcmVzcG9uc2UtY29udGVudC10eXBlPWltYWdlJTJGcG5nIn0.HRBm318NHgC-6StqlPbpvXTPbcENEiRiEB8bTzwS7Jo)

您可以测试是否能获得 GPS 读数：

```
gz topic -e -t /world/nbpark/model/my_roboboat/link/base_link/sensor/navsat/navsat

header {
  stamp {
    sec: 21
  }
  data {
    key: "seq"
    value: "21"
  }
}
latitude_deg: 27.377363426535567
longitude_deg: -82.452893421152822
altitude: 0.49600635841488838
velocity_east: 2.4807375001142368e-08
velocity_north: -1.8977398210552808e-10
velocity_up: 0.00099993929300069969
frame_id: "my_roboboat::base_link::navsat"
```


==============================================================================

# 第 11 章：2023 VRX 竞赛参与指南
# 第 11 章: 2023 VRX 竞赛参与指南
==============================================================================

# 如何参加 VRX 2023

VRX 2023 竞赛将使用 VRX 仿真环境运行，分为三个阶段进行。本页面提供了参加竞赛所需步骤的技术演练。

## 准备就绪

* 访问 [VRX 竞赛 2023 网站](https://robotx.org/vrx-2023) 了解竞赛任务、规则、评分和截止日期。
* 请注意，任何希望申请的团队都可以获得费用减免。
* 按照 [VRX 教程](https://github.com/osrf/vrx/wiki/) 学习如何安装和使用 VRX 仿真平台。
* 按照 [VRX 2023 竞赛任务教程](https://github.com/osrf/vrx/wiki/vrx_2023-task_tutorials) 了解每个任务的详细信息。

## 创建您的解决方案

参赛者必须开发并提交竞赛三个阶段中每个阶段的解决方案。请按照以下链接了解每个阶段的详细信息：

### [第一阶段：Hello World](https://github.com/osrf/vrx/wiki/vrx_2023-phase1_hello_world)

第一个 VRX 里程碑。团队提交演示他们能够运行仿真环境的视频。

* [Results](https://github.com/osrf/vrx/wiki/vrx_2023-phase1_results)

### [第二阶段：彩排](https://github.com/osrf/vrx/wiki/vrx_2023-phase2_dress_rehearsal)

竞赛的试运行。团队提交初步解决方案，并可以看到他们目前相对于所有其他参赛者的排名。

* 请参阅[第二阶段练习世界](https://github.com/osrf/vrx/wiki/vrx_2023-phase2_practice_worlds)教程，了解第二阶段练习的建议。
* [Results](https://github.com/osrf/vrx/wiki/vrx_2023-phase2_results)

### [第三阶段：VRX 挑战赛](https://github.com/osrf/vrx/wiki/vrx_2023-phase3_challenge)

主要赛事。团队提交完成的解决方案并获得分数和最终排名。

* [Results](https://github.com/osrf/vrx/wiki/vrx_2023-phase3_results)

## 测试您的解决方案

* 在提交之前，[确保您的 WAM-V 配置合规](https://github.com/osrf/vrx/wiki/vrx_2023-wamv_compliance)。
* 接下来，[验证您的解决方案](https://github.com/osrf/vrx/wiki/vrx_2023-validation)以确保它通过简单的健全性检查。
* 在开发解决方案时，我们强烈建议您通过运行 VRX 服务器的本地副本来[评估它](https://github.com/osrf/vrx/wiki/vrx_2023-testing)，以查看您在各种任务中的得分。

## 提交您的解决方案

* [提交流程](https://github.com/osrf/vrx/wiki/vrx_2023-submission_process)：
  如何为某个阶段提交您的解决方案。

## 查看您的成绩

* 我们将在每个阶段的结果可用时发布结果和经验教训。
* [下载日志](https://github.com/osrf/vrx/wiki/download_logs)：
  如何在阶段完成后下载您的日志文件。

| Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [第一阶段：Hello World](https://github.com/osrf/vrx/wiki/vrx_2023-phase1_hello_world) |
| --- | --- |


---


## 第一阶段: Hello World

这个简单的检查鼓励团队尽早开始，并且是识别仿真环境技术问题的一种手段。此阶段的目标是让团队展示他们可以在本地（自己的计算机上）运行 VRX 仿真环境，并展示 VRX 任务的早期原型解决方案。

**新内容**：为鼓励团队熟悉后续阶段的提交流程，我们现在要求团队在提交视频的同时也提交一个最小的 Docker 镜像和 WAM-V 配置。详情请见下文。

## 交付物

此阶段的交付物包括一个视频和一个包含 WAM-V 配置文件的最小参赛者镜像。

### 视频交付物

此 VRX 挑战阶段的主要交付物是在线视频。视频内容故意是开放式的。我们鼓励您创造性地思考如何展示您当前的状态。唯一的要求是您的视频演示您的团队已经在本地设置了 VRX 环境。为了更进一步，我们建议团队考虑包含更多关于他们状态的信息，并展示他们已经能够实现的内容。将提交视为您团队状态的精彩视频。

视频长度应限制在 5 分钟以内。

### 参赛者镜像交付物

除了视频外，团队还应创建并提交一个最小工作系统，打包为包含两个 WAM-V 配置文件的 Docker 镜像。请注意，系统不必解决任何任务；此练习的唯一目的是让您提前了解为第二和第三阶段打包提交的过程。此交付物所需的文件可以通过以下教程生成（无需自定义）：

* [VRX 参赛者 Docker 入门](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_orientation)
* [创建参赛者镜像](https://github.com/osrf/vrx/wiki/tutorials-vrx_competitor_image)
* [自定义 WAM-V](https://github.com/osrf/vrx/wiki/Customizing-the-WAM-V)

## 准备系统的资源

* 团队可以访问 [VRX 代码、文档和教程](https://github.com/osrf/vrx)以支持设置本地开发环境。
* VRX 2023 任务描述文档可在 [VRX 网站](https://robotx.org/vrx-2023) 上获取，其中包含有关任务以及未见竞赛场景中预期环境范围的所有详细信息。
* 如需技术支持，鼓励团队向 [VRX 问题跟踪器](https://github.com/osrf/vrx/issues) 提交。

## 提交流程

要提交参赛作品，您必须执行以下操作：

* 创建提交格式部分（如下）中描述的所需文件；
* 按照[提交流程教程](https://github.com/osrf/vrx/wiki/vrx_2023-submission_process)中的说明提交这些文件。

### 提交格式

我们期望在此次活动之前从每位参赛者那里收到四个文件：

* `video_link.txt`：仅包含团队在线视频提交的 URL
* `dockerhub_image.txt`：包含要从 DockerHub 拉取的镜像名称。
* `thruster_config.yaml`：定义 WAM-V 推进器配置（[参见教程示例](https://github.com/osrf/vrx/wiki/Customizing-the-WAM-V)）。
* `component_config.yaml`：定义 WAM-V 组件配置（[参见教程示例](https://github.com/osrf/vrx/wiki/Customizing-the-WAM-V)）。

#### 重要提示：

* 如果您使用私有 DockerHub 仓库，请授予 `virtualrobotx` DockerHub 用户访问权限。否则我们将无法评估您的提交。
* 按照 [WAMV 合规教程](https://github.com/osrf/vrx/wiki/wamv_compliance) 确保您的 WAM-V 配置符合竞赛指南。

### 提交测试

在提交 pull request 之前，我们建议您验证您的提交以确保其符合提交要求。以下教程将指导您完成此过程：

* [如何验证您的提交](https://github.com/osrf/vrx/wiki/vrx_2023-validation)。

一旦您提交了 pull request，VRX 技术团队将在合并（接受）提交之前做两件事：

1. 检查 WAM-V 推进器和组件配置是否符合 VRX 技术指南中描述的配置约束。
2. 检查 DockerHub 镜像是否可被 `virtualrobotx` DockerHub 用户访问。

一旦满足这两个要求，pull request 将被合并，您的提交将被接受。所有通过上述验证检查的提交都将被接受，提交团队将进入下一阶段，无论系统是否处于工作状态。

### 额外测试

还鼓励团队验证提交是否正常运行，并能够与最终用于为竞赛评分的 VRX 服务器通信。以下教程提供了测试是否如此的说明：

* [如何运行试运行并自行评分](https://github.com/osrf/vrx/wiki/vrx_2023-testing)。

如果运行正常，您应该看到 WAM-V 响应您从参赛者镜像发送的命令而移动。

## 重要日期

| 日期 | 描述 |
| --- | --- |
| 2023 年 9 月 5 日 23:59 PST | 提交文件截止日期 |
| 2023 年 9 月 6 日 23:59 PST | 提交更正截止日期 |
| 2023 年 9 月 7 日 | 结果发布 |

关于提交更正的说明：我们在日程中安排了一天的额外时间来修复与提交文件内容相关的任何问题（例如：`video_link.txt` 中视频 URL 的拼写错误、不合规的组件或推进器配置、指向 DockerHub 镜像的拼写错误、DockerHub 权限问题导致我们无法下载您的解决方案镜像等）。团队在初始提交截止日期之后不得修改视频或 Docker 镜像。

## 奖项

虽然第一阶段没有官方奖项，但如果有出色的参赛作品，我们希望能够适当地认可它们。

| Back: [Overview](https://github.com/osrf/vrx/wiki/vrx_2023-participation_overview) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [第二阶段：彩排](https://github.com/osrf/vrx/wiki/vrx_2023-phase2_dress_rehearsal) |
| --- | --- | --- |


---


## 第二阶段: 彩排

对于 2023 年第二阶段 - 彩排，团队将提交他们的系统，以便针对以前未见过的场景运行。
团队已经接触了彩排场景中将出现的所有任务。
必须设计一个单一系统来解决 VRX 2023 中的任何任务。

## 预期内容

VRX 2023 任务描述和技术指南可在 [VRX 2023 竞赛网站](https://robotx.org/programs/vrx-competition-2023/) 上获取，其中包含有关任务以及未见竞赛场景中预期环境范围的所有详细信息。

## 准备系统

为确保您的系统能够适应以前未见过的场景，团队应至少针对所有已发布的示例任务测试他们的系统。

为了帮助您准备，我们为 VRX 2023 中的每个任务提供了 3 个[示例世界](https://github.com/osrf/vrx/tree/main/vrx_gz/worlds/2023_practice)。这些示例世界大致代表了任务文档中指定的任务配置和环境条件。[第二阶段练习世界教程](https://github.com//osrf/vrx/wiki/vrx_2023-phase2_practice_worlds)解释了这些世界的标签以及如何使用它们来练习竞赛。

对于您第二阶段提交的评估，解决方案将通过每个示例世界运行，以及一些在第二阶段截止日期之前未发布的新配置。这种方法的目的是让团队练习他们在第二阶段竞赛中将看到的一些实际场景，以便他们可以将本地性能与阶段结果进行比较。我们还引入了新的、以前未见过的场景，以测试解决方案处理新配置和条件的能力。

## 提交流程

要提交参赛作品，您必须执行以下操作：

* 创建一个或多个任务的解决方案。
* [创建包含您解决方案的 Docker 镜像并上传到 Docker Hub。](https://github.com/osrf/vrx/wiki/tutorials-vrx_competitor_image)
* 创建提交格式部分（如下）中描述的所需文件；
* 按照[提交流程教程](https://github.com/osrf/vrx/wiki/vrx_2023-submission_process)中的说明提交这些文件。

### 提交格式

我们期望在此次活动之前从每位参赛者那里收到三个文件：

* `dockerhub_image.txt`：包含要从 DockerHub 拉取的镜像名称。
* `thruster_config.yaml`：定义 WAM-V 推进器配置（[参见教程示例](https://github.com/osrf/vrx/wiki/Customizing-the-WAM-V)）。
* `component_config.yaml`：定义 WAM-V 组件配置（[参见教程示例](https://github.com/osrf/vrx/wiki/Customizing-the-WAM-V)）。.

#### 重要提示：

* 如果您使用私有 DockerHub 仓库，请授予 `virtualrobotx` DockerHub 用户访问权限。否则我们将无法评估您的提交。
* 按照 [WAMV 合规教程](https://github.com/osrf/vrx/wiki/wamv_compliance) 确保您的 WAM-V 配置符合竞赛指南。

### 提交测试

在提交 pull request 之前，我们建议您验证您的提交并评估它以确保其按预期工作。请参阅以下教程了解此过程的详细信息：

* [如何验证您的提交](https://github.com/osrf/vrx/wiki/vrx_2023-validation)。
* [如何运行试运行并自行评分](https://github.com/osrf/vrx/wiki/vrx_2023-testing)。

一旦您提交了 pull request，VRX 技术团队将在合并（接受）提交之前做两件事：

1. 检查 WAM-V 推进器和组件配置是否符合 VRX 技术指南中描述的配置约束。
2. 检查 DockerHub 镜像是否可被 `virtualrobotx` DockerHub 用户访问。

一旦满足这两个要求，pull request 将被合并，您的提交将被视为准备就绪。

## 重要日期

| 日期 | 描述 |
| --- | --- |
| 2023 年 10 月 3 日 23:59 PDT | 提交解决方案截止日期 |
| 2023 年 10 月 4 日 23:59 PDT | 提交更正截止日期 |
| 2023 年 10 月 11 日 | 结果发布 |

**关于提交更正的说明**：我们在日程中安排了一天的额外时间来修复与提交文件内容相关的任何问题（例如：不合规的组件或推进器配置、指向 DockerHub 镜像的拼写错误、DockerHub 权限问题导致我们无法下载您的解决方案镜像等）。团队在解决方案提交截止日期之后不得修改 DockerHub 镜像。

## 评分与结果

根据 VRX 竞赛和任务描述，"要获得第三阶段的资格，团队必须按照 VRX 技术指南中描述的方式提交他们的解决方案以进行自动评估。"所有提供这三个文件的团队将进入第三阶段。

提交将由技术团队评分，分数将提供给团队，但彩排分数不计入最终分数。它仅用于帮助团队为决赛做准备。

### 关于 Docker 可选参数的说明

Docker 的某些功能需要额外的命令行参数。如果您的团队希望在执行提交的 docker 镜像时指定额外的命令行参数，请尽快通过在问题跟踪器中创建新问题告知 VRX 技术团队。我们将逐案处理这些请求。我们将尽最大努力在为所有团队提供公平竞争的同时，对团队解决方案保持尽可能的灵活性。

我们不期望许多团队需要添加额外命令行参数的灵活性，但在某些情况下，如果它可以让团队在不给他们不公平竞争优势的情况下进行创新，我们希望努力实现这一点。

| Back: [第一阶段：Hello World](https://github.com/osrf/vrx/wiki/vrx_2023-phase1_hello_world) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [第二阶段练习世界](https://github.com/osrf/vrx/wiki/vrx_2023-phase2_practice_worlds) |
| --- | --- | --- |


---


## 第二阶段练习世界

# 2023 第二阶段任务练习世界

本教程的目的是为准备[第二阶段 - 彩排](https://github.com/osrf/vrx/wiki/vrx_2023-phase2_dress_rehearsal)提供本地测试的分步指导。如 VRX 竞赛文档中所述，每个 VRX **任务**将通过多个**试验**进行评估，每个试验以不同的配置（例如不同的航点位置）和不同的环境条件（风、波浪、光照等）呈现任务。环境范围的规范（可能的环境参数范围）包含在 VRX 技术指南中，每个任务的规范在 VRX 竞赛和任务描述中 - 两个文档均可在 [VRX 网站](https://www.robotx.org/vrx-2023) 上获取。

[VRX 任务：示例](https://github.com/osrf/vrx/wiki/vrx_2023-task_tutorials) wiki 提供了每个任务的通用说明和单个试验示例，这是一个很好的起点。但是，如果您的解决方案要在竞赛中表现出色，它应该能够在广泛的任务和环境条件下运行。本教程的目的是向您展示...

1. How to locally test each task with multiple trials
2. How to evaluate and verify your performance

对于本教程，所有测试都将在本地（主机或本地 Docker 容器上）完成。对于实际竞赛，这些解决方案将被自动评估。如果您有兴趣测试评估的这一方面，您可以使用 [vrx-docker 仓库](https://github.com/osrf/vrx-docker/) 中描述的工具设置您自己的评估环境，等同于竞赛中使用的环境。

## 示例试验

我们为每个任务生成了三个试验，涵盖了大部分允许的任务和环境参数。这些试验在概念上是

0. Easy - simplified task in negligible wave, wind and visual (fog) environmental factors.
1. Medium - moderate task difficulty and environmental influence.
2. Hard - at or close to the limit of task difficulty and environmental factors.

我们的意图是使用非常相似（但不完全相同）的每个任务试验来执行第二阶段挑战提交的评估。每个试验由世界和模型组成，以定义任务实例和操作环境。

### 运行示例试验

所有示例都存储在 `vrx/vrx_gz/worlds/2023_practice` 目录中。

您应该能够按如下方式运行各个示例

```
TASK=stationkeeping
TRIAL=0
ros2 launch vrx_gz competition.launch.py world:=practice_2023_${TASK}${TRIAL}_task
```

您需要更改 `TASK` 和 `TRIAL` 变量的值以指定要运行的世界。

| Back: [第二阶段：彩排](https://github.com/osrf/vrx/wiki/vrx_2023-phase2_dress_rehearsal) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [第三阶段：VRX 挑战赛](https://github.com/osrf/vrx/wiki/vrx_2023-phase3_challenge) |
| --- | --- | --- |


---


## 第三阶段: VRX 挑战赛

第三阶段的过程和内容将与[第二阶段：彩排](https://github.com/osrf/vrx/wiki/vrx_2023-phase2_dress_rehearsal)非常相似。

## 预期内容

* VRX 2023 任务描述和技术指南可在 [VRX 2023 竞赛网站](https://robotx.org/programs/vrx-competition-2023/) 上获取，其中包含有关任务以及未见竞赛场景中预期环境范围的所有详细信息。
* 与第二阶段一样，团队表现将通过在每个任务的多个试验上运行团队提交来评估，每个试验实例化一个与一般任务描述一致的特定任务场景。
* 提交流程将与第二阶段相同。

## 准备系统

为确保您的系统能够适应以前未见过的场景，团队应至少针对所有已发布的示例任务测试他们的系统。

与第二阶段一样，您可以使用我们为 VRX 2023 中每个任务提供的 3 个[示例世界](https://github.com/osrf/vrx/tree/main/vrx_gz/worlds/2023_practice)来准备此阶段。这些示例世界大致代表了任务文档中指定的任务配置和环境条件。[第二阶段练习世界教程](https://github.com//osrf/vrx/wiki/vrx_2023-phase2_practice_worlds)解释了这些世界的标签以及如何使用它们来练习竞赛。

对于您第三阶段提交的评估，解决方案将通过在第三阶段截止日期之前未发布的新配置运行，以测试解决方案处理新配置和条件的能力。

## 提交流程

要提交参赛作品，您必须执行以下操作：

* [为您的 WAMV 定义合规的推进器和组件配置。](https://github.com/osrf/vrx/wiki/Customizing-the-WAM-V)
* 创建一个或多个任务的解决方案。
* [创建包含您解决方案的 Docker 镜像并上传到 Docker Hub。](https://github.com/osrf/vrx/wiki/tutorials-vrx_competitor_image)
* 创建提交格式部分（如下）中描述的所需文件；
* 按照[提交流程教程](https://github.com/osrf/vrx/wiki/vrx_2023-submission_process)中的说明提交这些文件。

### 提交格式

我们期望在此次活动之前从每位参赛者那里收到三个文件：

* `dockerhub_image.txt`：包含要从 DockerHub 拉取的镜像名称。
* `thruster_config.yaml`：定义 WAM-V 推进器配置（[参见教程示例](https://github.com/osrf/vrx/wiki/Customizing-the-WAM-V)）。
* `component_config.yaml`：定义 WAM-V 组件配置（[参见教程示例](https://github.com/osrf/vrx/wiki/Customizing-the-WAM-V)）。

#### 重要提示：

* 如果您使用私有 DockerHub 仓库，请授予 `virtualrobotx` DockerHub 用户访问权限。否则我们将无法评估您的提交。
* 按照 [WAMV 合规教程](https://github.com/osrf/vrx/wiki/wamv_compliance) 确保您的 WAM-V 配置符合竞赛指南。

### 提交测试

我们强烈建议每个团队按照[测试您的提交](https://github.com/osrf/vrx/wiki/vrx_2023-testing)\* wiki 测试他们的解决方案和 docker 镜像。这将使团队能够在提交之前重现评估和评分系统。

### 提交流程说明

在提交 pull request 之前，我们建议您验证您的提交并评估它以确保其按预期工作。请参阅以下教程了解此过程的详细信息：

* [如何验证您的提交](https://github.com/osrf/vrx/wiki/vrx_2023-validation)。
* [如何运行试运行并自行评分](https://github.com/osrf/vrx/wiki/vrx_2023-testing)。

一旦您提交了 pull request，VRX 技术团队将在合并（接受）提交之前做两件事：

1. Check that the WAM-V thruster and component configuration complies with the configuration constraints described in the VRX Technical Guide
2. 检查 DockerHub 镜像是否可被 virtualrobotx DockerHub 用户访问。

一旦满足这两个要求，pull request 将被合并，您的提交将被视为准备就绪。

## 重要日期

| 日期 | 描述 |
| --- | --- |
| 2023 年 11 月 1 日 23:59 PDT | 提交解决方案截止日期 |
| 2023 年 11 月 2 日 23:59 PDT | 提交更正截止日期 |
| 2023 年 11 月 16 日 | 结果发布 |

关于提交更正的说明：我们在日程中安排了一天的额外时间来修复与提交文件内容相关的任何问题（例如：不合规的组件或推进器配置、指向 dockerhub 镜像的拼写错误、dockerhub 权限问题导致 open robotics 无法下载您的解决方案镜像等）。团队在解决方案提交截止日期之后不得修改 dockerhub 镜像。

## 逾期提交

通常，逾期提交将不被接受。所有团队必须在解决方案截止日期之前提交所需要素。技术团队保留接受因提交过程中的不可预见问题而导致延迟的提交的权利。在解决方案截止日期之后接受任何提交将由 VRX 技术团队自行决定。

### 关于 Docker 可选参数的说明

Docker 的某些功能需要额外的命令行参数。如果您的团队希望在执行提交的 docker 镜像时指定额外的命令行参数，请尽快通过在问题跟踪器中创建新问题告知 VRX 技术团队。我们将逐案处理这些请求。我们将尽最大努力在为所有团队提供公平竞争的同时，对团队解决方案保持尽可能的灵活性。

我们不期望许多团队需要添加额外命令行参数的灵活性，但在某些情况下，如果它可以让团队在不给他们不公平竞争优势的情况下进行创新，我们希望努力实现这一点。

| Back: [第二阶段练习世界](https://github.com/osrf/vrx/wiki/vrx_2023-phase2_practice_worlds) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [提交流程](https://github.com/osrf/vrx/wiki/vrx_2023-submission_process) |
| --- | --- | --- |


---


## 提交流程

提交将通过 [vrx-events](https://github.com/osrf/vrx-events) 仓库进行。所有注册团队必须执行以下步骤来提交每个阶段的解决方案：

### 1. Fork `vrx-events` 仓库

* 点击[此链接](https://github.com/osrf/vrx-events/fork)。
* 您可以为仓库选择自定义名称，但这里我们将保留默认值 `vrx-events`。
* 完成 fork 过程后，您应该在 `https://github.com/<yourname>/vrx-events` 上拥有 `vrx-events` 的副本。

**注意：** 在这些教程中，将 <yourname> 替换为您的 GitHub 帐户用户名。

### 2. 克隆 `vrx-events` 仓库

现在您已经 fork 了 VRX events 仓库，下载本地副本并进入仓库根目录：

```
git clone git@github.com:<yourname>/vrx-events.git
cd vrx-events
```

### 3. 添加您的提交文件

导航到您计划参加的活动（例如：`2023/phase2_dress_rehearsal`）并创建一个以您的团队名称命名的目录：

```
cd <year>/<event>
mkdir <teamname>
cd <teamname>
```

* 将 <year> 和 <event> 替换为您计划参加的年份和活动，将 <teamname> 替换为您的团队名称。
* 将活动所需的所有文件复制到您的团队文件夹中。
* 按照活动页面中详细说明的操作获取要提交的文件的完整列表。

### 4. 测试您的提交（仅限第二和第三阶段）

在提交 pull request 之前，我们建议您验证您的提交并评估它以确保其按预期工作。请参阅以下教程了解此过程的详细信息：

* [检查您的 WAM-V 配置是否合规](https://github.com/osrf/vrx/wiki/vrx_2023-wamv_compliance)
* [如何验证您的提交](https://github.com/osrf/vrx/wiki/vrx_2023-validation)。
* [如何运行试运行并自行评分](https://github.com/osrf/vrx/wiki/vrx_2023-testing)。

### 5. 提交您的 pull request

1. 准备 pull request 时，在您的仓库中创建一个分支来跟踪您要提交的文件，例如：

   ```
   git checkout -b 2023_rehearsal_team_osrf
   ```
2. 使用 add 命令告诉 git 您要将刚刚复制的文件添加到您创建的新分支：

   ```
   git add *
   ```

   （请注意，在您提交更改之前，文件不会实际添加。）
3. 验证您添加的正是您想要的文件（没有其他文件）：

   ```
   git status
   ```

   **注意：** 此命令将给出 git 在下次提交时将进行的更改列表。如果这不是您想要的（特别是如果您意外添加了太多文件），请不要继续下一步！您需要在继续之前排除故障并纠正问题。如果您是 git 新手，您可能会发现[这些提示很有帮助](https://www.codementor.io/@citizen428/git-tutorial-10-common-git-problems-and-how-to-fix-them-aajv0katd)。
4. 将更改提交到本地 git 仓库：

   ```
   git commit -m "Team OSRF submission for 2023/rehearsal event."
   ```

   * `-m "TEXT"` 参数提供提交消息/注释。这是必需的。
5. 将本地更改推送到远程（在线）仓库。

   ```
   git push -u origin 2023_rehearsal_team_osrf
   ```
6. 为 `vrx-events` 打开一个 pull request。

   * 导航到您的在线 pull request 页面，链接如下：`https://github.com/<yourname>/vrx-events/pulls/new`
   * 点击 "New Pull Request"。
   * 勾选左侧两个框以确保您的 pull request 针对 `osrf/vrx-events` 仓库的 `master` 分支。
   * 右侧的两个框应指定您添加提交文件的分支。
   * 请务必在标题中包含年份、活动名称和您的团队名称。例如：2023 rehearsal team_osrf
   * 我们将假设您的解决方案适用于所有任务。如果不是这种情况，请在描述中告知我们。例如：*"此提交处理任务 #1、#2 和 #3，但不适用于任务 #4、#5 或 #6"*。此信息对我们非常有用，因为我们将在批准您的 pull request 之前对您的提交进行初步评估。如果我们观察到您的 WAM-V 没有移动，我们将在您的 pull request 中回复以确认这是预期的还是 Docker 镜像中存在问题，除非您之前在描述中警告过我们。
   * 点击 `Create pull request` 按钮，等待您的 pull request 被批准和合并。

![vrx_2023_submission](https://private-user-images.githubusercontent.com/8611855/238393530-2230a2a2-23c8-4327-a45b-001003fa1622.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3ODM1MjgyNDEsIm5iZiI6MTc4MzUyNzk0MSwicGF0aCI6Ii84NjExODU1LzIzODM5MzUzMC0yMjMwYTJhMi0yM2M4LTQzMjctYTQ1Yi0wMDEwMDNmYTE2MjIucG5nP1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI2MDcwOCUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNjA3MDhUMTYyNTQxWiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9MjEwYzY2ZGI5ZTVhZjQ5ZGE4MWYzZDllYmRjMTEzZjQzZmI2MTQ1ZmJlM2JjZmNmMTYyZjgzZWNhOGZhMTFmMSZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QmcmVzcG9uc2UtY29udGVudC10eXBlPWltYWdlJTJGcG5nIn0.TD4mO3F9puuOEgsIY0c8GMX4oFRKpD-34ZLojQY1_Mo)

| Back: [第三阶段：VRX 挑战赛](https://github.com/osrf/vrx/wiki/vrx_2023-phase3_challenge) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [WAM-V Compliance](https://github.com/osrf/vrx/wiki/vrx_2023-wamv_compliance) |
| --- | --- | --- |


---


## WAMV 合规性

# 欢迎访问 VRX Wiki

本 Wiki 为 VRX 仿真环境提供技术文档和教程，以及 VRX 环境支持的虚拟活动和竞赛。

## VRX 仿真环境

多功能 VRX 仿真环境是一个可扩展框架，致力于促进无人水面艇 (USV) 自主性的设计、开发和评估。

## [教程](https://github.com/osrf/vrx/wiki/tutorials)

如何使用 VRX 仿真环境。

## 竞赛

VRX 最初是为了满足各类竞赛的需求而设计的，例如：

* [2023 Virtual RobotX (VRX) Competition](https://github.com/osrf/vrx/wiki/vrx_2023-participation_overview)
* [2022 Virtual RobotX (VRX) Competition](https://robotx.org/programs/vrx-competition-2022/)
* 2020 Virtual Ocean Robotics Challenge (VORC)
* [2019 Virtual RobotX (VRX) Competition](https://robotx.org/programs/2019-virtual-robotx-competition/)
* [2019 RobotX Interactive Forum Hackathon (PDF)](https://robonation.org/app/uploads/sites/2/2019/09/2019-RobotX-Interactive-Forum-2019-Program.pdf)

鉴于与 [RobotBoat](https://robonation.org/programs/roboboat/) 的相似性，团队已通过引入世界和模型来扩展环境，专门支持开发 [RoboBoat 解决方案](https://github.com/osrf/vrx/wiki/tutorials#roboboat) 的团队。

## 研究

VRX 仿真环境已被海事机器人研发社区广泛采用，不断演进以适应感知、学习和控制方面的进步，同时探索 USV 能力的新应用。有关我们 OCEANS 论文的更多详情，请参阅[引用](https://ieeexplore.ieee.org/document/8962724/citations?tabFilter=papers#citations)。

* [Deep-Reinforcement-Learning-Based Motion Control for Unmanned Surface Vehicles with Environmental Disturbances](https://ieeexplore.ieee.org/document/10318284)
* [Dynamic Obstacle Avoidance for USVs Using Cross-Domain Deep Reinforcement Learning and Neural Network Model Predictive Controller](https://www.mdpi.com/1424-8220/23/7/3572)
* [Vision-Guided UAV Landing on a Swaying Ocean Platform in Simulation](https://ieeexplore.ieee.org/document/10249476)
* [COLREG-Compliant Simulation Environment for Verifying USV Motion Planning Algorithms](https://ieeexplore.ieee.org/document/10244676) with corresponding [source code](https://github.com/FieldRoboticsLab/MultiVessel_Simulation)
* [Multi-domain inspection of offshore wind farms using an autonomous surface vehicle](https://link.springer.com/article/10.1007/s42452-021-04451-5)

自 3.0 版本起，VRX 仿真环境默认使用 Gazebo Sim Harmonic 和 ROS 2 Jazzy。这是新用户的推荐起点，以下链接的教程均假设使用此配置。

## VRX Classic

希望保持与 Gazebo Classic 和 ROS 1 兼容性的用户，可以通过构建此仓库的 `gazebo_classic` 分支来实现。

* [点击此处访问 VRX Classic 的 Wiki 页面。](https://github.com/osrf/vrx/wiki/VRX-Classic-Home)
* **重要提示**：我们已于 2023 年春季将 `gazebo_classic` 分支从官方支持分支转变为社区支持分支。

## 如何引用

如果您在工作中使用了 VRX 仿真，请引用我们的总结论文 "Toward Maritime Robotic Simulation in Gazebo"。

```
@InProceedings{bingham19toward,
  Title                    = {Toward Maritime Robotic Simulation in Gazebo},
  Author                   = {Brian Bingham and Carlos Aguero and Michael McCarrin and Joseph Klamo and Joshua Malia and Kevin Allen and Tyler Lum and Marshall Rawson and Rumman Waqar},
  Booktitle                = {Proceedings of MTS/IEEE OCEANS Conference},
  Year                     = {2019},
  Address                  = {Seattle, WA},
  Month                    = {October}
}
```


---


## 验证

# 如何验证您的提交

本教程将允许您检查您对第二或第三阶段的提交是否符合 VRX 竞赛要求。

## 前提条件

我们假设您已经安装了 VRX 并准备好了要验证的提交。这意味着：

* 您已经[创建了一个包含您解决方案的 Docker 镜像](https://github.com/osrf/vrx/wiki/tutorials-vrx_competitor_image)并上传到 Docker hub。
* 您已经为[第一阶段](https://github.com/osrf/vrx/wiki/vrx_2023-phase1_hello_world)、[第二阶段](https://github.com/osrf/vrx/wiki/vrx_2023-phase2_dress_rehearsal)或[第三阶段](https://github.com/osrf/vrx/wiki/vrx_2023-phase3_challenge)提交创建了所需文件。

## 步骤 1：准备验证

1. Change into the `vrx_ws/src` directory you created when installing `vrx`.

   ```
   cd ~/vrx_ws/src
   ```
2. Clone the `vrx-docker` repository.

   ```
   git clone https://github.com/osrf/vrx-docker
   ```

   This should create a new `vrx-docker` directory alongside the original `vrx` repository directory.
3. Source your `bash.setup` file, change into the `vrx-docker` directory, and set the variable `TEAM` for later use:

   ```
   source ~/vrx_ws/devel/setup.bash
   cd vrx-docker
   TEAM=<your_team_name>
   ```

   Replace `<your_team_name>` with the team name you will use for your submission.
4. Create a new folder called `<your_team_name>` in the `vrx-docker/team_config` folder.

   ```
   mkdir team_config/$TEAM
   ```
5. Copy your submission files (yaml config files and docker hub image name) to the `vrx-docker/team_config/<your_team_name>` folder.

## 步骤 2：验证您的 dockerhub_image.txt 文件。

要测试您的 dockerhub_image.txt 文件是否包含可访问的 docker 镜像名称（和版本），请运行：

```
#!bash

cat "team_config/$TEAM/dockerhub_image.txt" | xargs docker pull
```

如果文件内容正确，docker 应该开始拉取您的镜像。一旦验证正常，您可以使用 `ctrl+c` 退出拉取。

## 步骤 3：检查推进和传感器合规性。

1. Run the prepare_team_wamv.bash script included with `vrx-docker` to set up your team's wamv configuration:

   ```
   ./prepare_team_wamv.bash "$TEAM"
   ```

   Note that this will produce a `REQUIRED process [wamv_config/wamv_generator-2] has died!` message, which is expected.
2. Check compliance:

   ```
   cat "generated/team_generated/$TEAM/compliant.txt"
   ```

   The output of the above should be **true** if your configuration passes compliance tests.

| Back: [提交流程](https://github.com/osrf/vrx/wiki/vrx_2023-submission_process) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [测试](https://github.com/osrf/vrx/wiki/vrx_2023-testing) |
| --- | --- | --- |


---


## 测试

本教程将允许您验证您的提交是否按预期工作。

## 依赖项

以下步骤假设您已安装并配置 Docker 以与您的 Nvidia 显卡配合工作。

* 如果您尚未在系统上设置 Docker，请在继续之前完成[本教程](https://github.com/Field-Robotics-Lab/dockwater/wiki/Install-Dependencies)的步骤 1 和 2。请注意，目前不需要步骤 3（安装 Rocker）。
* 此外，目前您必须*也*按照[基于主机的安装说明](https://github.com/osrf/vrx/wiki/preparing_system_tutorial)在主机系统上安装 VRX。
  + 注意：我们正在努力消除此依赖项。

## 前提条件

要完成本教程，您需要已经准备了一个或多个 VRX 任务的解决方案。这意味着：

* 您已经[创建了一个包含您解决方案的 Docker 镜像](https://github.com/osrf/vrx/wiki/tutorials-vrx_competitor_image)并上传到 Docker hub。
* 您已经为[第二阶段](https://github.com/osrf/vrx/wiki/vrx_2022-phase2_dress_rehearsal)或[第三阶段](https://github.com/osrf/vrx/wiki/vrx_2022-phase3_challenge)提交创建了所需文件。

## 步骤 1：准备本地测试环境

此步骤在您的系统上安装必要的测试工具。您应该只需要执行一次。

1. Change into the `vrx_ws/src` directory you created when installing `vrx`.

   ```
   cd ~/vrx_ws/src
   ```
2. Clone the `vrx-docker` repository.

   ```
   git clone https://github.com/osrf/vrx-docker
   ```

   This should create a new `vrx-docker` directory alongside the original `vrx` repository directory.
3. Source your `bash.setup` file, change into the `vrx-docker` directory, and set the variable `TEAM` for later use:

   ```
   source ~/vrx_ws/install/setup.bash
   cd vrx-docker
   TEAM=<your_team_name>
   ```

   Replace `<your_team_name>` with the team name you will use for your submission.
4. Build the vrx-server docker image (may take 30-60 minutes the first time):

   ```
   ./vrx_server/build_image.bash
   ```

## 步骤 2：配置您的测试

在此步骤中，您将设置平台并指定要评估的任务。

1. Copy your submission files (yaml config files and docker hub image name) to the `vrx-docker/team_config` folder.
2. Prepare your team's vehicle according to the configuration you provided:

   ```
   ./prepare_team_wamv.bash "$TEAM"
   ```
3. Define a TASK variable to indicate the task you wish to test against.

   ```
   TASK=stationkeeping
   ```

   Other valid options include `wayfinding`, `perception`, `acoustic_perception`,`wildlife`, `follow_path`, `acoustic_tracking`, and `scan_dock_deliver`.
4. Get the practice worlds provided for your task:

   ```
   cd ~/vrx_ws/src/vrx-docker
   mkdir -p generated/task_generated/$TASK/worlds
   for world in ~/vrx_ws/src/vrx/vrx_gz/worlds/2023_practice/practice_2023_$TASK*.sdf; do suffix=${world##*2023_}; name=${suffix%_task.sdf}.sdf; cp -v ${world} generated/task_generated/$TASK/worlds/$name; done
   ```
5. Define a TRIAL variable to indicate which of the task worlds you would like to run (options are 0,1,2).

   ```
   TRIAL=0
   ```

## 步骤 3：运行并评估您的解决方案

此步骤将在上述指定的任务和试验上测试您的提交：

1. Execute the `run_trial.bash` script with your team, task and trial variables as arguments:

   ```
   ./run_trial.bash $TEAM $TASK $TRIAL
   ```

   This command will run your submission image and the `vrx-server` image at the same time and generate multiple log files which are saved in the `vrx-docker/generated/logs` directory.
2. View your score for the task:

   ```
   cat generated/logs/$TEAM/$TASK/$TRIAL/trial_score.txt
   ```
3. See a replay of your system's performance on the task and verify that it behaved as you expected:

   ```
   ./replay_trial.bash $TEAM $TASK $TRIAL
   ```

## 故障排除

如果您已完成上述步骤且回放您的试验显示：

* 要么您的 WAMV 完全不移动，要么
* 您的 WAMV 在容器中的行为与在主机系统上直接运行时不匹配

那么您的 Docker 镜像设置可能存在问题。在这种情况下，请参阅我们的 [Docker 故障排除教程](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_troubleshooting) 获取一些检测和纠正 Docker 相关问题的建议。

| Back: [验证](https://github.com/osrf/vrx/wiki/vrx_2023-validation) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) |
| --- | --- |


---


## 如何练习

# VRX 任务：示例

对于 2023 年 VRX 竞赛中的每个单独任务，我们提供了仿真世界和 Gazebo 插件的示例，用于评估和评分任务性能。以下是适用于所有任务的通用说明以及运行各个示例的说明。

### 注意

* Detailed descriptions of each task, its application interface (API) and scoring is included in the Task Descriptions and Technical Guide, available on the [VRX 2023 website](https://robotx.org/programs/vrx-2023/).
* For instructions on how to participate in each phase of the VRX 2023 competition, please see the [VRX 2023 Overview](https://github.com/osrf/vrx/wiki/vrx_2023-overview).

## 通用说明：

### 初始状态

启动后，例如 `ros2 launch vrx_gz competition.launch.py world:=sydney_regatta`，下面给出的所有示例都应该从 wamv 漂浮在水面附近开始，如下所示：

![Initial position of WAM-V for VRX 2023](https://github.com/osrf/vrx/wiki/images/vrx_2023_initial.png)

附加的赛道元素将因任务而异。

### 监控任务状态

vrx 任务状态消息提供：

* 任务状态 {初始, 就绪, 运行中, 已完成}
* 当前分数
* 时间信息
* WAM-V 碰撞次数

任务状态发布到 `/vrx/task/info`。我们建议您在仿真期间监控任务状态。一种方法是运行：

```
ros2 topic echo /vrx/task/info
```

### 驾驶说明

在准备开发自动化解决方案时，我们建议使用游戏手柄驾驶 USV 通过赛道。要快速开始，请尝试：
`ros2 launch vrx_gz usv_joy_teleop.launch`

有关更多指导和信息，请查看[遥操作教程](https://github.com/osrf/vrx/wiki/teleop_tutorial)。

## 各任务详情

以下快速入门说明将引导您完成启动环境和订阅任何可用任务特定消息的初始过程。

### 任务 1：[定点保持](https://github.com/osrf/vrx/wiki/vrx_2023-stationkeeping_task)

### 任务 2：[寻路](https://github.com/osrf/vrx/wiki/vrx_2023-wayfinding_task)

### 任务 3：[感知](https://github.com/osrf/vrx/wiki/vrx_2023-perception_task)

### 任务 4：[声学感知](https://github.com/osrf/vrx/wiki/vrx_2023-acoustic_perception_task)

### 任务 5：[野生动物遭遇与避让](https://github.com/osrf/vrx/wiki/vrx_2023-wildlife_task)

### 任务 6：[沿路径行驶](https://github.com/osrf/vrx/wiki/vrx_2023-follow_the_path_task)

### 任务 7：[声学跟踪](https://github.com/osrf/vrx/wiki/vrx_2023-acoustic_tracking_task)

### 任务 8：[扫描、对接与交付](https://github.com/osrf/vrx/wiki/vrx_2023-scan_dock_deliver_task)

| Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [Stationkeeping](https://github.com/osrf/vrx/wiki/vrx_2023-stationkeeping_task) |
| --- | --- |


### 任务 1：定点保持

![](https://github.com/osrf/vrx/wiki/images/stationkeeping_2023.png)

## 总结

导航到目标位姿并保持驻留。最佳解决方案将在任务持续时间内最小化目标位姿与载具实际位姿之间的差异。

## 如何运行

### 步骤 1：启动示例

```
ros2 launch vrx_gz competition.launch.py world:=stationkeeping_task
```

### 步骤 2：订阅相关话题

* After starting the example, subscribe to the task-specific topics provided by the station-keeping scoring plugin.
  + To view the station-keeping goal (given as a [geometry_msgs/msg/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html) message):

```
ros2 topic echo /vrx/stationkeeping/goal
```

* 要获取超时计数器和当前分数，请订阅 `/vrx/task/info` 话题：

```
ros2 topic echo /vrx/task/info
```

### 步骤 3：完成任务

* In order to complete the task, you can manually drive the WAM-V to the goal and maintain that position with a gamepad:

```
ros2 launch vrx_gz usv_joy_teleop.launch
```

* Keep track of your position error by subscribing to the following topics:

```
ros2 topic echo /vrx/stationkeeping/pose_error
```

```
ros2 topic echo /vrx/stationkeeping/mean_pose_error
```

| Back: [序列概览](https://github.com/osrf/vrx/wiki/vrx_2023-task_tutorials) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [Wayfinding](https://github.com/osrf/vrx/wiki/vrx_2023-wayfinding_task) |
| --- | --- | --- |


### 任务 2：寻路

![](https://github.com/osrf/vrx/wiki/images/wayfinding_2023.png)

## 总结

导航通过每个发布的航点，使载具尽可能接近指定的位置和方向。

## 如何运行

### 步骤 1：启动示例

```
ros2 launch vrx_gz competition.launch.py world:=wayfinding_task
```

### 步骤 2：订阅相关话题

* After starting the example, subscribe to the task-specific topics provided by the wayfinding scoring plugin.
  + The list of waypoints (given as a [geometry_msgs/msg/PoseArray](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseArray.html) message):

```
rost2 opic echo /vrx/wayfinding/waypoints
```

* 要获取超时计数器和当前分数，请订阅 `/vrx/task/info` 话题：

```
ros2 topic echo /vrx/task/info
```

### 步骤 3：完成任务

* In order to complete the task, you can manually drive and orient the WAM-V to each of the waypoint positions with either a gamepad:

```
ros2 launch vrx_gz usv_joy_teleop.launch
```

有关更多指导和信息，请查看[遥操作教程](https://github.com/osrf/vrx/wiki/teleop_tutorial)。

* Keep track of your minimum errors for each waypoint so far,

```
ros2 topic echo /vrx/wayfinding/min_errors
```

* and the mean of the minimum errors:

```
ros2 topic echo /vrx/wayfinding/mean_error
```

| Back: [Stationkeeping](https://github.com/osrf/vrx/wiki/vrx_2023-stationkeeping_task) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [Perception](https://github.com/osrf/vrx/wiki/vrx_2023-perception_task) |
| --- | --- | --- |


### 任务 3：感知

![](https://github.com/osrf/vrx/wiki/images/perception_2023.png)

## 总结

在此任务中，载具保持在固定位置，标记将出现在视野中。目标是使用感知传感器识别标记并报告其位置。

## 如何运行

### 步骤 1：启动示例

```
ros2 launch vrx_gz competition.launch.py world:=perception_task
```

### 步骤 2：订阅相关话题并查看图像

* View the camera feeds from the front of the WAM-V:

```
ros2 run rqt_image_view rqt_image_view --ros-args --remap image:=/wamv/sensors/cameras/middle_right_camera/image_raw
```

* 要获取超时计数器和当前分数，请订阅 `/vrx/task/info` 话题：

```
ros2 topic echo /vrx/task/info
```

### 步骤 3：完成任务

* Trials will begin. Identify the type and location of the markers that appear during each trial.
* Publish landmark identification and localization solutions as a [geometry_msgs/msg/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html) message to the `/vrx/perception/landmark` topic. (Example: `ros2 topic pub -1 /vrx/perception/landmark geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "mb_marker_buoy_red"}, pose: {position: {x: -33.7227024, y: 150.67402097, z: 0.0}}}'`

### 注意:

* Each trial will last for 5 seconds
* Solutions must be submitted before the end of the trial
* Only the first submission for each trial will be considered

| Back: [Wayfinding](https://github.com/osrf/vrx/wiki/vrx_2023-wayfinding_task) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [Acoustic Perception](https://github.com/osrf/vrx/wiki/vrx_2023-acoustic_perception_task) |
| --- | --- | --- |


### 任务 4：声学感知

![](https://github.com/osrf/vrx/wiki/images/acoustic_perception_2023.png)

## 总结

水下声学信标广播距离、方位和仰角，指示其相对于 USV 的位置（带有噪声）。任务的目标是尽快导航到信标（1 米范围内）。

## 如何运行

### 步骤 1：启动示例

```
ros2 launch vrx_gz competition.launch.py world:=acoustic_perception_task
```

### 步骤 2：订阅相关话题

* The acoustic beacon advertises the range, bearing and elevation from the WAM-V. Subscribe to its topic (given as a [ros_gz_interfaces::msg::
  ParamVec](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_interfaces/msg/ParamVec.msg) message) via:

```
ros2 topic echo /wamv/pingers/pinger/range_bearing
```

* 要获取超时计数器和当前分数，请订阅 `/vrx/task/info` 话题：

```
ros2 topic echo /vrx/task/info
```

### 步骤 3：完成任务

* In order to complete the task, you can manually drive the WAM-V towards the beacon with a gamepad:

```
ros2 launch vrx_gz usv_joy_teleop.launch
```

每次运行的总体性能根据 WAM-V 到达信标位置所需的时间进行评分。

| Back: [Perception](https://github.com/osrf/vrx/wiki/vrx_2023-perception_task) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [Wildlife Encounter and Avoid](https://github.com/osrf/vrx/wiki/vrx_2023-wildlife_task) |
| --- | --- | --- |


### 任务 5：野生动物遭遇与避让

![](https://github.com/osrf/vrx/wiki/images/wildlife_2023.png)

## 总结

此任务要求系统跟踪一组代表动物生活的异构移动动物，并根据动物类型规划适当的操作。系统应规划并穿越一条路径，该路径

* circles clockwise around platypus markers,
* circles counterclockwise around turtle markers, and
* avoids (i.e. remains at a distance of 10m from) crocodile markers.

## 如何练习

### 步骤 1：启动示例

```
ros2 launch vrx_gz competition.launch.py world:=wildlife_task
```

### 步骤 2：订阅相关话题

以下 ROS 话题包含与此任务相关的信息：

```
/vrx/task/info
/vrx/wildlife/animalX/pose
```

with a separate pose topic for each animal in the world.

要获取超时计数器和当前分数，请订阅 `/vrx/task/info` 话题：

```
ros2 topic echo /vrx/task/info
```

要以球坐标接收每只动物的位置，请订阅感兴趣的 `/vrx/wildlife/animalX/pose` 话题：

```
ros2 topic echo /vrx/wildlife/animal0/pose
```

for the first animal, and replacing "X" with sequential integers for each additional animal.

### 步骤 3：完成任务

* Always stay 10 meters away from the crocodiles!
* Circumnavigate the platypus clockwise while staying within a 10 meter radius.
* Circumnavigate the turtles counterclockwise while staying within a 10 meter radius.

### 注意：

* If you hit an animal with the WAM-V, your current circumnavigation status is reset.
* The task finishes when all circumnavigable animals have been circumnavigated or the task timeouts.
* Your time bonus is always applied at the end of the task.
* For debugging purposes you can check the completion percentage in the console when you start circumnavigating an animal:

```
[ruby $(which gz) sim-1] [Dbg] [WildlifeScoringPlugin.cc:329] platypus::link Transition from NEVER_ENGAGED to ENGAGED
[ruby $(which gz) sim-1] [Dbg] [WildlifeScoringPlugin.cc:389] platypus::link Virtual gate incorrectly crossed counterclockwise! (0% completed)
```

| Back: [Acoustic Perception](https://github.com/osrf/vrx/wiki/vrx_2023-acoustic_perception_task) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [Follow the Path](https://github.com/osrf/vrx/wiki/vrx_2023-follow_the_path_task) |
| --- | --- | --- |


### 任务 6：路径跟随

![](https://github.com/osrf/vrx/wiki/images/follow_path_2023.png)

## 总结

此任务要求系统穿越由成对彩色浮标标记的通道，同时避开障碍物。

## 如何练习

### 步骤 1：启动示例

```
ros2 launch vrx_gz competition.launch.py world:=follow_path_task
```

### 步骤 2：订阅相关话题

要获取超时计数器和当前分数，请订阅 `/vrx/task/info` 话题：

```
rostopic echo /vrx/task/info
```

### 步骤 3：完成任务

* Navigate the channel, avoiding collisions with obstacles or buoys.
* To maximize score, the vehicle must traverse the gates in order.
* The first gate can be identified by the unique white buoy marking the left side of the channel. Crossing through this gate in the correct direction activates the course.
* Gates that are skipped or crossed in the wrong direction are out of play for the rest of the run and crossing them will not result in earning points. See the [VRX 2023 Task Description Document](https://robonation.org/app/uploads/sites/2/2023/05/VRX2023_Task-Descriptions_v1.1.pdf) for a full description of the rules and scoring for this task.

### 注意:

* Output in the terminal will let you know when a significant event has occurred - e.g. crossing a gate or colliding with an obstacle, as in this example:

```
[ruby $(which gz) sim-1] [Dbg] [NavigationScoringPlugin.cc:477] New gate crossed!
[ruby $(which gz) sim-1] [Dbg] [NavigationScoringPlugin.cc:493] Score: 10
[ruby $(which gz) sim-1] [Dbg] [ScoringPlugin.cc:631] [1] New collision counted between [wamv::wamv/base_link::wamv/base_link_fixed_joint_lump__left_front_float_collision_2] and [short_navigation_course_0::obstacle_0::link::collision]
[ruby $(which gz) sim-1] [Dbg] [NavigationScoringPlugin.cc:439] New penalty. score: 7
```

| Back: [Wildlife Encounter and Avoid](https://github.com/osrf/vrx/wiki/vrx_2023-wildlife_task) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [Acoustic Tracking](https://github.com/osrf/vrx/wiki/vrx_2023-acoustic_tracking_task) |
| --- | --- | --- |


### 任务 7：声学跟踪

![](https://github.com/osrf/vrx/wiki/images/acoustic_tracking_2023.png)

## 总结

在此任务中，载具将跟踪移动的水下声学信标，同时避开障碍物。

## 如何运行

### 步骤 1：启动示例

```
ros2 launch vrx_gz competition.launch.py world:=acoustic_tracking_task
```

### 步骤 2：订阅相关话题

* The acoustic beacon advertises the range, bearing and elevation from the WAM-V. Subscribe to its topic (given as a [ros_gz_interfaces::msg::
  ParamVec](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_interfaces/msg/ParamVec.msg) message) via:

```
ros2 topic echo /wamv/pingers/pinger/range_bearing
```

* 要获取超时计数器和当前分数，请订阅 `/vrx/task/info` 话题：

```
ros2 topic echo /vrx/task/info
```

### 步骤 3：完成任务

* In order to complete the task, you can manually drive the WAM-V to follow the beacon's path with a gamepad:

```
ros2 launch vrx_gz usv_joy_teleop.launch
```

每个时间步的目标位姿计算为声学信标到水面的垂直投影。您的分数根据 WAM-V 与当前目标位姿之间的距离递增 - 目标位姿会移动，因为信标在移动。与障碍物的碰撞将导致分数惩罚。

| Back: [Follow the Path](https://github.com/osrf/vrx/wiki/vrx_2023-follow_the_path_task) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [Scan and Dock and Deliver](https://github.com/osrf/vrx/wiki/vrx_2023-scan_dock_deliver_task) |
| --- | --- | --- |


### 任务 8：扫描、对接与交付

# 任务 8：扫描、对接与交付

![](https://github.com/osrf/vrx/wiki/images/scan_dock_2023.png)

## 总结

检测对接口并执行受控对接操作。系统应检测扫描码浮标发出的颜色序列，因为此颜色序列决定了正确的对接口。成功将弹丸推进穿过标牌头部两个孔之一的载具将获得额外分数。
the correct docking bay.

## 如何练习

### 步骤 1：启动示例

```
ros2 launch vrx_gz competition.launch.py world:=scan_dock_deliver_task
```

### 步骤 2：访问相关话题和服务

以下 ROS 话题和服务在此任务中使用：

#### 话题

```
/vrx/task/info
/wamv/shooters/ball_shooter/fire
```

#### 服务

```
/vrx/scan_dock_deliver/color_sequence
```

#### 使用这些话题和服务

1. To see the timeout counter and your current score, subscribe to the `/vrx/task/info` topic:

   ```
   ros2 topic echo /vrx/task/info
   ```
2. To shoot a projectile from the ball shooter, publish a `std_msgs/msgs/Bool` message to the `/wamv/shooters/ball_shooter/fire` topic:

   ```
   ros2 topic pub /wamv/shooters/ball_shooter/fire std_msgs/msg/Bool "{data: 0}" --once
   ```
3. To report the scan-the-code color sequence, publish a `ros_gz_interfaces/msg/StringVec` to the `/vrx/scan_dock_deliver/color_sequence` topic:

   ```
   ros2 topic pub --once vrx/scan_dock_deliver/color_sequence ros_gz_interfaces/msg/StringVec "{data: ['red', 'green', 'blue']}"
   ```

### 步骤 3：完成任务

* Get close to the scan-the-code buoy and report the correct color sequence.
* Identify the dock and the correct docking gate.
* Approach the correct gate and execute a smooth docking maneuver.
* Point to the right placard targets and shoot your projectiles.
* Exit the gate smoothly.

### 注意：

* Only four shots are allowed during this task. If you want to practice with unlimited shots remove the line `<num_shots>4</num_shots>` in `wamv_gazebo/urdf/ball_shooter/ball_shooter.xacro` and recompile VRX.
* Points are awarded for any successful docking maneuver, even in an incorrect gate.

| Back: [Acoustic Tracking](https://github.com/osrf/vrx/wiki/vrx_2023-acoustic_tracking_task) | Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) |
| --- | --- |


==============================================================================

# 第 12 章：使用 Docker 打包提交
# 第 12 章: Docker 打包提交
==============================================================================

# VRX Docker 镜像概述

如何为 VRX 竞赛生成、交互和调试参赛者 Docker 镜像。

## [VRX 参赛者 Docker 入门](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_orientation)

Docker 镜像和容器的简要说明，以及它们与 VRX 竞赛的关系。

## [创建参赛者镜像](https://github.com/osrf/vrx/wiki/tutorials-vrx_make_competitor_image)

本系列教程解释了如何创建和交互参赛者镜像。

### 本序列中的教程：

* [镜像创建概述](https://github.com/osrf/vrx/wiki/tutorials-vrx_make_competitor_image)
* [准备工作](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_setup)
* [使用 `docker commit` 的交互式流程](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_interactive)
* [使用 `Dockerfile` 的脚本化流程](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_scripted)
* [最小工作示例](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_mwes)

## [试运行故障排除](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_troubleshooting)

本系列教程解释了如何测试和调试参赛者镜像。

### 本序列中的教程：

* [故障排除概述](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_troubleshooting)
* [故障排除前提条件](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_before_trouble)
* [获取基本调试信息](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_debug_info)
* [检查正在运行的容器](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_examine)
* [手动运行您的容器](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_manual_run)
* [手动运行试运行](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_manual_trial)

| Top: [VRX 教程](https://github.com/osrf/vrx/wiki/tutorials) | Next: [VRX 参赛者 Docker 入门](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_orientation) |
| --- | --- |


---


## 参赛者 Docker 指南

# VRX 竞赛 Docker 指南

## 什么是 Docker？

* Docker 是一个用于创建和管理称为 **Docker 镜像** 的可执行包的工具。
* 您可以将这些镜像视为非常轻量级的虚拟机。
* 在内部，它们包含在原生环境中运行应用程序所需的一切：可执行文件、运行时、库、环境变量、配置文件等。
* 因为它们自带依赖项和环境，所以 Docker 镜像非常便携。
* **Docker 容器** 是镜像的运行时实例。
* 对于最终用户来说，Docker 容器的行为与任何其他可执行进程非常相似。

要了解有关 Docker 的更多信息，请参阅他们优秀的[教程和说明](https://docs.docker.com/get-started/)。

## Docker 与 VRX

* VRX 竞赛依赖 Docker 作为其自动评估系统的一个组件。
* 本质上，我们要求团队以代表其整个系统的最小"类虚拟机"文件形式提供他们的解决方案。
* 我们使用 Docker 镜像而不是虚拟机，因为它们更小、更易于管理、分发和自动化。

### 我们如何运行竞赛

我们使用两个 Docker 镜像来评估每个团队的性能。这些是：

* "参赛者镜像"。此镜像由团队提供，代表要评估的系统。
* VRX 服务器镜像。此镜像由 VRX 技术团队维护，模拟 VRX 任务环境。

在收到所有阶段提交后，我们通过将每个参赛者镜像与 VRX 服务器镜像的实例并发运行来模拟竞赛。这种方法有许多优势：

* 团队对其执行环境有更大的控制权。
* Docker 在提交的解决方案和评估它们的主机系统之间创建了一个抽象层：
  + 团队不必知道主机系统的详细信息即可确保其软件正确运行。
  + 同样，组织者不必知道参赛者容器环境的详细信息。

#### VRX 服务器镜像

* VRX 服务器镜像的所有代码和文档可在 [`vrx-docker` GitHub 仓库](https://github.com/osrf/vrx-docker) 中找到。
* 鼓励团队在本地构建此镜像并自行运行竞赛来为其提交评分。
* 请参阅[测试教程](https://github.com/osrf/vrx/wiki/vrx_2023-testing)了解如何操作的说明。

#### 参赛者镜像

* 参赛者镜像代表团队的平台。
* 此镜像只需要运行操作 WAMV 和完成每个任务所需的代码。
* 它不需要模拟任务本身，因为这是由 VRX 服务器镜像处理的。

| Up: [Overview](https://github.com/osrf/vrx/wiki/tutorials-vrx_competitor_image) | Next: [创建参赛者镜像](https://github.com/osrf/vrx/wiki/tutorials-vrx_make_competitor_image) |
| --- | --- |


---


## 创建参赛镜像

# Docker 镜像创建概述

本系列教程描述了必要的准备工作、创建参赛者镜像的两种替代流程，以及一些最小工作示例。

## [Preparation](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_setup)

如何安装所需的依赖项并设置 Dockerhub 帐户。

## 镜像开发

团队可以使用以下两个选项之一来创建参赛者镜像：

### 选项 1：[使用 `docker commit` 的交互式流程](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_interactive)

在本教程中，我们打开一个终端进入 Docker 镜像，并使用 Docker commit 命令保存更改。交互式流程更类似于直接在机器上开发的体验，因此对于首次使用 Docker 的用户来说进入门槛较低。这可能是新 Docker 用户最快的入门方式。

### 选项 2：[使用 `Dockerfile` 的脚本化流程](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_scripted)

本教程介绍了使用 `Dockerfile` 构建镜像的完全自动化和自文档化流程。此方法使用更常见的 Docker 用户工作流程，但需要更熟悉 Docker 的概念和实践。

## [最小工作示例](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_mwes)

如何查找和构建 `vrx-docker` 仓库中提供的参赛者镜像的最小工作示例。

| Back: [VRX 参赛者 Docker 入门](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_orientation) | Up: [VRX Docker Image Overview](https://github.com/osrf/vrx/wiki/tutorials-vrx_competitor_image) | Next: [Preparation](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_setup) |
| --- | --- | --- |


---


## Docker 准备工作

## 安装 Docker

* 创建 Dockerhub 镜像需要安装 Docker。
* 请按照 [VRX Docker 安装教程](https://github.com/osrf/vrx/wiki/tutorials-installDocker) 操作。
* 按照 [Nvidia Docker 安装教程](https://github.com/osrf/vrx/wiki/tutorials-installNvidiaDocker) 设置 Docker 以与您的 Nvidia GPU 配合工作。

## Dockerhub

* Dockerhub 是由 Docker 维护的在线 Docker 镜像仓库。
* 要向竞赛提交参赛作品，团队必须先将其镜像上传到 Dockerhub，以便与我们共享。
* 您需要一个免费的 Dockerhub 帐户来完成此操作。
* 如果您还没有 Dockerhub 帐户，请点击[此处](https://hub.docker.com/signup)创建一个。
* 请记下您的用户名。

| Back: [镜像创建概述] | Up: [VRX Docker 镜像概述] | Next: [选项 1：交互式] |
| --- | --- | --- |


---


## 交互式 docker commit 流程

# 选项 1：使用 `docker commit` 进行交互式构建

在本教程中，我们首先拉取一个干净的 ROS 2 Humble Docker 镜像。然后我们在终端中手动运行所需的命令来设置系统，并使用 `docker commit` 使我们的更改永久化。

* This is one of two options for building a competitor image.
* [The second option is described here](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_scripted).
* This development process is more similar to the experience of developing directly on a machine, and thus has a lower barrier of entry for first-time Docker users.
* For new Docker users, this is probably the fastest way to get started.

## 构建一个最小可用镜像

按照以下步骤获取参赛者镜像的最小版本并运行：

### 步骤 1：获取一个干净的 ROS Humble Docker 镜像

* 打开一个终端并执行以下命令：

  ```
  docker run --name my_container -it ros:humble-ros-base-jammy
  ```

  run 命令执行了多个操作：
  + 它将检查本地是否存在 `ros:humble-ros-base-jammy` 镜像。
  + 如果不存在，它将从 DockerHub 下载该镜像。（这可能需要几分钟。）
  + 下载完成后，它将运行镜像并创建一个容器，我们将其命名为 `my_container`。
  + `-it` 选项指定我们需要一个"交互式"会话和一个"终端"。
  + 命令完成后，它将为您打开一个交互式 Bash 会话以运行命令。
  + 请注意，您的终端提示符将更改为：

    ```
    root@<container_id>:/#
    ```

    其中容器 ID 是 Docker 分配给容器的哈希值。

### 步骤 2：添加开发工具

此 Bash 会话非常基础。它还没有文本编辑器，所以我们现在将安装一个。

* 在您刚刚打开的 Bash 会话中运行 `apt update && apt install -y nano`，或者将 `nano` 替换为您选择的文本编辑器。
* 请注意，在会话中运行此命令会将 nano 安装到容器*内部*，而不是您的主机系统上。
* 按照类似方式添加您需要的任何其他开发工具。

### 步骤 3：编辑入口点脚本

默认情况下，我们启动的 `ros:humble-ros-base-jammy` 镜像配置为在运行时查找并执行 `ros_entrypoint.sh` 脚本。我们将编辑此脚本，使其调用我们将提供的 `run_my_system.bash` 脚本。

* 使用文本编辑器编辑 `ros_entrypoint.sh`。例如：

  ```
  nano /ros_entrypoint.sh
  ```
* 将所有文本替换为以下内容：

  ```
  #!/bin/bash
  set -e

  # setup ros environment
  source "/opt/ros/$ROS_DISTRO/setup.bash"

  /run_my_system.bash
  ```
* 保存更改并退出。

### 步骤 4：创建 `run_my_system.bash`

现在我们已经告诉入口点脚本调用 `run_my_system.bash`，我们需要将该脚本添加到系统中。

* 在文本编辑器中打开新脚本：

  ```
  nano /run_my_system.bash
  ```
* 将以下文本复制到空白文件中：

  ```
  #!/bin/bash

  # Start ROS2 daemon for discovery  if not already started
  ros2 topic list

  # Send forward command
  RATE=1
  CMD=2
  echo "Sending forward command"
  ros2 topic pub -r ${RATE} -p 20 /wamv/thrusters/left/thrust std_msgs/msg/Float64 "{data: ${CMD}}" &
  ros2 topic pub -r ${RATE} -p 20 /wamv/thrusters/right/thrust std_msgs/msg/Float64  "{data: ${CMD}}"
  ```
* 运行 `chmod +x /run_my_system.bash` 使其可执行。

### 步骤 5：测试 `run_my_system.bash`

* 运行

  ```
  /run_my_system.bash &
  ```

  在后台执行该脚本。
* 您应该看到 ros core 服务启动，以及您刚刚创建的脚本中 echo 消息的输出 "Sending forward command"。
* 按回车键返回命令提示符。
* 检查您的脚本是否按预期发布数据。

  ```
  ros2 topic echo /wamv/thrusters/left/thrust
  ```
* 您应该收到 `/wamv/thrusters/left/thrust` 数据（在脚本中设置为 2.0）。

### 步骤 6：在本地保存更改

* 到目前为止您所做的一切都更改了 Docker *容器*，它在系统内存中运行。
* 我们现在需要将这些更改提交回 Docker *镜像*，以便它们能够持久保存。

为此：

* 运行 `exit` 退出此容器。您的提示符将更改以表明您已回到主机计算机上。
* 运行

  ```
  docker ps -a
  ```

  列出所有容器。您应该看到您的容器 `my_container`。`STATUS` 应该表明容器最近已退出。
* 从最左列复制 `my_container` 的 `Container ID`。
* 设置一些有用的变量，替换以下适当的值：

  ```
  AUTHOR_NAME=<your_name>
  CONTAINER_ID=<container_id>
  USERNAME=<dockerhub_username>
  IMAGE_NAME=<name_your_image>
  TAG=<image_version>
  ```

  + `AUTHOR_NAME` 是您的名字。
  + `USERNAME` 必须与您的 Dockerhub 帐户用户名匹配。
  + `CONTAINER_ID` 必须是您在上一步中复制的 ID。
  + `IMAGE_NAME` 应描述您的镜像。它将用于在本地和 Dockerhub 上查找您的镜像。
  + `TAG` 可以是任何内容，但我们建议您使用它来存储版本信息。
* 运行

  ```
  docker commit -m "<Write your commit message here.>" -a ${AUTHOR_NAME} ${CONTAINER_ID} ${USERNAME}/${IMAGE_NAME}:${TAG}
  ```
* 例如：

  ```
  AUTHOR_NAME=Michael_McCarrin
  CONTAINER_ID=a0e1e92cb6a5
  USERNAME=virtualrobotx
  IMAGE_NAME=vrx-competitor-example
  TAG=v2.2023
  docker commit -m "Start from ros-humble-base and add run_my_system.bash" -a ${AUTHOR_NAME} ${CONTAINER_ID} ${USERNAME}/${IMAGE_NAME}:${TAG}
  ```

### 步骤 7：将镜像推送到 Dockerhub

* 运行 `docker login` 并输入您的凭据。
* 推送您的镜像：

  ```
  docker push ${USERNAME}/${IMAGE_NAME}:${TAG}
  ```
* 您应该能够在 <https://hub.docker.com> 登录您的 Dockerhub 帐户并看到您的新仓库。

#### 可选：将仓库设为私有

* 如果您想将仓库设为私有，可以点击您的仓库，然后点击 Settings，然后点击 Make Private。
* 为确保我们可以访问和评估您的镜像，您可以点击 Collaborators 并添加 `virtualrobotx`。

## 进一步开发您的镜像

一旦您有了一个最小镜像在工作，您可以通过重复上述过程继续开发它。请务必使用 `docker commit` 和 `docker push` 定期保存更改并将其推送到 Dockerhub 仓库。

### 使用 `docker cp` 添加本地文件

在开发镜像时，您可能需要从主机系统添加文件。有几种方法可以做到这一点，但一种方便的方法是使用 `docker cp` 命令。

* 在容器运行时，打开一个新终端，导航到要复制的文件所在的目录。
* 运行

  ```
  docker cp <your_local_file> <container_name>:/path/to/file/in/container/
  ```
* 例如，如果您想将 `script.py` 复制到名为 `my_container` 的容器内的 `/root/scripts`，请运行：

  ```
  docker cp script.py my_container:/root/scripts
  ```
* 有关 `docker cp` 命令的完整文档，请参阅 <https://docs.docker.com/engine/reference/commandline/cp/>

| Back: [镜像创建概述] | Up: [VRX Docker 镜像概述] | Next: [选项 2：脚本化] |
| --- | --- | --- |


---


## Dockerfile 脚本化流程

# 选项 2：使用 `Dockerfile` 自动创建镜像

在本教程中，我们使用名为 `Dockerfile` 的脚本自动化创建参赛者镜像的整个过程。

* This is one of two options for building a competitor image.
* [The first option is described here](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_interactive).
* The primary advantage of this approach is that it is self-documenting and easily reproducible.

## 构建一个可用镜像

按照以下步骤获取参赛者镜像的最小版本并运行：

### 步骤 1：创建您的 Dockerfile

* 创建一个目录来存储您的 `Dockerfile` 和镜像所需的任何其他文件。

  ```
  mkdir ~/my_vrx_docker; cd ~/my_vrx_docker
  ```
* 创建一个名为 `Dockerfile` 的文件，在文本编辑器中打开它，并复制以下内容：

  ```
  FROM ros:humble-ros-base-jammy

  # Set up timezone
  ENV TZ=Etc/UTC
  RUN echo $TZ > /etc/timezone && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime

  # Install required utilities
  RUN apt update \
  && apt install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      gnupg2 \
      nano \
      python3-dbg \
      python3-setuptools \
      python3-vcstool \
      ruby \
      sudo \
  && rm -rf /var/lib/apt/lists/* \
  && apt clean -qq

  # Set up locale
  RUN sudo apt update && sudo apt install locales \
  && sudo locale-gen en_US en_US.UTF-8 \
  && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

  ARG ROSDIST=humble

  # Install example ROS package
  RUN apt update \
  && apt install -y --no-install-recommends \
    ros-${ROSDIST}-actuator-msgs \
  && rm -rf /var/lib/apt/lists/* \
  && apt clean -qq

  # Copy over script to Docker container
  COPY ./run_my_system.bash /

  # Use your ros_entrypoint
  COPY ./ros_entrypoint.sh /
  ```

  此脚本执行以下操作：

  + `FROM` 命令告诉 Docker 从 `ros:humble-ros-base-jammy` 镜像开始。
  + `RUN` 命令执行任意命令，就像在命令行上运行它们一样。在这种情况下，它们执行 `apt` 来安装实用程序和 ROS 包。我们实际上还不需要这些包。它们作为可能在以后有用的包的示例包含在内。
    - 请注意，`RUN` 命令的常见模式是使用 `&&` 将命令序列链接在一起，以便只有在前一个命令成功时才运行每个命令。这样分组命令的原因是环境变量不会从 `Dockerfile` 的一行持续到下一行。只有写入磁盘的更改才会持续。
    - 相反，我们在每次调用 `apt` 后清除临时文件，以避免将不需要的文件写入磁盘并使镜像变得杂乱。
  + `COPY` 命令将文件从主机文件系统复制到 docker 镜像中的指定位置。在这种情况下，引用的两个文件尚不存在。我们将在下一步中创建它们。

### 步骤 2：在本地创建脚本

默认情况下，我们启动的 `ros:humble-ros-base-jammy` 镜像配置为在运行时查找并执行 `ros_entrypoint.sh` 脚本。我们将创建此脚本并使用它来调用将控制我们 WAMV 的自定义脚本。

* 首先，使用文本编辑器创建一个名为 `ros_entrypoint.sh` 的文件，并将以下文本复制到文件中：

  ```
  #!/bin/bash
  set -e

  # setup ros environment
  source "/opt/ros/$ROS_DISTRO/setup.bash"

  /run_my_system.bash
  ```
* 运行 `chmod +x ros_entrypoint.sh` 使其可执行。
* 现在使用文本编辑器创建一个名为 `run_my_system.bash` 的文件，并将以下文本复制到文件中：

  ```
  #!/bin/bash

  # Start ROS2 daemon for discovery  if not already started
  ros2 topic list

  # Send forward command
  RATE=1
  CMD=2
  echo "Sending forward command"
  ros2 topic pub -r ${RATE} -p 20 /wamv/thrusters/left/thrust std_msgs/msg/Float64 "{data: ${CMD}}" &
  ros2 topic pub -r ${RATE} -p 20 /wamv/thrusters/right/thrust std_msgs/msg/Float64  "{data: ${CMD}}"
  ```
* 运行 `chmod +x run_my_system.bash` 使其可执行。

### 步骤 3：构建您的镜像

我们现在将使用 `docker build` 命令从 `Dockerfile` 创建镜像。

* 首先，设置一些有用的变量，替换以下适当的值：

  ```
  USERNAME=<dockerhub_username>
  IMAGE_NAME=<name_your_image>
  TAG=<image_version>
  ```

  + `USERNAME` 必须与您的 Dockerhub 帐户用户名匹配。
  + `IMAGE_NAME` 应描述您的镜像。它将用于在本地和 Dockerhub 上查找您的镜像。
  + `TAG` 可以是任何内容，但我们建议您使用它来存储版本信息。
* 构建镜像：

  ```
  docker build --tag ${USERNAME}/${IMAGE_NAME}:${TAG} .
  ```
* 例如：

  ```
  USERNAME=virtualrobotx
  IMAGE_NAME=vrx-competitor-example
  TAG=v1.2023
  docker build --tag ${USERNAME}/${IMAGE_NAME}:${TAG} .
  ```

### 步骤 4：测试镜像

* 运行您的镜像

  ```
  docker run -it ${USERNAME}/${IMAGE_NAME}:${TAG}
  ```

  + 这将使用您在上一步中创建的镜像来运行一个容器。
  + 容器将调用 `ros_entrypoint.sh` 脚本，该脚本将执行 `/run_my_system.bash`。
  + `-it` 选项指定我们需要一个交互式终端。这对我们当前的示例很有用，因为它允许我们查看终端输出，并在完成后使用 `CTRL+C` 终止容器。（没有此选项，我们需要使用 `docker stop` 命令来关闭容器。）
* 您应该看到类似以下的输出：

  ```
  /parameter_events
  /rosout
  Sending forward command
  publisher: beginning loop
  publishing #1: std_msgs.msg.Float64(data=2.0)

  publisher: beginning loop
  publishing #1: std_msgs.msg.Float64(data=2.0)
  ```
* 要验证脚本是否按预期工作，请打开一个新终端并获取容器 ID：

  ```
  docker container ls
  ```

  复制最左列的容器 ID。
* 现在我们将创建到正在运行的容器的第二个连接：

  ```
  docker exec -it <container_id> bash
  ```

  其中 `<container_id>` 是您刚刚复制的 ID。此命令将打开一个到容器的交互式 bash 会话。
* 在此会话中，检查您的脚本是否按预期发布数据。

  ```
  source /opt/ros/humble/setup.bash
  ros2 topic echo /wamv/thrusters/left/thrust
  ```
* 您应该收到 `/wamv/thrusters/left/thrust` 数据（在脚本中设置为 2.0）。
* 切换回您的原始终端并按 `CTRL+C` 退出容器。
* 请注意，这也将终止您的辅助 bash 会话。

### 步骤 5：将镜像推送到 Dockerhub

* 运行 `docker login` 并输入您的凭据。
* 推送您的镜像：

  ```
  docker push ${USERNAME}/${IMAGE_NAME}:${TAG}
  ```
* 您应该能够在 <https://hub.docker.com> 登录您的 Dockerhub 帐户并看到您的新仓库。

#### 可选：将仓库设为私有

* 如果您想将仓库设为私有，可以点击您的仓库，然后点击 Settings，然后点击 Make Private。
* 为确保我们可以访问和评估您的镜像，您**必须**点击 Collaborators 并添加 `virtualrobotx`。

## 进一步开发您的镜像

* 一旦您有了一个最小镜像在工作，您可以通过向 `Dockerfile` 添加命令并重新构建来继续开发它。
* 为了提高效率，Docker 将只重新运行从 Dockerfile 中第一行更改的行开始的构建命令，因此将不太可能更改（或运行时间较长）的命令放在文件顶部是有利的。
* 请参阅 Docker 的[编写 Dockerfile 最佳实践](https://docs.docker.com/develop/develop-images/dockerfile_best-practices/)，了解许多关于如何编写好的 `Dockerfile` 的有用技巧，以及可用命令的完整列表。

### 使用 Dockerfile 构建 ROS 工作空间

* 由于 Dockerfile 中的每个命令都在干净的环境中运行，依赖于源化许多环境变量的命令可能会比较棘手。
* 依赖 ROS 工具的命令是一个明显的例子。
* 解决方案通常是确保同一个 run 命令既设置环境又立即调用需要它的命令。
* 有关具体的工作示例，请参阅 [`vrx-repository`](https://github.com/osrf/vrx-docker/tree/master/mwes) 中提供的最小工作示例，并在下一个教程中讨论。

| Back: [选项 1：交互式] | Up: [VRX Docker 镜像概述] | Next: [最小工作示例] |
| --- | --- | --- |


---


## 最小工作示例

## vrx-docker 仓库中的资源

[`vrx-docker` 仓库](https://github.com/osrf/vrx-docker)包含运行 VRX 竞赛所需的脚本和 Docker 镜像。

* 这些包括 VRX 服务器 docker 镜像的 `Dockerfile`，以及三个参赛者镜像的示例：
  + `vrx_2019_simple`：用于在 VRX 2019 竞赛中测试 VRX 服务器的最小参赛者镜像
  + `vrx_2022_simple`：用于在 VRX 2022 竞赛中测试 VRX 服务器的最小参赛者镜像
  + `vrx_2022_starter`：为 VRX 2022 竞赛提供的稍微更完善的 `Dockerfile` 示例，从主机上的源代码构建工作空间。
  + `vrx_2023_simple`：VRX 2023 的示例 `Dockerfile`，使用 ROS 2 和 Gazebo Garden。
* 有关所有三个镜像的更多详细信息记录在 vrx-docker 仓库中，[此处](https://github.com/osrf/vrx-docker/tree/master/mwes)。

## `vrx_2023_simple` 镜像

在本教程中，我们使用为 `vrx_2023_simple` 提供的 `Dockerfile` 作为开发参赛者镜像的起点。

### 下载和构建：

* 构建此镜像的说明可在 vrx-docker 仓库中找到，[此处](https://github.com/osrf/vrx-docker/tree/master/mwes/2023/vrx_2023_simple)。
* 请注意，这些说明在构建镜像时省略了 Github 用户名和标签。
  + 这对于本地开发来说没问题。但是，在推送到 Dockerhub 仓库之前需要重命名镜像。
  + 您可以使用 [`docker tag` 命令](https://docs.docker.com/engine/reference/commandline/tag/)来完成此操作。例如：

  ```
  docker tag <my_image> <username>/<my_image>:<tag>
  ```

### 调整 `Dockerfile`

提供的 [`Dockerfile`](https://github.com/osrf/vrx-docker/blob/master/mwes/2023/vrx_2023_simple/Dockerfile) 按以下部分组织：

* 设置环境变量
* 设置语言环境信息
* 安装实用程序
* 安装 ROS 包
* 安装可选开发工具
* 从源代码导入和构建工作空间
* 添加启动脚本
* 添加其他自定义设置
* 设置入口点
  大多数部分都是不言自明的，可以轻松修改以设置变量或添加/删除包。以下示例演示了如何使用不同的源代码仓库进行替换和开发。

#### 构建您自己的仓库

在此示例中，我们假设团队想要构建不同的源代码仓库，而不是 `vrx` 仓库。这可以通过以下方式完成：

* 将您的仓库克隆到与 `Dockerfile` 相同的目录中：

  ```
  git clone git@github.com/myteam/myrepo.git
  ```

  默认情况下，这将创建一个名为 `myrepo` 的本地目录。
* 使用文本编辑器修改提供的 `Dockerfile` 中以下行的 `COPY` 命令

  ```
  COPY my_source /vrx_ws/src/vrx
  ```

  使其将您从主机文件系统克隆的仓库复制到镜像中的所需路径。例如：

  ```
  COPY myrepo /vrx_ws/src/myrepo
  ```
* 现在使用相同的 `docker build` 命令使用您的源代码构建镜像。

#### 开发工作流程

* 一旦您设置了镜像来构建您的代码，您就可以在本地的仓库中工作。
* 每次重新构建时，`COPY` 命令将检测源代码的任何更改并根据需要重新构建镜像。
* `COPY` 命令的此功能使其特别适合开发工作流程。
* 相反，如果您使用 `RUN` 命令将源仓库直接克隆到镜像中，Docker 将无法检测远程仓库中的更改，并且除非您修改 `Dockerfile` 本身或使用 `-no-cache` 选项强制完全重新构建，否则始终使用本地缓存版本。因此，这种方法可能导致您的镜像与最新更改不同步，通常不推荐使用。

## ROS1 <-> ROS2 桥接

虽然 VRX 2023 竞赛使用 ROS2 API 进行通信，但可以通过使用桥接器在参赛者 docker 容器中运行 ROS1 节点。这不是官方支持的，但是允许的。有关桥接 VRX 任务消息的 ROS1-ROS2 容器的最小工作示例，请参阅[此仓库](https://github.com/j-herman/humble-noetic-bridge)。

| Back: [选项 2：脚本化] | Up: [VRX Docker 镜像概述] | Next: [试运行故障排除] |
| --- | --- | --- |


---


## 试运行故障排除

本系列教程解释了如何对参赛者镜像进行故障排除。

## [Troubleshooting Prerequisites](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_before_trouble)

在进行故障排除之前，您必须先构建镜像并尝试使用它。

## [Get basic debugging information](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_debug_info)

使用 Docker 命令获取有关容器在试验期间行为的基本信息。

## [Examine a running container](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_examine)

在正在运行的容器上创建交互式 bash 会话，以获取有关其正在做什么的信息。

## [Run your container manually](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_manual_run)

手动运行您的参赛者镜像以隔离潜在问题。

## [Run a trial manually](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_manual_trial)

通过手动启动参赛者镜像和 vrx-docker 服务器来模拟整个试验。

| Back: [最小工作示例](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_mwes) | Up: [VRX Docker Image Overview](https://github.com/osrf/vrx/wiki/tutorials-vrx_competitor_image) | Next: [Troubleshooting Prerequisites](https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_before_trouble) |
| --- | --- | --- |


==============================================================================

# 第 13 章：技术文档
# 第 13 章: 技术文档
==============================================================================

## VRX 竞赛 2022

* RoboNation [VRX 主页](https://robotx.org/programs/vrx-competition-2023/)：包含 VRX 项目概述、注册信息、日期等。
* [VRX 2023 任务描述，版本 1.2](https://robonation.org/app/uploads/sites/2/2023/07/VRX2023_Task-Descriptions_v1.2.pdf)
* [VRX 2023 技术指南，版本 1.2](https://robonation.org/app/uploads/sites/2/2023/07/VRX2023_Technical-Guide_v1.2.pdf)

## VRX 仿真平台信息

1. [系统要求](https://github.com/osrf/vrx/wiki/system_requirements_classic)
2. [Gazebo 插件](https://github.com/osrf/vrx/wiki/vrx_classic_theory)：VRX 中使用的环境和模型插件描述
3. [VRX 约定](https://github.com/osrf/vrx/wiki/frame_conventions)：VRX 中使用的坐标约定等

## 如何引用 VRX

如果您在工作中使用了 VRX 仿真，请引用我们的总结论文 [Toward Maritime Robotic Simulation in Gazebo](https://wiki.nps.edu/display/BB/Publications?preview=/1173263776/1173263778/PID6131719.pdf)：

```
@InProceedings{bingham19toward,
  Title                    = {Toward Maritime Robotic Simulation in Gazebo},
  Author                   = {Brian Bingham and Carlos Aguero and Michael McCarrin and Joseph Klamo and Joshua Malia and Kevin Allen and Tyler Lum and Marshall Rawson and Rumman Waqar},
  Booktitle                = {Proceedings of MTS/IEEE OCEANS Conference},
  Year                     = {2019},
  Address                  = {Seattle, WA},
  Month                    = {October}
}
```

## 往届竞赛文档

## VRX 竞赛 2022

* [虚拟 RobotX (VRX) 竞赛 2022：简介](https://robonation.org/app/uploads/sites/2/2021/06/Introduction-Document-Virtual-RobotX-2022.pdf)
* [VRX 2022 竞赛和任务描述版本 1.0](https://robonation.org/app/uploads/sites/2/2021/09/VirtualRobotX2022_Task-Descriptions_v1.0.pdf)
* [VRX 2022 竞赛技术指南版本 1.1](https://robonation.org/app/uploads/sites/2/2021/11/VRX2022_Technical-Guide_v1.1.pdf)

### VRX 竞赛 2019

* [VRX 竞赛和任务描述，版本 1.4，2019 年 11 月 9 日](https://github.com/osrf/vrx/wiki/files/VRX2019_Task_Descriptions_v1.4.pdf)：定义了竞赛的三个阶段、评分、任务和每个任务的 API。
* [VRX 技术指南，版本 1.2，2019 年 8 月 26 日](https://github.com/osrf/vrx/wiki/files/VRX2019_Technical%20Guide_V1.2.pdf)：定义了 VRX 配置（WAM-V、传感器和推进）、环境范围（波浪、风和视觉条件）、任务接口、竞赛实施方式和参赛者提交解决方案的方式。


---


## 平台概述

# 虚拟 RobotX 仿真平台

VRX 活动和竞赛使用 VRX 仿真平台运行。

* 仿真平台的代码托管在 [VRX Github 仓库](https://github.com/osrf/vrx)。
* [系统要求](https://github.com/osrf/vrx/wiki/system_requirements)。
* [Gazebo 插件工作原理](https://github.com/osrf/vrx/wiki/vrx_theory)。
* [教程](https://github.com/osrf/vrx/wiki/tutorials)用于安装和使用 VRX 仿真平台。


---


## 坐标系约定

本页面记录了我们在 VRX 中使用的一些参考坐标系和约定。通常，VRX 遵循 [REP 103：标准度量单位和坐标约定](https://www.ros.org/reps/rep-0103.html)。

# 世界坐标系

Gazebo 的仿真总是在笛卡尔坐标系中执行（传统的 XYZ）。您可以通过在 GUI 菜单中选择 `View->Origin` 来可视化 Gazebo 原点。

![world_frame](https://user-images.githubusercontent.com/1440739/156785118-225d5973-9837-49d1-8a5b-be402e3c074c.png)

此外，Gazebo 支持在仿真中使用 WGS84 大地坐标系的真实世界经纬度坐标。世界的笛卡尔坐标对应于行星表面上给定点处的局部切平面。默认情况下，此平面遵循 ENU（东-北-天）约定，如下图所示：

![spherical_coordinates](https://user-images.githubusercontent.com/1440739/156786820-f81d3871-2299-4c3d-a4c2-c1047a4e3b18.png)

# 模型坐标系

Gazebo 遵循右手定则。机器人朝前的表面在正 x 方向，朝左的表面在正 y 方向，朝上的表面在正 z 方向，依此类推。

![model_frame_sm](https://user-images.githubusercontent.com/1440739/156787618-3795012f-3a77-4048-8a16-94d6ba163f2b.jpg)

下图显示了分别应用正滚转、俯仰和偏航时模型的变换。

![roll_pitch_yaw](https://user-images.githubusercontent.com/1440739/156788264-b98a5fda-5535-436b-91e7-9494a3148fbc.png)
*注意：此图片由 <https://www.pix4d.com/> 提供*

# 风

风插件发布应用于计算 WAM-V 上力的风矢量，而不是
wind reading like you would get from a weather report. Thus, the direction value that is reported is where
the wind is going to, rather than coming from, so the WAM-V will move in the direction of reported wind.
风矢量方向在 Gazebo 世界坐标系中以度数表示。因此，例如，360（或 0）度的风向将把 WAM-V 推向世界坐标系 x 轴的方向，即向东。090 度的风向将把 WAM-V 推向北方。报告的大小以米/秒为单位。

# 声学定位器

方位角在 WAM-V 坐标系和坐标系中报告：x 轴朝向车辆头部，
the y-axis is towards the port side, and the z-axis points upwards. Following the right-hand rule, bearing angles are measured counter-clockwise beginning from the x-axis. As an example, imagine that the acoustic pinger is located below the light buoy as depicted in the image below.

![pinger](https://user-images.githubusercontent.com/1440739/156793656-d84c593a-194c-4e07-8928-d34f7d7f169d.jpg)

来自 WAM-V 的测量值（带有一些噪声）是：

```
header:
  seq: 21
  stamp:
    secs: 22
    nsecs: 249000000
  frame_id: "wamv/pinger"
range: 64.8294677734375
bearing: 0.2934380769729614
elevation: -0.4265419840812683
```

**重要提示**：看起来我们在计算仰角的方式上有一个 bug，在此示例中它应该是正的。我们将在 2022 VRX 决赛**之后**修复此问题。


---


## 浮力插件文档

此模型插件是 usv_gazebo_plugins 的一部分；它模拟物体在流体中的浮力。

![buoyancy.gif](https://github.com/osrf/vrx/wiki/images/1837217943-buoyancy.gif)

## 工作原理

阿基米德原理指出，浸没在流体中的物体上的浮力等于物体排开的流体重量：

![$$F_{B} = -\rho V g$$](https://github.com/osrf/vrx/wiki/images/1666239173-eq1.png)

where ρ is the density of the fluid, *V* is the submerged volume of the body and *g* is the acceleration due to gravity. The buoyancy force is applied at the center of the submerged volume of the object.

![Untitled Diagram.png](https://github.com/osrf/vrx/wiki/images/1306625960-Untitled%20Diagram.png)

浮力与重力的耦合会导致振荡。在自然界中，振荡由于阻力而减弱。我们使用简化的力-阻力模型，将阻力近似为速度的线性函数：

![F_d = \beta_l m \frac{V}{V_T} (\boldsymbol{v_w} - \boldsymbol{v_c})](https://github.com/osrf/vrx/wiki/images/1163409051-eq2.png)

这里 *β_l* 是线性阻力系数，*m* 是物体的质量，*V* 是物体的浸没体积，*V_T* 是物体的总体积，*v_w* 是流体流的速度，*v_c* 是物体的速度。*F_d* 应用于物体浸没体积的中心。

为了耗散角速度，我们添加阻力矩如下：

![T_d = \beta_a m \frac{V}{V_T} L^2 \omega](https://github.com/osrf/vrx/wiki/images/4248525408-eq3.png)

这里 *β_a* 是角阻力系数，*L* 是物体的平均宽度，*Ω* 是物体的角速度。*T_d* 应用于物体的质心。

浮力插件目前仅支持球体、长方体和圆柱体形状。长方体和圆柱体被建模为多面体，其体积计算成本很高。球体的浸没体积通过积分计算，非常高效。因此，除非需要捕获更高保真度的物理交互，否则应使用球体近似。

## 插件用法

参数可以在模型定义中作为 SDF 参数指定。例如：

```
<model>
    ...
    <plugin name="BuoyancyPlugin" filename="libbuoyancy_gazebo_plugin.so">
        <wave_model>ocean_waves</wave_model> <!-- name of wave model object (optional) -->
        <fluid_density>1000</fluid_density>  <!-- density of fluid -->
        <fluid_level>0.0</fluid_level>       <!-- height of fluid / air interface [m] -->
        <linear_drag>25.0</linear_drag>      <!-- linear drag coefficient -->
        <angular_drag>2.0</angular_drag>     <!-- angular drag coefficient -->
        <buoyancy name="buoyancy_sphere">    <!-- describes volume properties -->
            <link_name>link</link_name>
            <pose>0 0 -0.08 0 0 0</pose>
            <geometry>
                <sphere>
                    <radius>0.23</radius>
                </sphere>
            </geometry>
        </buoyancy>
    </plugin>
</model>
```

物体的体积属性在 `<buoyancy />` 标签中指定。每个浮力元素与模型中的 `<link />` 关联。可以为物体设置多个浮力对象。`<geometry />` 标签支持长方体、球体和圆柱体形状。
您可以通过数值实验选择阻力系数。

有关更深入的示例，请参阅 `usv_gazebo_plugins/worlds/buoyancy_plugin_demo.world.xacro`。


---


## 波场生成

感谢 @Rhys Mainwaring 的贡献（参见 [PR #78](https://osrf-migration.github.io/vrx-gh-pages/#!/osrf/vrx/pull-requests/78/page/1) 和 [Issue #23](https://github.com/osrf/vrx/issues/23/)），VRX 环境实现了一个海面波浪模型，该模型将视觉表示与对环境内模型的物理影响同步。本页面旨在解释该实现。

# 模型

## 海洋模型

`wave_gazebo/world_models/ocean_waves` 模型包含三个插件：

1. **WavefieldModelPlugin:ModelPlugin**
   * Includes instances of
     + WavefieldEntity:gazebo::physics::Base which includes an instance of
       - WaveParameters (defined in Wavefield.hh) holds the current values to define the wave field (number, angle, scale, etc.)
   * Subscribed Gazebo Topics:
     + ~/request ([gazebo::msgs::Request](https://github.com/osrf/gazebo/blob/master/gazebo/msgs/request.proto)) Responds with a Param_V of all of the wave parameters (number, angle, scale, etc.)
     + ~/wave ([gazebo::msgs::Param_V](https://github.com/osrf/gazebo/blob/master/gazebo/msgs/param_v.proto)) - Allows for setting the values of the WaveParameters instance via gazebo topic. The WaveMsgPublisher utility is supplied to support.
   * Published Gazebo Topics
     + ~/response ([gazebo::msgs::Response](https://github.com/osrf/gazebo/blob/master/gazebo/msgs/response.proto))
2. **WavefieldVisualPlugin:VisualPlugin**
   * On initialization, requests wave parameters from the WavefieldModelPlugin via Gazebo ~/request message.
   * During the visual plugin update, uses rendering API to set wave parameters to OpenGL shader GernstnerWaves.vert.
     + Done using the Visual::SetMaterialShaderParam to pass the simulation time value to the GernstnerWaves.vert vertex shader program where the wave model is run to generate the 3D wave field shape. This feature doesn't seem to be terribly well documented, but here is the [PR](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2863/page/1) that implemented the feature and an example.
     + Note that it appears that the GernstnerWaves.vert is hardcoded to 3 component waves.
   * Subscribed Gazebo Topics:
     + ~/response ([gazebo::msgs::Response](https://github.com/osrf/gazebo/blob/master/gazebo/msgs/response.proto)). When receives a response from the model plugin, sets parameters to the vertex shader.
     + ~/wave ([gazebo::msgs::Param_V](https://github.com/osrf/gazebo/blob/master/gazebo/msgs/param_v.proto)) - Allows for setting the values of the WaveParameters instance via gazebo topic. The WaveMsgPublisher utility is supplied to support.
     + ~/world_stats
       \* Published Gazebo Topics
     + ~/request ([gazebo::msgs::Request](https://github.com/osrf/gazebo/blob/master/gazebo/msgs/request.proto)) Requests wave_param wave parameters.
3. A second **WavefieldVisualPlugin:VisualPlugin** for below the water surface. Uses the same parameters and shader.

## WAM-V USV 模型

1. **UsvDynamicsPlugin:ModelPlugin**
   * Uses the `wave_model` parameter to specify a model, by name, that includes an instance of the WavefieldModelPlugin
   * On Update
     + Uses the WavefieldModelPlugin API to get WaveParameters pointer so that this plugin is using the same parameters as used by the visual plugin.
       - This is done on each update, which seems like overkill. If we consider the wave parameters to be constant for a simulation run this could be simplified.
     + Calls a the static WavefieldSampler::ComputDepthDirectly function (see Wavefield.hh/cc) to implement the geometry of the wave height model at specific grid points.

## 浮标和障碍物模型（例如 surmark950400）

1. **UsvDynamicsPlugin:ModelPlugin**
   * Same implementation for wave height as done for the UsvDynamicsPlugin
   * Uses the `wave_model` parameter to specify a model, by name, that includes an instance of the WavefieldModelPlugin. Then on update retrieves the current WaveParameters pointer via the WavefieldModelPlugin and determines the water level at simulation time for the point location of the link via the WavefieldSampler::ComputDepthDirectly function.


---


## 波场包络

我们的目标是定义模拟波浪场，使场景反映在物理 RobotX 竞赛中可能遇到的全部条件范围，并且在 VRX 仿真的能力范围内。

## 波场规范

对于 VRX，我们使用文档中详细描述的 Pierson-Moskowitz 波谱采样（PMS）模型（合并 wave_visualization 分支后将显示）。波浪场模型的参数由用户在 ocean_waves 模型定义中作为 WavefieldModelPlugin 的 SDF 参数指定。例如：

```
<wave>
  <model>PMS</model>
  <number>3</number>
  <scale>1.5</scale>
  <steepness>0.0</steepness>
  <amplitude>0.1</amplitude>  <!-- No effect for the PMS model -->
  <period>2.0</period>
  <gain>1.0</gain>
  <direction>1.0 0.0</direction>
  <angle>0.4</angle>
  <tau>1.0</tau>
</wave>
```

唯一会变化的参数是周期、增益、方向和角度。方向和角度参数控制组成波浪的水平角度，可以在整个 360 度范围内变化。

## 周期和增益范围

允许的周期和增益参数限制范围是通过仿真的经验测试选择的，以找到合适的值来挑战参与者，同时保持与 RobotX 条件的相关性。

测试设置是一个自定义海洋世界，包含两个 WAM-V（带摄像头）、一些标记和波浪计（更多内容见下文）。

```
roslaunch wave_gazebo wave_wamv.launch verbose:=true paused:=true
```

应该会产生类似于此的视图。
![gz_wavefield_test_mr.png](https://github.com/osrf/vrx/wiki/images/695865771-gz_wavefield_test_mr.png)

基于在各种条件下运行仿真，我们得出了以下 VRX 评估的可接受波浪场参数范围。

![wavefield_envelope.png](https://github.com/osrf/vrx/wiki/images/2265349497-wavefield_envelope.png)

[此视频](https://vimeo.com/341005740)展示了范围内的部分波浪条件。

## 超出周期和增益限制

### 波场视觉和物理同步

生成波浪场的视觉表示和与波浪相关的物理力的过程在 <https://github.com/osrf/vrx/wiki/documentation-wavefield_generation> 中描述

这种方法在低海况下似乎效果很好，但随着海洋动力学的增加，可视化（对于模拟相机渲染很重要）和物理运动之间的差异越来越大。我们尝试测量差异的一种方法是使用波浪计插件和视觉模型。下图显示了波浪计模型。中心红色部分长 0.2 米，每个黑白部分长 0.1 米。波浪计插件将此对象的位置设置为波浪模型计算的当前波浪高度 - 与用于生成物理力的模型相同。因此，此模型是物理波浪高度的指示器，以可视化物理和视觉波浪高度之间的差异。

![wave_gauge.png](https://github.com/osrf/vrx/wiki/images/1580853313-wave_gauge.png)

我们可以运行一个简单的测试

```
roslaunch wave_gazebo ocean_world_buoys.launch
```

波场周期为 8.0 秒，增益为 1.0。这将产生三个分量波，周期/振幅分别为 12.0/0.14、8.0/0.7、5.3/0.46。

在[此视频](https://vimeo.com/341005805)中，我们可以看到物理波浪高度（如波浪计模型所示）与视觉波浪高度的差异大约为 +/-0.4 米。请注意，此海况的海洋表面波浪高度（峰到谷）约为 2.0 米，这远超我们对 RobotX 的预期。


---


## Docker 开发

官方 Docker 镜像存储在 Dockerhub 上的 [VRX 仓库](https://hub.docker.com/u/vrx/dashboard/) 中。

有关使用这些镜像或从其 Dockerfile 构建的说明，请参阅[系统设置 Docker](https://github.com/osrf/vrx/wiki/tutorials-SystemSetupDocker) 教程。

具有推送访问权限的 VRX 组织成员可以按以下方式上传 Docker 镜像的新版本：

1. 构建新镜像。
2. 从命令行登录 Docker：

   ```
        $docker login
   ```
3. 为要推送到仓库的镜像打标签。例如，如果镜像名称为 new_nvidia_docker，请运行：

   ```
        $docker tag new_nvidia_docker osrf/vrx_nvidia:v4
   ```
4. 上传新镜像。继续上面的示例，您将运行：

   ```
        $docker push osrf/vrx_nvidia:v4
   ```


==============================================================================

# 第 14 章：VRX Classic（旧版）
# 第 14 章: VRX 经典版 (旧版)
==============================================================================

# 虚拟 RobotX (VRX) 竞赛 2022

本 Wiki 为 VRX 竞赛 2022 的参与者提供技术文档和教程。

* 有关竞赛信息（日程、注册、规则、技术指南等），请参阅 [VRX 网站](https://www.robotx.org/vrx-2022)。
* 有关竞赛相关问题和讨论，请参阅 [VRX 竞赛 2022 论坛](https://robonationforum.vbulletin.net/forum/robotx/-2022-virtual-robotx-competition)。
* 有关 VRX 仿真平台的一般信息，请参阅 [VRX 仿真平台概述](https://github.com/osrf/vrx/wiki/platform_overview)。

## [如何参与](https://github.com/osrf/vrx/wiki/vrx_2022-overview)

竞赛三个阶段的概述，包括如何为每个阶段准备和提交解决方案。

## [教程](https://github.com/osrf/vrx/wiki/VRX-Classic-Tutorials)

一组帮助您开始使用 VRX 的教程。

## [文档](https://github.com/osrf/vrx/wiki/documentation)

VRX 文档、系统要求和 Gazebo 插件的工作原理。

## 其他资源

* [VRX 问题跟踪器](https://github.com/osrf/vrx/issues)用于提交和跟踪错误、功能请求和增强
* [VRX 软件更新日志](https://github.com/osrf/vrx/blob/master/Changelog.md)了解 VRX 开发的简要历史。
* [Maritime RobotX 网站（一般信息）](https://www.robotx.org/)
* [往届活动](https://github.com/osrf/vrx/wiki/event_list)包含与涉及 VRX 仿真平台的往届活动相关的存档 Wiki 页面。

## 如何引用

如果您在工作中使用了 VRX 仿真，请引用我们的总结论文 "Toward Maritime Robotic Simulation in Gazebo"。

```
@InProceedings{bingham19toward,
  Title                    = {Toward Maritime Robotic Simulation in Gazebo},
  Author                   = {Brian Bingham and Carlos Aguero and Michael McCarrin and Joseph Klamo and Joshua Malia and Kevin Allen and Tyler Lum and Marshall Rawson and Rumman Waqar},
  Booktitle                = {Proceedings of MTS/IEEE OCEANS Conference},
  Year                     = {2019},
  Address                  = {Seattle, WA},
  Month                    = {October}
}
```


---


## VRX 经典版教程

我们建议按照数字顺序学习教程，因为每个教程都建立在前一个的基础上。完成这些教程后，您将熟悉 Gazebo、ROS 以及如何使用 VRX。

## VRX 仿真环境介绍

### [系统设置](https://github.com/osrf/vrx/wiki/vrx_classic_system_setup_tutorials)

配置个人电脑和安装 VRX 软件的指南。

### [示例环境](https://github.com/osrf/vrx/wiki/vrx_classic_environments_tutorials)

这些教程展示了如何在 Gazebo 中启动示例环境作为开发的起点。

### [VRX 接口](https://github.com/osrf/vrx/wiki/vrx_classic_api_tutorials)

如何连接到 VRX API 以执行驾驶和与组件（如传感器和球发射器）交互等任务。

### [自定义 VRX](https://github.com/osrf/vrx/wiki/vrx_classic_customizing_vrx_tutorials)

修改 VRX 环境或 WAM-V 平台的教程。

### [故障排除](https://github.com/osrf/vrx/wiki/Troubleshooting)

诊断和修复 VRX 的常见问题。

## 特定 VRX 活动教程

### [VRX 竞赛 2022](https://github.com/osrf/vrx/wiki/vrx_2022-task_tutorials)

即将举行的 VRX 竞赛的任务分解。

### [RobotX 交互论坛 2019](https://github.com/osrf/vrx/wiki/rxi_2019-overview)

2019 年在新加坡举办的黑客马拉松挑战赛概述和演练。

### [VRX 竞赛 2019](https://github.com/osrf/vrx/wiki/vrx_2019-task_tutorials)

VRX 2019 任务的存档教程。


==============================================================================

# 第 15 章：故障排除
# 第 15 章: 故障排除
==============================================================================

# 目录

* [设置和安装](#Setup-and-Install)
* [在网络问题下添加软件包](#Adding-a-package-with-network-trouble)
* [网络问题](#Network-trouble)
* [ROS 密钥问题](#ROS-key-issues)
* [如何更新现有 Docker 容器](#How-to-update-existing-docker-container)

# 设置和安装

## Docker

### 权限

如果这是您第一次在此机器上使用 docker，当您运行上述命令时，您可能会收到类似以下的错误...

```
docker: Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Post http://%2Fvar%2Frun%2Fdocker.sock/v1.37/containers/create: dial unix /var/run/docker.sock: connect: permission denied. See 'docker run --help'.
```

您需要将您的用户帐户添加到 docker 组，

```
sudo usermod -a -G docker $USER
```

然后注销并重新登录以使更改生效。

### 网络

#### Docker DNS 问题

**症状**
症状是您无法从 Docker 容器内获得网络访问。我们可以使用以下命令测试容器网络访问...

`docker run busybox nslookup google.com`

如果系统返回类似以下内容...

```
$ docker run busybox nslookup google.com
Server:    172.20.20.11
Address 1: 172.20.20.11 lee.ern.nps.edu

Name:      google.com
Address 1: 2607:f8b0:4005:804::200e sfo07s13-in-x0e.1e100.net
Address 2: 216.58.195.78 sfo07s16-in-f78.1e100.net
```

则系统具有网络访问权限，我们可以继续构建 VRX 容器。

如果系统挂起很长时间（约 5 分钟）并返回类似以下内容

```
Server:    8.8.8.8
Address 1: 8.8.8.8

nslookup: can't resolve 'google.com'
```

则我们遇到了网络问题。可能的原因是我们无法使用默认 DNS。

**临时修复**

我们可以通过临时向 docker run 命令提供显式 DNS 来验证这一点。

1. 查找您的主机使用的是什么 DNS...

```
$ nmcli dev show | grep 'IP4.DNS'
IP4.DNS[1]:                             172.20.20.11
IP4.DNS[2]:                             172.10.20.12
```

2. 使用您的 DNS 重复 nslookup

```
$ docker run --dns 172.20.20.11 busybox nslookup google.com
Server:    172.20.20.11
Address 1: 172.20.20.11 lee.ern.nps.edu

Name:      google.com
Address 1: 2607:f8b0:4005:807::200e sfo07s16-in-x0e.1e100.net
Address 2: 216.58.195.78 sfo07s16-in-f78.1e100.net
```

不幸的是，docker build 命令似乎没有 `dns` 命令行选项来在构建步骤中显式设置此选项。

我们尝试了两种方式配置 docker 使用特定 DNS

1. 将以下行添加到 /etc/docker/daemon.json 文件 `{ "dns": ["172.20.20.11", "172.10.20.12"] }`
2. 将以下行添加到 /etc/default/docker 文件

```
# Use DOCKER_OPTS to modify the daemon startup options.
DOCKER_OPTS="--dns 172.20.20.11 --dns 172.10.20.12"
```

对于两种尝试，您都需要重启 docker：`docker run busybox less /etc/resolv.conf`，然后重新运行 nslookup 测试或尝试 `docker run busybox less /etc/resolv.conf`。如果此测试返回

```
# Dynamic resolv.conf(5) file for glibc resolver(3) generated by resolvconf(8)
#     DO NOT EDIT THIS FILE BY HAND -- YOUR CHANGES WILL BE OVERWRITTEN

nameserver 8.8.8.8
nameserver 8.8.4.4
```

则我们可能仍然没有访问权限。

**解决方案**
解决此问题的一种方法是...

* 编辑 NetworkManager 文件

```
sudo cp /run/resolvconf/interface/NetworkManager /run/resolvconf/interface/NetworkManager.orig
sudo nano /run/resolvconf/interface/NetworkManager
```

* 添加您的特定 DNS 条目，例如，

```
nameserver 172.20.20.11
nameserver 172.10.20.12
```

* 更新 resolveconf

```
sudo resolvconf -u
```

* 验证更改现在在容器中可见

```
docker run busybox less /etc/resolv.conf
```

应该列出相同的 DNS IP，例如，

```
# Dynamic resolv.conf(5) file for glibc resolver(3) generated by resolvconf(8)
#     DO NOT EDIT THIS FILE BY HAND -- YOUR CHANGES WILL BE OVERWRITTEN
nameserver 172.20.20.11
nameserver 172.10.20.12
```

参考：

* <https://stackoverflow.com/questions/24151129/docker-network-calls-fail-during-image-build-on-corporate-network>
* <https://development.robinwinslow.uk/2016/06/23/fix-docker-networking-dns/>

# 在网络问题下添加软件包

我们想要添加一个软件包 - 比如 iputils-ping，以便我们可以使用 ping 命令

# 网络问题

如上所述，由于假定的防火墙问题，我们无法连接到网络。

1. 启动容器 - 在容器内...
2. 手动添加 DNS 地址：`vim /etc/resolv.conf` 并如上添加 nameserver 条目。
3. `sudo apt install iputils-ping nano`

# ROS 密钥问题

当我们尝试 `sudo apt update` 时，我们收到类似以下的错误

```
W: An error occurred during the signature verification. The repository is not updated and the previous index files will be used. GPG error: http://packages.ros.org/ros/ubuntu bionic InRelease: The following signatures couldn't be verified because the public key is not available: NO_PUBKEY F42ED6FBAB17C654
```

这是一个非 docker 特定的问题，在此描述：<https://answers.ros.org/question/325039/apt-update-fails-cannot-install-pkgs-key-not-working/>

# 如何更新现有 Docker 容器

这些说明对 BSB 的设置有些特定。

## 主机上的网络设置

由于 NPS 网络问题。我们需要在主机上执行以下操作（来自上述内容）。

* 编辑 NetworkManager 文件

```
sudo cp /run/resolvconf/interface/NetworkManager /run/resolvconf/interface/NetworkManager.orig
sudo nano /run/resolvconf/interface/NetworkManager
```

* 添加您的特定 DNS 条目，例如，

```
nameserver 172.20.20.11
nameserver 172.10.20.12
```

* 更新 resolveconf

```
sudo resolvconf -u
```

* 验证更改现在在容器中可见

```
docker run busybox less /etc/resolv.conf
```

应该列出相同的 DNS IP，例如，

```
# Dynamic resolv.conf(5) file for glibc resolver(3) generated by resolvconf(8)
#     DO NOT EDIT THIS FILE BY HAND -- YOUR CHANGES WILL BE OVERWRITTEN
nameserver 172.20.20.11
nameserver 172.10.20.12
```


==============================================================================

# 第 16 章：贡献者
# 第 16 章: 贡献者
==============================================================================

社区的贡献对于继续增长海事机器人仿真能力极为重要。我们特别感谢以下项目、组织和个人的重要贡献：

* 原始海洋表面模型（着色器、纹理等）来自 [uuv_simulator](https://github.com/uuvsimulator/uuv_simulator) 项目。
* @JonathanWheare 帮助维护了对 Kinetic/Gazebo7 的支持，添加了声学定位器模型，并改进了仿真处理激光雷达和水面的方式。
* @srmainwaring 为他的波浪模拟器找到了一种同步视觉和物理海洋表面模型的方法，并通过 [wave_sim_vrx](https://github.com/srmainwaring/wave_sim_vrx) 项目慷慨地与 VRX 分享。

有关贡献的更多详细信息（以及它们如何实现的详细信息），可以浏览 [Pull requests](https://github.com/osrf/vrx/pull-requests/) 项目历史。有关旧 BitBucket 仓库的 pull request 历史，请参阅[此处](https://osrf-migration.github.io/vrx-gh-pages/#!/osrf/vrx/pull-requests/page/1)。


==============================================================================

# 附录
==============================================================================


### 其他资源 / 其他资源

- [VRX GitHub Repository](https://github.com/osrf/vrx)
- [VRX Wiki (Online)](https://github.com/osrf/vrx/wiki)
- [VRX Classic Wiki](https://github.com/osrf/vrx/wiki/VRX-Classic-Home)
- [RobotX Competition](https://robotx.org/)

### 如何引用 / 如何引用

如果您在工作中使用了 VRX 仿真，请引用：
如果你在工作中使用了 VRX 仿真，请引用：

```bibtex
@InProceedings{bingham19toward,
  Title     = {Toward Maritime Robotic Simulation in Gazebo},
  Author    = {Brian Bingham and Carlos Aguero and Michael McCarrin and
              Joseph Klamo and Joshua Malia and Kevin Allen and
              Tyler Lum and Marshall Rawson and Rumman Waqar},
  Booktitle = {Proceedings of MTS/IEEE OCEANS Conference},
  Year      = {2019},
  Address   = {Seattle, WA},
  Month     = {October}
}
```

---

*文档编译自 GitHub 上的 VRX Wiki。*
*已处理页面：69/69*
*文档编译自 GitHub 上的 VRX Wiki。已处理页面：69/69*