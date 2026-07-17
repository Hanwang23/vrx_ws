# 动态目标与 COLREGs 学习指南

## 1. 为什么下一阶段先学这个

当前项目已经具备一条稳定的静态航行链：

```text
robot_localization EKF
  -> 滚动占据栅格
  -> Dubins State Lattice A*
  -> ILOS 路径跟踪
  -> 航速/艏摇闭环
```

下一步不应立刻让 NMPC 接管推进器。先学习动态目标与 COLREGs，可以复用上述全部模块，同时补上真实水面航行必须具备的“预测别人将去哪里”能力。动态船不能只当成一枚静态浮标：它可能在几秒后离开，也可能与本船形成碰撞趋势。

本阶段目标是：在官方 Wayfinding 任务中生成红绿航道浮标和两艘可见运动船，跟踪多目标速度，计算 CPA/TCPA，选择最紧急的交叉会遇，并让现有 ILOS/PID 控制器执行右转和降速约束。

## 2. 先建立四个概念

### 2.1 相对位置和相对速度

在 `codex_odom` 的东-北坐标系中，定义：

```text
p_rel = p_target - p_own
v_rel = v_target - v_own
```

`p_rel` 表示目标当前相对本船的位置，`v_rel` 表示从本船视角看目标如何运动。只看当前距离不够：距离很近但正在远离的船，通常不如距离较远但快速逼近航线的船危险。

### 2.2 TCPA

TCPA 是按当前速度继续运动时，到达最近会遇点还需要的时间：

```text
TCPA = -(p_rel dot v_rel) / |v_rel|^2
```

- `TCPA > 0`：最近会遇点还在未来。
- `TCPA < 0`：最近会遇点已经过去，双方总体正在分离。
- 相对速度接近零时，不计算 TCPA，避免除以很小的数。

### 2.3 DCPA

把相对位置推进到 TCPA 时刻：

```text
p_cpa = p_rel + v_rel * TCPA
DCPA = |p_cpa|
```

DCPA 是预测的最近会遇距离。当前实现只有同时满足以下条件才认为存在风险：

```text
0 < TCPA <= 120 s
DCPA < 15 m
目标已被连续观测并确认正在稳定运动
```

### 2.4 相对方位

目标位置被转换到本船坐标：前方为 0 度，左舷为正，右舷为负。当前教学控制器使用简化会遇分类：

| 会遇 | 判定要点 | 动作 |
| --- | --- | --- |
| 对遇 `head_on` | 目标在前方约正负 15 度，航向近似相反 | 右转并降速 |
| 右舷交叉 `crossing_starboard` | 目标位于本船右前/右侧 | 本船让路，右转并降速 |
| 左舷交叉 `crossing_port` | 目标位于本船左前/左侧 | 保持航向并持续监视 |
| 追越 `overtaking` | 目标位于较大后方角度 | 保持足够净空，默认右侧避让 |
| 近距离不确定 `close_quarters` | 有 CPA 风险但不属于上述稳定分类 | 保守右转并降速 |

这是面向仿真学习的监督层，不是可直接取代船员判断或完整法规系统的认证实现。

## 3. 软件如何分层

```text
Gazebo 可见目标船
  -> /autonomous_usv/dynamic_targets (理想化位置检测)
  -> alpha-beta 动态轨迹跟踪
  -> CPA/TCPA + 会遇分类
  -> 航向偏置和速度缩放
  -> State Lattice / ILOS 参考
  -> 原有艏摇和航速控制器
```

安全优先级从高到低是：

1. 传感器失效停车、近距离制动和碰撞走廊。
2. COLREGs 动态目标监督，提前右转或降速。
3. 激光局部避障，应对预测失准或突然接近。
4. State Lattice 静态全局绕行。
5. ILOS 贴合参考路径和完成航点。

动态目标会从“静态地图融合副本”中屏蔽，但不会从原始激光安全层中删除。这样运动船不会在滚动栅格中留下持续 8 秒的虚假墙，同时近距离时仍能触发紧急避障。

## 4. 第一次运行

### 4.1 构建

打开终端：

```bash
cd /home/han/Ai_ws/Study/vrx_ws
source /opt/ros/humble/setup.bash
export GZ_VERSION=garden
colcon build --merge-install --base-paths codex_fix \
  --packages-select codex_usv_controller
source install/setup.bash
```

看到 `Finished <<< codex_usv_controller` 表示构建完成。

### 4.2 启动带界面的学习场景

```bash
ros2 launch codex_usv_controller colregs_learning.launch.py
```

它会执行以下过程：

1. 启动官方 Wayfinding 场景和控制器。
2. 约 5 秒后依次生成两艘橙白色目标船。
3. 约 6.5 秒开始依次生成 6 对红绿航道浮标。
4. 第一艘船以 `1.0 m/s` 向西运动，第二艘稍晚以 `0.7 m/s` 向南运动。
5. RViz 用橙色船体框和速度箭头显示两条动态轨迹；风险目标会变红。
6. WAM-V 正常完成三个官方航点，并在预测交会时提前让路。

### 4.3 观察动态目标话题

另开一个终端，并重新加载环境：

```bash
cd /home/han/Ai_ws/Study/vrx_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic hz /autonomous_usv/dynamic_targets
```

期望约为 `9-10 Hz`。由于仿真负载和 ROS 调度，略低于 10 Hz 正常。

查看一个完整检测样本：

```bash
ros2 topic echo /autonomous_usv/dynamic_targets --once
```

这里的位置是 `codex_odom` 局部 ENU 坐标，不是 Gazebo 世界绝对坐标。

### 4.4 观察决策状态

```bash
ros2 topic echo /autonomous_usv/status --full-length
```

重点字段：

| 字段 | 正常含义 |
| --- | --- |
| `dynamic_track_count` | 默认场景稳定后应为 2，表示两艘动态船 |
| `colregs_target_source` | 应为 `dedicated_topic` |
| `dynamic_target_age_s` | 通常明显小于 1 秒 |
| `colregs_active` | 只有存在未来 CPA 风险时为 `true` |
| `colregs_encounter` | 默认主要出现 `crossing_starboard` |
| `colregs_tcpa_s` | 最近会遇点还需多少秒 |
| `colregs_dcpa_m` | 预测最近会遇距离 |
| `colregs_heading_bias_deg` | 让路时为负值，表示右转偏置 |
| `colregs_speed_scale` | 让路时小于 1.0 |
| `dynamic_masked_scan_beams` | 激光照到目标船时可能大于 0 |
| `num_collisions` | 必须保持为 0 |

不要期待 `colregs_active` 从头到尾为真。一直为真反而说明风险解除逻辑错误。

## 5. 自动评测

完成学习观察后，运行完整官方闭环：

```bash
cd /home/han/Ai_ws/Study/vrx_ws
bash codex_fix/scripts/run_colregs_learning.sh
```

脚本会自动构建、清理残留仿真、启动无界面官方任务、采集状态并判断：

- 三个航点全部完成。
- 碰撞数为 0。
- 官方平均误差不超过 `3.0 m`。
- 任一航点误差不超过 `3.5 m`。
- COLREGs 激活样本不少于 20。
- 最多动态跟踪数不少于 2，确认两艘目标船都进入了判断链。
- State Lattice 没有持续 fallback。
- `robot_localization` 持续提供估计。

成功时末尾应看到：

```text
TRIAL 1/1: PASS all thresholds satisfied
```

最新结果可查看：

```bash
python3 -m json.tool codex_fix/evaluation/latest.json | less
```

本次已验证基线位于 `codex_fix/evaluation/20260717T002427Z/aggregate.json`：

| 指标 | 结果 |
| --- | ---: |
| 完成率 | 100% |
| 碰撞 | 0 |
| 最大动态跟踪数 | 2 |
| COLREGs 激活样本 | 189 |
| 会遇分类 | `close_quarters` 87、`crossing_starboard` 65、`crossing_port` 37 |
| 最小净空 | 5.952 m |
| 官方平均误差 | 1.076 m |
| 最大横向误差 | 4.196 m |
| 最大艏摇角速度 | 11.244 deg/s |
| 最大规划耗时 | 6.151 ms |
| State Lattice fallback | 0 |
| 最大动态激光屏蔽数 | 83 条光束 |

最大横向误差比纯静态任务大是可解释的：让路动作有意离开名义路径。评价动态避碰时，应先保证无碰撞和足够 DCPA，再比较路径误差，不能只追求贴线。

## 6. 三个循序练习

### 练习一：只改变目标速度

```bash
ros2 launch codex_usv_controller colregs_learning.launch.py \
  target_velocity_x:=-0.7
```

记录首次 `colregs_active=true` 时的 TCPA、DCPA。再用 `-1.3` 重复。思考：速度更快是否一定更危险，还是要结合相对几何判断？

### 练习二：改变启动时刻

```bash
ros2 launch codex_usv_controller colregs_learning.launch.py \
  target_motion_delay:=25.0
```

目标更晚出发后，双方到达交会区的时刻改变。观察是否仍有 `crossing_starboard`，以及激活持续时间如何变化。

### 练习三：让目标反向航行

```bash
ros2 launch codex_usv_controller colregs_learning.launch.py \
  target_start_x:=-590.0 target_start_y:=205.0 \
  target_velocity_x:=1.0 target_velocity_y:=0.0
```

这不是保证形成某一规则会遇的标准场景。练习重点是先画出双方航线，再用 TCPA/DCPA 解释控制器为何触发或不触发，避免把参数调试变成猜测。

每次实验只改一组参数，并保存以下指标：完成、碰撞、最小净空、COLREGs 激活样本、会遇分类计数、横向误差和任务耗时。

## 7. 常见问题

### 7.1 目标船没有出现

```bash
ros2 service list | rg /world/wayfinding_task/set_pose
ros2 node list | rg 'moving_target|spawn_colregs_target'
```

若服务不存在，检查 `ros_gz_bridge` 是否安装，以及构建后是否重新 `source install/setup.bash`。

### 7.2 有目标轨迹，但从不触发 COLREGs

依次检查：

1. `dynamic_target_age_s` 是否小于轨迹超时。
2. 目标是否实际运动超过 2 秒和 2 米。
3. TCPA 是否为未来正值。
4. DCPA 是否小于 `colregs.safety_radius`。
5. 目标速度是否大于 `colregs.minimum_target_speed`。

不要先扩大安全半径。先判断是不是双方本来就在分离。

### 7.3 静态场景误触发 COLREGs

确认 `colregs.use_pointcloud_tracks: false`。匿名点云簇可能在相邻浮标间跳变，形成假的速度；默认只允许专用动态检测话题激活规则层。

### 7.4 目标船在地图中留下障碍尾迹

查看 `dynamic_masked_scan_beams` 和 `dynamic_masked_cloud_tracks`。目标进入雷达视场时二者至少有一个应出现非零峰值。还可适度增大 `colregs.map_mask_radius`，但过大会误删目标附近真实静态障碍。

### 7.5 让路后横向误差变大

先检查碰撞和最小净空。动态会遇中短时离开路径是合理行为；风险解除后，State Lattice/ILOS 会重新收敛。只有长期无法回线或无法完成航点时，才调整航向偏置、速度缩放或重规划参数。

## 8. 下一阶段学习路线

完成本指南后，按以下顺序继续：

1. **多目标数据关联**：从单个理想化 `PoseArray` 扩展到多个带 ID、协方差和时间戳的目标。
2. **传感器目标检测**：使用雷达/点云或视觉产生目标量测，专门评估漏检、误检和 ID 切换。
3. **轨迹预测不确定性**：让 DCPA 安全半径随协方差和目标机动扩大，而不是固定阈值。
4. **完整 COLREGs 场景集**：分别构建对遇、左/右交叉和追越官方任务回归。
5. **3-DOF 系统辨识**：在无碰撞场景采集持续激励数据，验证模型残差和跨航速泛化。
6. **影子 NMPC**：先只计算不执行，与 ILOS/PID 对比预测误差、求解时间和约束违例。
7. **受门控接管**：只有模型质量、实时性和安全回退全部通过后，才允许 NMPC 输出推进器命令。

当前最重要的学习成果不是“看见船就转弯”，而是能用相对速度、TCPA 和 DCPA解释每一次触发与解除。
