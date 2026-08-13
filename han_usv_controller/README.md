# VRX WAM-V 自主闭环控制器

这是 `CLAUDE.md` 方案 C 的可运行实现，本次新增的源文件都位于 `han_usv_controller`。工作区根目录原有的 `autonomous_controller.py` 和 `launch_wayfinding.sh` 未被本方案修改。控制器订阅 GNSS、IMU、激光雷达扫描与点云和 VRX 任务目标，使用速度 PID 与受限角速度级联航向控制生成左右推进器命令。

当前直接支持：

- `wayfinding_task`：依次到达官方发布的经纬度航点，并在航点对准指定艏向。
- `stationkeeping_task`：驶向官方目标并持续抵抗风浪漂移。
- `custom`：按 YAML 中自定义的 GNSS 航点航行。
- 所有模式：6 状态 GNSS/IMU EKF、100 m 滚动占据栅格、Dubins State Lattice A*、点云水面分离与低浮标跟踪、极坐标自由间隙规划、主动反推制动、安全倒车恢复、传感器超时停车、推力限幅、重复控制器检测和状态 JSON 话题。

## 无竞赛倒计时的学习世界

`simulation.launch.py` 和 `buoy_course.launch.py` 默认使用 `han_usv_controller/worlds/wayfinding_task.sdf`。场景、航点、话题和 Gazebo 内部世界名都与官方版本一致，只把 `running_state_duration` 从 300 秒改为 100 个仿真年，因此学习时不会在约 320 秒后自动结束。

直接运行即可进入学习模式，不需要再传 `world` 参数：

```bash
ros2 launch han_usv_controller simulation.launch.py
```

需要复现实战竞赛倒计时时，可显式切回官方世界：

```bash
ros2 launch han_usv_controller simulation.launch.py timed_competition:=True
```

计时开关与 VRX 的调试话题开关相互独立。还要模拟正式竞赛的话题可见性时，再加 `competition_mode:=True`；平时学习保持默认 `False` 更便于诊断。

`world:=stationkeeping_task` 等其他任务仍加载其官方世界；无限时长替换只对 `wayfinding_task` 生效。

## 八航点算法压力课程

`multi_waypoint_course.launch.py` 会在启动时用 XML 解析无限时长的
`wayfinding_task.sdf`，只替换官方评分插件的航点块，生成 8 个确定性航点。
课程包含 1 条短于 `2 x 8 m` Dubins 转弯直径的直线 ILOS 航段、3 条超过
40 m 的 State Lattice 分段规划航段、连续左右转和 8 个不同最终艏向。
控制器在这个 launch 中严格保留 XML 发布顺序，不执行最近邻重排。

带 Gazebo 和 RViz 观察完整课程：

```bash
ros2 launch han_usv_controller multi_waypoint_course.launch.py
```

自动连续运行三轮并检查 `8/8`、0 碰撞、官方逐点误差、角速度、最长原地
调整、单航点耗时、State Lattice fallback 和估计器健康状态：

```bash
bash han_usv_controller/scripts/run_multi_waypoint_evaluations.sh
```

评估器会把课程坐标、每轮日志、每航点耗时和判定写到
`han_usv_controller/evaluation/<UTC时间>/`，聚合结果同时写入
`han_usv_controller/evaluation/latest.json`。若另一个 Gazebo 正在运行，脚本默认拒绝
启动；先在旧 launch 终端按 `Ctrl+C`。仅在确定可以关闭残留仿真时使用：

```bash
FORCE_CLEAN_STALE=True bash han_usv_controller/scripts/run_multi_waypoint_evaluations.sh
```

2026-07-17 的最终 v4 三轮回归保存在
`han_usv_controller/evaluation/20260717T055622Z/aggregate.json`：`3/3 PASS`、每轮
`8/8`、合计 0 碰撞、0 State Lattice fallback；聚合平均耗时
`574.1 s`，平均官方误差 `1.083 m`，三轮最差单点误差 `1.712 m`，
最小净空 `16.01 m`，全程峰值角速度 `14.27 deg/s`，最长连续原地对准
`58.53 s`，最长单航点 `143.66 s`。

本压力课程还修复了三个由长课程暴露的控制问题：局部 40 m 路径接力失败
时不再沿 ILOS 掉头返回旧端点；终端直线 ILOS 带 8 m 滞环，只有被障碍
从 16 m 终端区推出到 24 m 外才恢复碰撞检查的 State Lattice/Dubins 路径，
避免在捕获圈附近生成只前进 Dubins 回环；最终艏向对准检测到 4 m 内障碍时进入
`alignment_blocked` 并制动艏摇，不继续旋转扫碰。正常导航的大角度转向还会
用“安全半径 + 船体半宽”检查全向扫掠净空；紧急制动按航速符号工作，船向后
漂向岸体时会给正推制动，不再误当成静止原地转向。

## 密集浮标测试场

新增的 `buoy_course.launch.py` 会在官方 Wayfinding 世界里额外生成 16 个锚定浮标：6 对红绿浮标组成约 30 米宽门，4 个橙色浮标位于对应航段中心线外约 12 米，用于触发激光雷达避障并保留船体转弯扫掠净空。浮标具有碰撞体和雷达可见几何，使用静态锚定方式，不会被波浪冲离测试位置。

完成首次构建后，用下面一条命令启动带额外浮标的仿真和控制器：

```bash
ros2 launch han_usv_controller buoy_course.launch.py
```

该命令现在会同时打开 Gazebo 和预配置的 RViz 点云视图。RViz 默认延迟 6 秒启动，等待 WAM-V 的 TF 和雷达话题就绪。

`simulation.launch.py`、`random_buoy_course.launch.py`、
`buoy_course.launch.py`、`lattice_stress.launch.py` 和
`colregs_learning.launch.py` 都显式声明并逐层透传同一个 `rviz_config`，默认
统一加载 `config/pointcloud.rviz`。因此航点、规划范围、安全圈、运动矢量、
系统健康和规划器面板会在这五种场景中保持一致；也可以给任一命令追加
`rviz_config:=/绝对路径/其他配置.rviz` 临时覆盖。

无界面运行：

```bash
ros2 launch han_usv_controller buoy_course.launch.py headless:=True rviz:=False
```

浮标会从 `spawn_delay` 开始每隔 0.25 秒依次生成，避免同时请求 Gazebo 导致服务超时。如果机器启动 Gazebo 较慢，可延后开始时间：

```bash
ros2 launch han_usv_controller buoy_course.launch.py spawn_delay:=10.0
```

浮标场按 `wayfinding_task` 的坐标设计。运行原始、无额外浮标的任务时，仍使用 `simulation.launch.py`。

`buoy_course.launch.py` 会从 `world` 别名或 SDF 文件名自动推导 Gazebo 创建服务名。只有自定义 SDF 的文件名和内部 `<world name>` 不一致时，才需要额外传 `gz_world_name:=内部名字`。

## State Lattice 压力场

`lattice_stress.launch.py` 在密集浮标场的第一条长航段中心再放置三枚间隔 4 m 的橙色浮标。经过 3 m 栅格膨胀后，它们形成一条必须绕行的短障碍带，用于验证 A* 确实展开，而不是只走无遮挡解析 Dubins 连接。

带界面观察点云、地图和规划路径：

```bash
ros2 launch han_usv_controller lattice_stress.launch.py
```

自动压力评测会重建工作区，并要求 `max_lattice_expanded_states >= 1`、0 碰撞和 0 持续 fallback。若检测到另一套 ROS/Gazebo 仿真正在运行，评测器默认拒绝启动并列出 PID，避免误杀其它学习会话：

```bash
bash han_usv_controller/scripts/run_lattice_stress.sh
```

## 动态目标与 COLREGs 学习场

`colregs_learning.launch.py` 在官方 Wayfinding 中默认生成 6 对红绿航道浮标和两艘约 7 米长的运动目标船。第一艘由东向西横穿第二航段，第二艘稍晚由北向南横穿后续航段；它们都在 `han_usv_odom` 发布理想化动态目标检测。控制器跟踪多目标速度、计算 CPA/TCPA、选择最紧急会遇，并对现有 ILOS/PID 施加右转和降速约束。动态目标只从静态栅格融合副本中屏蔽，未屏蔽的原始激光仍用于近距离紧急避障。

RViz 中橙色长方体表示动态船，橙色箭头表示预测速度方向；被 COLREGs 选中的风险船会变红，并显示 TCPA、DCPA 和会遇类型。红绿浮标是静态雷达参照物，只进入占据栅格和局部避障，不会触发 COLREGs。

RViz 左侧的项目自定义显示项已使用中文名称。画面内的 `TEXT_VIEW_FACING` Marker 标签保留英文，因为 ROS 2 Humble 当前 RViz/Ogre 默认 Marker 字体实测不包含中文字形，中文会显示为空白。这些英文只是可视化标签，不影响中文面板名称和算法运行。

`浮标候选 (激光聚类)` 图层将三维激光点云经过水面平面滤除、空间聚类和多帧确认后的静态小障碍物显示为黄色圆环，标签 `BUOY #编号 | 距离`会自动向画面中心偏移。专用动态船轨迹附近的聚类会被排除。该图层表示“形状和尺寸类似浮标的雷达候选”，不声称仅凭激光就能识别红色/绿色语义。实时数量同时写入状态字段 `buoy_candidate_count`。

旧的红色通用障碍聚类 Marker 与黄色浮标候选来自同一批点云数据，会造成重复和坐标偏移观感，因此已删除。黄色圆环是唯一的静态小目标聚类标记；该改动只影响 RViz，内部避障、滚动栅格和规划仍使用原始障碍数据。

黄色浮标候选现在只对已经消失的 Marker ID 做定点删除，不再每帧执行 `DELETEALL`，因此 RViz 不会在“全部清空”和“重新绘制”之间闪烁。

`simulation.launch.py` 还带有两层重复启动保护：启动前检查真实运行中的 `gz sim`，可识别升级前残留或外部启动的 Gazebo；随后获取工作区级独占锁，解决两个新 launch 同时启动的竞态。若另一个终端已经有仿真，第二次 `simulation.launch.py`、`colregs_learning.launch.py` 或包含基础仿真的其他 launch 会在启动第二个 Gazebo/WAM-V 前退出，并显示占用 PID。锁随 launch 进程退出自动释放，磁盘上保留的 `.lock` 文件本身不代表仍被占用。

带 Gazebo 和 RViz 运行：

```bash
ros2 launch han_usv_controller colregs_learning.launch.py
```

自动运行一次完整官方闭环，并要求至少 20 个真实 COLREGs 激活样本：

```bash
bash han_usv_controller/scripts/run_colregs_learning.sh
```

2026-07-17 已验证结果位于 `han_usv_controller/evaluation/20260717T002427Z/aggregate.json`：官方任务完成、0 碰撞、同时跟踪 2 艘动态船、COLREGs 激活 189 个样本、最小净空 5.952 m、官方平均误差 1.076 m、State Lattice fallback 为 0。自动脚本同时要求 `max_dynamic_track_count >= 2`，避免第二艘船失效时误报通过。

完整原理、状态字段、参数练习和故障诊断见 [`动态目标与COLREGs学习指南.md`](动态目标与COLREGs学习指南.md)。当前范围是“理想化目标检测后的跟踪与决策”，尚未实现可部署的雷达/视觉船舶检测器，也不是经过认证的完整 COLREGs 系统。

## 为什么没有加入 SLAM / Nav2

这套 VRX 任务位于开阔水面，官方目标以 WGS84 经纬度发布，艇上已有 GNSS、IMU 和雷达。水面缺少长期稳定的墙面特征，波浪又会带来姿态扰动，二维占据栅格 SLAM 通常不是航点任务的主定位源。这里采用：

`GNSS/IMU EKF + 雷达滚动局部地图 + State Lattice A* + PID 船体控制`

这比直接套用陆地机器人的 Nav2/SLAM 更贴合当前任务。进入无 GNSS 水域、港池精细靠泊或需要保存固定岸线地图时，再增加雷达/视觉定位和地图规划更合适。

## EKF、滚动地图与 State Lattice

状态估计器融合 `[东, 北, 东速, 北速, 航向, 艏摇角速度]` 六个状态。GNSS 提供位置和相邻采样速度观测，IMU 提供带角度环绕处理的航向与角速度观测；Joseph 形式协方差更新和 `6 sigma` 创新门限用于抑制跳点。状态话题会发布位置、速度和航向标准差，以及异常观测拒绝数。

官方任务会话重新开始时，目标、EKF、速度估计、传感器时间戳、点云轨迹和滚动地图会一起原子重置。控制器必须重新收到 GNSS、IMU 和雷达样本后才恢复动作，不会把上一任务的位置或障碍带入下一任务。

激光束以逆传感器模型写入 `100 m x 100 m`、`0.5 m` 分辨率的滚动栅格。每帧每个栅格只更新一次且占用命中优先，避免相邻自由射线重复清除低浮标。达到 5 次确认的点云轨迹也会作为占用证据写图；自由空间降低 log-odds，命中提高 log-odds，超过 8 秒的证据变为未知。`+inf` 被解释为无回波自由射线，`NaN`、`-inf` 和小于量程下限的值不会清空地图；局部安全层会把 `-inf` 或过近值当作紧急近障碍。长时间传感器中断会按完整间隔衰减旧证据。窗口按整格平移，因此艇运动时不会擦掉仍在窗口内的障碍。地图写入和规划都使用当前被选中的估计器状态源，规划快照按 `3 m` 安全半径膨胀，原始概率地图发布到 `/autonomous_usv/rolling_grid`。发布时地图原点会从内部 ENU 转到 `wamv/wamv/base_link`，可直接与点云一起在附带 RViz 配置中查看。

这里的规划器更准确地说是受 Dubins 曲率约束、以栅格与离散航向作为状态键的 Hybrid A*/State-Lattice 风格规划器。它使用 `L/S/R` 三种只前进运动基元，全部满足 `|kappa| <= 1/R_min`。无遮挡时先使用经过栅格碰撞检查的解析 Dubins 连接；连接被占用栅格阻断时才展开 A*。目标超过滚动地图时先规划 40 m 子目标，路径剩余小于 6 m 后从最新 EKF 状态继续规划。两个独立地图版本持续阻断未来路径时触发 `lattice_obstacle` 在线重规划，并带 5 秒冷却。只有起点栅格本身被占用时，4 m 豁免才允许路径连续驶出起点占用团；遇到第一个自由栅格后立即失效，路径离开后重新进入该区域仍正常碰撞检查。在线搜索或分段续规划失败会保留当前已碰撞检查路径并交给局部安全层，不用未经检查的 Dubins 结果覆盖它。

## State Lattice + ILOS 分层制导

正常航行不再直接把“船到目标点的方位角”交给推进器。当前控制链为：

```text
GNSS/IMU EKF 状态 + 滚动占据栅格
  -> Dubins State Lattice A* / 碰撞检查后的解析连接
  -> ILOS 抗横流路径跟踪
  -> 避障解除后的事件触发路径重规划
  -> 曲率预瞄限速 + v*kappa 角速度前馈
  -> 雷达局部安全航向过滤
  -> 角速度参考治理器 + 航速增益调度闭环
  -> 左右推进器
```

求解器实现并测试了 `LSL/RSR/LSR/RSL/RLR/LRL` 六类只前进 Dubins 路径。WAM-V 默认只在四类 CSC 路径中选最短解，因为 RLR/LRL 会在圆弧连接处瞬时反转曲率，真实船体无法瞬时反转角动量；需要做纯几何实验时可打开 `dubins_allow_three_turn_paths`。

最小转弯半径与航速必须共同满足近似动力学约束：

```text
R_min >= cruise_speed / max_yaw_rate
```

当前几何半径为 `8 m`，但不会以 1.6 m/s 硬闯圆弧。控制器在未来一个 ILOS 前视距离内预瞄解析曲率，并按 `v_curve <= sqrt(a_lat_max / |kappa|)` 限速；`a_lat_max=0.12 m/s^2` 时，8 m 圆弧约限为 `0.98 m/s`，只需约 `7.0 deg/s` 艏摇角速度。目标包含最终 yaw 且距离足够时使用 Dubins；没有目标 yaw 或距离太短时自动退化为直线 ILOS。

对路径局部切向 `chi_p`，定义左侧为正的横向误差：

```text
e_y = -sin(chi_p) * (E-E_p) + cos(chi_p) * (N-N_p)
chi_d = chi_p - atan2(e_y + b_I, Delta)
```

所以船位于路径左侧时 `e_y > 0`，ILOS 会给出更小的期望航向，向右回到路径。`b_I` 是受限积分偏置，用于抵消稳定横流和模型偏差；雷达绕障、制动、倒车、低速、最终艏向对准以及横向误差过大时积分冻结，避免把临时避障轨迹学习成“海流”。

路径圆弧的解析曲率直接生成前馈角速度：

```text
r_ff = gain * desired_speed * path_curvature
r_cmd = rate_limit(k_heading * heading_error + r_ff)
```

`r_cmd` 还受到 `5 deg/s^2` 的变化率限制，并限制为 `12 deg/s`，因此局部避障从左侧间隙切到右侧间隙时不会一帧反满舵。低航速时水动力艏摇阻尼下降，差动转向推力会从高速上限 160 自动调度到低速上限 80。接近目标 22 m 内允许最多 240 的平滑反推消除惯性；曲率预瞄发现实际航速高于弯道速度包络时，即使还在远航段也允许受限反推，不再只撤油门长距离滑行。

最终艏向对准使用独立的低速级联闭环。外环把航向误差压缩为不超过 `5 deg/s` 的目标艏摇角速度，再由 `2.5 deg/s^2` 参考治理器限制目标变化，内环用 IMU 艏摇角速度消除角动量。正常转向和反向制动推力分别限制为 60 和 90；即使船跨过目标艏向，参考角速度也不会从左转一帧跳成右转，避免低阻尼 WAM-V 在 `FINAL HEADING ALIGNMENT` 中形成饱和极限环。评估器分别记录进入对准时的船体实测角速度和控制器目标角速度，避免把进点残余角动量误判成限幅器失效。

局部避障采用到目标为止的有限检查视距。目标只有 5 m 远时，位于目标后方 18 m 的浮标不会把船从航点拉走；无进展倒车还必须同时具备近障碍证据，并在终端捕获区完全禁用。

绕过浮标后，控制器不会永远强追最初生成的路径。安全层记录一次完整避障过程，雷达连续清空 `0.5 s` 后才认为避障结束；如果此时船到旧路径投影点的距离超过 `8 m`，就从当前 GNSS 位置和 IMU 艏向重新求一条 Dubins/ILOS 路径。重规划只用于带目标艏向的 Dubins 路径，并带 `5 s` 冷却。进入离目标两个转弯半径的终端区后，控制器会一次性从 Dubins/State Lattice 切换为直接 ILOS 进近，清除旧路径阻断计数；因此不会沿已经走到末端的曲线路径绕点，也不会被单帧雷达空白反复触发重规划或破坏终点捕获。

关键参数位于 `guidance`：

- `dubins_turn_radius`：动力学允许的最小转弯半径；太小会让路径不可跟踪，太大会绕远。
- `dubins_sample_step`：路径离散间距，只影响跟踪和显示精度，不改变解析路径。
- `dubins_allow_three_turn_paths`：是否允许 RLR/LRL；有明显转动惯量的 WAM-V 默认关闭。
- `ilos_lookahead`：增大后更平滑但回线较慢；减小后贴线更紧但容易摆动。
- `ilos_integral_gain`：稳定横流补偿速度，实船应从小值逐步增加。
- `ilos_integral_limit`：积分等效横向偏置上限，防止长期绕障后积分饱和。
- `ilos_correction_limit_deg`：ILOS 最大几何修正角，局部安全层仍拥有更高优先级。
- `curvature_feedforward_gain`：曲率到目标角速度的前馈比例，理论起点为 1.0。
- `max_lateral_acceleration`：曲率预瞄速度包络，降低后弯道更稳但更慢。
- `replan_path_deviation`：避障结束后允许继续跟踪旧路径的最大欧氏距离。
- `replan_cooldown`：两次事件重规划之间的最短时间，防止路径频繁变化。

推荐调参顺序：先关闭积分并确定 `turn_radius/lookahead`，再确认无流条件下横向误差收敛，最后在恒定横流中缓慢增加 `integral_gain`。不要同时调整路径、积分和推进器增益，否则无法判断变化来自哪一层。

对于后续真实 USV，我推荐按需求继续增加，而不是一次堆满算法：

- 默认启动 `robot_localization`，融合 GNSS 局部里程计、GNSS 差分速度、IMU yaw 和 yaw rate；自研 EKF 同时运行，用于一致性检查和失效回退。
- COLREGs 层已实现动态轨迹、CPA/TCPA、会遇分类和右转/降速约束，但只接受专用动态目标检测话题。匿名点云簇曾实测产生静态浮标误关联，因此默认不允许它触发航行规则。
- 3-DOF 数据记录、离线一阶水动力拟合和 NMPC 模型门控已经实现；NMPC 求解器尚未接入推进器，任何模型缺失或质量不足都保持 ILOS/PID。
- 当前静态浮标 Wayfinding 中，Dubins + ILOS + 局部间隙规划比直接引入 NMPC 更容易验证，也更符合任务复杂度。

## 首次构建

```bash
cd /home/han/Ai_ws/Study/vrx_ws
source /opt/ros/humble/setup.bash
export GZ_VERSION=garden
colcon build --merge-install --base-paths han_usv_controller --packages-select han_usv_controller
source install/setup.bash
```

上面假定当前工作区的 VRX 已经编译好；本机现在就是这个状态。若是全新的未构建工作区，先执行：

```bash
cd /home/han/Ai_ws/Study/vrx_ws
source /opt/ros/humble/setup.bash
export GZ_VERSION=garden
colcon build --merge-install --base-paths src --cmake-args "-DBUILD_TESTING=OFF"
source install/setup.bash
colcon build --merge-install --base-paths han_usv_controller --packages-select han_usv_controller
source install/setup.bash
```

## 推荐运行方式

只开一个终端，同时启动 Wayfinding 仿真和控制器：

```bash
cd /home/han/Ai_ws/Study/vrx_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_VERSION=garden
ros2 launch han_usv_controller simulation.launch.py
```

电脑性能紧张时可用无界面模式：

```bash
ros2 launch han_usv_controller simulation.launch.py headless:=True
```

测试停驻任务时只替换世界名：

```bash
ros2 launch han_usv_controller simulation.launch.py world:=stationkeeping_task
```

如果仿真已经由另一个终端启动，只启动控制器：

```bash
source /opt/ros/humble/setup.bash
source /home/han/Ai_ws/Study/vrx_ws/install/setup.bash
export GZ_VERSION=garden
ros2 launch han_usv_controller controller.launch.py
```

Wayfinding 会周期发布航点，后启动控制器通常能恢复；Stationkeeping 目标主要在任务 `ready` 阶段发布，因此停驻任务应优先用 `simulation.launch.py` 同时启动仿真和控制器。

不要同时运行根目录中的 `virtual_joystick.py`、`auto_pilot.py`、`autonomous_controller.py` 或其它推进器发布程序。节点检测到第二个 ROS 推进器发布者时会主动输出零推力。

## 观察运行状态

```bash
ros2 topic echo /autonomous_usv/status
ros2 topic echo /wamv/sensors/gps/gps/fix --once
ros2 topic hz /wamv/sensors/imu/imu/data
ros2 topic hz /wamv/sensors/lidars/lidar_wamv_sensor/scan
ros2 topic info /wamv/thrusters/left/thrust --verbose
ros2 topic hz /odometry/filtered
```

正常时终端每两秒打印当前状态、目标编号、距离、艏向误差、速度、选定航迹净空和左右推力。常见状态包括 `navigating`、`avoiding`、`approach_braking`、`braking`、`pivoting` 和 `backing_away`。推进器话题的 `Publisher count` 应为 `1`。

点云仿真运行后，可在另一个终端打开预配置的 RViz：

```bash
source /opt/ros/humble/setup.bash
source /home/han/Ai_ws/Study/vrx_ws/install/setup.bash
ros2 launch han_usv_controller pointcloud.launch.py
```

上面的窗口是近距离路径跟踪视图。需要确认远处岸线是否被最大 `130 m` 的雷达扫描时，另开一个终端启动广角雷达视图：

```bash
source /opt/ros/humble/setup.bash
source /home/han/Ai_ws/Study/vrx_ws/install/setup.bash
ros2 launch han_usv_controller lidar_overview.launch.py
```

广角视图使用 `10 m` 网格和较小缩放比例，亮绿色点列就是原始二维回波。船位固定在画面中心，岸线随船运动和转向在周围移动。它只启动 RViz，不会重复启动 Gazebo、控制器或推进器发布者，可以和路径跟踪视图同时打开。

启动后先找画面中央的船体，再看左下方的 `TRACKING`、右下方的 `SYSTEM HEALTH` 和上方的 `PLANNER` 是否持续更新。为给 Gazebo/RViz 半屏布局留下足够画布，左右 Dock 默认收起；需要切换图层时，用顶部 `Panels -> Displays` 重新打开显示列表。默认视图使用亮绿色 `2D lidar returns (130 m)` 显示原始二维激光回波，因此远处岸线也能直接看到；局部规划地图和原始 3D 点云默认关闭，需要检查地图融合或点云高度时，再在 `Displays` 中勾选 `局部规划地图` 或 `WAM-V 三维激光雷达`。三维点云已使用与传感器发布端匹配的 `Best Effort` QoS，手动打开后使用 0.55 透明度、2 像素和 0.35 秒衰减，既能看清又不会长期拖影；控制器确认的浮标候选由黄色圆环单独表示。

不要把原始雷达范围和规划地图范围混为一谈：Gazebo 激光的 `range_max` 是 `130 m`，但滚动占据栅格只融合船周围 `40 m`，用于局部避障和 State Lattice 碰撞检查。`40-130 m` 的岸基可以显示为亮绿色原始回波，但不会提前写入局部规划地图。启用地图后看到的半透明灰色区域表示已观测自由空间，不表示“雷达没有扫描”；黑色/深色栅格才是占用证据。

以船体为中心观察各标记：

- `TARGET`（蓝色箭头）只表示驶向航点坐标的位置方向；当前航点和控制方向已经表达同类信息，因此它默认隐藏。需要专门比较“目标方位”和“控制器选择方位”时，可在 Marker namespace 中临时勾选 `target_direction`。
- 灰色细线连接控制器实际采用的完整航点顺序；每个 `WP i/n` 都带 2 m 捕获圆和最终艏向箭头。绿色表示已完成，亮蓝表示当前目标，灰白表示尚未执行；当前航点额外显示 4 m 捕获后退出圆。
- 船周围由内到外显示 3 m 最小净空、5.5 m 紧急距离、22 m 感知警戒距离和 40 m State Lattice 规划范围。它们是算法参数的可视化，不是检测到的实体障碍。
- 亮绿色散点是最大 `130 m` 的原始二维激光回波；连续岸线通常呈弧线或带状点列。黄色圆环是经过点云聚类和多帧确认的浮标候选，两者不是同一层数据。
- 青色粗线是当前 Dubins/ILOS 未来规划路径，黄色粗线是 WAM-V 在当前路径版本下实际走过的轨迹。路径重规划时黄色轨迹和 20 秒统计会一起重新开始，防止拿旧规划的历史航迹与新规划误比较。两条线在船附近越贴合，直观跟踪越准确。
- 洋红线连接船体中心和当前路径投影点，洋红圆点是投影位置，旁边的 `XTE` 是横向误差绝对值。
- `TRACKING: ON TRACK` 表示当前误差和最近 20 秒正常跟踪平均误差都较小；`RECOVERING` 表示正在回线；`OFF PATH` 表示正常巡航时明显偏离；`SAFETY MANEUVER` 表示正在避障或执行 COLREGs；`GOAL APPROACH` 表示正在制动、终点捕获或对准。安全动作和终点阶段都不计入正常 20 秒平均值。
- 白色 `ILOS NOMINAL COURSE` 箭头是路径跟踪期望航向。
- `CHOSEN` 是控制器实际命令：绿色巡航、黄色避障、红色制动/原地转向、紫色倒车；到点后橙色 `FINAL HEADING ALIGNMENT` 表示正在对准竞赛航点规定的最终艏向。
- 青色矩形是船头前方的安全走廊；红球进入或靠近走廊时，控制器会避让或减速。
- 船尾的左右箭头分别表示左、右推进器；箭头方向表示正推或反推，长度表示推力大小。
- `MOTION` 中绿色箭头是当前纵向速度，蓝色箭头是期望速度与控制方向，紫色横向箭头表示目标艏摇方向；文字同时列出实测/期望速度和实测/期望艏摇角速度。
- `SYSTEM HEALTH` 分别显示 GPS、IMU、二维雷达、点云数据年龄和当前定位源；`PLANNER` 显示地图版本、已知/占用栅格数、路径版本、重规划原因、State Lattice 展开状态数、耗时和 fallback。

`TRACKING` 中的 `NORMAL 20s: AVG / MAX` 是最近 20 秒正常路径跟踪的平均/最大绝对横向误差；安全避让样本会被排除，避免把主动绕障误判为跟踪失败。`CURVE` 是当前路径曲率，`MOTION` 中的目标 `YAW` 是经过前馈、反馈和变化率限制后的目标角速度；最终艏向对准时也会显示独立 governor 的真实参考。直线段曲率应接近 0；进入青色圆弧前，速度会先下降，随后目标 `YAW` 平滑建立。RViz 的 Fixed Frame 默认是随船转动的 `wamv/wamv/base_link`，画面图例也会显示 `VIEW: VESSEL-FIXED`。因此船原地转向时黄色历史轨迹、航点和环境都会在屏幕中旋转；只有 `PLANNER` 中的 `PATH REV` 增加才表示规划路径真的更新。Marker 以最高 5 Hz 发布且保留 0.85 秒，丰富显示不会按 20 Hz 占用控制循环，也不会因轻微调度抖动闪烁。

如果拖动画面后视图跑偏，在右侧 `Views` 面板点击 `Zero` 恢复。灰色点云太密时，可在左侧 `Displays` 面板取消勾选 `WAM-V 3D lidar` 临时隐藏它，不会影响控制器运行。

## 参数与调参

源参数文件是 `/home/han/Ai_ws/Study/vrx_ws/han_usv_controller/config/controller.yaml`。建议一次只改一类参数：

- 转弯摆动：先降低 `limits.max_navigation_yaw_acceleration_deg_s2` 或 `limits.max_low_speed_turn_thrust`，不要先提高 D 增益。
- 转向太慢：小幅提高 `limits.navigation_heading_rate_gain`；角速度跟踪偏弱时再提高 `limits.navigation_yaw_rate_gain`。
- 航速太慢：提高 `navigation.cruise_speed`，再微调 `speed_pid.kp`。
- 接近航点冲过头：降低 `navigation.approach_gain`，或提高 `navigation.normal_brake_distance`；不要把正常制动推力调到紧急制动等级。
- 避障太晚：提高 `avoidance.warning_distance`。
- 正前方来不及停：提高 `avoidance.brake_time_horizon` 或 `avoidance.brake_gain`，不要只增大安全半径。
- 可通行的宽门被判为无路：小幅降低 `avoidance.safety_radius`；默认 3.0 米用于选间隙，实际直行走廊半宽由 `avoidance.path_half_width: 2.4` 单独控制。
- 侧面障碍导致不必要减速：减小 `avoidance.path_half_width`，但不建议低于 WAM-V 碰撞半宽约 1.27 米加定位余量。
- 点云把水面误判成障碍：先提高 `avoidance.cloud_min_above_water` 或 `avoidance.cloud_cluster_min_points`。`cloud_min_height` 只是水面拟合失败时的绝对高度下限。
- 低矮浮标偶尔漏检：降低 `avoidance.cloud_min_above_water`，或提高 `avoidance.cloud_track_timeout`；轨迹会投影回当前船体坐标，不会把旧点固定在雷达画面中。
- 船长时间原地转：降低 `avoidance.stuck_timeout`；超时后会按 `backup_duration` 和 `backup_thrust` 做有限倒车恢复。
- 雷达话题中断：scan 和点云缓存分别过期，超过 `scan_cache_timeout` 没有任何新雷达数据时立即零推力停车。
- 航点最终艏向来回摆：先降低 `limits.max_alignment_yaw_acceleration_deg_s2`，再小幅降低 `max_alignment_yaw_rate_deg_s` 或 `alignment_yaw_rate_gain`；`max_alignment_thrust` 限制正常转动，`max_alignment_brake_thrust` 只在消除已有角动量时使用。驻留使用严格进入、宽松退出的艏向滞回。
- 航点捕获边缘反复切换：`navigation.waypoint_exit_tolerance` 是捕获后的退出半径，应大于 `navigation.position_tolerance`，避免轻微漂移重新触发位置追踪。

`nearest_obstacle_m` 仅用于诊断传感器最近目标，`path_clearance_m` 决定正常航速，`collision_clearance_m` 和当前速度共同决定是否反推制动。三者分开是为了避免位于侧面的浮标把船锁死。

状态 JSON 还包含 `gps_age_s`、`imu_age_s`、`scan_age_s`、`cloud_age_s` 与 `yaw_rate_valid`，可直接判断画面异常来自控制逻辑还是传感器陈旧。控制看门狗使用稳态时钟，即使 `/clock` 暂停也会继续执行超时停车。

`estimator_source` 正常应主要为 `robot_localization`；外部滤波器超时、协方差过大或与独立 EKF 相差超过 8 m 时会自动显示 `custom_ekf`。`estimator_fallback_count` 记录本次任务的切换次数，`estimator_disagreement_m` 给出两套估计的位置差。

## 动态目标与 COLREGs

专用检测器应向 `/autonomous_usv/dynamic_targets` 发布 `geometry_msgs/PoseArray`，`header.frame_id` 必须为 `han_usv_odom`，每个 pose 的 `position.x/y` 是目标的东/北坐标。控制器会进行轨迹关联和速度估计，只有满足轨迹年龄、净位移、连续同向运动、协方差、CPA 和 TCPA 门槛后才触发 `colregs_give_way`。

静态浮标场中 `colregs_active_samples` 必须为 0。不要打开 `colregs.use_pointcloud_tracks`，除非上游点云已经完成实例级船舶分类和稳定 ID 关联；原始浮标点云继续由滚动栅格和局部安全层处理。

## 3-DOF 辨识与 NMPC 门控

先在 `controller.yaml` 中把 `model_identification.enabled` 改为 `true`，运行包含直航、加减速和左右转向的任务。样本写入：

```text
han_usv_controller/model_data/usv_identification.csv
```

离线拟合：

```bash
cd /home/han/Ai_ws/Study/vrx_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run han_usv_controller fit_3dof_model \
  han_usv_controller/model_data/usv_identification.csv \
  --output han_usv_controller/model_data/fitted_model.json
```

只有样本数、推力激励范围、稳定阻尼和 surge/yaw 拟合 `R^2` 全部达标时，输出才会包含 `"nmpc_ready": true`。这只表示模型进入下一阶段求解器验证，不表示 NMPC 已接管；当前 `model_control_active_backend` 始终保持 `ilos_pid`。

推力默认限制在 1800，低于 H 型推进器约 2353 的上限。当前 Garden 仿真实测为 `forward_thrust_sign: 1.0`（正推力前进）；若以后更换了推进器插件或模型，应先做低推力极性测试再改这个参数。

## 自定义 GNSS 航点

在源参数文件的 `autonomous_usv.ros__parameters` 下加入：

```yaml
autonomous_usv:
  ros__parameters:
    task_mode: custom
    start_without_task: true
    custom_waypoints:
      - "-33.72267,150.67406,70"
      - "-33.72208,150.67379,57"
```

每项格式为 `纬度,经度,艏向角度`，艏向可省略。直接把源 YAML 传给 launch，不需要重新构建：

```bash
ros2 launch han_usv_controller simulation.launch.py \
  world:=sydney_regatta \
  config:=/home/han/Ai_ws/Study/vrx_ws/han_usv_controller/config/controller.yaml
```

## 本地验证

```bash
cd /home/han/Ai_ws/Study/vrx_ws
source /opt/ros/humble/setup.bash
PYTHONPATH=han_usv_controller:${PYTHONPATH:-} python3 -m pytest -q han_usv_controller/test
colcon test --merge-install --packages-select han_usv_controller
colcon test-result --verbose
```

完整三次自动评测只需一条命令。它会先重建 copy-install 包，确认没有另一套 ROS/Gazebo 实例，再为每次试验启动唯一的官方 Wayfinding 学习世界：

```bash
cd /home/han/Ai_ws/Study/vrx_ws
bash han_usv_controller/scripts/run_three_evaluations.sh
```

30 个可复现随机浮标布局使用：

```bash
cd /home/han/Ai_ws/Study/vrx_ws
BASE_SEED=1000 bash han_usv_controller/scripts/run_30_randomized_evaluations.sh
```

第 `N` 轮 seed 为 `BASE_SEED + N - 1`。生成器同时约束浮标两两间距不小于 7 m、官方航点净空不小于 10 m、WAM-V 出生点净空不小于 12 m，并限制在已验证的开阔水域课程边界内；这些数据和场景实体会写入每轮 `scenario.json`。汇总包含完成率、碰撞轮数/总数、最小净空、平均/最坏规划耗时、横向误差、峰值 yaw rate、原地转向总时长与最长连续时长。单轮约 4 到 7 分钟，30 轮通常需要数小时。

每次结果保存在 `han_usv_controller/evaluation/<UTC时间>/trial_XX/result.json`，原始进程输出分别在 `launch.log` 和 `monitor.log`；全部试验汇总写入 `aggregate.json`，最新一份“已完成汇总”的结果另写 `han_usv_controller/evaluation/latest.json`。验收要求每个请求的 trial 都完成、0 碰撞、官方平均误差不超过 2.5 m、任一航点最小误差不超过 3 m、最大角速度不超过 20 deg/s、双 EKF 状态与指标存在且健康、滚动地图已获得足够观测、State Lattice 确实进入控制链，以及 0 个持续 fallback 样本。碰撞数、峰值角速度或末态估计器字段缺失时会直接判失败，不再按 0 或健康处理。

评测器不会默认结束其它终端里的仿真。先手动 `Ctrl-C` 停止旧 Launch；只有确认所有匹配进程都应被结束时，才给底层命令显式追加 `--force-clean-stale`。`--no-clean-stale` 会跳过冲突预检，仅适合已经由外部进程隔离工具管理的环境。

单独给已运行仿真采集指标时仍可用：

```bash
ros2 run han_usv_controller regression_monitor --timeout 600
```

这里的 `600` 秒是自检进程的墙钟看门狗，不是 Gazebo 世界或 VRX 任务倒计时。学习世界仍没有约 320 秒的竞赛终止限制；把看门狗留得更宽，是因为开启雷达、地图和 RViz 后 Gazebo 的实时因子可能低于 1，仿真 300 秒可能需要更长的真实时间。

2026-07-17 逐步实测。目录中的结果是对应生成时源码的历史证据，不应把 `latest.json` 理解成任何后续改动都会自动重新验证：

- 普通学习世界最终验收：`3/3 complete`，墙钟 `264.6 s`，0 碰撞，官方平均误差 `1.210 m`，各航点最小误差 `0.248 / 1.510 / 1.872 m`，最大角速度 `12.88 deg/s`，0 个 State Lattice fallback。进入 16 m 终端区后一次性切换到直接 ILOS，使总原地转向时间从修改前一轮的 `41.44 s` 降到 `15.94 s`、最长连续转向从 `25.89 s` 降到 `9.12 s`。汇总见 `han_usv_controller/evaluation/20260717T020638Z/aggregate.json`。

- 本轮干净学习世界回归：任务进入 `running` 后约 `281.6 s` 完成 3 个航点，官方平均误差 `1.722 m`，各航点最小误差 `0.335 / 1.975 / 2.857 m`，0 碰撞；末态 `complete`、双推进器为 0。评测器加固没有改变控制律。

- EKF 基线：259.1 秒，`3/3 complete`，0 碰撞，官方平均误差 `1.441 m`，末态位置标准差 `0.435 m`。
- EKF + 滚动地图 + State Lattice：277.8 秒，`3/3 complete`，0 碰撞，官方平均误差 `1.587 m`，4 次分段重规划，最大角速度 `10.14 deg/s`。
- 最终连续三次自动评测：`3/3 PASS`，用时 `275.5 / 271.1 / 266.0 s`，官方平均误差 `1.540 / 1.576 / 1.575 m`，三轮均为 0 碰撞、4 次分段重规划、0 次 State Lattice 回退。汇总见 `han_usv_controller/evaluation/20260716T184749Z/aggregate.json`。
- 在线地图重规划收紧后的普通官方回归：`270.4 s`，平均误差 `1.594 m`，最差航点 `2.425 m`，0 碰撞、0 fallback、0 次额外地图障碍重规划。结果见 `han_usv_controller/evaluation/20260716T193937Z/aggregate.json`。
- 强制 A* 压力回归：`342.4 s`，平均误差 `1.432 m`，最差航点 `2.456 m`，0 碰撞，A* 最大扩展 143，1 次 `lattice_obstacle` 在线重规划，0 fallback。结果见 `han_usv_controller/evaluation/20260716T194435Z/aggregate.json`。
- 随机 seed `1000`：`384.9 s`，平均误差 `0.723 m`，0 碰撞，最小净空 `5.47 m`，A* 最大扩展 98，最坏规划 `12.51 ms`，0 fallback。结果见 `han_usv_controller/evaluation/20260716T201314Z/aggregate.json`。
- 最终艏向 governor 优化后复测相同随机 seed `1000`：`3/3 complete`，墙钟 `370.9 s`，0 碰撞，官方平均误差 `0.428 m`，各航点最小误差 `0.241 / 0.529 / 0.514 m`，对准阶段峰值角速度 `4.24 deg/s`，0 fallback、0 EKF 回退。第 2 点连续对准由现场旧版本的约 `117.6 s` 降到约 `45 s`，且不再反复退出捕获。结果见 `han_usv_controller/evaluation/20260717T025430Z/aggregate.json`，`latest.json` 已同步。
- 静态测试当前覆盖评测判定、双 EKF 接口、随机场景、滚动栅格、State Lattice、Dubins/ILOS、动态跟踪/COLREGs、3-DOF 拟合、NMPC 门控、避障与控制器状态机。

这里的秒数是本机墙钟时间；学习世界完成后 `task_state` 仍为 `running`，剩余时长约 100 个仿真年。Gazebo 在 Ctrl-C 后可能在 `libWaveVisual.so` 析构中报段错误，这是 VRX/Gazebo 的退出阶段问题，判断控制器成败应看段错误之前的 `complete` 与指标汇总。

修改 `core.py`、`node.py` 或 YAML 后必须重新运行 `colcon build` 并再次 `source install/setup.bash`，否则 launch 仍可能启动旧安装版。

Wayfinding 和 Stationkeeping 是运动控制任务。`navigation_task` 的红绿浮标识别、`wildlife_task` 的动物分类、扫描靠泊和声学任务还需要各自的视觉/声学任务层；本包为这些任务提供底层航行与避障能力，但不虚构尚未实现的识别器。
