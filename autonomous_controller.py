#!/usr/bin/env python3
"""
WAM-V 自主控制器 — Wayfinding 航路点导航 + 避障 + 视觉感知
============================================================
使用方法:
  1. 启动仿真:  ros2 launch vrx_gz competition.launch.py world:=wayfinding_task
  2. 运行本脚本: python3 autonomous_controller.py

功能模块:
  - PIDController: 标准 PID 控制器（带抗饱和）
  - GPSConverter:  WGS84 经纬度 → 本地 ENU 米制坐标
  - ObstacleAvoidance: 激光雷达虚拟力场避障（VFF）
  - VisualPerception: OpenCV HSV 颜色目标检测
  - AutonomousController: ROS 2 主节点，50Hz 控制循环

推力约定: 正值沿 base_link +X，负值沿 -X，最大推力 2353 N。
"""

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import NavSatFix, Imu, LaserScan, Image
from geometry_msgs.msg import PoseArray
from ros_gz_interfaces.msg import Float32Array, ParamVec

# ============================================================================
#  PID 控制器
# ============================================================================

class PIDController:
    """标准 PID 控制器，带积分抗饱和和输出限幅。"""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = float('inf'),
                 integral_limit: float = float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        # 比例项
        p = self.kp * error
        # 积分项（带限幅）
        self._integral += error * dt
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))
        i = self.ki * self._integral
        # 微分项
        if self._first:
            d = 0.0
            self._first = False
        else:
            d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error
        # 输出限幅
        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True


# ============================================================================
#  GPS 坐标转换
# ============================================================================

class GPSConverter:
    """WGS84 经纬度 ↔ 本地 ENU 坐标转换。"""

    # WGS84 椭球参数
    EARTH_RADIUS = 6371000.0  # 米

    def __init__(self):
        self.ref_lat = None
        self.ref_lon = None
        self._lat_scale = None
        self._lon_scale = None

    def set_reference(self, lat: float, lon: float):
        """设置参考原点（首次收到的目标点）。"""
        self.ref_lat = lat
        self.ref_lon = lon
        self._lat_scale = self.EARTH_RADIUS * math.radians(1.0)
        self._lon_scale = self.EARTH_RADIUS * math.cos(math.radians(lat)) * math.radians(1.0)

    def to_local(self, lat: float, lon: float):
        """经纬度 → 本地 (north, east) 米制坐标。"""
        if self.ref_lat is None:
            return None, None
        north = (lat - self.ref_lat) * self._lat_scale
        east = (lon - self.ref_lon) * self._lon_scale
        return north, east

    @staticmethod
    def quaternion_to_yaw(q) -> float:
        """四元数 → 偏航角（弧度），返回 [-π, π]。"""
        # yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


# ============================================================================
#  虚拟力场避障（VFF）
# ============================================================================

class ObstacleAvoidance:
    """
    基于前向激光雷达扇区的局部避障。

    输入和输出航向均为船体坐标系下的相对航向，避免将世界坐标
    的航向与船体坐标的雷达角度直接相加。
    """

    def __init__(self, danger_dist: float = 7.0, warning_dist: float = 16.0,
                 max_steer: float = math.radians(65.0)):
        self.danger_dist = danger_dist
        self.warning_dist = warning_dist
        self.max_steer = max_steer

    @staticmethod
    def _clearance(samples, default):
        """使用较小分位数，既能看到浮标，又不被单个噪点触发。"""
        if not samples:
            return default
        return float(np.percentile(samples, 10.0))

    def _risk(self, clearance: float) -> float:
        return clamp(
            (self.warning_dist - clearance) /
            (self.warning_dist - self.danger_dist),
            0.0, 1.0
        )

    def compute(self, ranges: list, angle_min: float, angle_increment: float,
                desired_heading_error: float, desired_speed: float):
        """
        返回 ``(调整后相对航向, 调整后推力, 正前方净空)``。
        """
        if not ranges:
            return desired_heading_error, desired_speed, float('inf')

        front = []
        port = []
        starboard = []
        for i, distance in enumerate(ranges):
            if not math.isfinite(distance) or distance <= 0.2:
                continue
            angle = angle_min + i * angle_increment
            if abs(angle) <= math.radians(32.0):
                front.append(distance)
            if math.radians(18.0) <= angle <= math.radians(100.0):
                port.append(distance)
            if math.radians(-100.0) <= angle <= math.radians(-18.0):
                starboard.append(distance)

        front_clearance = self._clearance(front, float('inf'))
        port_clearance = self._clearance(port, self.warning_dist * 2.0)
        starboard_clearance = self._clearance(
            starboard, self.warning_dist * 2.0)

        front_risk = self._risk(front_clearance)
        port_risk = self._risk(port_clearance)
        starboard_risk = self._risk(starboard_clearance)

        # 右侧障碍向左转（正），左侧障碍向右转（负）。
        side_bias = math.radians(30.0) * (starboard_risk - port_risk)
        front_bias = 0.0
        if front_risk > 0.0:
            if abs(desired_heading_error) > math.radians(12.0):
                turn_sign = 1.0 if desired_heading_error > 0.0 else -1.0
            else:
                turn_sign = 1.0 if port_clearance >= starboard_clearance else -1.0
            front_bias = turn_sign * self.max_steer * front_risk

        adjusted_error = clamp(
            desired_heading_error + side_bias + front_bias,
            -math.radians(95.0), math.radians(95.0)
        )

        # 进入危险区后停止前进，保留差速转向能力。
        speed_factor = 1.0 - 0.75 * front_risk
        if front_clearance < self.danger_dist:
            speed_factor = 0.0

        return adjusted_error, desired_speed * speed_factor, front_clearance


# ============================================================================
#  视觉感知（HSV 颜色检测）
# ============================================================================

class VisualPerception:
    """
    使用 OpenCV HSV 色彩空间检测目标颜色。
    可用于视觉伺服微调航向，或识别航标。
    """

    def __init__(self):
        self._cv2 = None
        self._bridge = None
        try:
            import cv2
            self._cv2 = cv2
            from cv_bridge import CvBridge
            self._bridge = CvBridge()
        except ImportError:
            pass  # 无 OpenCV 时视觉功能静默禁用

    @property
    def available(self) -> bool:
        return self._cv2 is not None and self._bridge is not None

    def detect_target(self, ros_image, color: str = 'red'):
        """
        检测指定颜色的目标。

        返回:
          (detected: bool, offset_x: float, offset_y: float)
          offset_x/y 归一化到 [-1, 1]，(0,0) = 图像中心
        """
        if not self.available:
            return False, 0.0, 0.0

        cv2 = self._cv2

        try:
            # ROS Image → OpenCV
            frame = self._bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        except Exception:
            return False, 0.0, 0.0

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 根据目标颜色设置 HSV 范围
        if color == 'red':
            # 红色在 HSV 中跨越 0°，需要两段
            mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
            mask2 = cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))
            mask = cv2.bitwise_or(mask1, mask2)
        elif color == 'green':
            mask = cv2.inRange(hsv, (35, 100, 100), (85, 255, 255))
        elif color == 'blue':
            mask = cv2.inRange(hsv, (100, 100, 100), (130, 255, 255))
        elif color == 'yellow':
            mask = cv2.inRange(hsv, (20, 100, 100), (35, 255, 255))
        else:
            mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))

        # 形态学操作：去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找最大轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, 0.0, 0.0

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        # 面积阈值：至少占图像 0.5%
        min_area = w * h * 0.005
        if area < min_area:
            return False, 0.0, 0.0

        # 计算质心
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return False, 0.0, 0.0

        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']

        # 归一化到 [-1, 1]
        offset_x = (cx - w / 2.0) / (w / 2.0)
        offset_y = (cy - h / 2.0) / (h / 2.0)

        return True, offset_x, offset_y


# ============================================================================
#  辅助函数
# ============================================================================

def normalize_angle(angle: float) -> float:
    """将角度归一化到 [-π, π]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


# ============================================================================
#  主控制器节点
# ============================================================================

class AutonomousController(Node):
    """
    WAM-V 自主控制器 ROS 2 节点。

    功能:
      - Wayfinding 多航点导航
      - 激光雷达 VFF 避障
      - 摄像头 HSV 颜色视觉感知
      - 双环 PID 控制（航向 + 速度）
      - 差速驱动推进器输出
    """

    # 推力参数
    MAX_THRUST = 2353.0   # H 型配置最大推力
    MAX_CONTROL_THRUST = 1200.0  # 控制器使用的保守上限

    # 航点参数
    WAYPOINT_CAPTURE_DIST = 2.5
    WAYPOINT_RELEASE_DIST = 5.0
    WAYPOINT_HEADING_TOL = math.radians(10.0)
    WAYPOINT_DWELL_TIME = 1.0

    # 状态枚举
    STATE_WAITING = 0
    STATE_REVERSING = 1   # 阶段1：倒退远离岸边
    STATE_TURNING = 2     # 阶段2：180°转向
    STATE_NAVIGATING = 3
    STATE_DONE = 4

    def __init__(self):
        super().__init__('autonomous_controller')

        # ---- 状态变量 ----
        self.state = self.STATE_WAITING
        self.task_state = 'unknown'
        self.current_lat = None
        self.current_lon = None
        self.current_yaw = None
        self.current_yaw_rate = 0.0
        self.waypoints = []  # [(lat, lon, yaw), ...]
        self.current_wp_idx = 0
        self.laser_ranges = []
        self.laser_angle_min = 0.0
        self.laser_angle_inc = 0.0
        self.latest_image = None
        self._last_time = time.monotonic()
        self._turn_settle_start = None
        self._aligning_waypoint = False
        self._waypoint_dwell_start = None
        self._init_wp_min_dist = float('inf')
        self._log_counter = 0
        self.mean_error = None
        self.min_errors = []

        # ---- 初始化阶段变量 ----
        self._init_start_lat = None
        self._init_start_lon = None
        self._init_start_yaw = None
        self._turn_target_yaw = None
        self.REVERSE_DIST = 20.0
        self.REVERSE_THRUST = 500.0

        # ---- 速度估计 ----
        self._prev_gps_lat = None
        self._prev_gps_lon = None
        self._prev_gps_time = None
        self._estimated_speed = 0.0  # m/s

        # ---- 子模块 ----
        self.gps_conv = GPSConverter()
        # 倒车保持航向使用的 PID。航点转向直接用 IMU 角速度做 PD。
        self.heading_pid = PIDController(
            kp=260.0, ki=0.0, kd=80.0,
            output_limit=250.0, integral_limit=0.0
        )
        self.avoidance = ObstacleAvoidance(
            danger_dist=7.0, warning_dist=16.0
        )
        self.vision = VisualPerception()

        # ---- QoS 配置 ----
        # 传感器数据用 BEST_EFFORT（兼容 Gazebo 桥接）
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # ---- 发布者（4 个） ----
        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.left_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/left/pos', 10)
        self.right_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/right/pos', 10)

        # ---- 订阅者（6 个） ----
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix',
                                 self._gps_callback, sensor_qos)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data',
                                 self._imu_callback, sensor_qos)
        self.create_subscription(LaserScan, '/wamv/sensors/lidars/lidar_wamv_sensor/scan',
                                 self._laser_callback, sensor_qos)
        self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera/image_raw',
                                 self._image_callback, sensor_qos)
        self.create_subscription(PoseArray, '/vrx/wayfinding/waypoints',
                                 self._waypoints_callback, 10)
        self.create_subscription(ParamVec, '/vrx/task/info',
                                 self._task_info_callback, 10)
        self.create_subscription(Float32, '/vrx/wayfinding/mean_error',
                                 self._mean_error_callback, 10)
        self.create_subscription(Float32Array, '/vrx/wayfinding/min_errors',
                                 self._min_errors_callback, 10)

        # ---- 控制循环定时器（50 Hz） ----
        self.create_timer(0.02, self._control_loop)

        # ---- 日志 ----
        self.get_logger().info('=== 自主控制器已启动 ===')
        self.get_logger().info(f'视觉感知: {"可用" if self.vision.available else "不可用（缺少 OpenCV/CvBridge）"}')
        self.get_logger().info('等待任务状态和航点数据...')

    # ========================================================================
    #  回调函数
    # ========================================================================

    def _gps_callback(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        # 首次收到 GPS 时，以当前位置为参考原点
        if self.gps_conv.ref_lat is None:
            self.gps_conv.set_reference(msg.latitude, msg.longitude)
            self.get_logger().info(f'GPS 参考原点: ({msg.latitude:.6f}, {msg.longitude:.6f})')

        # 速度估计
        now = time.monotonic()
        if self._prev_gps_lat is not None and self._prev_gps_time is not None:
            dt = now - self._prev_gps_time
            if dt > 0.01:
                dn, de = self.gps_conv.to_local(msg.latitude, msg.longitude)
                pn, pe = self.gps_conv.to_local(self._prev_gps_lat, self._prev_gps_lon)
                if dn is not None and pn is not None:
                    dist = math.sqrt((dn - pn)**2 + (de - pe)**2)
                    self._estimated_speed = 0.7 * self._estimated_speed + 0.3 * (dist / dt)  # 低通滤波
        self._prev_gps_lat = msg.latitude
        self._prev_gps_lon = msg.longitude
        self._prev_gps_time = now

    def _imu_callback(self, msg: Imu):
        self.current_yaw = GPSConverter.quaternion_to_yaw(msg.orientation)
        self.current_yaw_rate = msg.angular_velocity.z

    def _laser_callback(self, msg: LaserScan):
        self.laser_ranges = list(msg.ranges)
        self.laser_angle_min = msg.angle_min
        self.laser_angle_inc = msg.angle_increment

    def _image_callback(self, msg: Image):
        self.latest_image = msg

    def _waypoints_callback(self, msg: PoseArray):
        if not self.waypoints:
            # position.x/y = 纬经度，orientation = 评分使用的目标航向。
            for pose in msg.poses:
                lat = pose.position.x
                lon = pose.position.y
                yaw = GPSConverter.quaternion_to_yaw(pose.orientation)
                self.waypoints.append((lat, lon, yaw))
            self.get_logger().info(f'收到 {len(self.waypoints)} 个航点')
            for i, (lat, lon, yaw) in enumerate(self.waypoints):
                self.get_logger().info(
                    f'  航点 {i}: ({lat:.6f}, {lon:.6f}), '
                    f'目标航向={math.degrees(yaw):.1f}°')

    def _task_info_callback(self, msg: ParamVec):
        for param in msg.params:
            if param.name == 'state':
                new_state = param.value.string_value
                if new_state != self.task_state:
                    old = self.task_state
                    self.task_state = new_state
                    self.get_logger().info(f'任务状态: {old} → {new_state}')
                    if new_state == 'finished':
                        self._all_done('VRX 任务已结束')

    def _mean_error_callback(self, msg: Float32):
        self.mean_error = float(msg.data)

    def _min_errors_callback(self, msg: Float32Array):
        self.min_errors = [float(value) for value in msg.data]

    # ========================================================================
    #  控制主循环（50 Hz）
    # ========================================================================

    def _control_loop(self):
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now

        # 推进器角度始终朝前
        self._publish_float(self.left_pos_pub, 0.0)
        self._publish_float(self.right_pos_pub, 0.0)

        # 状态分发
        if self.state == self.STATE_WAITING:
            if (self.task_state == 'running' and self.current_lat is not None and
                    self.current_yaw is not None and self.waypoints):
                self._init_start_lat = self.current_lat
                self._init_start_lon = self.current_lon
                self._init_start_yaw = self.current_yaw
                self.state = self.STATE_REVERSING
                self.get_logger().info('>>> 开始初始化：倒退 20m 远离岸边 <<<')
            self._stop_thrusters()
            return

        if self.state == self.STATE_DONE:
            self._stop_thrusters()
            return

        # 检查数据就绪
        if self.current_lat is None or self.current_yaw is None:
            self._stop_thrusters()
            return

        # ---- 阶段1：倒退远离岸边 ----
        if self.state == self.STATE_REVERSING:
            self._do_reverse_phase(dt)
            return

        # ---- 阶段2：180°转向 ----
        if self.state == self.STATE_TURNING:
            self._do_turn_phase()
            return

        # ---- 阶段3：航点导航 ----
        if self.state == self.STATE_NAVIGATING:
            self._do_navigate_phase()
            return

    # ========================================================================
    #  初始化阶段方法
    # ========================================================================

    def _do_reverse_phase(self, dt: float):
        """阶段1：倒退远离岸边。正推力 = 后退。"""
        if self._init_start_lat is None:
            self._init_start_lat = self.current_lat
            self._init_start_lon = self.current_lon

        # 计算已倒退距离
        dn, de = self.gps_conv.to_local(self.current_lat, self.current_lon)
        sn, se = self.gps_conv.to_local(self._init_start_lat, self._init_start_lon)
        if dn is None or sn is None:
            self._stop_thrusters()
            return
        reversed_dist = math.sqrt((dn - sn) ** 2 + (de - se) ** 2)

        if reversed_dist >= self.REVERSE_DIST:
            # 倒退完成，进入转向阶段
            self._init_start_yaw = self.current_yaw
            # 离岸轨迹会穿过第一航点，不再掉头返回岸边重复访问。
            if (self.current_wp_idx == 0 and
                    self._init_wp_min_dist <= self.WAYPOINT_CAPTURE_DIST + 0.5):
                self.current_wp_idx = 1
                self.get_logger().info(
                    f'离岸阶段已经过航点 1，最小距离 '
                    f'{self._init_wp_min_dist:.2f}m')

            # 目标航向：对准当前尚未访问的航点。
            if self.current_wp_idx < len(self.waypoints):
                tgt_lat, tgt_lon, _ = self.waypoints[self.current_wp_idx]
                tgt_n, tgt_e = self.gps_conv.to_local(tgt_lat, tgt_lon)
                cur_n, cur_e = self.gps_conv.to_local(self.current_lat, self.current_lon)
                if tgt_n is not None and cur_n is not None:
                    self._turn_target_yaw = math.atan2(tgt_n - cur_n, tgt_e - cur_e)
                else:
                    self._turn_target_yaw = normalize_angle(self._init_start_yaw + math.pi)
            else:
                self._turn_target_yaw = normalize_angle(self._init_start_yaw + math.pi)

            self.heading_pid.reset()
            self._turn_settle_start = None
            self.state = self.STATE_TURNING
            self.get_logger().info(
                f'倒退 {reversed_dist:.1f}m 完成！开始转向（目标航向 {math.degrees(self._turn_target_yaw):.1f}°）')
            return

        # 倒退：正推力 = 后退，保持航向
        heading_error = (
            normalize_angle(self._init_start_yaw - self.current_yaw)
            if self._init_start_yaw is not None else 0.0
        )
        heading_cmd = self.heading_pid.compute(heading_error, dt)
        heading_cmd = clamp(heading_cmd, -200.0, 200.0)
        self._command_thrusters(self.REVERSE_THRUST, heading_cmd)

        if self.current_wp_idx < len(self.waypoints):
            wp_lat, wp_lon, _ = self.waypoints[self.current_wp_idx]
            wp_n, wp_e = self.gps_conv.to_local(wp_lat, wp_lon)
            if wp_n is not None:
                wp_dist = math.hypot(wp_n - dn, wp_e - de)
                self._init_wp_min_dist = min(self._init_wp_min_dist, wp_dist)

        # 日志
        self._log_counter += 1
        if self._log_counter % 50 == 0:
            self.get_logger().info(f'[倒退] 已倒退 {reversed_dist:.1f}/{self.REVERSE_DIST:.0f}m')

    def _do_turn_phase(self):
        """阶段2：用正反差速就地转向，并等待角速度稳定。"""
        heading_error = normalize_angle(self._turn_target_yaw - self.current_yaw)
        abs_err = abs(heading_error)

        if abs_err < math.radians(7.0) and abs(self.current_yaw_rate) < 0.12:
            if self._turn_settle_start is None:
                self._turn_settle_start = time.monotonic()
            elif time.monotonic() - self._turn_settle_start >= 0.6:
                self.heading_pid.reset()
                self.state = self.STATE_NAVIGATING
                self.get_logger().info(
                    f'转向完成！航向误差 {math.degrees(heading_error):.1f}° '
                    f'→ 开始航点导航')
                return
        else:
            self._turn_settle_start = None

        heading_cmd = self._yaw_effort(heading_error, 850.0)
        left, right = self._command_thrusters(0.0, heading_cmd)

        # 日志
        self._log_counter += 1
        if self._log_counter % 50 == 0:
            self.get_logger().info(
                f'[转向] 航向误差={math.degrees(heading_error):.1f}°  推力 L={left:.0f} R={right:.0f}')

    def _do_navigate_phase(self):
        """阶段3：航点跟踪、局部避障与目标姿态对齐。"""
        if not self.waypoints:
            self._stop_thrusters()
            return

        if self.current_wp_idx >= len(self.waypoints):
            self._all_done()
            return

        target_lat, target_lon, target_yaw = self.waypoints[self.current_wp_idx]

        if self.gps_conv.ref_lat is None:
            self._stop_thrusters()
            return

        cur_n, cur_e = self.gps_conv.to_local(self.current_lat, self.current_lon)
        tgt_n, tgt_e = self.gps_conv.to_local(target_lat, target_lon)

        if cur_n is None or tgt_n is None:
            self._stop_thrusters()
            return

        dn = tgt_n - cur_n
        de = tgt_e - cur_e
        distance = math.sqrt(dn * dn + de * de)
        bearing = math.atan2(dn, de)

        if not self._aligning_waypoint and distance <= self.WAYPOINT_CAPTURE_DIST:
            self._aligning_waypoint = True
            self._waypoint_dwell_start = None
            self.get_logger().info(
                f'航点 {self.current_wp_idx + 1} 距离 {distance:.1f}m，'
                f'开始对齐目标航向 {math.degrees(target_yaw):.1f}°')

        if self._aligning_waypoint:
            if distance > self.WAYPOINT_RELEASE_DIST:
                self._aligning_waypoint = False
                self._waypoint_dwell_start = None
                self.get_logger().warn('对齐时偏离航点，重新接近')
                return

            heading_error = normalize_angle(target_yaw - self.current_yaw)
            heading_cmd = self._yaw_effort(heading_error, 520.0)

            # 接近航点后用小幅反推消除前进惯性。
            brake_thrust = 0.0
            if self._estimated_speed > 0.45:
                brake_thrust = -clamp(
                    100.0 + self._estimated_speed * 90.0, 0.0, 320.0)
            left, right = self._command_thrusters(brake_thrust, heading_cmd)

            stable = (
                distance <= self.WAYPOINT_CAPTURE_DIST + 0.5 and
                abs(heading_error) <= self.WAYPOINT_HEADING_TOL and
                abs(self.current_yaw_rate) < 0.15 and
                self._estimated_speed < 0.8
            )
            if stable:
                if self._waypoint_dwell_start is None:
                    self._waypoint_dwell_start = time.monotonic()
                elif (time.monotonic() - self._waypoint_dwell_start >=
                      self.WAYPOINT_DWELL_TIME):
                    self._finish_waypoint()
                    return
            else:
                self._waypoint_dwell_start = None

            self._log_counter += 1
            if self._log_counter % 50 == 0:
                self.get_logger().info(
                    f'[WP{self.current_wp_idx + 1} 对齐] dist={distance:.1f}m '
                    f'herr={math.degrees(heading_error):.1f}° '
                    f'speed={self._estimated_speed:.2f}m/s '
                    f'L={left:.0f} R={right:.0f}')
            return

        heading_error = normalize_angle(bearing - self.current_yaw)
        abs_heading_error = abs(heading_error)

        # 距离越远推力越大；靠近航点保留足够推力，不再落入死区。
        forward_magnitude = clamp(170.0 + 18.0 * distance, 220.0, 900.0)
        if distance < 8.0:
            forward_magnitude = clamp(120.0 + 25.0 * distance, 170.0, 320.0)

        if abs_heading_error > math.radians(70.0):
            heading_speed_factor = 0.0
        elif abs_heading_error > math.radians(45.0):
            heading_speed_factor = 0.25
        elif abs_heading_error > math.radians(25.0):
            heading_speed_factor = 0.55
        else:
            heading_speed_factor = 1.0

        speed_cmd = forward_magnitude * heading_speed_factor
        adjusted_error, speed_cmd, front_clearance = self.avoidance.compute(
            self.laser_ranges,
            self.laser_angle_min,
            self.laser_angle_inc,
            heading_error,
            speed_cmd
        )

        heading_cmd = self._yaw_effort(adjusted_error, 700.0)
        left_thrust, right_thrust = self._command_thrusters(
            speed_cmd, heading_cmd)

        self._log_counter += 1
        if self._log_counter % 50 == 0:
            front_text = (
                f'{front_clearance:.1f}m'
                if math.isfinite(front_clearance) else 'clear'
            )
            score_text = (
                f'{self.mean_error:.2f}' if self.mean_error is not None else 'n/a'
            )
            self.get_logger().info(
                f'[WP{self.current_wp_idx + 1}] dist={distance:.1f}m '
                f'herr={math.degrees(heading_error):.1f}° '
                f'adj={math.degrees(adjusted_error):.1f}° front={front_text} '
                f'L={left_thrust:.0f} R={right_thrust:.0f} score={score_text}')

    # ========================================================================
    #  工具方法
    # ========================================================================

    def _yaw_effort(self, heading_error: float, limit: float) -> float:
        """航向 PD：误差产生转向推力，IMU 角速度抑制过冲。"""
        effort = 720.0 * heading_error - 280.0 * self.current_yaw_rate
        return clamp(effort, -limit, limit)

    def _command_thrusters(self, surge: float, yaw_effort: float):
        """
        将纵向和转向推力混合到左右推进器。

        本模型正推力沿 base_link +X，负推力沿 -X。允许一正一负是为了在
        大航向误差时转向，而不是带着船向岸边画大圈。
        """
        left = clamp(
            surge - yaw_effort,
            -self.MAX_CONTROL_THRUST,
            self.MAX_CONTROL_THRUST
        )
        right = clamp(
            surge + yaw_effort,
            -self.MAX_CONTROL_THRUST,
            self.MAX_CONTROL_THRUST
        )
        self._publish_float(self.left_thrust_pub, left)
        self._publish_float(self.right_thrust_pub, right)
        return left, right

    def _finish_waypoint(self):
        completed = self.current_wp_idx + 1
        self.current_wp_idx += 1
        self._aligning_waypoint = False
        self._waypoint_dwell_start = None
        self.heading_pid.reset()

        score_text = (
            f'{self.mean_error:.3f}' if self.mean_error is not None else 'n/a'
        )
        self.get_logger().info(
            f'航点 {completed} 已到达并对齐，当前平均误差={score_text}')

        if self.current_wp_idx >= len(self.waypoints):
            self._all_done('所有航点已到达')
        else:
            self.get_logger().info(
                f'前往航点 {self.current_wp_idx + 1}/{len(self.waypoints)}')

    @staticmethod
    def _publish_float(pub, value: float):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def _stop_thrusters(self):
        try:
            self._publish_float(self.left_thrust_pub, 0.0)
            self._publish_float(self.right_thrust_pub, 0.0)
        except Exception:
            pass  # shutdown 后 publisher 可能已失效

    def _all_done(self, reason: str = '导航任务完成'):
        if self.state == self.STATE_DONE:
            return
        self.state = self.STATE_DONE
        self._stop_thrusters()
        score_text = (
            f'{self.mean_error:.3f}' if self.mean_error is not None else 'n/a'
        )
        min_text = ', '.join(f'{value:.3f}' for value in self.min_errors)
        self.get_logger().info(
            f'=== {reason} | 平均误差={score_text} '
            f'| 各航点最小误差=[{min_text}] ===')


# ============================================================================
#  入口
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousController()
    try:
        while rclpy.ok() and node.state != node.STATE_DONE:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().warn(f'退出: {e}')
    finally:
        try:
            node._stop_thrusters()
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
