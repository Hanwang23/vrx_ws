"""ROS 2 adapter for the VRX autonomous controller."""

import json
import math
import signal
import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import Point, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from ros_gz_interfaces.msg import ParamVec
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, NavSatStatus, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker, MarkerArray

from .core import (
    EARTH_RADIUS_M,
    ControlConfig,
    ControllerCore,
    GeoTarget,
    GroundSpeedEstimator,
    PIDController,
    VesselState,
    distance_and_bearing,
    enu_to_geodetic,
    extract_obstacle_points,
    geodetic_delta_m,
    nearest_neighbor_order,
    normalize_angle,
    validate_control_config,
    validated_quaternion_yaw,
)
from .colregs import (
    DynamicTargetTracker,
    assess_encounter,
    is_confirmed_moving,
    select_most_urgent,
)
from .debug_visualization import (
    circle_points,
    enu_history_to_body,
    enu_offset_to_body,
    filter_buoy_candidates,
    freshness_state,
    stale_buoy_marker_ids,
    tracking_quality,
    tracking_statistics,
    waypoint_visual_state,
)
from .dynamic_map import mask_dynamic_scan_ranges
from .estimator import EstimatorConfig, PlanarEKF
from .occupancy_grid import (
    OccupancyGridConfig,
    RollingOccupancyGrid,
    enu_grid_origin_in_body,
)
from .model_control import gate_model_control
from .system_identification import IdentificationLogger, IdentificationSample


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


class AutonomousUSVNode(Node):
    """Sensor watchdog, task adapter, and actuator I/O for ControllerCore."""

    def __init__(self) -> None:
        super().__init__('autonomous_usv')
        self._declare_parameters()

        self.task_mode = str(self.get_parameter('task_mode').value).lower()
        self.start_without_task = bool(self.get_parameter('start_without_task').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout').value)
        self.scan_cache_timeout = self._float_parameter(
            'avoidance.scan_cache_timeout')
        self.cloud_track_timeout = self._float_parameter(
            'avoidance.cloud_track_timeout')
        self.require_lidar = bool(self.get_parameter('avoidance.require_lidar').value)
        self.cloud_min_height = self._float_parameter(
            'avoidance.cloud_min_height')
        self.cloud_min_above_water = self._float_parameter(
            'avoidance.cloud_min_above_water')
        self.cloud_cluster_cell_size = self._float_parameter(
            'avoidance.cloud_cluster_cell_size')
        self.cloud_cluster_min_points = int(
            self.get_parameter('avoidance.cloud_cluster_min_points').value)
        self.cloud_track_min_hits = int(
            self.get_parameter('avoidance.cloud_track_min_hits').value)
        self.cloud_track_match_distance = self._float_parameter(
            'avoidance.cloud_track_match_distance')
        self.cloud_pose_tolerance = self._float_parameter(
            'avoidance.cloud_pose_tolerance')
        self.cloud_point_stride = max(
            1, int(self.get_parameter('avoidance.cloud_point_stride').value))
        self.cloud_sensor_offset_x = self._float_parameter(
            'avoidance.cloud_sensor_offset_x')
        self.cloud_sensor_offset_y = self._float_parameter(
            'avoidance.cloud_sensor_offset_y')
        self.stop_on_control_conflict = bool(
            self.get_parameter('stop_on_control_conflict').value)
        self.publish_debug_markers = bool(
            self.get_parameter('debug.publish_markers').value)
        self.debug_frame = str(self.get_parameter('debug.frame').value)
        self.debug_publish_period = 1.0 / max(
            1.0, self._float_parameter('debug.publish_rate_hz'))
        self.last_debug_publish_time = 0.0
        self.control_conflict = False
        self.task_name = 'unknown'
        self.task_state = 'unknown'
        self.num_collisions = 0
        self.active_mode = self.task_mode if self.task_mode != 'auto' else 'unknown'

        config = self._control_config()
        validate_control_config(config)
        heading_pid = PIDController(
            self._float_parameter('heading_pid.kp'),
            self._float_parameter('heading_pid.ki'),
            self._float_parameter('heading_pid.kd'),
            config.max_turn_thrust,
            self._float_parameter('heading_pid.integral_limit'),
        )
        speed_pid = PIDController(
            self._float_parameter('speed_pid.kp'),
            self._float_parameter('speed_pid.ki'),
            self._float_parameter('speed_pid.kd'),
            config.max_surge_thrust,
            self._float_parameter('speed_pid.integral_limit'),
        )
        self.controller = ControllerCore(config, heading_pid, speed_pid)
        self.speed_estimator = GroundSpeedEstimator()
        self.ekf_enabled = bool(self.get_parameter('ekf.enabled').value)
        self.estimator = PlanarEKF(EstimatorConfig(
            position_process_noise=self._float_parameter(
                'ekf.position_process_noise'),
            velocity_process_noise=self._float_parameter(
                'ekf.velocity_process_noise'),
            yaw_process_noise=math.radians(self._float_parameter(
                'ekf.yaw_process_noise_deg')),
            yaw_rate_process_noise=math.radians(self._float_parameter(
                'ekf.yaw_rate_process_noise_deg_s')),
            default_gps_std=self._float_parameter('ekf.default_gps_std'),
            default_yaw_std=math.radians(self._float_parameter(
                'ekf.default_yaw_std_deg')),
            default_yaw_rate_std=math.radians(self._float_parameter(
                'ekf.default_yaw_rate_std_deg_s')),
            velocity_measurement_std=self._float_parameter(
                'ekf.velocity_measurement_std'),
            innovation_gate_sigma=self._float_parameter(
                'ekf.innovation_gate_sigma'),
            max_speed=self._float_parameter('ekf.max_speed'),
            max_position_std=self._float_parameter('ekf.max_position_std'),
        ))
        self.estimator_healthy = not self.ekf_enabled
        self.estimator_position_std: Optional[float] = None
        self.estimator_velocity_std: Optional[float] = None
        self.estimator_yaw_std: Optional[float] = None
        self.robot_localization_enabled = bool(
            self.get_parameter('robot_localization.enabled').value)
        self.robot_localization_timeout = self._float_parameter(
            'robot_localization.timeout')
        self.robot_localization_max_position_std = self._float_parameter(
            'robot_localization.max_position_std')
        self.robot_localization_max_yaw_std = math.radians(
            self._float_parameter('robot_localization.max_yaw_std_deg'))
        self.robot_localization_max_disagreement = self._float_parameter(
            'robot_localization.max_disagreement_m')
        self.robot_localization_origin: Optional[Tuple[float, float]] = None
        self.robot_localization_state = None
        self.robot_localization_last_time: Optional[float] = None
        self.robot_localization_healthy = False
        self.estimator_source = 'custom_ekf'
        self.estimator_disagreement_m: Optional[float] = None
        self.estimator_fallback_count = 0
        self.navigation_east: Optional[float] = None
        self.navigation_north: Optional[float] = None
        self.map_enabled = bool(self.get_parameter('map.enabled').value)
        self.occupancy_grid = RollingOccupancyGrid(OccupancyGridConfig(
            width_m=self._float_parameter('map.width_m'),
            height_m=self._float_parameter('map.height_m'),
            resolution=self._float_parameter('map.resolution'),
            max_range=self._float_parameter('map.max_range'),
            hit_log_odds=self._float_parameter('map.hit_log_odds'),
            miss_log_odds=self._float_parameter('map.miss_log_odds'),
            decay_rate=self._float_parameter('map.decay_rate'),
            stale_after=self._float_parameter('map.stale_after'),
            occupied_probability=self._float_parameter(
                'map.occupied_probability'),
            ray_stride=int(self.get_parameter('map.ray_stride').value),
        ))
        self.map_frame = str(self.get_parameter('map.frame').value)
        self.map_publish_period = 1.0 / max(
            0.1, self._float_parameter('map.publish_rate_hz'))
        self.last_map_publish_time = 0.0
        self.map_known_cells = 0
        self.map_occupied_cells = 0
        self.map_inflation_radius = self._float_parameter(
            'map.inflation_radius')
        self.map_cloud_track_min_hits = int(
            self.get_parameter('map.cloud_track_min_hits').value)
        self.latest_occupancy_snapshot = None
        self.colregs_enabled = bool(
            self.get_parameter('colregs.enabled').value)
        self.colregs_use_pointcloud_tracks = bool(
            self.get_parameter('colregs.use_pointcloud_tracks').value)
        self.colregs_target_frame = str(
            self.get_parameter('colregs.target_frame').value)
        self.colregs_min_hits = int(
            self.get_parameter('colregs.minimum_hits').value)
        self.colregs_minimum_speed = self._float_parameter(
            'colregs.minimum_target_speed')
        self.colregs_safety_radius = self._float_parameter(
            'colregs.safety_radius')
        self.colregs_time_horizon = self._float_parameter(
            'colregs.time_horizon')
        self.colregs_map_mask_radius = self._float_parameter(
            'colregs.map_mask_radius')
        self.dynamic_tracker = DynamicTargetTracker(
            match_distance=self._float_parameter('colregs.match_distance'),
            timeout=self._float_parameter('colregs.track_timeout'),
            position_gain=self._float_parameter('colregs.position_gain'),
            velocity_gain=self._float_parameter('colregs.velocity_gain'),
            max_speed=self._float_parameter('colregs.max_target_speed'),
        )
        self.current_encounter = None
        self.colregs_risk_count = 0
        self.last_dynamic_target_time: Optional[float] = None
        self.dynamic_masked_scan_beams = 0
        self.dynamic_masked_cloud_tracks = 0
        self.identification_enabled = bool(
            self.get_parameter('model_identification.enabled').value)
        self.identification_logger = (
            IdentificationLogger(
                str(self.get_parameter('model_identification.log_path').value),
                1.0 / max(
                    0.1,
                    self._float_parameter('model_identification.log_rate_hz')),
            )
            if self.identification_enabled else None
        )
        fitted_model = None
        fitted_model_path = str(
            self.get_parameter('model_control.fitted_model_path').value)
        if fitted_model_path:
            try:
                with open(fitted_model_path, encoding='utf-8') as stream:
                    fitted_model = json.load(stream)
            except (OSError, ValueError, json.JSONDecodeError) as error:
                self.get_logger().warning(
                    f'Could not load fitted model {fitted_model_path!r}: {error}')
        self.model_control_status = gate_model_control(
            str(self.get_parameter('model_control.requested_backend').value),
            fitted_model,
        )

        self.latitude: Optional[float] = None
        self.longitude: Optional[float] = None
        self.yaw: Optional[float] = None
        self.yaw_rate = 0.0
        self.yaw_rate_valid = False
        self.last_imu_sample_time: Optional[float] = None
        self.last_imu_yaw: Optional[float] = None
        self.speed = 0.0
        self.laser_ranges: Tuple[float, ...] = ()
        self.laser_angle_min = 0.0
        self.laser_angle_increment = 0.0
        self.laser_range_min = 0.1
        self.obstacle_points: Tuple[Tuple[float, float], ...] = ()
        self.cloud_tracks: List[Tuple[float, float, float, int]] = []
        self.buoy_candidate_count = 0
        self.previous_buoy_candidate_marker_count = 0
        self.pose_history: List[Tuple[float, float, float, float]] = []
        self.trajectory_history: List[Tuple[float, float]] = []
        self.tracking_error_history: List[Tuple[float, float, bool]] = []
        self.debug_path_revision = -1
        self.last_gps_time: Optional[float] = None
        self.last_imu_time: Optional[float] = None
        self.last_scan_time: Optional[float] = None
        self.last_cloud_time: Optional[float] = None
        self.last_control_time: Optional[float] = None
        self.last_log_time = 0.0
        self.last_conflict_check = 0.0
        self.waypoint_signature: Optional[Tuple[Tuple[float, ...], ...]] = None
        self.stationkeeping_signature: Optional[Tuple[float, float, float]] = None

        self.left_topic = str(self.get_parameter('topics.left_thrust').value)
        self.right_topic = str(self.get_parameter('topics.right_thrust').value)
        self.left_pos_topic = str(self.get_parameter('topics.left_position').value)
        self.right_pos_topic = str(self.get_parameter('topics.right_position').value)
        self.left_pub = self.create_publisher(Float64, self.left_topic, 10)
        self.right_pub = self.create_publisher(Float64, self.right_topic, 10)
        self.left_pos_pub = self.create_publisher(Float64, self.left_pos_topic, 10)
        self.right_pos_pub = self.create_publisher(Float64, self.right_pos_topic, 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.debug_marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('topics.debug_markers').value),
            10,
        )
        self.buoy_marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('topics.buoy_candidates').value),
            10,
        )
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter('topics.rolling_grid').value),
            2,
        )

        self.create_subscription(
            NavSatFix,
            str(self.get_parameter('topics.gps').value),
            self._gps_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter('topics.imu').value),
            self._imu_callback,
            qos_profile_sensor_data,
        )
        origin_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            NavSatFix,
            str(self.get_parameter('topics.gps_origin').value),
            self._gps_origin_callback,
            origin_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('topics.filtered_odometry').value),
            self._filtered_odometry_callback,
            20,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('topics.lidar').value),
            self._lidar_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('topics.lidar_points').value),
            self._point_cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter('topics.waypoints').value),
            self._waypoints_callback,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter('topics.dynamic_targets').value),
            self._dynamic_targets_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('topics.stationkeeping_goal').value),
            self._stationkeeping_callback,
            10,
        )
        self.create_subscription(
            ParamVec,
            str(self.get_parameter('topics.task_info').value),
            self._task_callback,
            10,
        )

        self._load_custom_waypoints()
        rate = max(2.0, self._float_parameter('control_rate_hz'))
        self.control_period = 1.0 / rate
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            self.control_period,
            self._control_loop,
            clock=self.steady_clock,
        )
        self.get_logger().info(
            'Autonomous USV ready: waiting for GNSS, IMU, lidar, and task target')
        self.get_logger().warning(
            'Keep exactly one publisher on each thruster command topic')

    def _declare_parameters(self) -> None:
        defaults = {
            'task_mode': 'auto',
            'start_without_task': False,
            'control_rate_hz': 20.0,
            'sensor_timeout': 2.5,
            'stop_on_control_conflict': True,
            # A non-empty default lets Humble infer STRING_ARRAY type.
            'custom_waypoints': [''],
            'topics.gps': '/wamv/sensors/gps/gps/fix',
            'topics.imu': '/wamv/sensors/imu/imu/data',
            'topics.lidar': '/wamv/sensors/lidars/lidar_wamv_sensor/scan',
            'topics.lidar_points': '/wamv/sensors/lidars/lidar_wamv_sensor/points',
            'topics.waypoints': '/vrx/wayfinding/waypoints',
            'topics.stationkeeping_goal': '/vrx/stationkeeping/goal',
            'topics.task_info': '/vrx/task/info',
            'topics.left_thrust': '/wamv/thrusters/left/thrust',
            'topics.right_thrust': '/wamv/thrusters/right/thrust',
            'topics.left_position': '/wamv/thrusters/left/pos',
            'topics.right_position': '/wamv/thrusters/right/pos',
            'topics.debug_markers': '/autonomous_usv/debug_markers',
            'topics.buoy_candidates': '/autonomous_usv/buoy_candidates',
            'topics.rolling_grid': '/autonomous_usv/rolling_grid',
            'topics.filtered_odometry': '/odometry/filtered',
            'topics.gps_origin': '/autonomous_usv/gps_origin',
            'topics.dynamic_targets': '/autonomous_usv/dynamic_targets',
            'ekf.enabled': True,
            'ekf.position_process_noise': 0.15,
            'ekf.velocity_process_noise': 0.8,
            'ekf.yaw_process_noise_deg': 2.0,
            'ekf.yaw_rate_process_noise_deg_s': 8.0,
            'ekf.default_gps_std': 1.5,
            'ekf.default_yaw_std_deg': 3.0,
            'ekf.default_yaw_rate_std_deg_s': 2.0,
            'ekf.velocity_measurement_std': 0.8,
            'ekf.innovation_gate_sigma': 6.0,
            'ekf.max_speed': 15.0,
            'ekf.max_position_std': 8.0,
            'robot_localization.enabled': True,
            'robot_localization.timeout': 1.0,
            'robot_localization.max_position_std': 8.0,
            'robot_localization.max_yaw_std_deg': 15.0,
            'robot_localization.max_disagreement_m': 8.0,
            'map.enabled': True,
            'map.frame': 'wamv/wamv/base_link',
            'map.width_m': 100.0,
            'map.height_m': 100.0,
            'map.resolution': 0.5,
            'map.max_range': 40.0,
            'map.hit_log_odds': 0.85,
            'map.miss_log_odds': -0.40,
            'map.decay_rate': 0.08,
            'map.stale_after': 8.0,
            'map.occupied_probability': 0.65,
            'map.inflation_radius': 3.0,
            'map.cloud_track_min_hits': 5,
            'map.ray_stride': 2,
            'map.publish_rate_hz': 2.0,
            'colregs.enabled': True,
            'colregs.use_pointcloud_tracks': False,
            'colregs.target_frame': 'han_usv_odom',
            'colregs.match_distance': 4.0,
            'colregs.track_timeout': 4.0,
            'colregs.position_gain': 0.65,
            'colregs.velocity_gain': 0.35,
            'colregs.minimum_hits': 8,
            'colregs.minimum_target_speed': 0.5,
            'colregs.max_target_speed': 15.0,
            'colregs.safety_radius': 15.0,
            'colregs.time_horizon': 120.0,
            'colregs.map_mask_radius': 4.0,
            'model_identification.enabled': False,
            'model_identification.log_path': (
                'han_usv_controller/model_data/usv_identification.csv'),
            'model_identification.log_rate_hz': 10.0,
            'model_control.requested_backend': 'ilos_pid',
            'model_control.fitted_model_path': '',
            'limits.max_thrust': 1800.0,
            'limits.max_surge_thrust': 500.0,
            'limits.max_reverse_thrust': 500.0,
            'limits.max_turn_thrust': 160.0,
            'limits.max_low_speed_turn_thrust': 80.0,
            'limits.turn_full_gain_speed': 1.2,
            'limits.max_alignment_thrust': 60.0,
            'limits.max_alignment_brake_thrust': 90.0,
            'limits.alignment_heading_rate_gain': 0.35,
            'limits.alignment_yaw_rate_gain': 350.0,
            'limits.max_alignment_yaw_rate_deg_s': 5.0,
            'limits.max_alignment_yaw_acceleration_deg_s2': 2.5,
            'limits.navigation_heading_rate_gain': 0.55,
            'limits.navigation_yaw_rate_gain': 700.0,
            'limits.max_navigation_yaw_rate_deg_s': 12.0,
            'limits.max_navigation_yaw_acceleration_deg_s2': 5.0,
            'limits.yaw_rate_slowdown_start_deg_s': 8.0,
            'limits.yaw_rate_slowdown_stop_deg_s': 20.0,
            'forward_thrust_sign': 1.0,
            'navigation.cruise_speed': 1.6,
            'navigation.minimum_approach_speed': 0.35,
            'navigation.approach_gain': 0.14,
            'navigation.position_tolerance': 2.0,
            'navigation.waypoint_exit_tolerance': 4.0,
            'navigation.station_exit_tolerance': 4.0,
            'navigation.heading_tolerance_deg': 8.0,
            'navigation.yaw_rate_tolerance_deg_s': 3.0,
            'navigation.heading_exit_tolerance_deg': 12.0,
            'navigation.yaw_rate_exit_tolerance_deg_s': 8.0,
            'navigation.waypoint_dwell_time': 1.0,
            'navigation.speed_feedforward': 260.0,
            'navigation.max_normal_brake_thrust': 240.0,
            'navigation.speed_brake_deadband': 0.20,
            'navigation.normal_brake_distance': 22.0,
            'navigation.nearest_neighbor_order': True,
            'guidance.enabled': True,
            'guidance.dubins_turn_radius': 8.0,
            'guidance.dubins_sample_step': 0.5,
            'guidance.dubins_allow_three_turn_paths': False,
            'guidance.ilos_lookahead': 8.0,
            'guidance.ilos_integral_gain': 0.015,
            'guidance.ilos_integral_limit': 3.0,
            'guidance.ilos_correction_limit_deg': 45.0,
            'guidance.ilos_integral_min_speed': 0.4,
            'guidance.curvature_feedforward_gain': 1.0,
            'guidance.max_lateral_acceleration': 0.12,
            'guidance.replan_path_deviation': 8.0,
            'guidance.replan_cooldown': 5.0,
            'guidance.lattice_enabled': True,
            'guidance.lattice_heading_bins': 16,
            'guidance.lattice_planning_horizon': 40.0,
            'guidance.lattice_analytic_expansion_distance': 12.0,
            'guidance.lattice_max_expansions': 2500,
            'guidance.lattice_turn_penalty': 0.05,
            'guidance.lattice_replan_distance': 6.0,
            'guidance.lattice_blocked_path_confirmations': 2,
            'guidance.lattice_path_check_stride': 4,
            'guidance.lattice_start_clearance_radius': 4.0,
            'avoidance.enabled': True,
            'avoidance.require_lidar': True,
            'avoidance.warning_distance': 22.0,
            'avoidance.emergency_distance': 5.5,
            'avoidance.front_angle_deg': 65.0,
            'avoidance.planning_angle_deg': 100.0,
            'avoidance.safety_radius': 3.0,
            'avoidance.path_half_width': 2.4,
            'avoidance.bin_size_deg': 3.0,
            'avoidance.direction_hysteresis_deg': 12.0,
            'avoidance.cluster_range_tolerance': 0.8,
            'avoidance.brake_time_horizon': 2.5,
            'avoidance.emergency_time_horizon': 0.5,
            'avoidance.caution_speed': 0.55,
            'avoidance.brake_gain': 450.0,
            'avoidance.clear_hold_time': 0.5,
            'avoidance.stuck_timeout': 5.0,
            'avoidance.backup_duration': 2.0,
            'avoidance.backup_thrust': 350.0,
            'avoidance.terminal_recovery_disable_radius': 8.0,
            'avoidance.scan_cache_timeout': 0.5,
            'avoidance.cloud_track_timeout': 2.5,
            # Heights are relative to the lidar; the fitted water plane is primary.
            'avoidance.cloud_min_height': -1.75,
            'avoidance.cloud_min_above_water': 0.10,
            'avoidance.cloud_cluster_cell_size': 0.45,
            'avoidance.cloud_cluster_min_points': 3,
            'avoidance.cloud_track_min_hits': 2,
            'avoidance.cloud_track_match_distance': 0.8,
            'avoidance.cloud_pose_tolerance': 0.5,
            'avoidance.cloud_point_stride': 4,
            # Lidar origin relative to the GNSS antenna in body coordinates.
            'avoidance.cloud_sensor_offset_x': 1.55,
            'avoidance.cloud_sensor_offset_y': 0.0,
            'heading_pid.kp': 600.0,
            'heading_pid.ki': 18.0,
            'heading_pid.kd': 120.0,
            'heading_pid.integral_limit': math.radians(60.0),
            'speed_pid.kp': 260.0,
            'speed_pid.ki': 35.0,
            'speed_pid.kd': 20.0,
            'speed_pid.integral_limit': 8.0,
            'debug.publish_markers': True,
            'debug.frame': 'wamv/wamv/base_link',
            'debug.publish_rate_hz': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _control_config(self) -> ControlConfig:
        return ControlConfig(
            max_thrust=self._float_parameter('limits.max_thrust'),
            max_surge_thrust=self._float_parameter('limits.max_surge_thrust'),
            max_reverse_thrust=self._float_parameter(
                'limits.max_reverse_thrust'),
            max_turn_thrust=self._float_parameter('limits.max_turn_thrust'),
            max_low_speed_turn_thrust=self._float_parameter(
                'limits.max_low_speed_turn_thrust'),
            turn_full_gain_speed=self._float_parameter(
                'limits.turn_full_gain_speed'),
            max_alignment_thrust=self._float_parameter(
                'limits.max_alignment_thrust'),
            max_alignment_brake_thrust=self._float_parameter(
                'limits.max_alignment_brake_thrust'),
            alignment_heading_rate_gain=self._float_parameter(
                'limits.alignment_heading_rate_gain'),
            alignment_yaw_rate_gain=self._float_parameter(
                'limits.alignment_yaw_rate_gain'),
            max_alignment_yaw_rate=math.radians(self._float_parameter(
                'limits.max_alignment_yaw_rate_deg_s')),
            max_alignment_yaw_acceleration=math.radians(
                self._float_parameter(
                    'limits.max_alignment_yaw_acceleration_deg_s2')),
            navigation_heading_rate_gain=self._float_parameter(
                'limits.navigation_heading_rate_gain'),
            navigation_yaw_rate_gain=self._float_parameter(
                'limits.navigation_yaw_rate_gain'),
            max_navigation_yaw_rate=math.radians(self._float_parameter(
                'limits.max_navigation_yaw_rate_deg_s')),
            max_navigation_yaw_acceleration=math.radians(
                self._float_parameter(
                    'limits.max_navigation_yaw_acceleration_deg_s2')),
            yaw_rate_slowdown_start=math.radians(self._float_parameter(
                'limits.yaw_rate_slowdown_start_deg_s')),
            yaw_rate_slowdown_stop=math.radians(self._float_parameter(
                'limits.yaw_rate_slowdown_stop_deg_s')),
            forward_thrust_sign=self._float_parameter('forward_thrust_sign'),
            cruise_speed=self._float_parameter('navigation.cruise_speed'),
            minimum_approach_speed=self._float_parameter(
                'navigation.minimum_approach_speed'),
            approach_gain=self._float_parameter('navigation.approach_gain'),
            position_tolerance=self._float_parameter(
                'navigation.position_tolerance'),
            waypoint_exit_tolerance=self._float_parameter(
                'navigation.waypoint_exit_tolerance'),
            station_exit_tolerance=self._float_parameter(
                'navigation.station_exit_tolerance'),
            heading_tolerance=math.radians(
                self._float_parameter('navigation.heading_tolerance_deg')),
            yaw_rate_tolerance=math.radians(
                self._float_parameter(
                    'navigation.yaw_rate_tolerance_deg_s')),
            heading_exit_tolerance=math.radians(self._float_parameter(
                'navigation.heading_exit_tolerance_deg')),
            yaw_rate_exit_tolerance=math.radians(self._float_parameter(
                'navigation.yaw_rate_exit_tolerance_deg_s')),
            waypoint_dwell_time=self._float_parameter(
                'navigation.waypoint_dwell_time'),
            speed_feedforward=self._float_parameter(
                'navigation.speed_feedforward'),
            max_normal_brake_thrust=self._float_parameter(
                'navigation.max_normal_brake_thrust'),
            speed_brake_deadband=self._float_parameter(
                'navigation.speed_brake_deadband'),
            normal_brake_distance=self._float_parameter(
                'navigation.normal_brake_distance'),
            guidance_enabled=bool(
                self.get_parameter('guidance.enabled').value),
            dubins_turn_radius=self._float_parameter(
                'guidance.dubins_turn_radius'),
            dubins_sample_step=self._float_parameter(
                'guidance.dubins_sample_step'),
            dubins_allow_three_turn_paths=bool(self.get_parameter(
                'guidance.dubins_allow_three_turn_paths').value),
            ilos_lookahead=self._float_parameter(
                'guidance.ilos_lookahead'),
            ilos_integral_gain=self._float_parameter(
                'guidance.ilos_integral_gain'),
            ilos_integral_limit=self._float_parameter(
                'guidance.ilos_integral_limit'),
            ilos_correction_limit=math.radians(self._float_parameter(
                'guidance.ilos_correction_limit_deg')),
            ilos_integral_min_speed=self._float_parameter(
                'guidance.ilos_integral_min_speed'),
            curvature_feedforward_gain=self._float_parameter(
                'guidance.curvature_feedforward_gain'),
            max_lateral_acceleration=self._float_parameter(
                'guidance.max_lateral_acceleration'),
            guidance_replan_path_deviation=self._float_parameter(
                'guidance.replan_path_deviation'),
            guidance_replan_cooldown=self._float_parameter(
                'guidance.replan_cooldown'),
            lattice_enabled=bool(
                self.get_parameter('guidance.lattice_enabled').value),
            lattice_heading_bins=int(
                self.get_parameter('guidance.lattice_heading_bins').value),
            lattice_planning_horizon=self._float_parameter(
                'guidance.lattice_planning_horizon'),
            lattice_analytic_expansion_distance=self._float_parameter(
                'guidance.lattice_analytic_expansion_distance'),
            lattice_max_expansions=int(
                self.get_parameter('guidance.lattice_max_expansions').value),
            lattice_turn_penalty=self._float_parameter(
                'guidance.lattice_turn_penalty'),
            lattice_replan_distance=self._float_parameter(
                'guidance.lattice_replan_distance'),
            lattice_blocked_path_confirmations=int(self.get_parameter(
                'guidance.lattice_blocked_path_confirmations').value),
            lattice_path_check_stride=int(self.get_parameter(
                'guidance.lattice_path_check_stride').value),
            lattice_start_clearance_radius=self._float_parameter(
                'guidance.lattice_start_clearance_radius'),
            obstacle_warning_distance=self._float_parameter(
                'avoidance.warning_distance'),
            obstacle_emergency_distance=self._float_parameter(
                'avoidance.emergency_distance'),
            obstacle_front_angle=math.radians(
                self._float_parameter('avoidance.front_angle_deg')),
            obstacle_planning_angle=math.radians(
                self._float_parameter('avoidance.planning_angle_deg')),
            obstacle_safety_radius=self._float_parameter(
                'avoidance.safety_radius'),
            obstacle_path_half_width=self._float_parameter(
                'avoidance.path_half_width'),
            obstacle_bin_size=math.radians(
                self._float_parameter('avoidance.bin_size_deg')),
            obstacle_direction_hysteresis=math.radians(
                self._float_parameter('avoidance.direction_hysteresis_deg')),
            obstacle_cluster_range_tolerance=self._float_parameter(
                'avoidance.cluster_range_tolerance'),
            obstacle_brake_time_horizon=self._float_parameter(
                'avoidance.brake_time_horizon'),
            obstacle_emergency_time_horizon=self._float_parameter(
                'avoidance.emergency_time_horizon'),
            obstacle_caution_speed=self._float_parameter(
                'avoidance.caution_speed'),
            obstacle_brake_gain=self._float_parameter(
                'avoidance.brake_gain'),
            obstacle_clear_hold_time=self._float_parameter(
                'avoidance.clear_hold_time'),
            obstacle_stuck_timeout=self._float_parameter(
                'avoidance.stuck_timeout'),
            obstacle_backup_duration=self._float_parameter(
                'avoidance.backup_duration'),
            obstacle_backup_thrust=self._float_parameter(
                'avoidance.backup_thrust'),
            terminal_recovery_disable_radius=self._float_parameter(
                'avoidance.terminal_recovery_disable_radius'),
            obstacle_avoidance_enabled=bool(
                self.get_parameter('avoidance.enabled').value),
        )

    def _gps_callback(self, message: NavSatFix) -> None:
        if message.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if not math.isfinite(message.latitude) or not math.isfinite(message.longitude):
            return
        now = time.monotonic()
        sample_time = self._stamp_seconds(message.header.stamp)
        if sample_time <= 0.0:
            sample_time = 1e-9 * float(self.get_clock().now().nanoseconds)
        if self.ekf_enabled:
            position_variances = None
            covariance = message.position_covariance
            if (
                len(covariance) >= 5
                and math.isfinite(covariance[0]) and covariance[0] > 0.0
                and math.isfinite(covariance[4]) and covariance[4] > 0.0
            ):
                position_variances = (covariance[0], covariance[4])
            self.estimator.update_gps(
                message.latitude, message.longitude, sample_time,
                position_variances)
            self._apply_estimator_state()
        else:
            self.latitude = message.latitude
            self.longitude = message.longitude
        self.speed_estimator.update(
            message.latitude, message.longitude, sample_time)
        if not self.ekf_enabled and self.yaw is None:
            self.speed = 0.0
        elif not self.ekf_enabled:
            self.speed = (
                self.speed_estimator.velocity_east * math.cos(self.yaw)
                + self.speed_estimator.velocity_north * math.sin(self.yaw)
            )
        if self.yaw is not None and self.latitude is not None:
            self.pose_history.append((
                sample_time,
                self.latitude,
                self.longitude,
                self.yaw,
            ))
            self.pose_history = self.pose_history[-100:]
        self.last_gps_time = now

    def _imu_callback(self, message: Imu) -> None:
        q = message.orientation
        if not all(math.isfinite(value) for value in (q.x, q.y, q.z, q.w)):
            return
        if message.orientation_covariance[0] == -1.0:
            return
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 0.5 or norm > 1.5:
            return
        yaw = quaternion_to_yaw(
            q.x / norm, q.y / norm, q.z / norm, q.w / norm)
        sample_time = self._stamp_seconds(message.header.stamp)
        if sample_time <= 0.0:
            sample_time = 1e-9 * float(self.get_clock().now().nanoseconds)
        angular_velocity_valid = (
            message.angular_velocity_covariance[0] != -1.0
            and math.isfinite(message.angular_velocity.z)
        )
        if angular_velocity_valid:
            self.yaw_rate = float(message.angular_velocity.z)
            self.yaw_rate_valid = True
        elif (
            self.last_imu_sample_time is not None
            and self.last_imu_yaw is not None
            and sample_time > self.last_imu_sample_time
        ):
            self.yaw_rate = normalize_angle(yaw - self.last_imu_yaw) / (
                sample_time - self.last_imu_sample_time)
            self.yaw_rate_valid = math.isfinite(self.yaw_rate)
        else:
            self.yaw_rate = 0.0
            self.yaw_rate_valid = False
        if self.ekf_enabled:
            yaw_variance = None
            if (
                len(message.orientation_covariance) >= 9
                and math.isfinite(message.orientation_covariance[8])
                and message.orientation_covariance[8] > 0.0
            ):
                yaw_variance = message.orientation_covariance[8]
            yaw_rate_variance = None
            if (
                angular_velocity_valid
                and len(message.angular_velocity_covariance) >= 9
                and math.isfinite(message.angular_velocity_covariance[8])
                and message.angular_velocity_covariance[8] > 0.0
            ):
                yaw_rate_variance = message.angular_velocity_covariance[8]
            self.estimator.update_imu(
                yaw,
                self.yaw_rate if self.yaw_rate_valid else None,
                sample_time,
                yaw_variance,
                yaw_rate_variance,
            )
            self._apply_estimator_state()
        else:
            self.yaw = yaw
        self.last_imu_sample_time = sample_time
        self.last_imu_yaw = yaw
        self.last_imu_time = time.monotonic()

    def _apply_estimator_state(self) -> None:
        estimate = self.estimator.estimate()
        if estimate is None:
            self.estimator_healthy = False
            self._select_estimator_state(time.monotonic())
            return
        self.estimator_healthy = estimate.healthy
        self.estimator_position_std = estimate.position_std
        self.estimator_velocity_std = estimate.velocity_std
        self.estimator_yaw_std = estimate.yaw_std
        self._select_estimator_state(time.monotonic())

    def _gps_origin_callback(self, message: NavSatFix) -> None:
        if message.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if not self._valid_geodetic(message.latitude, message.longitude):
            return
        self.robot_localization_origin = (
            float(message.latitude), float(message.longitude))

    def _filtered_odometry_callback(self, message: Odometry) -> None:
        if self.robot_localization_origin is None:
            return
        position = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = validated_quaternion_yaw(q.x, q.y, q.z, q.w)
        values = (
            position.x,
            position.y,
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
        )
        if yaw is None or not all(math.isfinite(value) for value in values):
            return
        covariance = message.pose.covariance
        position_variance = max(0.0, float(covariance[0])) + max(
            0.0, float(covariance[7]))
        yaw_variance = max(0.0, float(covariance[35]))
        position_std = math.sqrt(position_variance)
        yaw_std = math.sqrt(yaw_variance)
        latitude, longitude = enu_to_geodetic(
            self.robot_localization_origin[0],
            self.robot_localization_origin[1],
            float(position.x),
            float(position.y),
        )
        self.robot_localization_state = {
            'latitude': latitude,
            'longitude': longitude,
            'east': float(position.x),
            'north': float(position.y),
            'yaw': yaw,
            'yaw_rate': float(message.twist.twist.angular.z),
            'speed': float(message.twist.twist.linear.x),
            'position_std': position_std,
            'yaw_std': yaw_std,
        }
        self.robot_localization_last_time = time.monotonic()
        self._select_estimator_state(self.robot_localization_last_time)

    def _select_estimator_state(self, now: float) -> None:
        custom = self.estimator.estimate() if self.ekf_enabled else None
        external = self.robot_localization_state
        external_fresh = (
            self.robot_localization_enabled
            and external is not None
            and self.robot_localization_last_time is not None
            and now - self.robot_localization_last_time
            <= self.robot_localization_timeout
        )
        self.estimator_disagreement_m = None
        if external_fresh and custom is not None:
            disagreement_east, disagreement_north = geodetic_delta_m(
                custom.latitude,
                custom.longitude,
                external['latitude'],
                external['longitude'],
            )
            self.estimator_disagreement_m = math.hypot(
                disagreement_east, disagreement_north)
        self.robot_localization_healthy = bool(
            external_fresh
            and external['position_std']
            <= self.robot_localization_max_position_std
            and external['yaw_std'] <= self.robot_localization_max_yaw_std
            and (
                self.estimator_disagreement_m is None
                or self.estimator_disagreement_m
                <= self.robot_localization_max_disagreement
            )
        )
        previous_source = self.estimator_source
        if self.robot_localization_healthy:
            self.estimator_source = 'robot_localization'
            self.latitude = external['latitude']
            self.longitude = external['longitude']
            self.navigation_east = external['east']
            self.navigation_north = external['north']
            self.yaw = external['yaw']
            self.yaw_rate = external['yaw_rate']
            self.yaw_rate_valid = True
            self.speed = external['speed']
            self.estimator_position_std = external['position_std']
            self.estimator_yaw_std = external['yaw_std']
        elif custom is not None:
            self.estimator_source = 'custom_ekf'
            self.latitude = custom.latitude
            self.longitude = custom.longitude
            self.navigation_east = custom.east
            self.navigation_north = custom.north
            self.yaw = custom.yaw
            self.yaw_rate = custom.yaw_rate
            self.yaw_rate_valid = True
            self.speed = custom.forward_speed
            self.estimator_position_std = custom.position_std
            self.estimator_velocity_std = custom.velocity_std
            self.estimator_yaw_std = custom.yaw_std
        if (
            previous_source == 'robot_localization'
            and self.estimator_source == 'custom_ekf'
        ):
            self.estimator_fallback_count += 1

    def _lidar_callback(self, message: LaserScan) -> None:
        if (
            not message.ranges
            or not math.isfinite(message.angle_min)
            or not math.isfinite(message.angle_increment)
            or message.angle_increment <= 0.0
        ):
            return
        self.laser_ranges = tuple(message.ranges)
        self.laser_angle_min = message.angle_min
        self.laser_angle_increment = message.angle_increment
        if math.isfinite(message.range_min) and message.range_min >= 0.0:
            self.laser_range_min = message.range_min
        now = time.monotonic()
        if self.map_enabled:
            self._select_estimator_state(now)
            if (
                self.navigation_east is not None
                and self.navigation_north is not None
                and self.yaw is not None
            ):
                sensor_east = (
                    self.navigation_east
                    + self.cloud_sensor_offset_x * math.cos(self.yaw)
                    - self.cloud_sensor_offset_y * math.sin(self.yaw)
                )
                sensor_north = (
                    self.navigation_north
                    + self.cloud_sensor_offset_x * math.sin(self.yaw)
                    + self.cloud_sensor_offset_y * math.cos(self.yaw)
                )
                dynamic_targets = tuple(
                    (track.east, track.north)
                    for track in self._confirmed_dynamic_tracks(now)
                )
                map_ranges, self.dynamic_masked_scan_beams = (
                    mask_dynamic_scan_ranges(
                        sensor_east,
                        sensor_north,
                        self.yaw,
                        message.ranges,
                        message.angle_min,
                        message.angle_increment,
                        dynamic_targets,
                        self.colregs_map_mask_radius,
                    )
                )
                self.occupancy_grid.update_scan(
                    sensor_east,
                    sensor_north,
                    self.yaw,
                    map_ranges,
                    message.angle_min,
                    message.angle_increment,
                    message.range_min,
                    message.range_max,
                    now,
                )
        self.last_scan_time = now

    def _point_cloud_callback(self, message: PointCloud2) -> None:
        warning_distance = self.controller.config.obstacle_warning_distance
        planning_angle = self.controller.config.obstacle_planning_angle
        raw_points = []
        try:
            points = point_cloud2.read_points(
                message,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
            )
            for index, point in enumerate(points):
                if index % self.cloud_point_stride != 0:
                    continue
                raw_points.append(
                    (float(point[0]), float(point[1]), float(point[2])))
        except (KeyError, TypeError, ValueError):
            return
        if not raw_points:
            return
        obstacles = extract_obstacle_points(
            raw_points,
            warning_distance,
            planning_angle,
            minimum_height=self.cloud_min_height,
            minimum_above_water=self.cloud_min_above_water,
            cluster_cell_size=self.cloud_cluster_cell_size,
            cluster_min_points=self.cloud_cluster_min_points,
        )
        now = time.monotonic()
        cloud_time = self._stamp_seconds(message.header.stamp)
        pose = self._pose_for_timestamp(cloud_time)
        self.obstacle_points = tuple(obstacles)
        self._update_cloud_tracks(obstacles, now, pose)
        self._update_map_from_cloud_tracks(now)
        self.last_cloud_time = now

    def _update_cloud_tracks(
        self,
        obstacles: List[Tuple[float, float]],
        now: float,
        pose: Optional[Tuple[float, float, float]],
    ) -> None:
        self.cloud_tracks = [
            track for track in self.cloud_tracks
            if now - track[2] <= self.cloud_track_timeout
        ]
        if pose is None:
            return
        detections = [
            self._body_point_to_geodetic(distance, angle, pose)
            for distance, angle in obstacles
        ]
        if self.colregs_use_pointcloud_tracks and self.estimator.origin is not None:
            origin_latitude, origin_longitude = self.estimator.origin
            self.dynamic_tracker.update(
                (
                    geodetic_delta_m(
                        origin_latitude, origin_longitude,
                        latitude, longitude)
                    for latitude, longitude in detections
                ),
                now,
            )
        used_tracks = set()
        for latitude, longitude in detections:
            best_index = None
            best_distance = self.cloud_track_match_distance
            for index, track in enumerate(self.cloud_tracks):
                if index in used_tracks:
                    continue
                east, north = geodetic_delta_m(
                    latitude, longitude, track[0], track[1])
                distance = math.hypot(east, north)
                if distance <= best_distance:
                    best_distance = distance
                    best_index = index
            if best_index is None:
                self.cloud_tracks.append((latitude, longitude, now, 1))
                used_tracks.add(len(self.cloud_tracks) - 1)
            else:
                old = self.cloud_tracks[best_index]
                self.cloud_tracks[best_index] = (
                    latitude, longitude, now, min(10, old[3] + 1))
                used_tracks.add(best_index)

    def _dynamic_targets_callback(self, message: PoseArray) -> None:
        if message.header.frame_id != self.colregs_target_frame:
            self.get_logger().warning(
                'Ignored dynamic targets in frame '
                f'{message.header.frame_id!r}; expected '
                f'{self.colregs_target_frame!r}',
                throttle_duration_sec=5.0,
            )
            return
        detections = [
            (float(pose.position.x), float(pose.position.y))
            for pose in message.poses
            if math.isfinite(pose.position.x)
            and math.isfinite(pose.position.y)
        ]
        now = time.monotonic()
        self.dynamic_tracker.update(detections, now)
        self.last_dynamic_target_time = now

    def _update_map_from_cloud_tracks(self, now: float) -> None:
        if not self.map_enabled or self.estimator.origin is None:
            return
        self._select_estimator_state(now)
        if self.navigation_east is None or self.navigation_north is None:
            return
        origin_latitude, origin_longitude = self.estimator.origin
        points = []
        dynamic_targets = tuple(
            (track.east, track.north)
            for track in self._confirmed_dynamic_tracks(now)
        )
        self.dynamic_masked_cloud_tracks = 0
        for latitude, longitude, timestamp, hits in self.cloud_tracks:
            if (
                hits < self.map_cloud_track_min_hits
                or now - timestamp > self.cloud_track_timeout
            ):
                continue
            point = geodetic_delta_m(
                origin_latitude,
                origin_longitude,
                latitude,
                longitude,
            )
            if any(
                math.hypot(
                    point[0] - target[0], point[1] - target[1]
                ) <= self.colregs_map_mask_radius
                for target in dynamic_targets
            ):
                self.dynamic_masked_cloud_tracks += 1
                continue
            points.append(point)
        self.occupancy_grid.update_obstacles(
            self.navigation_east,
            self.navigation_north,
            points,
            now,
        )

    def _confirmed_dynamic_tracks(self, now: float):
        return tuple(
            track for track in self.dynamic_tracker.active_tracks(now)
            if is_confirmed_moving(
                track,
                minimum_target_speed=self.colregs_minimum_speed,
                minimum_hits=self.colregs_min_hits,
            )
        )

    def _body_point_to_geodetic(
        self,
        distance: float,
        angle: float,
        pose: Tuple[float, float, float],
    ) -> Tuple[float, float]:
        latitude_origin, longitude_origin, yaw = pose
        forward = self.cloud_sensor_offset_x + distance * math.cos(angle)
        left = self.cloud_sensor_offset_y + distance * math.sin(angle)
        east = forward * math.cos(yaw) - left * math.sin(yaw)
        north = forward * math.sin(yaw) + left * math.cos(yaw)
        latitude = latitude_origin + math.degrees(north / EARTH_RADIUS_M)
        longitude_scale = EARTH_RADIUS_M * max(
            0.01, abs(math.cos(math.radians(latitude_origin))))
        longitude = longitude_origin + math.degrees(east / longitude_scale)
        return latitude, longitude

    def _pose_for_timestamp(
        self, timestamp: float,
    ) -> Optional[Tuple[float, float, float]]:
        if self.pose_history and timestamp > 0.0:
            sample = min(
                self.pose_history,
                key=lambda candidate: abs(candidate[0] - timestamp),
            )
            if abs(sample[0] - timestamp) > self.cloud_pose_tolerance:
                return None
            return sample[1], sample[2], sample[3]
        if self.latitude is None or self.longitude is None or self.yaw is None:
            return None
        return self.latitude, self.longitude, self.yaw

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    def _tracked_obstacle_points(self, now: float) -> Tuple[Tuple[float, float], ...]:
        if self.latitude is None or self.longitude is None or self.yaw is None:
            if (
                self.last_cloud_time is not None
                and now - self.last_cloud_time <= self.scan_cache_timeout
            ):
                return self.obstacle_points
            return ()
        points = []
        retained_tracks = []
        for track in self.cloud_tracks:
            if now - track[2] > self.cloud_track_timeout:
                continue
            retained_tracks.append(track)
            if track[3] < self.cloud_track_min_hits:
                continue
            east, north = geodetic_delta_m(
                self.latitude, self.longitude, track[0], track[1])
            forward = east * math.cos(self.yaw) + north * math.sin(self.yaw)
            left = -east * math.sin(self.yaw) + north * math.cos(self.yaw)
            distance = math.hypot(forward, left)
            if distance <= self.controller.config.obstacle_warning_distance:
                points.append((distance, math.atan2(left, forward)))
        self.cloud_tracks = retained_tracks
        return tuple(points)

    def _colregs_supervision(self, now: float):
        self.current_encounter = None
        self.colregs_risk_count = 0
        if not self.colregs_enabled:
            return None
        estimate = self.estimator.estimate()
        if estimate is None:
            return None
        assessments = [
            assess_encounter(
                (estimate.east, estimate.north),
                (estimate.velocity_east, estimate.velocity_north),
                estimate.yaw,
                track,
                safety_radius=self.colregs_safety_radius,
                time_horizon=self.colregs_time_horizon,
                minimum_target_speed=self.colregs_minimum_speed,
                minimum_hits=self.colregs_min_hits,
            )
            for track in self.dynamic_tracker.active_tracks(now)
        ]
        self.colregs_risk_count = sum(
            assessment.risk for assessment in assessments)
        self.current_encounter = select_most_urgent(assessments)
        return self.current_encounter

    def _waypoints_callback(self, message: PoseArray) -> None:
        if (
            self.task_mode == 'auto'
            and self.task_name not in ('unknown', 'wayfinding')
        ):
            return
        targets: List[GeoTarget] = []
        signature = []
        for pose in message.poses:
            latitude = float(pose.position.x)
            longitude = float(pose.position.y)
            if not self._valid_geodetic(latitude, longitude):
                continue
            q = pose.orientation
            yaw = validated_quaternion_yaw(q.x, q.y, q.z, q.w)
            if yaw is None:
                self.get_logger().error('Rejected waypoint with invalid orientation')
                continue
            targets.append(GeoTarget(latitude, longitude, yaw))
            signature.append((
                round(latitude, 10),
                round(longitude, 10),
                round(yaw, 7),
            ))
        frozen_signature = tuple(signature)
        if not targets or frozen_signature == self.waypoint_signature:
            return
        if self.task_mode in ('auto', 'wayfinding'):
            nearest_order = bool(
                self.get_parameter('navigation.nearest_neighbor_order').value)
            if nearest_order and (self.latitude is None or self.longitude is None):
                # VRX republishes goals at 1 Hz; defer so the configured order
                # does not depend on a startup callback race.
                return
            if nearest_order:
                targets = nearest_neighbor_order(
                    self.latitude, self.longitude, targets)
            self.waypoint_signature = frozen_signature
            self.active_mode = 'wayfinding'
            self.controller.set_targets(targets, 'wayfinding')
            self.get_logger().info(f'Loaded {len(targets)} competition waypoints')

    def _stationkeeping_callback(self, message: PoseStamped) -> None:
        if (
            self.task_mode == 'auto'
            and self.task_name not in ('unknown', 'stationkeeping')
        ):
            return
        latitude = float(message.pose.position.x)
        longitude = float(message.pose.position.y)
        if not self._valid_geodetic(latitude, longitude):
            self.get_logger().error('Rejected invalid stationkeeping goal')
            return
        q = message.pose.orientation
        yaw = validated_quaternion_yaw(q.x, q.y, q.z, q.w)
        if yaw is None:
            self.get_logger().error('Rejected invalid stationkeeping orientation')
            return
        signature = (
            round(latitude, 10), round(longitude, 10), round(yaw, 7))
        if signature == self.stationkeeping_signature:
            return
        if self.task_mode in ('auto', 'stationkeeping'):
            self.stationkeeping_signature = signature
            self.active_mode = 'stationkeeping'
            self.controller.set_targets(
                [GeoTarget(latitude, longitude, yaw)], 'stationkeeping')
            self.get_logger().info('Loaded competition stationkeeping goal')

    def _task_callback(self, message: ParamVec) -> None:
        old_state = self.task_state
        old_name = self.task_name
        for parameter in message.params:
            if parameter.name == 'state':
                self.task_state = parameter.value.string_value.lower()
            elif parameter.name == 'name':
                self.task_name = parameter.value.string_value.lower()
            elif parameter.name == 'num_collisions':
                self.num_collisions = int(parameter.value.integer_value)
        restarted = (
            self.task_name != old_name
            or (
                old_state in ('finished', 'timed_out')
                and self.task_state in ('initial', 'ready', 'running')
            )
        )
        if restarted:
            self.waypoint_signature = None
            self.stationkeeping_signature = None
            self.active_mode = (
                self.task_mode if self.task_mode != 'auto' else 'unknown')
            self.controller.set_targets([], self.active_mode)
            self._reset_runtime_state()
        if old_state != self.task_state:
            self.get_logger().info(
                f'Task {self.task_name}: {old_state} -> {self.task_state}')

    def _reset_runtime_state(self) -> None:
        """Make task restarts wait for fresh state and perception samples."""
        self.estimator.reset()
        self.speed_estimator.reset()
        self.occupancy_grid.reset()
        self.latitude = None
        self.longitude = None
        self.yaw = None
        self.yaw_rate = 0.0
        self.yaw_rate_valid = False
        self.speed = 0.0
        self.last_imu_sample_time = None
        self.last_imu_yaw = None
        self.last_gps_time = None
        self.last_imu_time = None
        self.last_scan_time = None
        self.last_cloud_time = None
        self.last_control_time = None
        self.laser_ranges = ()
        self.obstacle_points = ()
        self.cloud_tracks = []
        self.buoy_candidate_count = 0
        self.previous_buoy_candidate_marker_count = 0
        self.dynamic_tracker.reset()
        self.current_encounter = None
        self.colregs_risk_count = 0
        self.last_dynamic_target_time = None
        self.dynamic_masked_scan_beams = 0
        self.dynamic_masked_cloud_tracks = 0
        self.pose_history = []
        self.trajectory_history = []
        self.tracking_error_history = []
        self.debug_path_revision = -1
        self.last_debug_publish_time = 0.0
        self.latest_occupancy_snapshot = None
        self.last_map_publish_time = 0.0
        self.map_known_cells = 0
        self.map_occupied_cells = 0
        self.estimator_healthy = not self.ekf_enabled
        self.estimator_position_std = None
        self.estimator_velocity_std = None
        self.estimator_yaw_std = None
        self.robot_localization_state = None
        self.robot_localization_last_time = None
        self.robot_localization_healthy = False
        self.estimator_source = 'custom_ekf'
        self.estimator_disagreement_m = None
        self.estimator_fallback_count = 0
        self.navigation_east = None
        self.navigation_north = None

    def _load_custom_waypoints(self) -> None:
        raw_waypoints = [
            value for value in self.get_parameter('custom_waypoints').value
            if str(value).strip()
        ]
        if self.task_mode != 'custom' or not raw_waypoints:
            return
        targets = []
        for raw in raw_waypoints:
            fields = [field.strip() for field in str(raw).split(',')]
            if len(fields) not in (2, 3):
                raise ValueError(
                    f'Invalid custom waypoint {raw!r}; expected lat,lon[,yaw_deg]')
            latitude = float(fields[0])
            longitude = float(fields[1])
            yaw = math.radians(float(fields[2])) if len(fields) == 3 else None
            if not self._valid_geodetic(latitude, longitude):
                raise ValueError(f'Invalid custom waypoint {raw!r}')
            targets.append(GeoTarget(latitude, longitude, yaw))
        self.active_mode = 'custom'
        self.controller.set_targets(targets, 'wayfinding')
        self.get_logger().info(f'Loaded {len(targets)} custom GNSS waypoints')

    @staticmethod
    def _valid_geodetic(latitude: float, longitude: float) -> bool:
        return (
            math.isfinite(latitude)
            and math.isfinite(longitude)
            and -90.0 <= latitude <= 90.0
            and -180.0 <= longitude <= 180.0
        )

    def _control_loop(self) -> None:
        now = time.monotonic()
        self._select_estimator_state(now)
        control_time = 1e-9 * float(self.get_clock().now().nanoseconds)
        if self.last_control_time is None:
            dt = self.control_period
        else:
            dt = control_time - self.last_control_time
            if dt <= 0.0 or dt > 0.5:
                dt = self.control_period
        self.last_control_time = control_time
        self._publish_float(self.left_pos_pub, 0.0)
        self._publish_float(self.right_pos_pub, 0.0)

        self._check_control_conflict(now)
        blocked_reason = self._blocked_reason(now)
        if blocked_reason is not None:
            command = self.controller.stop(blocked_reason)
            obstacle_points: Tuple[Tuple[float, float], ...] = ()
        else:
            scan_is_fresh = (
                self.last_scan_time is not None
                and now - self.last_scan_time <= self.scan_cache_timeout
            )
            obstacle_points = self._tracked_obstacle_points(now)
            encounter = self._colregs_supervision(now)
            vessel = VesselState(
                latitude=float(self.latitude),
                longitude=float(self.longitude),
                yaw=float(self.yaw),
                speed=self.speed,
                yaw_rate=self.yaw_rate,
                yaw_rate_valid=self.yaw_rate_valid,
                laser_ranges=self.laser_ranges if scan_is_fresh else (),
                laser_angle_min=self.laser_angle_min,
                laser_angle_increment=self.laser_angle_increment,
                laser_range_min=self.laser_range_min,
                obstacle_points=obstacle_points,
                east=self.navigation_east,
                north=self.navigation_north,
                occupancy_grid=self.latest_occupancy_snapshot,
                colregs_active=encounter is not None,
                colregs_heading_bias=(
                    encounter.heading_bias if encounter is not None else 0.0),
                colregs_speed_scale=(
                    encounter.speed_scale if encounter is not None else 1.0),
                colregs_action=(
                    encounter.action if encounter is not None else 'none'),
            )
            command = self.controller.update(vessel, dt)

        buoy_candidates = self._buoy_candidate_points(obstacle_points, now)
        self.buoy_candidate_count = len(buoy_candidates)

        if command.guidance_replanned:
            self.get_logger().info(
                'Guidance path replanned: '
                f'reason={command.guidance_replan_reason} '
                f'revision={command.path_revision}')

        self._publish_float(self.left_pub, command.left_thrust)
        self._publish_float(self.right_pub, command.right_thrust)
        self._update_debug_history(control_time, command)
        self._log_identification(control_time, command)
        self._publish_status(command)
        if self.map_enabled and now - self.last_map_publish_time >= self.map_publish_period:
            self.last_map_publish_time = now
            self._publish_occupancy_grid(now)
        if self.publish_debug_markers:
            if now - self.last_debug_publish_time >= self.debug_publish_period:
                self.last_debug_publish_time = now
                self._publish_debug_markers(command, obstacle_points)
                self._publish_buoy_candidate_markers(buoy_candidates)
        if now - self.last_log_time >= 2.0:
            self.last_log_time = now
            distance = command.distance
            distance_text = f'{distance:.1f}m' if math.isfinite(distance) else '-'
            displayed_target = min(
                command.target_index + 1, command.target_count)
            if command.target_count == 0:
                displayed_target = 0
            nearest_text = (
                f'{command.nearest_obstacle:.1f}'
                if math.isfinite(command.nearest_obstacle)
                else '-'
            )
            collision_text = (
                f'{command.collision_clearance:.1f}'
                if math.isfinite(command.collision_clearance)
                else '-'
            )
            self.get_logger().info(
                f'{command.state} mode={self.active_mode} '
                f'wp={displayed_target}/{command.target_count} '
                f'dist={distance_text} yaw_err={math.degrees(command.heading_error):.1f}deg '
                f'yaw_rate={math.degrees(self.yaw_rate):.1f}deg/s '
                f'speed={self.speed:.2f}/{command.desired_speed:.2f}m/s '
                f'obs={nearest_text}m collision={collision_text}m '
                f'tracks={len(self.cloud_tracks)} '
                f'thrust=({command.left_thrust:.0f},{command.right_thrust:.0f})')

    def _update_debug_history(self, timestamp: float, command) -> None:
        if (
            command.path_valid
            and command.path_revision != self.debug_path_revision
        ):
            self.trajectory_history = []
            self.tracking_error_history = []
            self.debug_path_revision = command.path_revision

        if (
            self.navigation_east is not None
            and self.navigation_north is not None
            and math.isfinite(self.navigation_east)
            and math.isfinite(self.navigation_north)
        ):
            position = (self.navigation_east, self.navigation_north)
            if (
                not self.trajectory_history
                or math.hypot(
                    position[0] - self.trajectory_history[-1][0],
                    position[1] - self.trajectory_history[-1][1],
                ) >= 0.25
            ):
                self.trajectory_history.append(position)
                self.trajectory_history = self.trajectory_history[-1200:]

        if command.path_valid and math.isfinite(command.cross_track_error):
            terminal_active = command.state in (
                'approach_braking', 'braking', 'aligning',
                'alignment_blocked',
                'waypoint_dwell', 'stationkeeping', 'complete',
            )
            safety_active = bool(
                command.avoidance_override
                or command.avoidance_episode_active
                or command.colregs_active
                or command.state in ('pivoting', 'backing_away')
            )
            self.tracking_error_history.append((
                timestamp,
                abs(command.cross_track_error),
                safety_active or terminal_active,
            ))
        cutoff = timestamp - 25.0
        self.tracking_error_history = [
            sample for sample in self.tracking_error_history
            if sample[0] >= cutoff
        ]

    def _log_identification(self, timestamp: float, command) -> None:
        if self.identification_logger is None:
            return
        estimate = self.estimator.estimate()
        if estimate is None:
            return
        cos_yaw = math.cos(estimate.yaw)
        sin_yaw = math.sin(estimate.yaw)
        surge = (
            estimate.velocity_east * cos_yaw
            + estimate.velocity_north * sin_yaw)
        sway = (
            -estimate.velocity_east * sin_yaw
            + estimate.velocity_north * cos_yaw)
        self.identification_logger.append(IdentificationSample(
            time_s=timestamp,
            east_m=estimate.east,
            north_m=estimate.north,
            yaw_rad=estimate.yaw,
            surge_mps=surge,
            sway_mps=sway,
            yaw_rate_rps=estimate.yaw_rate,
            left_thrust=command.left_thrust,
            right_thrust=command.right_thrust,
            state=command.state,
        ))

    def _blocked_reason(self, now: float) -> Optional[str]:
        if self.control_conflict:
            return 'control_conflict'
        if self.latitude is None or self.last_gps_time is None:
            return 'waiting_for_gps'
        if self.yaw is None or self.last_imu_time is None:
            return 'waiting_for_imu'
        if (
            self.ekf_enabled
            and self.estimator_source == 'custom_ekf'
            and not self.estimator_healthy
        ):
            return 'ekf_unhealthy'
        if now - self.last_gps_time > self.sensor_timeout:
            return 'gps_timeout'
        if now - self.last_imu_time > self.sensor_timeout:
            return 'imu_timeout'
        if self.controller.config.obstacle_avoidance_enabled and self.require_lidar:
            lidar_times = [
                timestamp for timestamp in (
                    self.last_scan_time, self.last_cloud_time)
                if timestamp is not None
            ]
            if not lidar_times:
                return 'waiting_for_lidar'
            if now - max(lidar_times) > self.scan_cache_timeout:
                return 'lidar_timeout'
        task_is_running = self.task_state == 'running'
        if not task_is_running and not self.start_without_task:
            return f'task_{self.task_state}'
        return None

    def _check_control_conflict(self, now: float) -> None:
        if now - self.last_conflict_check < 1.0:
            return
        self.last_conflict_check = now
        conflict = (
            self.count_publishers(self.left_topic) > 1
            or self.count_publishers(self.right_topic) > 1
            or self.count_publishers(self.left_pos_topic) > 1
            or self.count_publishers(self.right_pos_topic) > 1
        )
        if conflict and not self.control_conflict:
            self.get_logger().error(
                'Multiple thruster publishers detected; commands are inhibited')
        self.control_conflict = conflict and self.stop_on_control_conflict

    def _publish_status(self, command) -> None:
        now = time.monotonic()
        control_time = 1e-9 * float(self.get_clock().now().nanoseconds)
        tracking = tracking_statistics(
            self.tracking_error_history, control_time)

        def age(timestamp: Optional[float]) -> Optional[float]:
            return None if timestamp is None else max(0.0, now - timestamp)

        message = String()
        message.data = json.dumps({
            'state': command.state,
            'mode': self.active_mode,
            'task_state': self.task_state,
            'task_name': self.task_name,
            'num_collisions': self.num_collisions,
            'target_index': command.target_index,
            'target_count': command.target_count,
            'distance_m': command.distance if math.isfinite(command.distance) else None,
            'heading_error_deg': math.degrees(command.heading_error),
            'yaw_rate_deg_s': math.degrees(self.yaw_rate),
            'yaw_rate_valid': self.yaw_rate_valid,
            'ekf_enabled': self.ekf_enabled,
            'ekf_healthy': self.estimator_healthy,
            'ekf_position_std_m': self.estimator_position_std,
            'ekf_velocity_std_mps': self.estimator_velocity_std,
            'ekf_yaw_std_deg': (
                None if self.estimator_yaw_std is None
                else math.degrees(self.estimator_yaw_std)),
            'ekf_rejected_measurements': self.estimator.rejected_measurements,
            'estimator_source': self.estimator_source,
            'robot_localization_enabled': self.robot_localization_enabled,
            'robot_localization_healthy': self.robot_localization_healthy,
            'robot_localization_age_s': age(
                self.robot_localization_last_time),
            'estimator_disagreement_m': self.estimator_disagreement_m,
            'estimator_fallback_count': self.estimator_fallback_count,
            'map_enabled': self.map_enabled,
            'map_revision': self.occupancy_grid.revision,
            'map_known_cells': self.map_known_cells,
            'map_occupied_cells': self.map_occupied_cells,
            'speed_mps': self.speed,
            'desired_speed_mps': command.desired_speed,
            'nearest_obstacle_m': (
                command.nearest_obstacle
                if math.isfinite(command.nearest_obstacle)
                else None
            ),
            'path_clearance_m': (
                command.path_clearance
                if math.isfinite(command.path_clearance)
                else None
            ),
            'collision_clearance_m': (
                command.collision_clearance
                if math.isfinite(command.collision_clearance)
                else None
            ),
            'guidance_mode': command.guidance_mode,
            'path_valid': command.path_valid,
            'path_revision': command.path_revision,
            'path_segment_index': command.path_segment_index,
            'path_remaining_m': (
                command.path_remaining
                if math.isfinite(command.path_remaining)
                else None
            ),
            'cross_track_error_m': command.cross_track_error,
            'tracking_error_mean_20s_m': tracking.mean_abs_m,
            'tracking_error_max_20s_m': tracking.max_abs_m,
            'trajectory_point_count': len(self.trajectory_history),
            'path_deviation_m': (
                command.path_deviation if command.path_valid else None),
            'nominal_heading_error_deg': math.degrees(
                command.nominal_heading_error),
            'avoidance_override': command.avoidance_override,
            'avoidance_episode_active': command.avoidance_episode_active,
            'guidance_replan_pending': command.guidance_replan_pending,
            'guidance_replanned': command.guidance_replanned,
            'guidance_replan_reason': command.guidance_replan_reason,
            'guidance_replan_cooldown_remaining_s': (
                command.guidance_replan_cooldown_remaining),
            'lattice_expanded_states': command.lattice_expanded_states,
            'lattice_map_revision': command.lattice_map_revision,
            'lattice_partial_path': command.lattice_partial_path,
            'lattice_fallback': command.lattice_fallback,
            'lattice_blocked_confirmations': (
                command.lattice_blocked_confirmations),
            'lattice_planning_time_ms': command.lattice_planning_time_ms,
            'dynamic_track_count': len(
                self.dynamic_tracker.active_tracks(now)),
            'colregs_enabled': self.colregs_enabled,
            'colregs_target_source': (
                'pointcloud_experimental'
                if self.colregs_use_pointcloud_tracks else 'dedicated_topic'),
            'dynamic_target_age_s': age(self.last_dynamic_target_time),
            'dynamic_masked_scan_beams': self.dynamic_masked_scan_beams,
            'dynamic_masked_cloud_tracks': self.dynamic_masked_cloud_tracks,
            'colregs_active': command.colregs_active,
            'colregs_risk_count': self.colregs_risk_count,
            'colregs_encounter': (
                self.current_encounter.encounter
                if self.current_encounter is not None else 'none'),
            'colregs_action': command.colregs_action,
            'colregs_tcpa_s': (
                self.current_encounter.tcpa_s
                if self.current_encounter is not None else None),
            'colregs_dcpa_m': (
                self.current_encounter.dcpa_m
                if self.current_encounter is not None else None),
            'colregs_heading_bias_deg': math.degrees(
                command.colregs_heading_bias),
            'colregs_speed_scale': command.colregs_speed_scale,
            'model_identification_enabled': self.identification_enabled,
            'model_control_requested_backend': (
                self.model_control_status.requested_backend),
            'model_control_active_backend': (
                self.model_control_status.active_backend),
            'nmpc_model_ready': self.model_control_status.nmpc_ready,
            'nmpc_gate_reason': self.model_control_status.reason,
            'ilos_integral_bias_m': command.ilos_integral_bias,
            'path_curvature_1pm': command.path_curvature,
            'upcoming_curvature_1pm': command.upcoming_curvature,
            'yaw_rate_feedforward_deg_s': math.degrees(
                command.yaw_rate_feedforward),
            'desired_yaw_rate_deg_s': math.degrees(
                command.desired_yaw_rate),
            'cloud_track_count': len(self.cloud_tracks),
            'buoy_candidate_count': self.buoy_candidate_count,
            'gps_age_s': age(self.last_gps_time),
            'imu_age_s': age(self.last_imu_time),
            'scan_age_s': age(self.last_scan_time),
            'cloud_age_s': age(self.last_cloud_time),
            'left_thrust': command.left_thrust,
            'right_thrust': command.right_thrust,
        }, separators=(',', ':'))
        self.status_pub.publish(message)

    def _publish_occupancy_grid(self, now: float) -> None:
        try:
            snapshot = self.occupancy_grid.snapshot(
                now, inflation_radius=self.map_inflation_radius)
        except RuntimeError:
            return
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.info.resolution = snapshot.resolution
        message.info.width = snapshot.width
        message.info.height = snapshot.height
        estimate = self.estimator.estimate()
        if estimate is not None and self.map_frame == self.debug_frame:
            origin_x, origin_y, origin_yaw = enu_grid_origin_in_body(
                snapshot.origin_east,
                snapshot.origin_north,
                estimate.east,
                estimate.north,
                estimate.yaw,
            )
            message.info.origin.position.x = origin_x
            message.info.origin.position.y = origin_y
            message.info.origin.orientation.z = math.sin(0.5 * origin_yaw)
            message.info.origin.orientation.w = math.cos(0.5 * origin_yaw)
        else:
            message.info.origin.position.x = snapshot.origin_east
            message.info.origin.position.y = snapshot.origin_north
            message.info.origin.orientation.w = 1.0
        message.data = list(snapshot.probabilities)
        self.map_known_cells = sum(value >= 0 for value in snapshot.probabilities)
        self.map_occupied_cells = sum(value >= 65 for value in snapshot.probabilities)
        self.latest_occupancy_snapshot = snapshot
        self.map_pub.publish(message)

    def _publish_debug_markers(
        self,
        command,
        obstacle_points: Tuple[Tuple[float, float], ...],
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        def marker(namespace: str, marker_id: int, marker_type: int) -> Marker:
            item = Marker()
            item.header.frame_id = self.debug_frame
            item.header.stamp = stamp
            item.ns = namespace
            item.id = marker_id
            item.type = marker_type
            item.action = Marker.ADD
            item.pose.orientation.w = 1.0
            item.lifetime.sec = 0
            item.lifetime.nanosec = 850000000
            return item

        def text_marker(
            namespace: str,
            marker_id: int,
            text: str,
            x: float,
            y: float,
            z: float,
            color: Tuple[float, float, float],
            size: float = 0.52,
        ) -> Marker:
            item = marker(namespace, marker_id, Marker.TEXT_VIEW_FACING)
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = z
            item.scale.z = size
            item.color.r, item.color.g, item.color.b = color
            item.color.a = 1.0
            item.text = text
            return item

        vessel = marker('vessel_reference', 0, Marker.LINE_LIST)
        vessel.scale.x = 0.14
        vessel.color.r = 0.95
        vessel.color.g = 0.95
        vessel.color.b = 0.95
        vessel.color.a = 0.95
        hull_front = 2.35
        hull_rear = -2.35
        hull_half_width = 0.24
        for hull_center in (-1.05, 1.05):
            corners = (
                (hull_rear, hull_center - hull_half_width),
                (hull_front, hull_center - hull_half_width),
                (hull_front, hull_center + hull_half_width),
                (hull_rear, hull_center + hull_half_width),
            )
            for start, end in zip(corners, corners[1:] + corners[:1]):
                vessel.points.extend((
                    Point(x=start[0], y=start[1], z=2.25),
                    Point(x=end[0], y=end[1], z=2.25),
                ))
        # The bow chevron makes the body-frame +X direction unambiguous.
        for start, end in (
            ((2.85, 0.0), (2.25, 0.45)),
            ((2.85, 0.0), (2.25, -0.45)),
            ((-1.2, -1.05), (-1.2, 1.05)),
            ((1.2, -1.05), (1.2, 1.05)),
        ):
            vessel.points.extend((
                Point(x=start[0], y=start[1], z=2.25),
                Point(x=end[0], y=end[1], z=2.25),
            ))
        markers.markers.append(vessel)
        markers.markers.append(text_marker(
            'vessel_reference', 1, 'WAM-V / FRONT',
            3.15, 0.0, 2.9, (1.0, 1.0, 1.0), 0.48,
        ))

        limit_specs = (
            (
                self.controller.config.obstacle_safety_radius,
                (1.00, 0.18, 0.12),
                'SAFETY',
                -0.15,
            ),
            (
                self.controller.config.obstacle_emergency_distance,
                (1.00, 0.48, 0.08),
                'EMERGENCY',
                -0.75,
            ),
            (
                self.controller.config.obstacle_warning_distance,
                (1.00, 0.82, 0.10),
                'WARNING',
                -0.55,
            ),
            (
                self.controller.config.lattice_planning_horizon,
                (0.28, 0.62, 1.00),
                'PLANNING HORIZON',
                0.72,
            ),
        )
        for marker_id, (radius, color, label, label_angle) in enumerate(
            limit_specs
        ):
            limit = marker('navigation_limits', marker_id, Marker.LINE_STRIP)
            limit.scale.x = 0.12 if marker_id < 2 else 0.08
            limit.color.r, limit.color.g, limit.color.b = color
            limit.color.a = 0.72 if marker_id < 2 else 0.38
            for x, y in circle_points(radius):
                limit.points.append(Point(x=x, y=y, z=1.50))
            markers.markers.append(limit)
            if marker_id >= 2:
                markers.markers.append(text_marker(
                    'navigation_limits', 10 + marker_id,
                    f'{radius:g}m {label}',
                    (radius + 0.8) * math.cos(label_angle),
                    (radius + 0.8) * math.sin(label_angle),
                    2.45,
                    color,
                    0.46,
                ))

        waypoint_body_points = []
        if (
            self.latitude is not None
            and self.longitude is not None
            and self.yaw is not None
        ):
            for target in self.controller.targets:
                relative_east, relative_north = geodetic_delta_m(
                    self.latitude,
                    self.longitude,
                    target.latitude,
                    target.longitude,
                )
                waypoint_body_points.append(enu_offset_to_body(
                    relative_east, relative_north, self.yaw))

        if waypoint_body_points:
            route = marker('mission_route', 0, Marker.LINE_STRIP)
            route.scale.x = 0.14
            route.color.r = 0.62
            route.color.g = 0.70
            route.color.b = 0.82
            route.color.a = 0.55
            for forward, left in waypoint_body_points:
                route.points.append(Point(x=forward, y=left, z=1.64))
            markers.markers.append(route)

            completed_count = min(
                command.target_index, len(waypoint_body_points))
            if completed_count >= 2:
                completed_route = marker(
                    'mission_route', 1, Marker.LINE_STRIP)
                completed_route.scale.x = 0.24
                completed_route.color.r = 0.25
                completed_route.color.g = 0.95
                completed_route.color.b = 0.38
                completed_route.color.a = 0.75
                for forward, left in waypoint_body_points[:completed_count]:
                    completed_route.points.append(
                        Point(x=forward, y=left, z=1.68))
                markers.markers.append(completed_route)

            for index, ((forward, left), target) in enumerate(zip(
                waypoint_body_points, self.controller.targets
            )):
                visual = waypoint_visual_state(index, command.target_index)
                base_id = 100 * (index + 1)
                waypoint = marker(
                    'mission_waypoints', base_id, Marker.SPHERE)
                waypoint.pose.position.x = forward
                waypoint.pose.position.y = left
                waypoint.pose.position.z = 2.30
                waypoint.scale.x = visual.scale
                waypoint.scale.y = visual.scale
                waypoint.scale.z = 0.42
                waypoint.color.r, waypoint.color.g, waypoint.color.b = (
                    visual.color)
                waypoint.color.a = 0.98
                markers.markers.append(waypoint)

                capture_ring = marker(
                    'mission_waypoints', base_id + 1, Marker.LINE_STRIP)
                capture_ring.scale.x = 0.11
                capture_ring.color.r, capture_ring.color.g, (
                    capture_ring.color.b) = visual.color
                capture_ring.color.a = (
                    0.90 if visual.label == 'CURRENT' else 0.42)
                for x, y in circle_points(
                    self.controller.config.position_tolerance, 32
                ):
                    capture_ring.points.append(Point(
                        x=forward + x,
                        y=left + y,
                        z=1.78,
                    ))
                markers.markers.append(capture_ring)

                yaw_text = ''
                if target.yaw is not None:
                    relative_yaw = normalize_angle(target.yaw - self.yaw)
                    yaw_arrow = marker(
                        'mission_waypoints', base_id + 2, Marker.ARROW)
                    yaw_arrow.scale.x = 0.13
                    yaw_arrow.scale.y = 0.35
                    yaw_arrow.scale.z = 0.45
                    yaw_arrow.color.r, yaw_arrow.color.g, yaw_arrow.color.b = (
                        visual.color)
                    yaw_arrow.color.a = 0.88
                    yaw_arrow.points = [
                        Point(x=forward, y=left, z=2.45),
                        Point(
                            x=forward + 4.0 * math.cos(relative_yaw),
                            y=left + 4.0 * math.sin(relative_yaw),
                            z=2.45,
                        ),
                    ]
                    markers.markers.append(yaw_arrow)
                    if visual.label == 'CURRENT':
                        yaw_text = (
                            f'TARGET YAW '
                            f'{math.degrees(target.yaw):+.0f} deg')

                waypoint_label = f'WP {index + 1}/{len(waypoint_body_points)}'
                if visual.label != 'PENDING':
                    waypoint_label += f' | {visual.label}'
                if yaw_text:
                    waypoint_label += f'\n{yaw_text}'
                markers.markers.append(text_marker(
                    'mission_waypoints', base_id + 3,
                    waypoint_label,
                    forward,
                    left + 2.65,
                    3.20,
                    visual.color,
                    0.60 if visual.label == 'CURRENT' else 0.44,
                ))

                if visual.label == 'CURRENT':
                    exit_ring = marker(
                        'mission_waypoints', base_id + 4,
                        Marker.LINE_STRIP)
                    exit_ring.scale.x = 0.07
                    exit_ring.color.r, exit_ring.color.g, exit_ring.color.b = (
                        visual.color)
                    exit_ring.color.a = 0.35
                    for x, y in circle_points(
                        self.controller.config.waypoint_exit_tolerance, 40
                    ):
                        exit_ring.points.append(Point(
                            x=forward + x,
                            y=left + y,
                            z=1.74,
                        ))
                    markers.markers.append(exit_ring)

        planned_path = marker('planned_path', 0, Marker.LINE_STRIP)
        planned_path.scale.x = 0.38
        planned_path.color.r = 0.05
        planned_path.color.g = 0.95
        planned_path.color.b = 0.95
        planned_path.color.a = 1.0
        for forward, left in command.path_points_body:
            planned_path.points.append(Point(x=forward, y=left, z=1.8))
        markers.markers.append(planned_path)

        actual_track = marker('actual_trajectory', 0, Marker.LINE_STRIP)
        actual_track.scale.x = 0.34
        actual_track.color.r = 1.0
        actual_track.color.g = 0.76
        actual_track.color.b = 0.08
        actual_track.color.a = 1.0
        if (
            self.navigation_east is not None
            and self.navigation_north is not None
            and self.yaw is not None
        ):
            body_history = enu_history_to_body(
                self.trajectory_history,
                self.navigation_east,
                self.navigation_north,
                self.yaw,
            )
            for forward, left in body_history:
                actual_track.points.append(Point(x=forward, y=left, z=2.15))
        markers.markers.append(actual_track)

        cross_track = marker('cross_track_error', 0, Marker.LINE_LIST)
        cross_track.scale.x = 0.34
        cross_track.color.r = 1.0
        cross_track.color.g = 0.15
        cross_track.color.b = 0.80
        cross_track.color.a = 0.95
        if command.path_projection_body is not None:
            projection_forward, projection_left = command.path_projection_body
            cross_track.points = [
                Point(x=0.0, y=0.0, z=2.1),
                Point(x=projection_forward, y=projection_left, z=2.1),
            ]
        markers.markers.append(cross_track)
        if command.path_projection_body is not None:
            projection_forward, projection_left = command.path_projection_body
            projection = marker('cross_track_error', 1, Marker.SPHERE)
            projection.pose.position.x = projection_forward
            projection.pose.position.y = projection_left
            projection.pose.position.z = 2.2
            projection.scale.x = 0.72
            projection.scale.y = 0.72
            projection.scale.z = 0.30
            projection.color.r = 1.0
            projection.color.g = 0.15
            projection.color.b = 0.80
            projection.color.a = 1.0
            markers.markers.append(projection)
            markers.markers.append(text_marker(
                'cross_track_error', 2,
                f'XTE {abs(command.cross_track_error):.2f} m',
                0.5 * projection_forward,
                0.5 * projection_left - 0.65,
                3.0,
                (1.0, 0.15, 0.80),
                0.55,
            ))

        corridor = marker('safety_corridor', 0, Marker.LINE_STRIP)
        corridor.scale.x = 0.14
        corridor.color.r = 0.10
        corridor.color.g = 0.85
        corridor.color.b = 0.90
        corridor.color.a = 0.75
        half_width = self.controller.config.obstacle_path_half_width
        corridor_length = self.controller.config.obstacle_warning_distance
        for x, y in (
            (0.0, -half_width),
            (corridor_length, -half_width),
            (corridor_length, half_width),
            (0.0, half_width),
            (0.0, -half_width),
        ):
            corridor.points.append(Point(x=x, y=y, z=2.0))
        markers.markers.append(corridor)
        corridor_fill = marker('safety_corridor', 2, Marker.CUBE)
        corridor_fill.pose.position.x = corridor_length * 0.5
        corridor_fill.pose.position.z = 1.95
        corridor_fill.scale.x = corridor_length
        corridor_fill.scale.y = half_width * 2.0
        corridor_fill.scale.z = 0.04
        corridor_fill.color.r = 0.10
        corridor_fill.color.g = 0.85
        corridor_fill.color.b = 0.90
        corridor_fill.color.a = 0.10
        markers.markers.append(corridor_fill)

        target_relative = None
        if (
            self.latitude is not None
            and self.longitude is not None
            and self.yaw is not None
            and command.target_index < len(self.controller.targets)
        ):
            target = self.controller.targets[command.target_index]
            _, target_bearing = distance_and_bearing(
                self.latitude,
                self.longitude,
                target.latitude,
                target.longitude,
            )
            target_relative = normalize_angle(target_bearing - self.yaw)

        if target_relative is not None and command.state not in (
            'aligning', 'alignment_blocked', 'waypoint_dwell',
            'stationkeeping', 'complete',
        ):
            target_arrow = marker('target_direction', 0, Marker.ARROW)
            target_arrow.scale.x = 0.16
            target_arrow.scale.y = 0.40
            target_arrow.scale.z = 0.55
            target_arrow.color.r = 0.20
            target_arrow.color.g = 0.55
            target_arrow.color.b = 1.0
            target_arrow.color.a = 0.90
            target_arrow.points = [
                Point(x=0.0, y=0.0, z=2.65),
                Point(
                    x=13.0 * math.cos(target_relative),
                    y=13.0 * math.sin(target_relative),
                    z=2.65,
                ),
            ]
            markers.markers.append(target_arrow)
            target_number = min(command.target_index + 1, command.target_count)
            distance_text = (
                f'{command.distance:.1f} m'
                if math.isfinite(command.distance)
                else 'DISTANCE -'
            )
            markers.markers.append(text_marker(
                'target_direction', 1,
                f'GOAL {target_number}/{command.target_count} | {distance_text}\n'
                'TARGET DIRECTION',
                13.7 * math.cos(target_relative),
                13.7 * math.sin(target_relative),
                3.1,
                (0.20, 0.55, 1.0),
                0.62,
            ))

        if command.path_valid and command.state not in (
            'aligning', 'alignment_blocked', 'waypoint_dwell',
            'stationkeeping', 'complete',
        ):
            nominal_arrow = marker('ilos_nominal_heading', 0, Marker.ARROW)
            nominal_arrow.scale.x = 0.14
            nominal_arrow.scale.y = 0.34
            nominal_arrow.scale.z = 0.45
            nominal_arrow.color.r = 0.95
            nominal_arrow.color.g = 0.95
            nominal_arrow.color.b = 0.95
            nominal_arrow.color.a = 0.95
            nominal_length = 7.0
            nominal_arrow.points = [
                Point(x=0.0, y=0.0, z=2.45),
                Point(
                    x=nominal_length * math.cos(
                        command.nominal_heading_error),
                    y=nominal_length * math.sin(
                        command.nominal_heading_error),
                    z=2.45,
                ),
            ]
            markers.markers.append(nominal_arrow)
            markers.markers.append(text_marker(
                'ilos_nominal_heading', 1, 'ILOS NOMINAL COURSE',
                nominal_arrow.points[1].x,
                nominal_arrow.points[1].y,
                3.0,
                (0.95, 0.95, 0.95),
                0.38,
            ))

        selected_arrow = marker('selected_heading', 0, Marker.ARROW)
        selected_arrow.scale.x = 0.22
        selected_arrow.scale.y = 0.52
        selected_arrow.scale.z = 0.68
        selected_arrow.color.a = 1.0
        if command.state in ('braking', 'pivoting', 'alignment_blocked'):
            selected_arrow.color.r = 1.0
            selected_arrow.color.g = 0.20
            selected_arrow.color.b = 0.10
        elif command.state == 'aligning':
            selected_arrow.color.r = 1.0
            selected_arrow.color.g = 0.55
            selected_arrow.color.b = 0.10
        elif command.state == 'backing_away':
            selected_arrow.color.r = 0.95
            selected_arrow.color.g = 0.20
            selected_arrow.color.b = 0.90
        elif command.state in ('avoiding', 'curve_braking'):
            selected_arrow.color.r = 1.0
            selected_arrow.color.g = 0.75
            selected_arrow.color.b = 0.10
        else:
            selected_arrow.color.r = 0.20
            selected_arrow.color.g = 1.0
            selected_arrow.color.b = 0.35
        selected_arrow.points = [
            Point(x=0.0, y=0.0, z=2.95),
            Point(
                x=11.0 * math.cos(command.heading_error),
                y=11.0 * math.sin(command.heading_error),
                z=2.95,
            ),
        ]
        markers.markers.append(selected_arrow)
        selected_color = (
            selected_arrow.color.r,
            selected_arrow.color.g,
            selected_arrow.color.b,
        )
        turn_direction = 'LEFT' if command.heading_error >= 0.0 else 'RIGHT'
        if command.state == 'aligning':
            control_text = (
                f'FINAL HEADING ALIGNMENT: {turn_direction} '
                f'{abs(math.degrees(command.heading_error)):.0f} deg'
            )
        elif command.state == 'alignment_blocked':
            control_text = 'FINAL ALIGNMENT BLOCKED: OBSTACLE TOO CLOSE'
        elif command.state == 'waypoint_dwell':
            control_text = 'GOAL CAPTURED: HOLDING'
        elif command.state == 'complete':
            control_text = 'COURSE COMPLETE'
        elif command.state == 'avoiding':
            control_text = (
                f'CONTROL: AVOID {turn_direction} '
                f'{abs(math.degrees(command.heading_error)):.0f} deg'
            )
        elif command.state == 'curve_braking':
            control_text = 'CONTROL: CURVE SPEED BRAKING'
        elif command.state == 'pivoting':
            control_text = f'CONTROL: PIVOT {turn_direction}'
        elif command.state == 'braking':
            control_text = 'CONTROL: BRAKE'
        elif command.state == 'backing_away':
            control_text = f'CONTROL: REVERSE / TURN {turn_direction}'
        else:
            control_text = 'CONTROL: CRUISE'
        markers.markers.append(text_marker(
            'selected_heading', 1, control_text,
            6.8 * math.cos(command.heading_error),
            6.8 * math.sin(command.heading_error),
            3.45,
            selected_color,
            0.48,
        ))

        if (
            self.navigation_east is not None
            and self.navigation_north is not None
            and self.yaw is not None
        ):
            cosine = math.cos(self.yaw)
            sine = math.sin(self.yaw)
            risky_track_id = (
                self.current_encounter.track_id
                if self.current_encounter is not None else None)
            for track in self.dynamic_tracker.active_tracks(time.monotonic()):
                relative_east = track.east - self.navigation_east
                relative_north = track.north - self.navigation_north
                forward = relative_east * cosine + relative_north * sine
                left = -relative_east * sine + relative_north * cosine
                velocity_forward = (
                    track.velocity_east * cosine
                    + track.velocity_north * sine)
                velocity_left = (
                    -track.velocity_east * sine
                    + track.velocity_north * cosine)
                risky = track.track_id == risky_track_id

                vessel_track = marker(
                    'dynamic_vessels', 10 * track.track_id, Marker.CUBE)
                vessel_track.pose.position.x = forward
                vessel_track.pose.position.y = left
                vessel_track.pose.position.z = 2.6
                relative_course = math.atan2(
                    velocity_left, velocity_forward)
                vessel_track.pose.orientation.z = math.sin(
                    0.5 * relative_course)
                vessel_track.pose.orientation.w = math.cos(
                    0.5 * relative_course)
                vessel_track.scale.x = 7.0
                vessel_track.scale.y = 2.8
                vessel_track.scale.z = 0.8
                vessel_track.color.r = 1.0
                vessel_track.color.g = 0.12 if risky else 0.55
                vessel_track.color.b = 0.05
                vessel_track.color.a = 0.90
                markers.markers.append(vessel_track)

                velocity_arrow = marker(
                    'dynamic_vessels', 10 * track.track_id + 1, Marker.ARROW)
                velocity_arrow.scale.x = 0.28
                velocity_arrow.scale.y = 0.70
                velocity_arrow.scale.z = 0.80
                velocity_arrow.color.r = vessel_track.color.r
                velocity_arrow.color.g = vessel_track.color.g
                velocity_arrow.color.b = vessel_track.color.b
                velocity_arrow.color.a = 1.0
                velocity_arrow.points = [
                    Point(x=forward, y=left, z=3.2),
                    Point(
                        x=forward + 8.0 * velocity_forward,
                        y=left + 8.0 * velocity_left,
                        z=3.2,
                    ),
                ]
                markers.markers.append(velocity_arrow)

                encounter_text = ''
                if risky and self.current_encounter is not None:
                    tcpa = self.current_encounter.tcpa_s
                    dcpa = self.current_encounter.dcpa_m
                    encounter_text = (
                        f'\n{self.current_encounter.encounter.upper()} | '
                        f'TCPA {tcpa:.0f}s DCPA {dcpa:.1f}m'
                        if tcpa is not None and dcpa is not None else '')
                markers.markers.append(text_marker(
                    'dynamic_vessels', 10 * track.track_id + 2,
                    f'DYNAMIC VESSEL {track.track_id} | '
                    f'{track.speed:.1f} m/s{encounter_text}',
                    forward,
                    left + 2.5,
                    4.0,
                    (
                        vessel_track.color.r,
                        vessel_track.color.g,
                        vessel_track.color.b,
                    ),
                    0.52,
                ))
                if risky:
                    risk_zone = marker(
                        'dynamic_vessels',
                        10 * track.track_id + 3,
                        Marker.CYLINDER,
                    )
                    risk_zone.pose.position.x = forward
                    risk_zone.pose.position.y = left
                    risk_zone.pose.position.z = 1.75
                    risk_zone.scale.x = 2.0 * self.colregs_safety_radius
                    risk_zone.scale.y = 2.0 * self.colregs_safety_radius
                    risk_zone.scale.z = 0.05
                    risk_zone.color.r = 1.0
                    risk_zone.color.g = 0.10
                    risk_zone.color.b = 0.05
                    risk_zone.color.a = 0.10
                    markers.markers.append(risk_zone)

        actual_speed_arrow = marker('motion_vectors', 0, Marker.ARROW)
        actual_speed_arrow.scale.x = 0.14
        actual_speed_arrow.scale.y = 0.38
        actual_speed_arrow.scale.z = 0.48
        actual_speed_arrow.color.r = 0.20
        actual_speed_arrow.color.g = 1.00
        actual_speed_arrow.color.b = 0.42
        actual_speed_arrow.color.a = 0.95
        actual_speed_length = 4.0 * self.speed
        actual_speed_arrow.points = [
            Point(x=0.0, y=-1.55, z=2.75),
            Point(x=actual_speed_length, y=-1.55, z=2.75),
        ]
        markers.markers.append(actual_speed_arrow)

        desired_speed_arrow = marker('motion_vectors', 1, Marker.ARROW)
        desired_speed_arrow.scale.x = 0.13
        desired_speed_arrow.scale.y = 0.34
        desired_speed_arrow.scale.z = 0.44
        desired_speed_arrow.color.r = 0.20
        desired_speed_arrow.color.g = 0.62
        desired_speed_arrow.color.b = 1.00
        desired_speed_arrow.color.a = 0.92
        desired_speed_length = 4.0 * command.desired_speed
        desired_speed_arrow.points = [
            Point(x=0.0, y=1.55, z=2.72),
            Point(
                x=desired_speed_length * math.cos(command.heading_error),
                y=(
                    1.55
                    + desired_speed_length * math.sin(command.heading_error)
                ),
                z=2.72,
            ),
        ]
        markers.markers.append(desired_speed_arrow)

        if abs(command.desired_yaw_rate) > math.radians(0.05):
            yaw_direction = math.copysign(1.0, command.desired_yaw_rate)
            yaw_limit = max(
                math.radians(0.1),
                self.controller.config.max_navigation_yaw_rate,
            )
            yaw_scale = min(1.0, abs(command.desired_yaw_rate) / yaw_limit)
            yaw_arrow = marker('motion_vectors', 2, Marker.ARROW)
            yaw_arrow.scale.x = 0.12
            yaw_arrow.scale.y = 0.34
            yaw_arrow.scale.z = 0.44
            yaw_arrow.color.r = 0.85
            yaw_arrow.color.g = 0.35
            yaw_arrow.color.b = 1.00
            yaw_arrow.color.a = 0.95
            yaw_arrow.points = [
                Point(x=2.75, y=0.0, z=3.00),
                Point(
                    x=2.75,
                    y=yaw_direction * (1.8 + 2.7 * yaw_scale),
                    z=3.00,
                ),
            ]
            markers.markers.append(yaw_arrow)

        turn_word = (
            'LEFT' if command.desired_yaw_rate > 0.0
            else 'RIGHT' if command.desired_yaw_rate < 0.0
            else 'STRAIGHT'
        )
        markers.markers.append(text_marker(
            'motion_vectors', 3,
            f'MOTION | GREEN ACTUAL / BLUE TARGET\n'
            f'SPEED {self.speed:+.2f} / {command.desired_speed:+.2f} m/s | '
            f'YAW {math.degrees(self.yaw_rate):+.1f} / '
            f'{math.degrees(command.desired_yaw_rate):+.1f} deg/s {turn_word}',
            8.0,
            -18.0,
            3.45,
            (0.82, 0.88, 1.00),
            0.42,
        ))

        for marker_id, (name, thrust, lateral) in enumerate((
            ('left', command.left_thrust, 1.0),
            ('right', command.right_thrust, -1.0),
        )):
            thrust_arrow = marker('thrusters', marker_id, Marker.ARROW)
            thrust_arrow.scale.x = 0.12
            thrust_arrow.scale.y = 0.28
            thrust_arrow.scale.z = 0.30
            thrust_arrow.color.a = 0.95
            if thrust >= 0.0:
                thrust_arrow.color.r = 0.15
                thrust_arrow.color.g = 0.95
                thrust_arrow.color.b = 0.30
            else:
                thrust_arrow.color.r = 1.0
                thrust_arrow.color.g = 0.20
                thrust_arrow.color.b = 0.10
            length = 2.5 * thrust / max(1.0, self.controller.config.max_thrust)
            thrust_arrow.points = [
                Point(x=-2.2, y=lateral, z=2.55),
                Point(x=-2.2 + length, y=lateral, z=2.55),
            ]
            markers.markers.append(thrust_arrow)
            markers.markers.append(text_marker(
                'thrusters', marker_id + 2,
                f'{name.upper()} {thrust:.0f}',
                -2.8,
                lateral,
                3.05,
                (
                    thrust_arrow.color.r,
                    thrust_arrow.color.g,
                    thrust_arrow.color.b,
                ),
                0.38,
            ))

        tracking = tracking_statistics(
            self.tracking_error_history,
            1e-9 * float(self.get_clock().now().nanoseconds),
        )
        safety_active = bool(
            command.avoidance_override
            or command.avoidance_episode_active
            or command.colregs_active
            or command.state in (
                'pivoting', 'backing_away', 'alignment_blocked')
        )
        terminal_active = command.state in (
            'approach_braking', 'braking', 'aligning',
            'alignment_blocked',
            'waypoint_dwell', 'stationkeeping', 'complete',
        )
        quality = tracking_quality(
            command.path_valid,
            abs(command.cross_track_error),
            tracking,
            safety_active,
            terminal_active,
        )
        status = marker('controller_status', 0, Marker.TEXT_VIEW_FACING)
        status.pose.position.x = -5.0
        status.pose.position.y = 18.0
        status.pose.position.z = 4.0
        status.scale.z = 0.56
        status.color.r, status.color.g, status.color.b = quality.color
        status.color.a = 1.0
        target_number = min(command.target_index + 1, command.target_count)
        nearest = (
            f'{command.nearest_obstacle:.1f}m'
            if math.isfinite(command.nearest_obstacle)
            else '-'
        )
        collision = (
            f'{command.collision_clearance:.1f}m'
            if math.isfinite(command.collision_clearance)
            else '-'
        )
        distance = (
            f'{command.distance:.1f} m'
            if math.isfinite(command.distance)
            else '-'
        )
        mean_error = (
            f'{tracking.mean_abs_m:.2f}'
            if tracking.mean_abs_m is not None else '-')
        max_error = (
            f'{tracking.max_abs_m:.2f}'
            if tracking.max_abs_m is not None else '-')
        status.text = (
            f'TRACKING: {quality.label} | '
            f'XTE: {abs(command.cross_track_error):.2f} m\n'
            f'NORMAL 20s: AVG {mean_error} m | MAX {max_error} m\n'
            f'STATE: {command.state.upper()} | '
            f'GOAL: {target_number}/{command.target_count} | '
            f'DIST: {distance}\n'
            f'DEV: {command.path_deviation:.2f} m | '
            f'I: {command.ilos_integral_bias:+.2f} m | '
            f'CURVE: {command.path_curvature:+.3f} 1/m\n'
            f'CLEARANCE: {collision} | NEAREST: {nearest} | '
            f'HITS: {self.num_collisions}'
        )
        markers.markers.append(status)

        debug_now = time.monotonic()

        def sample_age(timestamp: Optional[float]) -> Optional[float]:
            if timestamp is None:
                return None
            return max(0.0, debug_now - timestamp)

        def age_text(age_s: Optional[float]) -> str:
            return '-' if age_s is None else f'{age_s:.2f}s'

        gps_age = sample_age(self.last_gps_time)
        imu_age = sample_age(self.last_imu_time)
        scan_age = sample_age(self.last_scan_time)
        cloud_age = sample_age(self.last_cloud_time)
        gps_health = freshness_state(gps_age, self.sensor_timeout)
        imu_health = freshness_state(imu_age, self.sensor_timeout)
        scan_health = freshness_state(scan_age, self.scan_cache_timeout)
        cloud_health = freshness_state(cloud_age, self.scan_cache_timeout)
        lidar_healthy = 'OK' in (scan_health, cloud_health)
        localization_healthy = bool(
            (
                self.estimator_source == 'robot_localization'
                and self.robot_localization_healthy
            )
            or (
                self.estimator_source == 'custom_ekf'
                and (self.estimator_healthy or not self.ekf_enabled)
            )
        )
        critical_stale = bool(
            'STALE' in (gps_health, imu_health)
            or (
                not lidar_healthy
                and 'STALE' in (scan_health, cloud_health)
            )
            or self.control_conflict
        )
        critical_waiting = bool(
            'WAIT' in (gps_health, imu_health)
            or not lidar_healthy
            or not localization_healthy
        )
        if critical_stale:
            health_label = 'FAULT'
            health_color = (1.00, 0.22, 0.12)
        elif critical_waiting:
            health_label = 'WARMING / DEGRADED'
            health_color = (1.00, 0.74, 0.10)
        else:
            health_label = 'READY'
            health_color = (0.24, 1.00, 0.40)
        position_std = (
            f'{self.estimator_position_std:.2f}m'
            if self.estimator_position_std is not None
            else '-'
        )
        markers.markers.append(text_marker(
            'sensor_health', 0,
            f'SYSTEM HEALTH: {health_label}\n'
            f'GPS [{gps_health}] {age_text(gps_age)} | '
            f'IMU [{imu_health}] {age_text(imu_age)}\n'
            f'SCAN [{scan_health}] {age_text(scan_age)} | '
            f'CLOUD [{cloud_health}] {age_text(cloud_age)}\n'
            f'LOCALIZATION [{"OK" if localization_healthy else "WARN"}] '
            f'{self.estimator_source.upper()} | POS STD {position_std}',
            -5.0,
            -18.0,
            4.1,
            health_color,
            0.46,
        ))

        terminal_states = (
            'approach_braking', 'braking', 'aligning',
            'alignment_blocked', 'waypoint_dwell',
            'stationkeeping', 'complete',
        )
        if command.lattice_fallback:
            planner_label = 'FALLBACK'
            planner_color = (1.00, 0.42, 0.10)
        elif command.path_valid:
            planner_label = 'PATH VALID'
            planner_color = (0.20, 0.90, 1.00)
        elif command.state in terminal_states:
            planner_label = 'GOAL HOLD'
            planner_color = (0.35, 0.72, 1.00)
        else:
            planner_label = 'WAITING FOR PATH'
            planner_color = (1.00, 0.74, 0.10)
        map_label = (
            'OK' if self.map_enabled and self.map_known_cells > 0
            else 'WARM' if self.map_enabled
            else 'OFF'
        )
        path_remaining = (
            f'{command.path_remaining:.1f}m'
            if math.isfinite(command.path_remaining)
            else '-'
        )
        replan_label = (
            command.guidance_replan_reason.upper()
            if command.guidance_replan_reason else 'NONE'
        )
        markers.markers.append(text_marker(
            'planner_health', 0,
            f'PLANNER: {planner_label} | {command.guidance_mode.upper()}\n'
            f'MAP [{map_label}] REV {self.occupancy_grid.revision} | '
            f'{self.map_known_cells} KNOWN / {self.map_occupied_cells} OCC\n'
            f'PATH REV {command.path_revision} | REMAIN {path_remaining} | '
            f'REPLAN {replan_label}\n'
            f'LATTICE {command.lattice_expanded_states} STATES / '
            f'{command.lattice_planning_time_ms:.1f} ms | '
            f'FALLBACK {"YES" if command.lattice_fallback else "NO"}',
            23.0,
            -18.0,
            4.1,
            planner_color,
            0.44,
        ))
        markers.markers.append(text_marker(
            'sensor_legend', 0,
            'VIEW: VESSEL-FIXED / FRONT = +X\n'
            'CYAN = LOCAL PLANNED PATH\n'
            'YELLOW = ACTUAL / CURRENT REV\n'
            'GREEN / BLUE / GRAY WP = DONE / CURRENT / PENDING\n'
            'RINGS: RED 3m / ORANGE 5.5m / YELLOW 22m / BLUE 40m\n'
            'MAGENTA = CURRENT XTE | ORANGE BOX = DYNAMIC VESSEL',
            23.0,
            18.0,
            4.0,
            (0.90, 0.95, 1.0),
            0.42,
        ))
        self.debug_marker_pub.publish(markers)

    def _buoy_candidate_points(
        self,
        obstacle_points: Tuple[Tuple[float, float], ...],
        now: float,
    ) -> Tuple[Tuple[float, float], ...]:
        dynamic_points = []
        if (
            self.navigation_east is not None
            and self.navigation_north is not None
            and self.yaw is not None
        ):
            cosine = math.cos(self.yaw)
            sine = math.sin(self.yaw)
            for track in self.dynamic_tracker.active_tracks(now):
                relative_east = track.east - self.navigation_east
                relative_north = track.north - self.navigation_north
                dynamic_points.append((
                    relative_east * cosine + relative_north * sine,
                    -relative_east * sine + relative_north * cosine,
                ))
        return filter_buoy_candidates(
            obstacle_points,
            dynamic_points,
            self.colregs_map_mask_radius,
            (self.cloud_sensor_offset_x, self.cloud_sensor_offset_y),
        )

    def _publish_buoy_candidate_markers(
        self,
        candidates: Tuple[Tuple[float, float], ...],
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        for marker_id in stale_buoy_marker_ids(
            self.previous_buoy_candidate_marker_count,
            len(candidates),
        ):
            stale = Marker()
            stale.header.frame_id = self.debug_frame
            stale.header.stamp = stamp
            stale.ns = 'buoy_candidates'
            stale.id = marker_id
            stale.action = Marker.DELETE
            markers.markers.append(stale)

        for index, (distance, angle) in enumerate(candidates, start=1):
            forward = self.cloud_sensor_offset_x + distance * math.cos(angle)
            left = self.cloud_sensor_offset_y + distance * math.sin(angle)

            ring = Marker()
            ring.header.frame_id = self.debug_frame
            ring.header.stamp = stamp
            ring.ns = 'buoy_candidates'
            ring.id = 10 * index
            ring.type = Marker.LINE_STRIP
            ring.action = Marker.ADD
            ring.pose.orientation.w = 1.0
            ring.scale.x = 0.24
            ring.color.r = 1.0
            ring.color.g = 0.88
            ring.color.b = 0.05
            ring.color.a = 1.0
            ring.lifetime.nanosec = 850000000
            radius = 1.1
            for step in range(25):
                phase = 2.0 * math.pi * step / 24.0
                ring.points.append(Point(
                    x=forward + radius * math.cos(phase),
                    y=left + radius * math.sin(phase),
                    z=2.65,
                ))
            markers.markers.append(ring)

            center = Marker()
            center.header.frame_id = self.debug_frame
            center.header.stamp = stamp
            center.ns = 'buoy_candidates'
            center.id = 10 * index + 1
            center.type = Marker.SPHERE
            center.action = Marker.ADD
            center.pose.position.x = forward
            center.pose.position.y = left
            center.pose.position.z = 2.65
            center.pose.orientation.w = 1.0
            center.scale.x = 0.72
            center.scale.y = 0.72
            center.scale.z = 0.34
            center.color.r = 1.0
            center.color.g = 0.88
            center.color.b = 0.05
            center.color.a = 0.95
            center.lifetime.nanosec = 850000000
            markers.markers.append(center)

            label = Marker()
            label.header.frame_id = self.debug_frame
            label.header.stamp = stamp
            label.ns = 'buoy_candidates'
            label.id = 10 * index + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = forward
            label.pose.position.y = left + (-1.35 if left >= 0.0 else 1.35)
            label.pose.position.z = 3.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.48
            label.color.r = 1.0
            label.color.g = 0.92
            label.color.b = 0.20
            label.color.a = 1.0
            label.text = f'BUOY #{index} | {distance:.1f} m'
            label.lifetime.nanosec = 850000000
            markers.markers.append(label)

        self.buoy_marker_pub.publish(markers)
        self.previous_buoy_candidate_marker_count = len(candidates)

    @staticmethod
    def _publish_float(publisher, value: float) -> None:
        message = Float64()
        message.data = float(value)
        publisher.publish(message)

    def stop_thrusters(self) -> None:
        try:
            for _ in range(3):
                self._publish_float(self.left_pub, 0.0)
                self._publish_float(self.right_pub, 0.0)
        except Exception:
            # The ROS context may already be invalid during external shutdown.
            pass


def main(args=None) -> None:
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = AutonomousUSVNode()

    def request_shutdown(_signum, _frame):
        node.stop_thrusters()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_thrusters()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
