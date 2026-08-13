"""Convert VRX GNSS fixes into local ENU odometry for robot_localization."""

from dataclasses import dataclass
import copy
import math
from typing import Optional, Tuple

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from .core import geodetic_delta_m


@dataclass(frozen=True)
class ProjectedFix:
    east: float
    north: float
    velocity_east: float
    velocity_north: float
    position_variances: Tuple[float, float]
    velocity_variance: float


class GnssProjector:
    """Project WGS84 fixes and derive a bounded, low-pass ENU velocity."""

    def __init__(
        self,
        velocity_smoothing: float = 0.25,
        max_speed: float = 15.0,
        default_position_std: float = 1.5,
        default_velocity_std: float = 0.8,
    ) -> None:
        self.velocity_smoothing = min(1.0, max(0.0, velocity_smoothing))
        self.max_speed = max(0.1, float(max_speed))
        self.default_position_std = max(0.01, float(default_position_std))
        self.default_velocity_std = max(0.01, float(default_velocity_std))
        self.reset()

    def reset(self) -> None:
        self.origin: Optional[Tuple[float, float]] = None
        self.previous: Optional[Tuple[float, float, float]] = None
        self.velocity_east = 0.0
        self.velocity_north = 0.0

    def update(
        self,
        latitude: float,
        longitude: float,
        timestamp: float,
        position_variances: Optional[Tuple[float, float]] = None,
    ) -> Optional[ProjectedFix]:
        if not all(math.isfinite(value) for value in (
            latitude, longitude, timestamp,
        )):
            return None
        if self.origin is None:
            self.origin = (latitude, longitude)
        east, north = geodetic_delta_m(
            self.origin[0], self.origin[1], latitude, longitude)

        velocity_variance = self.default_velocity_std ** 2
        if self.previous is not None:
            previous_east, previous_north, previous_time = self.previous
            dt = timestamp - previous_time
            if 0.05 <= dt <= 3.0:
                measured_east = (east - previous_east) / dt
                measured_north = (north - previous_north) / dt
                if math.hypot(measured_east, measured_north) <= self.max_speed:
                    alpha = self.velocity_smoothing
                    self.velocity_east += alpha * (
                        measured_east - self.velocity_east)
                    self.velocity_north += alpha * (
                        measured_north - self.velocity_north)
                    # Differentiated GNSS is noisier than position. Inflate it
                    # when samples are close together instead of over-trusting it.
                    velocity_variance = max(
                        velocity_variance,
                        sum(position_variances or (0.0, 0.0))
                        / max(dt * dt, 0.01),
                    )
        self.previous = (east, north, timestamp)
        default_variance = self.default_position_std ** 2
        east_variance, north_variance = position_variances or (
            default_variance, default_variance)
        return ProjectedFix(
            east=east,
            north=north,
            velocity_east=self.velocity_east,
            velocity_north=self.velocity_north,
            position_variances=(
                max(1e-6, east_variance), max(1e-6, north_variance)),
            velocity_variance=max(1e-6, velocity_variance),
        )


class GnssOdometryNode(Node):
    """ROS adapter providing position, body velocity, IMU, and datum topics."""

    def __init__(self) -> None:
        super().__init__('gnss_odometry_adapter')
        self.declare_parameter('gps_topic', '/wamv/sensors/gps/gps/fix')
        self.declare_parameter('imu_topic', '/wamv/sensors/imu/imu/data')
        self.declare_parameter('odom_topic', '/autonomous_usv/gps_odometry')
        self.declare_parameter('imu_output_topic', '/autonomous_usv/imu')
        self.declare_parameter('origin_topic', '/autonomous_usv/gps_origin')
        self.declare_parameter('odom_frame', 'han_usv_odom')
        self.declare_parameter('base_link_frame', 'wamv/wamv/base_link')
        self.declare_parameter('velocity_smoothing', 0.25)
        self.declare_parameter('max_speed', 15.0)
        self.declare_parameter('default_position_std', 1.5)
        self.declare_parameter('default_velocity_std', 0.8)

        self.projector = GnssProjector(
            velocity_smoothing=float(
                self.get_parameter('velocity_smoothing').value),
            max_speed=float(self.get_parameter('max_speed').value),
            default_position_std=float(
                self.get_parameter('default_position_std').value),
            default_velocity_std=float(
                self.get_parameter('default_velocity_std').value),
        )
        self.latest_yaw: Optional[float] = None
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_link_frame = str(self.get_parameter('base_link_frame').value)
        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter('odom_topic').value), 10)
        self.imu_pub = self.create_publisher(
            Imu, str(self.get_parameter('imu_output_topic').value),
            qos_profile_sensor_data)
        origin_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.origin_pub = self.create_publisher(
            NavSatFix, str(self.get_parameter('origin_topic').value), origin_qos)
        self.create_subscription(
            NavSatFix, str(self.get_parameter('gps_topic').value),
            self._gps_callback, qos_profile_sensor_data)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value),
            self._imu_callback, qos_profile_sensor_data)

    @staticmethod
    def _stamp_seconds(message) -> float:
        return float(message.sec) + 1e-9 * float(message.nanosec)

    @staticmethod
    def _yaw(message: Imu) -> Optional[float]:
        q = message.orientation
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            return None
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 0.5 or norm > 1.5:
            return None
        x, y, z, w = (value / norm for value in values)
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    def _imu_callback(self, message: Imu) -> None:
        yaw = self._yaw(message)
        if yaw is not None:
            self.latest_yaw = yaw
        output = copy.deepcopy(message)
        output.header.frame_id = self.base_link_frame
        self.imu_pub.publish(output)

    def _gps_callback(self, message: NavSatFix) -> None:
        if message.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        timestamp = self._stamp_seconds(message.header.stamp)
        if timestamp <= 0.0:
            timestamp = 1e-9 * float(self.get_clock().now().nanoseconds)
        covariance = message.position_covariance
        position_variances = None
        if (
            len(covariance) >= 5
            and math.isfinite(covariance[0]) and covariance[0] > 0.0
            and math.isfinite(covariance[4]) and covariance[4] > 0.0
        ):
            position_variances = (float(covariance[0]), float(covariance[4]))
        projected = self.projector.update(
            float(message.latitude), float(message.longitude), timestamp,
            position_variances)
        if projected is None or self.projector.origin is None:
            return

        origin = copy.deepcopy(message)
        origin.latitude, origin.longitude = self.projector.origin
        origin.altitude = 0.0
        self.origin_pub.publish(origin)

        odometry = Odometry()
        odometry.header = copy.deepcopy(message.header)
        odometry.header.frame_id = self.odom_frame
        odometry.child_frame_id = self.base_link_frame
        odometry.pose.pose.position.x = projected.east
        odometry.pose.pose.position.y = projected.north
        odometry.pose.pose.orientation.w = 1.0
        odometry.pose.covariance[0] = projected.position_variances[0]
        odometry.pose.covariance[7] = projected.position_variances[1]
        odometry.pose.covariance[14] = 1e6
        odometry.pose.covariance[21] = 1e6
        odometry.pose.covariance[28] = 1e6
        odometry.pose.covariance[35] = 1e6
        if self.latest_yaw is not None:
            cos_yaw = math.cos(self.latest_yaw)
            sin_yaw = math.sin(self.latest_yaw)
            odometry.twist.twist.linear.x = (
                projected.velocity_east * cos_yaw
                + projected.velocity_north * sin_yaw)
            odometry.twist.twist.linear.y = (
                -projected.velocity_east * sin_yaw
                + projected.velocity_north * cos_yaw)
            odometry.twist.covariance[0] = projected.velocity_variance
            odometry.twist.covariance[7] = projected.velocity_variance
        else:
            odometry.twist.covariance[0] = 1e6
            odometry.twist.covariance[7] = 1e6
        self.odom_pub.publish(odometry)


def main() -> None:
    rclpy.init()
    node = GnssOdometryNode()
    try:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
