"""Move a visible Gazebo target and publish its idealized ENU detections."""

import math
from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


@dataclass(frozen=True)
class TargetPose:
    x: float
    y: float
    yaw: float


def target_pose(
    start_x: float,
    start_y: float,
    velocity_x: float,
    velocity_y: float,
    elapsed: float,
    start_delay: float,
    duration: float,
) -> TargetPose:
    moving_elapsed = min(
        max(0.0, duration), max(0.0, elapsed - max(0.0, start_delay)))
    yaw = math.atan2(velocity_y, velocity_x)
    return TargetPose(
        start_x + velocity_x * moving_elapsed,
        start_y + velocity_y * moving_elapsed,
        yaw,
    )


class MovingTargetNode(Node):
    def __init__(self) -> None:
        super().__init__('moving_target')
        defaults = {
            'use_sim_time': True,
            'world_name': 'wayfinding_task',
            'entity_name': 'codex_target_vessel',
            'target_topic': '/autonomous_usv/dynamic_targets',
            'frame_id': 'codex_odom',
            'world_origin_x': -532.0,
            'world_origin_y': 162.0,
            'start_x': -500.0,
            'start_y': 205.0,
            'start_z': 0.7,
            'velocity_x': -1.0,
            'velocity_y': 0.0,
            'start_delay': 15.0,
            'motion_duration': 90.0,
            'publish_rate_hz': 10.0,
        }
        for name, value in defaults.items():
            if name == 'use_sim_time' and self.has_parameter(name):
                continue
            self.declare_parameter(name, value)
        self.world_name = str(self.get_parameter('world_name').value)
        self.entity_name = str(self.get_parameter('entity_name').value)
        self.origin_x = float(self.get_parameter('world_origin_x').value)
        self.origin_y = float(self.get_parameter('world_origin_y').value)
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.start_z = float(self.get_parameter('start_z').value)
        self.velocity_x = float(self.get_parameter('velocity_x').value)
        self.velocity_y = float(self.get_parameter('velocity_y').value)
        self.start_delay = max(0.0, float(
            self.get_parameter('start_delay').value))
        self.motion_duration = max(1.0, float(
            self.get_parameter('motion_duration').value))
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publisher = self.create_publisher(
            PoseArray, str(self.get_parameter('target_topic').value), 10)
        self.pose_client = self.create_client(
            SetEntityPose, f'/world/{self.world_name}/set_pose')
        self.started_at: Optional[float] = None
        self.pending_pose = None
        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._update)

    def _update(self) -> None:
        now = 1e-9 * float(self.get_clock().now().nanoseconds)
        if now <= 0.0:
            return
        if self.started_at is None:
            self.started_at = now
        elapsed = max(0.0, now - self.started_at)
        target = target_pose(
            self.start_x,
            self.start_y,
            self.velocity_x,
            self.velocity_y,
            elapsed,
            self.start_delay,
            self.motion_duration,
        )
        world_x, world_y, yaw = target.x, target.y, target.yaw

        detection = PoseArray()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.header.frame_id = self.frame_id
        pose = Pose()
        pose.position.x = world_x - self.origin_x
        pose.position.y = world_y - self.origin_y
        pose.orientation.z = math.sin(0.5 * yaw)
        pose.orientation.w = math.cos(0.5 * yaw)
        detection.poses.append(pose)
        self.publisher.publish(detection)

        if not self.pose_client.service_is_ready():
            return
        if self.pending_pose is not None and not self.pending_pose.done():
            return
        request = SetEntityPose.Request()
        request.entity.name = self.entity_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = world_x
        request.pose.position.y = world_y
        request.pose.position.z = self.start_z
        request.pose.orientation.z = math.sin(0.5 * yaw)
        request.pose.orientation.w = math.cos(0.5 * yaw)
        self.pending_pose = self.pose_client.call_async(request)


def main() -> None:
    rclpy.init()
    node = MovingTargetNode()
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
            rclpy.shutdown()
