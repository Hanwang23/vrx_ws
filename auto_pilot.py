#!/usr/bin/env python3
"""
WAM-V 自动前进测试：直接发布左右推进器命令。

运行前请只保留一个控制源：不要同时运行 usv_joy_teleop.py、virtual_joystick.py
或其它 /wamv/thrusters/* 发布节点。
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class AutoPilot(Node):
    def __init__(self):
        super().__init__('auto_pilot')

        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.left_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/left/pos', 10)
        self.right_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/right/pos', 10)

        self.start_time = time.monotonic()
        self.duration = 300.0
        self.thrust_value = -2353.0
        self.last_print = -1
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

        self.get_logger().info('=== WAM-V 自动前进测试 ===')
        self.get_logger().info(f'前进推力 {self.thrust_value:.0f}，持续 {self.duration:.0f} 秒，Ctrl+C 可停止')

    def timer_callback(self):
        elapsed = time.monotonic() - self.start_time
        if elapsed > self.duration:
            self.stop_thrusters()
            self.get_logger().info('测试完成，已停止推进器')
            rclpy.shutdown()
            return

        self.publish_float(self.left_pos_pub, 0.0)
        self.publish_float(self.right_pos_pub, 0.0)
        self.publish_float(self.left_thrust_pub, self.thrust_value)
        self.publish_float(self.right_thrust_pub, self.thrust_value)

        second = int(elapsed)
        if second % 2 == 0 and second != self.last_print:
            self.last_print = second
            self.get_logger().info(f'前进中... 剩余 {self.duration - second:.0f}s')

    @staticmethod
    def publish_float(pub, value: float):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def stop_thrusters(self):
        self.publish_float(self.left_thrust_pub, 0.0)
        self.publish_float(self.right_thrust_pub, 0.0)


def main():
    rclpy.init()
    node = AutoPilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_thrusters()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
