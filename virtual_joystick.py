#!/usr/bin/env python3
"""
键盘虚拟摇杆：直接控制 WAM-V 左/右推进器。

运行前请只保留一个控制源：不要同时运行 usv_joy_teleop.py、auto_pilot.py
或其它 /wamv/thrusters/* 发布节点。
"""

import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class VirtualJoystick(Node):
    def __init__(self):
        super().__init__('virtual_joystick')

        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.left_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/left/pos', 10)
        self.right_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/right/pos', 10)

        # VRX H 配置 max_thrust_cmd 约 2353，设低一点更稳定。
        self.thrust_scale = 2353.0
        self.command = 'stop'
        self.command_until = 0.0
        self._last_log = 0.0

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

        self.get_logger().info('=== 键盘虚拟摇杆已启动 ===')
        self.get_logger().info('W 前进 | S 后退 | A 左转 | D 右转 | SPACE 停止 | Q 退出')

    def set_command(self, command: str):
        self.command = command
        # 普通键盘只能收到按键事件，收不到可靠的“松开”事件；保留 0.25s。
        # 按住按键时终端自动重复输入，会持续刷新这个时间。
        self.command_until = time.monotonic() + 0.25

    def timer_callback(self):
        now = time.monotonic()
        command = self.command if now <= self.command_until else 'stop'

        left = 0.0
        right = 0.0
        # VRX/WAM-V 的 H 型推进器中，前进对应负推力。
        # 这和很多键盘直觉相反，但与实际 joystick 轴向一致：摇杆上推通常是负轴值。
        if command == 'forward':
            left = right = -self.thrust_scale
        elif command == 'backward':
            left = right = self.thrust_scale
        elif command == 'left':
            left = 0.5 * self.thrust_scale
            right = -0.5 * self.thrust_scale
        elif command == 'right':
            left = -0.5 * self.thrust_scale
            right = 0.5 * self.thrust_scale

        self.publish_float(self.left_pos_pub, 0.0)
        self.publish_float(self.right_pos_pub, 0.0)
        self.publish_float(self.left_thrust_pub, left)
        self.publish_float(self.right_thrust_pub, right)

        if command != 'stop' and now - self._last_log > 0.5:
            self._last_log = now
            self.get_logger().info(f'{command}: left={left:.0f}, right={right:.0f}')

    @staticmethod
    def publish_float(pub, value: float):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def stop_thrusters(self):
        for pub in (self.left_thrust_pub, self.right_thrust_pub):
            self.publish_float(pub, 0.0)


def keyboard_input_thread(node: VirtualJoystick):
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'q':
                    node.get_logger().info('退出虚拟摇杆')
                    rclpy.shutdown()
                    return
                if key == 'w':
                    node.set_command('forward')
                elif key == 's':
                    node.set_command('backward')
                elif key == 'a':
                    node.set_command('left')
                elif key == 'd':
                    node.set_command('right')
                elif key == ' ':
                    node.set_command('stop')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    print('\n=== 键盘虚拟摇杆 ===')
    print('W 前进 | S 后退 | A 左转 | D 右转 | SPACE 停止 | Q 退出')
    print('注意：请不要同时运行 usv_joy_teleop.py 或 auto_pilot.py\n')

    rclpy.init()
    node = VirtualJoystick()
    input_thread = threading.Thread(target=keyboard_input_thread, args=(node,), daemon=True)
    input_thread.start()

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
