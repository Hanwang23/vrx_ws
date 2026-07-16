#!/usr/bin/env python3
"""
WAM-V 控制器 - 直接使用 Gazebo 命令（持续发送）
"""

import subprocess
import time
import sys
import threading
import select
import termios
import tty

class DirectGazeboController:
    def __init__(self):
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.running = True
        self.current_key = None

        print("=== WAM-V 直接控制器已启动 ===")
        print("控制说明：")
        print("  W: 前进")
        print("  S: 后退")
        print("  A: 左转")
        print("  D: 右转")
        print("  Q: 退出")
        print("========================")
        print("请持续按住按键...")

        # 启动发送循环
        self.send_thread = threading.Thread(target=self.send_loop, daemon=True)
        self.send_thread.start()

    def send_gz_command(self, topic, value):
        """直接发送 Gazebo 命令"""
        cmd = f'gz topic -t {topic} -m gz.msgs.Double -p "data: {value:.1f}"'
        try:
            subprocess.run(cmd, shell=True, capture_output=True, timeout=1)
        except:
            pass

    def send_loop(self):
        """持续发送循环"""
        while self.running:
            # 发送推进器命令
            self.send_gz_command('/wamv/thrusters/left/thrust', self.left_thrust)
            self.send_gz_command('/wamv/thrusters/right/thrust', self.right_thrust)
            time.sleep(0.05)  # 20Hz

    def set_thrust(self, left, right):
        """设置推进器推力"""
        self.left_thrust = left
        self.right_thrust = right

    def stop(self):
        """停止"""
        self.running = False
        self.set_thrust(0, 0)
        # 发送停止命令
        self.send_gz_command('/wamv/thrusters/left/thrust', 0)
        self.send_gz_command('/wamv/thrusters/right/thrust', 0)

def keyboard_input(controller):
    """键盘输入处理"""
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while controller.running:
            if select.select([sys.stdin], [], [], 0.01)[0]:
                key = sys.stdin.read(1).lower()

                if key == 'q':
                    print("退出...")
                    controller.stop()
                    return
                elif key == 'w':
                    if controller.current_key != 'w':
                        print("前进")
                        controller.current_key = 'w'
                    controller.set_thrust(3000, 3000)
                elif key == 's':
                    if controller.current_key != 's':
                        print("后退")
                        controller.current_key = 's'
                    controller.set_thrust(-3000, -3000)
                elif key == 'a':
                    if controller.current_key != 'a':
                        print("左转")
                        controller.current_key = 'a'
                    controller.set_thrust(-1500, 1500)
                elif key == 'd':
                    if controller.current_key != 'd':
                        print("右转")
                        controller.current_key = 'd'
                    controller.set_thrust(1500, -1500)
            else:
                # 没有按键时停止
                if controller.current_key is not None:
                    controller.current_key = None
                    controller.set_thrust(0, 0)
    except Exception as e:
        print(f"错误: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    controller = DirectGazeboController()

    try:
        keyboard_input(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        print("已停止")

if __name__ == '__main__':
    main()
