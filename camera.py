import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class CamViewer(Node):
    def __init__(self):
        super().__init__('cam_viewer')
        self.sub = self.create_subscription(Image, 
            '/wamv/sensors/cameras/front_left_camera_sensor/image_raw', 
            self.cb, 10)
    def cb(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        cv2.imshow('WAM-V Camera', img)
        cv2.waitKey(1)

rclpy.init()
rclpy.spin(CamViewer())
