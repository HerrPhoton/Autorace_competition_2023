import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

import numpy as np

import cv2


class TrafficLightDetector(Node):
    """ Определение зеленого цвета светофора. 
    После обнаружения посылает сигнал о начале старта."""

    def __init__(self):
        super().__init__('TrafficLightDetector')

        self.pub = self.create_publisher(
            Bool,
            '/moving_enable',
            1)
        
        self.sub = self.create_subscription(
            Image,
            '/color/image',
            self.subscription_callback,
            1)

        self.cv_bridge = CvBridge()

    def subscription_callback(self, msg):

        # Считывание изображения и перевод в HSV
        image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Маскирование зеленого цвета
        green_mask = cv2.inRange(image, (50, 100, 100), (70, 255, 255))

        # Подача сигнала на движение, если есть хоть один зеленый пиксел
        if np.any(green_mask != 0):
            self.pub.publish(Bool(data = True))
            rclpy.shutdown()


def main():
    rclpy.init()

    node = TrafficLightDetector()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()