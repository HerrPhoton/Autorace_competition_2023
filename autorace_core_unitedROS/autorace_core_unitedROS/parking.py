import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import cv2
import numpy as np


class PedestrianDetector(Node):
  
    def __init__(self):
        super().__init__('PedestrianDetector')

        self.enable_following_pub = self.create_publisher(
            Bool,
            '/enable_following',
            1)
        
        self.img_proj_sub = self.create_subscription(
            Image,
            '/color/image_projected',
            self.projected_callback,
            1)

        self.sign_sub = self.create_subscription(
            String,
            '/sign',
            self.handle_sign,
            1)
        
        self.image_sub = self.create_subscription(
            Image,
            '/depth/image',
            self.image_callback,
            1)
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1)
        
        self.stop = 0 
        self.wait = 0
        self.cv_bridge = CvBridge()
        self.sign_flag = False

    def handle_sign(self, msg):

        if msg.data == 'pedestrian_crossing_sign' and not self.sign_flag:
            self.sign_flag = True

    def projected_callback(self, msg):
        
        # Считывание изображения и перевод в HSV
        image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Маска для линии
        line_mask = cv2.inRange(image_hsv, (20, 100, 100),(20, 255, 255))

        # Определяем условия остановки 
        if np.any(line_mask) != 0 and self.sign_flag:
            self.stop += 1
        
        if self.stop == 35 and np.all(line_mask) == 0:

            self.enable_following_pub.publish(Bool(data = False))
            self.start = True
            self.cmd_vel_pub.publish(Twist())


    def image_callback(self, msg):

        # Вводим таймер ожидания пешехода 
        if self.stop > 35:
            self.wait += 1

        # Определяем расстояние от машинки до препятствия 
        image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        image = image[int(msg.height * 0.4) : -int(msg.height * 0.4), 
                      int(msg.width * 0.4)  : -int(msg.width * 0.4)]
        distance = np.min(image)

        # Проверяем, есть ли препятствие на пешеходном переходе за время ожидания 
        if distance > 0.3 and self.wait > 35:
            self.enable_following_pub.publish(Bool(data = True))
            rclpy.shutdown()


def main():
    rclpy.init()

    node = PedestrianDetector()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()