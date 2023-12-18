import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from cv_bridge import CvBridge

import cv2
import numpy as np


class Finish_Handler(Node):
    """ Останавливает робота после пересечения финишной линии.
    Измеряет расстояние с помощью лидара до светофора и фиксирует наличие черных пикселей 
    для определения остановки."""

    def __init__(self):
        super().__init__('Finish_Handler')

        # Publishers
        self.enable_following_pub = self.create_publisher(Bool, '/enable_following', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.finish_pub = self.create_publisher(String, '/robot_finish', 1)
        
        # Subscribers
        self.img_proj_sub = self.create_subscription(Image, '/color/image_projected', self.image_processing, 1)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.get_distance, 1)  
        self.finish_enable_sub = self.create_subscription(Bool, '/finish_enable', self.finish_state, 1)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.get_imu, 1)
        
        self.timer = self.create_timer(0.1, self.stop_robot)

        self.distance = np.inf # Расстояние до светофора
        self.finish_enable = False # Активирован ли режим обнаружения финиша

        self.stop = False

        self.cv_bridge = CvBridge()
        self.linear_acceleration = np.inf

    def image_processing(self, msg):

        if self.finish_enable:

            image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
            image = image[:, int(msg.width * 0.4) : -int(msg.width * 0.4)]

            # Маска черного цвета
            black_mask = cv2.inRange(image, (0, 0, 0), (10, 10, 10))
            
            if np.any(black_mask != 0):
                self.stop = True
                
    def get_distance(self, msg):       
        self.distance = np.min(msg.ranges[225 : 360])

    def finish_state(self, msg):
        self.finish_enable = msg.data

    def stop_robot(self):

        if self.stop and self.distance < 0.4:
            self.enable_following_pub.publish(Bool(data = False))
            self.cmd_vel_pub.publish(Twist())

            if self.linear_acceleration < 9.72:
                self.finish_pub.publish(String(data = 'unitedROS'))

    def get_imu(self, msg):
        self.linear_acceleration = np.sum([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])


def main():
    rclpy.init()

    node = Finish_Handler()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()