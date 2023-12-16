import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, String, Int8
from robot_rotate_interface.msg import Rotate
from sensor_msgs.msg import LaserScan

import numpy as np


class AvoidObstacles(Node):
    def __init__(self):
        super().__init__('AvoidObstacles')

        self.enable_following_pub = self.create_publisher(
            Bool,
            '/enable_following',
            1)

        self.max_vel_pub = self.create_publisher(
            Float64,
            '/max_vel',
            1)
        
        self.rotate_pub = self.create_publisher(
            Rotate,
            '/rotate',
            1)
        
        self.rotate_done_sub = self.create_subscription(
            Int8,
            '/rotate_done',
            self.set_rotate_done,
            1)
        
        self.sign_sub = self.create_subscription(
            String,
            '/sign',
            self.handle_sign,
            1)
           
        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.get_distance,
            1)
        
        self.enable_detection_pub = self.create_subscription(
            Bool,
            '/enable_detection',
            self.get_detection_state,
            1)

        self.ID = 1

        self.enable_detection = False

        self.dir = True     # Направление поворота: True - налево, False - направо
        self.enable = False # Режим объезда препятствий
        self.turned = False # Флаг окончания поворота

        self.obstaclesIn_speed = self.declare_parameter('obstaclesIn_speed', 0.0).get_parameter_value().double_value

    def handle_sign(self, msg):
        
        # Действия при проезде знака дорожных работ
        if msg.data == 'road_works_sign' and self.enable_detection:
            self.max_vel_pub.publish(Float64(data = self.obstaclesIn_speed))
            self.enable = True

    def get_distance(self, msg):
        
        # При нахождении знака определяем расстояние до препятствия 
        if self.enable:
            front_distance = np.min(np.concatenate((msg.ranges[345:360], msg.ranges[0:15]), axis = 0))

            if front_distance < 0.55:
                self.enable_following_pub.publish(Bool(data = False))

                # Поворот налево 
                if self.dir:
                    self.rotate_pub.publish(Rotate(angle = 50.0, linear_x = 0.15, angular_z = 1.0, id = self.ID))

                # Поворот направо 
                if not self.dir and self.turned:
                    self.rotate_pub.publish(Rotate(angle = -70.0, linear_x = 0.175, angular_z = 1.0, id = self.ID))
               
    def set_rotate_done(self, msg):
        
        if msg.data == self.ID:
            self.turned = True

            # Смена направления поворота
            if self.dir:
                self.dir = False
            else:
                # Отключение ноды после заершения второго поворота
                self.enable_following_pub.publish(Bool(data = True))
                rclpy.shutdown()

    def get_detection_state(self, msg):
        self.enable_detection = msg.data


def main():
    rclpy.init()

    node = AvoidObstacles()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()