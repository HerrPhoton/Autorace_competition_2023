from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np


class AvoidObstacles(Node):
    def __init__(self):
        super().__init__('AvoidObstacles')

        self.enable_following_pub = self.create_publisher(
            Bool,
            '/enable_following',
            1)
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1)

        self.max_vel_pub = self.create_publisher(
            Float64,
            '/max_vel',
            1)

        self.offset_pub = self.create_publisher(
            Float64,
            '/offset',
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

        self.enable = False
        self.get_turn = True
        self.turn = None # 0 left 1 right
        self.obstaclesIn_speed = self.declare_parameter('obstaclesIn_speed', 0.0).get_parameter_value().double_value

    def handle_sign(self, msg):
        
        # Действия при проезде знака дорожных работ
        if msg.data == 'road_works_sign':
            self.max_vel_pub.publish(Float64(data = self.obstaclesIn_speed))
            self.enable = True

    def get_distance(self, msg):
        
        if self.enable:
            front_distance = np.min(np.concatenate((msg.ranges[345:360],msg.ranges[0:15]), axis=0))
            left_distance = np.min(msg.ranges[65:135])
            right_distance = np.min(msg.ranges[155:325])

            if front_distance < 0.4 and self.get_turn:
                if right_distance > left_distance:
                    # self.get_logger().info(f'front_distance = {front_distance}')
                    self.turn = 0
                    self.get_logger().info('left')
                else:
                    self.turn = 1
                    self.get_logger().info('right')



def main():
    rclpy.init()

    node = AvoidObstacles()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()