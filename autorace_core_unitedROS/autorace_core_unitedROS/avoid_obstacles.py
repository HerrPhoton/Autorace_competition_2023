from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class AvoidObstacles(Node):
    def __init__(self):
        super().__init__('AvoidObstacles')

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1)
        
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            1)

        self.is_obstacle = False
        #self.timer = self.create_timer(0.1, self.publisher_callback)

    def lidar_callback(self, msg):
        self.get_logger().info(msg.ranges)
        #self.is_obstacle = min(msg.ranges) < 2.0


def main():
    rclpy.init()

    node = AvoidObstacles()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()