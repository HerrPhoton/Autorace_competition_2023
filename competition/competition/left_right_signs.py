import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist


class LeftRigtSigns(Node):

    def __init__(self):
        super().__init__('LeftRigtSigns')

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1)
        
        self.sign_sub = self.create_subscription(
            String,
            '/sign',
            self.rotate_robot,
            1)
        
        self.cnt = 0

    def rotate_robot(self, msg):
        
        cur_sign = msg.data

        if cur_sign == 'turn_left_sign':
            
            self.cnt += 1

            twist = Twist()
            twist.linear.x = 0.20   
            twist.angular.z = 2.0

            self.pub.publish(twist)

            if self.cnt == 50:
                rclpy.shutdown()


def main():
    rclpy.init()

    node = LeftRigtSigns()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()