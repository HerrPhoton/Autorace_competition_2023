from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist


class LaneFollower(Node):
    """ Программа следования по полосе.
    Принимает координату центра между белой и желтой линиями, после чего
    производит корректировку направления движения, чтобы робот двигался по цетру полосы."""

    def __init__(self):
        super().__init__('LaneFollower')

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        self.center_lane_sub = self.create_subscription(
            Float64,
            '/center_lane',
            self.move_robot,
            10)
        
        self.moving_enable_sub = self.create_subscription(
            Bool,
            '/moving_enable',
            self.moving_state,
            10)

        self.moving_enable = False

        self.is_first_call = True
        self.true_center = None

        self.instant_errors = deque([0], maxlen = 1000)
        self.error_differences = deque(maxlen = 1000)
        
        self.max_vel = 0.15

    def move_robot(self, msg):

        # Если не разрешено движение, то ничего не делать
        if not self.moving_enable:
            return

        cur_center = msg.data

        # Записываем начальное положение центра при первом коллбэке
        if self.is_first_call:

            self.is_first_call = False
            self.true_center = cur_center

            return

        error = self.true_center - cur_center

        self.error_differences.append(error - self.instant_errors[-1])
        self.instant_errors.append(error)

        Kp = 0.01
        Ki = 0.0
        Kd = 0.0

        angular_z = Kp * error + Ki * np.sum(self.instant_errors) + Kd * np.sum(self.error_differences)
        linear_x = self.max_vel #- Kp * error
        

        twist = Twist()   
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        self.pub.publish(twist)

    def moving_state(self, msg):
        self.moving_enable = msg.data


def main():
    rclpy.init()

    node = LaneFollower()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()