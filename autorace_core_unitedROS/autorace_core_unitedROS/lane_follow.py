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

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1)
        
        self.center_lane_sub = self.create_subscription(
            Float64,
            '/center_lane',
            self.move_robot,
            1)
        
        self.offset_sub = self.create_subscription(
            Float64,
            '/offset',
            self.get_offset,
            1)
            
        self.enable_following_sub = self.create_subscription(
            Bool,
            '/enable_following',
            self.moving_state,
            1)
        
        self.max_vel_sub = self.create_subscription(
            Float64,
            '/max_vel',
            self.set_max_vel,
            1)

        self.enable_following = False # Разрешено ли движение роботу вдоль полосы

        self.is_first_call = True # Проверка на первый вызов для записи начального центра
        self.true_center = None # Координата x истинного центра

        maxlen = self.declare_parameter('max_len', 1).get_parameter_value().integer_value  

        self.instant_errors = deque([0], maxlen = maxlen) # Массив мгновенных ошибок
        self.error_differences = deque(maxlen = maxlen) # Массив разниц мнгновенных ошибок
        
        # Максимальная скорость робота
        self.max_vel = self.declare_parameter('start_speed', 0.0).get_parameter_value().double_value 
        
        # Смещение центра от начального центра полосы
        self.offset = self.declare_parameter('start_offset', 0.0).get_parameter_value().double_value 

        # PID константы для угла поворота
        self.Kp_ang = self.declare_parameter('Kp_ang', 0.0).get_parameter_value().double_value
        self.Ki_ang = self.declare_parameter('Ki_ang', 0.0).get_parameter_value().double_value
        self.Kd_ang = self.declare_parameter('Kd_ang', 0.0).get_parameter_value().double_value

        # PID константы для скорости
        self.Kp_vel = self.declare_parameter('Kp_vel', 0.0).get_parameter_value().double_value
        self.Ki_vel = self.declare_parameter('Ki_vel', 0.0).get_parameter_value().double_value
        self.Kd_vel = self.declare_parameter('Kd_vel', 0.0).get_parameter_value().double_value


    def get_offset(self, msg):
        self.offset = msg.data

    def move_robot(self, msg):

        # Если не разрешено движение, то ничего не делать
        if not self.enable_following:
            return

        cur_center = msg.data

        # Записываем начальное положение центра при первом коллбэке
        if self.is_first_call:

            self.is_first_call = False
            self.true_center = cur_center

            return

        # Запись текущей ошибки
        error = self.true_center + self.offset - cur_center

        self.error_differences.append(error - self.instant_errors[-1])
        self.instant_errors.append(error)

        twist = Twist()   
        twist.linear.x = self.max_vel - (self.Kp_vel * np.abs(error) + self.Ki_vel * np.sum(np.abs(self.instant_errors)) + self.Kd_vel * np.sum(np.abs(self.error_differences)))
        twist.angular.z = self.Kp_ang * error + self.Ki_ang * np.sum(self.instant_errors) + self.Kd_ang * np.sum(self.error_differences)

        self.cmd_vel_pub.publish(twist)

    def moving_state(self, msg):
        self.enable_following = msg.data

    def set_max_vel(self, msg):
        self.max_vel = msg.data


def main():
    rclpy.init()

    node = LaneFollower()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()