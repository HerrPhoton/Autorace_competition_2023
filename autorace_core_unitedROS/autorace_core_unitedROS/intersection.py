import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np


class Intersection_Handler(Node):
    """ Обработка перекрестка.
    При знаке "поворот налево" или "поворот направо" принудительно поворачивает робота в соответствующую сторону,
    после чего робот снова следует вдоль полосы по PID. """

    def __init__(self):
        super().__init__('Intersection_Handler')

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
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.get_odom,
            1)
        
        self.timer = self.create_timer(0.1, self.rotate_robot)

        self.enable_rotate = True # Разрешено ли принудительно поворачивать робота

        self.cur_angle = None   # Текущий абсолютный угол поворота
        self.start_angle = None # Абсолютный угол поворота при начале поворота
        self.angle = None       # Относительный угол поворота

        # Скорости движения по перекрестку
        self.interIn_speed = self.declare_parameter('interIn_speed', 0.0).get_parameter_value().double_value
        self.inter_speed = self.declare_parameter('inter_speed', 0.0).get_parameter_value().double_value
        self.interOut_speed = self.declare_parameter('interOut_speed', 0.0).get_parameter_value().double_value

        # Смещения
        self.interIn_offset = self.declare_parameter('interIn_offset', 0.0).get_parameter_value().double_value
        self.interL_offset = self.declare_parameter('interL_offset', 0.0).get_parameter_value().double_value
        self.interR_offset = self.declare_parameter('interR_offset', 0.0).get_parameter_value().double_value
        self.interOut_offset = self.declare_parameter('interOut_offset', 0.0).get_parameter_value().double_value       

    def handle_sign(self, msg):
        
        cur_sign = msg.data

        # Действия при проезде знака перекрестка
        if cur_sign == 'intersection_sign':
            self.max_vel_pub.publish(Float64(data = self.interIn_speed))
            self.offset_pub.publish(Float64(data = self.interIn_offset))

        # Действия при повороте налево или направо
        if (cur_sign == 'turn_left_sign' or cur_sign == 'turn_right_sign'):
            self.max_vel_pub.publish(Float64(data = self.inter_speed))

            if self.enable_rotate:

                self.enable_following_pub.publish(Bool(data = False))

                if cur_sign == 'turn_left_sign':
                    self.offset_pub.publish(Float64(data = self.interL_offset))
                    self.angle = 80

                if cur_sign == 'turn_right_sign':
                    self.offset_pub.publish(Float64(data = self.interR_offset))
                    self.angle = 0

    def rotate_robot(self):
        
        # Принудительно повернуть робота на заданный угол
        if self.angle is not None and self.start_angle is not None and self.cur_angle is not None:

            if np.abs(self.cur_angle - self.start_angle) >= np.abs(self.angle):

                self.enable_rotate = False
                self.angle = None
                self.enable_following_pub.publish(Bool(data = True))
                
                return
            
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = 4.0 * np.sign(self.angle)

            self.cmd_vel_pub.publish(twist)

    def get_odom(self, msg):

        # Координата x робота
        pose_x = msg.pose.pose.position.x

        # Текущий угол поворота в градусах
        self.cur_angle = self.euler_from_quaternion(msg.pose.pose.orientation)[2]
        self.cur_angle = np.degrees(self.cur_angle)
        self.cur_angle = 360 + self.cur_angle if self.cur_angle < 0 else self.cur_angle

        # Фиксация начального угла перед поворотом робота
        if self.angle is not None and self.start_angle is None:
            self.start_angle = self.cur_angle

        # Изменить скорость и смещение при выезде из перекрестка
        if pose_x <= -0.1:
            self.offset_pub.publish(Float64(data = self.interOut_offset))
            self.max_vel_pub.publish(Float64(data = self.interOut_speed))
            rclpy.shutdown()

    def euler_from_quaternion(self, quaternion):
        
        # Перевод кватерниона в углы Эйлера
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        
        # Перевод углов Эйлера в кватернион
        cy = np.math.cos(yaw * 0.5)
        sy = np.math.sin(yaw * 0.5)
        cp = np.math.cos(pitch * 0.5)
        sp = np.math.sin(pitch * 0.5)
        cr = np.math.cos(roll * 0.5)
        sr = np.math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

                      
def main():
    rclpy.init()

    node = Intersection_Handler()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()