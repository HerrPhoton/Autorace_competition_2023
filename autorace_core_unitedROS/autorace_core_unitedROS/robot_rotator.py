import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_rotate_interface.msg import Rotate

import numpy as np

class Rotator(Node):
  
    def __init__(self):
        super().__init__('Rotator')

        self.rotate_done_pub = self.create_publisher(
            Int8,
            '/rotate_done',
            1)

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1)
       
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.get_odom,
            1)
        
        self.rotate_sub = self.create_subscription(
            Rotate,
            '/rotate',
            self.get_data,
            1)

        self.timer = self.create_timer(0.1, self.rotate_robot)

        self.cur_angle = None   # Текущий абсолютный угол поворота
        self.start_angle = None # Абсолютный угол поворота при начале поворота
        
        self.angle = None       # Относительный угол поворота
        self.linear_x = None    # Линейная скорость при повороте
        self.angular_z = None   # Угловая скорость при повороте
        self.id = None          # Номер испытания

    def rotate_robot(self):
        
        # Принудительно повернуть робота на заданный угол
        if self.angle is not None and self.start_angle is not None and self.cur_angle is not None:
            
            # Определить текущий относительный поворот
            diff = self.cur_angle - self.start_angle
            diff += -360 if diff > 180 else 360 if diff < -180 else 0

            # Проверка на окончание поворота
            if np.abs(diff) >= np.abs(self.angle):
                
                self.rotate_done_pub.publish(Int8(data = self.id))

                self.angle = None
                self.start_angle = None
                self.cur_angle = None
                self.id = None

        
            else:
                twist = Twist()
                twist.linear.x = self.linear_x
                twist.angular.z = self.angular_z * np.sign(self.angle)

                self.cmd_vel_pub.publish(twist)

    def get_odom(self, msg):
        
        if self.angle is not None:

            # Текущий угол поворота в градусах
            self.cur_angle = self.euler_from_quaternion(msg.pose.pose.orientation)[2]
            self.cur_angle = np.degrees(self.cur_angle)
            self.cur_angle = 360 + self.cur_angle if self.cur_angle < 0 else self.cur_angle

            # Фиксация начального угла перед поворотом робота
            if self.start_angle is None:
                self.start_angle = self.cur_angle

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
    
    def get_data(self, msg):

        self.angle = msg.angle
        self.linear_x = msg.linear_x
        self.angular_z = msg.angular_z
        self.id = msg.id


def main():
    rclpy.init()

    node = Rotator()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()