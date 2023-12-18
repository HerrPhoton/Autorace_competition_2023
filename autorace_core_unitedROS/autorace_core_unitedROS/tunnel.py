import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pi

import numpy as np


class Tunnel_Handler(Node):
    """Проезд туннеля.
    При заезде внутрь туннеля начинает двигаться к указанной точке (выходу). 
    С помощью лидара отслеживается наличие препятствий. В зависимости от того, к какой стороне робота
    ближе препятсвие, выбирается направление объезда.
    """

    def __init__(self):
        super().__init__('Tunnel_Handler')

        # Publishers
        self.enable_following_pub = self.create_publisher(Bool, '/enable_following', 1)
        self.max_vel_pub = self.create_publisher(Float64, '/max_vel', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.offset_pub = self.create_publisher(Float64, '/offset', 1)
        self.finish_enable_pub = self.create_publisher(Bool, '/finish_enable', 1)
        
        # Subscribers
        self.sign_sub = self.create_subscription(String, '/sign', self.handle_sign, 1)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.get_distance, 1)  
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.get_odom, 1)

        self.timer = self.create_timer(0.1, self.move2goal)

        self.tunnel = False # Запущено ли испытание
        self.follow = False # Запущено ли движение по туннелю

        self.angle_offset = 0.0 # Отклонение для объезда препятствий

        # Текущие величины
        self.x = None
        self.y = None
        self.angular = None

        # Целевые величины
        self.target_x = self.declare_parameter('target_x', 0.0).get_parameter_value().double_value
        self.target_y = self.declare_parameter('target_y', 0.0).get_parameter_value().double_value
        self.threshold = self.declare_parameter('threshold', 0.0).get_parameter_value().double_value

        # Скорости движения по туннелю
        self.speed = self.declare_parameter('speed', 0.0).get_parameter_value().double_value
        self.out_speed = self.declare_parameter('out_speed', 0.0).get_parameter_value().double_value

        # Смещения
        self.out_offset = self.declare_parameter('out_offset', 0.0).get_parameter_value().double_value

        self.max_distance = self.declare_parameter('max_distance', 0.0).get_parameter_value().double_value

    def move2goal(self):

        if self.angular is not None:

            twist = Twist()

            # Угол для движения к целевой точке
            angle = self.find_angular() - self.angular
            angle += -2 * pi if angle > pi else 2 * pi if angle < -pi else 0

            # Смещение угловой скорости, если есть препятствие
            angular_z = np.clip(angle + self.angle_offset, -1.0, 1.0)

            twist.angular.z = angular_z
            twist.linear.x = self.speed - (self.speed / 2) * np.abs(angular_z)

            self.cmd_vel_pub.publish(twist)

    def get_odom(self, msg):

        if self.follow:

            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            
            self.angular = self.euler_from_quaternion(msg.pose.pose.orientation)[2]
            self.angular = 2 * pi + self.angular if self.angular < 0 else self.angular
            
            if self.find_distance() < self.threshold:

                self.enable_following_pub.publish(Bool(data = True))
                self.finish_enable_pub.publish(Bool(data = True))
                self.max_vel_pub.publish(Float64(data = self.out_speed))
                self.offset_pub.publish(Float64(data = self.out_offset))

                rclpy.shutdown()

    def find_distance(self):
        return sqrt((self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2)

    def find_angular(self):
        return atan2(self.target_y - self.y, self.target_x - self.x)
    
    def get_distance(self, msg):

        if self.follow:
            
            front_range = np.concatenate([msg.ranges[330 : 360], msg.ranges[0 : 30]], axis = 0)
            front_distance = np.min(front_range)

            self.angle_offset = 0.0

            if front_distance < self.max_distance:
                self.angle_offset = 1 / (4 * front_distance) * (1 if np.argmin(front_range) <= len(front_range) / 2 else -1)

        elif self.tunnel and np.min(msg.ranges[88:92]) < 0.25:
            self.enable_following_pub.publish(Bool(data = False))
            self.tunnel = False
            self.follow = True        

    def handle_sign(self, msg):
        
        # Действия при проезде знака 
        if msg.data == 'tunnel_sign' and not self.tunnel:
            self.tunnel = True

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


def main():
    rclpy.init()

    node = Tunnel_Handler()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()