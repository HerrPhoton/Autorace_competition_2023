import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np


class Pedestrian_Crossing_Handler(Node):
    """ Проезд пешеходного перехода.
    После проезда знака "пещеходный переход" считывает расстояние с лидара перед собой. 
    Если обнаружено препятствие на заданном расстоянии, то робот останавливается до тех пор,
    пока проезд перед ним не будет свободным, чтобы возобновить движение.
    """

    def __init__(self):
        super().__init__('Pedestrian_Crossing_Handler')

        # Publishers
        self.enable_following_pub = self.create_publisher(Bool, '/enable_following', 1)  
        self.max_vel_pub = self.create_publisher(Float64, '/max_vel', 1)
        self.offset_pub = self.create_publisher(Float64, '/offset', 1)

        # Subscribers
        self.sign_sub = self.create_subscription(String, '/sign', self.handle_sign, 1)
        self.laser_scan_sub = self.create_subscription(LaserScan, '/scan', self.get_distance, 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # Расстояние до пешехода для реагирования
        self.stop_distance = self.declare_parameter('stop_distance', 0.0).get_parameter_value().double_value

        # Скорости
        self.in_speed = self.declare_parameter('in_speed', 0.0).get_parameter_value().double_value
        self.out_speed = self.declare_parameter('out_speed', 0.0).get_parameter_value().double_value

        # Смещения
        self.in_offset = self.declare_parameter('in_offset', 0.0).get_parameter_value().double_value
        self.out_offset = self.declare_parameter('out_offset', 0.0).get_parameter_value().double_value

        self.distance = 0 # Текущее расстояние до пешехода
        
        self.sign_flag = False # Обнаружен ли знак
        self.stop = False # Была ли остановка

    def handle_sign(self, msg):

        if msg.data == 'pedestrian_crossing_sign' and not self.sign_flag:
            self.max_vel_pub.publish(Float64(data = self.in_speed))
            self.offset_pub.publish(Float64(data = self.in_offset))

            self.sign_flag = True

    def get_distance(self, msg):
        
        # При нахождении знака определяем расстояние до препятствия 
        if self.sign_flag:
            self.distance = np.min(np.concatenate((msg.ranges[340:360], msg.ranges[0:20]), axis = 0))

            # Если расстояние меньше указанного, то необходимо затормозить
            if self.distance < self.stop_distance and not self.stop:
                self.enable_following_pub.publish(Bool(data = False))
                self.cmd_vel_pub.publish(Twist())
                self.stop = True

            # После остановки порверяем отсутствие препятствия
            if self.stop and self.distance > self.stop_distance:

                self.enable_following_pub.publish(Bool(data = True))
                self.max_vel_pub.publish(Float64(data = self.out_speed))
                self.offset_pub.publish(Float64(data = self.out_offset))

                rclpy.shutdown()

def main():
    rclpy.init()

    node = Pedestrian_Crossing_Handler()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()