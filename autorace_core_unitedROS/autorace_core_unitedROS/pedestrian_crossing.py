import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


import numpy as np


class Pedestrian_Crossing_Handler(Node):
  
    def __init__(self):
        super().__init__('Pedestrian_Crossing_Handler')

        self.enable_following_pub = self.create_publisher(
            Bool,
            '/enable_following',
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
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1)
        
        self.sign_flag = False
        self.distance = 0
        self.stop = False 

    def handle_sign(self, msg):

        if msg.data == 'pedestrian_crossing_sign' and not self.sign_flag:
            self.sign_flag = True

    def get_distance(self, msg):
        
        # При нахождении знака определяем расстояние до препятствия 
        if self.sign_flag:
            self.distance = np.min(np.concatenate((msg.ranges[340:360], msg.ranges[0:20]), axis = 0))

            # Если расстояние меньше указанного, то необходимо затормозить
            if self.distance < 0.4 and not self.stop:
                self.enable_following_pub.publish(Bool(data = False))
                self.cmd_vel_pub.publish(Twist())
                self.stop = True

            # После остановки порверяем отсутствие препятствия
            if self.stop and self.distance > 0.4:
                self.enable_following_pub.publish(Bool(data = True))
                rclpy.shutdown()

def main():
    rclpy.init()

    node = Pedestrian_Crossing_Handler()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()