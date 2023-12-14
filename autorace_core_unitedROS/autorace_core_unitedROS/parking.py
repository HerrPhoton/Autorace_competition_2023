from time import sleep

import rclpy
from rclpy.node import Node

from robot_rotate_interface.msg import Rotate
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Bool, String, Float64, Int8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge


import cv2
import numpy as np


class Parking_Handler(Node):
  
    def __init__(self):
        super().__init__('Parking_Handler')

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

        self.parking_pub = self.create_publisher(
            Bool,
            '/is_parking',
            1)

        self.rotate_pub = self.create_publisher(
            Rotate,
            '/rotate',
            1)
        
        self.rotate_done_sub = self.create_subscription(
            Int8,
            '/rotate_done',
            self.set_rotate_done,
            1)

        self.sign_sub = self.create_subscription(
            String,
            '/sign',
            self.handle_sign,
            1)
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.get_distance,
            1)
        
        self.img_proj_sub = self.create_subscription(
            Image,
            '/color/image_projected',
            self.image_processing,
            1)
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.get_odom,
            1)
 
        self.cv_bridge = CvBridge()

        self.is_parking = False # Режим заезда на парковку
        self.find_parking_place = False # Режим поворота на парковочное место
        self.exit = False # Режим выезда с перекрестка

        self.shutdown = False

        self.angle = None # Угол для поворота на парковочное место 
        self.id = 2

    def handle_sign(self, msg):

        if msg.data == 'parking_sign' and not self.is_parking and not self.exit:
            self.is_parking = True
            self.max_vel_pub.publish(Float64(data = 0.40))
            self.offset_pub.publish(Float64(data = 60.0))

    def get_distance(self, msg):

        # Дожидаемся, пока не доедем до знака
        if self.is_parking:
            
            self.distance = np.min(msg.ranges)

            if self.distance < 0.3:
                self.parking_pub.publish(Bool(data = True))

        # Определяем в какую сторону нужно повернуть для парковки
        if self.find_parking_place:

            min_id = np.argmin(msg.ranges)

            # Налево
            if min_id >= 180:
                self.angle = 80.0
                self.rotate_pub.publish(Rotate(angle = self.angle, linear_x = 0.23, angular_z = 1.0, id = self.id))
            # Направо
            else:
                self.angle = -70.0
                self.rotate_pub.publish(Rotate(angle = self.angle, linear_x = 0.20, angular_z = 1.0, id = self.id))

            self.find_parking_place = False
            
    def image_processing(self, msg):

        if self.is_parking:

            # Считывание изображения и перевод в HSV
            image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            yellow_mask = cv2.inRange(hsv_image, (20, 100, 100), (30, 255, 255))
            yellow_mask = cv2.blur(yellow_mask, (3, 3))
            yellow_mask[yellow_mask != 0] = 255

            contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # Начать поворот на парковочное место после конца желтых линий
            if len(contours) == 2 and not self.find_parking_place:

                if np.min(np.argwhere(yellow_mask)[:, 0]) >= yellow_mask.shape[0] / 2.1:
                    self.enable_following_pub.publish(Bool(data = False))
                    self.cmd_vel_pub.publish(Twist())
                    self.find_parking_place = True
                    self.is_parking = False

    def get_odom(self, msg):
        
        if self.exit:
            pose_y = msg.pose.pose.position.y

            # Продолжить движение по полосе после выезда с парковки
            if pose_y >= 3.10:

                self.parking_pub.publish(Bool(data = False))
                self.enable_following_pub.publish(Bool(data = False))
                self.rotate_pub.publish(Rotate(angle = 45.0, linear_x = 0.22, angular_z = 1.0, id = self.id))

                self.max_vel_pub.publish(Float64(data = 0.40))
                self.offset_pub.publish(Float64(data = 30.0))
                
                self.shutdown = True

    def set_rotate_done(self, msg):
    
        if msg.data == self.id:

            if not self.exit:
                self.cmd_vel_pub.publish(Twist())

                sleep(1.5)

                self.angle = 60 * np.sign(self.angle)
                self.rotate_pub.publish(Rotate(angle = self.angle, linear_x = -0.1, angular_z = 0.5, id = self.id))

                self.exit = True
            else:
                self.enable_following_pub.publish(Bool(data = True))

                if self.shutdown:
                    rclpy.shutdown()





def main():
    rclpy.init()

    node = Parking_Handler()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()