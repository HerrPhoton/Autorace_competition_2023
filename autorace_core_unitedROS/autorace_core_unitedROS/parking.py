from time import sleep
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float64, Int8
from robot_rotate_interface.msg import Rotate
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge

import cv2
import numpy as np


class Parking_Handler(Node):
    """ Проезд парковки.
    При приближении к знаку переключает режим следования по полосе на 
    режим следования по двум желтым полосам. После доезда до парковочного места с помощью лидара 
    определяет, где стоит машина, и поворачивает на парковочное место в противоположную сторону.
    Для выезда с парковочного места сдает назад, после чего вновь включается движение по двум желтым полосам до поворота.
    """

    def __init__(self):
        super().__init__('Parking_Handler')

        # Publishers
        self.enable_following_pub = self.create_publisher(Bool, '/enable_following', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.max_vel_pub = self.create_publisher(Float64, '/max_vel', 1)   
        self.offset_pub = self.create_publisher(Float64, '/offset', 1)
        self.parking_pub = self.create_publisher(Bool, '/is_parking', 1)
        self.rotate_pub = self.create_publisher(Rotate, '/rotate', 1)
        
        # Subscribers
        self.rotate_done_sub = self.create_subscription(Int8, '/rotate_done', self.set_rotate_done, 1)
        self.sign_sub = self.create_subscription(String, '/sign', self.handle_sign, 1)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.get_distance, 1)
        self.img_proj_sub = self.create_subscription(Image, '/color/image_projected', self.image_processing, 1)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.get_odom, 1)
 
        self.ID = 2 # Идентификатор ноды

        self.cv_bridge = CvBridge()

        # Скорости
        self.in_speed = self.declare_parameter('in_speed', 0.0).get_parameter_value().double_value
        self.parking_speed = self.declare_parameter('parking_speed', 0.0).get_parameter_value().double_value
        self.out_speed = self.declare_parameter('out_speed', 0.0).get_parameter_value().double_value
        
        # Смещения
        self.in_offset = self.declare_parameter('in_offset', 0.0).get_parameter_value().double_value
        self.out_offset = self.declare_parameter('out_offset', 0.0).get_parameter_value().double_value

        # Углы поворота
        self.angle_L = self.declare_parameter('angle_L', 0.0).get_parameter_value().double_value
        self.angle_R = self.declare_parameter('angle_R', 0.0).get_parameter_value().double_value
        self.angle_B = self.declare_parameter('angle_B', 0.0).get_parameter_value().double_value

        # Линейные скорости при повороте
        self.linear_x_L = self.declare_parameter('linear_x_L', 0.0).get_parameter_value().double_value
        self.linear_x_R = self.declare_parameter('linear_x_R', 0.0).get_parameter_value().double_value
        self.linear_x_BL = self.declare_parameter('linear_x_BL', 0.0).get_parameter_value().double_value
        self.linear_x_BR = self.declare_parameter('linear_x_BR', 0.0).get_parameter_value().double_value

        # Угловые скорости при повороте
        self.angular_z_L = self.declare_parameter('angular_z_L', 0.0).get_parameter_value().double_value
        self.angular_z_R = self.declare_parameter('angular_z_R', 0.0).get_parameter_value().double_value
        self.angular_z_BL = self.declare_parameter('angular_z_BL', 0.0).get_parameter_value().double_value
        self.angular_z_BR = self.declare_parameter('angular_z_BR', 0.0).get_parameter_value().double_value

        self.is_parking = False # Режим заезда на парковку
        self.find_parking_place = False # Режим поворота на парковочное место
        self.exit = False # Режим выезда с перекрестка

        self.shutdown = False # Нужно ли выключить ноду после завершения поворота

        self.angle = None     # Угол для поворота на парковочное место 
        self.linear_x = None  # Линейная скорость при повороте
        self.angular_z = None # Угловая скорость при повороте

    def handle_sign(self, msg):

        if msg.data == 'parking_sign' and not self.is_parking and not self.exit:
            self.is_parking = True
            self.max_vel_pub.publish(Float64(data = self.in_speed))
            self.offset_pub.publish(Float64(data = self.in_offset))

    def get_distance(self, msg):

        # Дожидаемся, пока не доедем до знака
        if self.is_parking:
            
            self.distance = np.min(msg.ranges[270:360])

            if self.distance < 0.3:
                self.parking_pub.publish(Bool(data = True))
                self.max_vel_pub.publish(Float64(data = self.parking_speed))

        # Определяем в какую сторону нужно повернуть для парковки
        if self.find_parking_place:

            min_id = np.argmin(msg.ranges)

            # Налево
            if min_id >= 180:
                self.angle = self.angle_L
                self.linear_x = self.linear_x_L
                self.angular_z = self.angular_z_L

            # Направо
            else:
                self.angle = self.angle_R
                self.linear_x = self.linear_x_R
                self.angular_z = self.angular_z_R

            self.rotate_pub.publish(Rotate(angle = self.angle, linear_x = self.linear_x, angular_z = self.angular_z, id = self.ID))
            self.find_parking_place = False
            
    def image_processing(self, msg):

        if self.is_parking:

            # Считывание изображения и перевод в HSV
            image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Определяем желтые полосы
            yellow_mask = cv2.inRange(hsv_image, (20, 100, 100), (30, 255, 255))
            yellow_mask = cv2.blur(yellow_mask, (3, 3))
            yellow_mask[yellow_mask != 0] = 255

            contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # Начать поворот на парковочное место после того, как желтые линии окажутся дальше середины изображения
            if len(contours) == 2 and not self.find_parking_place:

                if np.min(np.argwhere(yellow_mask)[:, 0]) >= yellow_mask.shape[0] / 2.0:
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
                self.rotate_pub.publish(Rotate(angle = 45.0, linear_x = 0.22, angular_z = 1.0, id = self.ID))

                self.max_vel_pub.publish(Float64(data = self.out_speed))
                self.offset_pub.publish(Float64(data = self.out_offset))
                
                self.shutdown = True 

    def set_rotate_done(self, msg):
        
        if msg.data == self.ID:
            # Поворот на парковочное место
            if not self.exit:
                self.cmd_vel_pub.publish(Twist())

                # Ждем секунду и пока робот полностью остановится
                sleep(1.5)

                # Запускаем выезд задом
                self.angle = self.angle_B * np.sign(self.angle)
                self.linear_x = self.linear_x_BL if self.angle > 0 else self.linear_x_BR
                self.angular_z = self.angular_z_BL if self.angle > 0 else self.angular_z_BR

                self.rotate_pub.publish(Rotate(angle = self.angle, linear_x = self.linear_x, angular_z = self.angular_z, id = self.ID))

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