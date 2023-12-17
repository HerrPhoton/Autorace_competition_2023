import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2

class LineDetector(Node):
    """ Определение белой и желтой линий на трансформированном изображении, 
    поступающего с камеры. Публикует координату x, на которой находится центр
    между линиями."""

    def __init__(self):
        super().__init__('LineDetector')

        self.center_lane_pub = self.create_publisher(
            Float64,
            '/center_lane',
            1)
        
        self.img_proj_sub = self.create_subscription(
            Image,
            '/color/image_projected',
            self.image_processing,
            1)

        self.parking_sub = self.create_subscription(
            Bool,
            '/is_parking',
            self.get_parking,
            1)

        self.cv_bridge = CvBridge()
        self.is_parking = False 
        
    def image_processing(self, msg):
        
        # Считывание изображения и перевод в HSV
        image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Центры линий
        centers_x = []

        # Маска желтой линии
        yellow_mask = cv2.inRange(hsv_image, (20, 100, 100), (30, 255, 255))
        yellow_mask = cv2.blur(yellow_mask, (3, 3))
        yellow_mask[yellow_mask != 0] = 255
 
        if self.is_parking:
            contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            
            if len(contours) != 2:
                centers_x.append(20)

            for contour in contours:
                M = cv2.moments(contour, binaryImage = True)
                centers_x.append(M['m10'] // M['m00'])

        else:
            # Маска белой линии
            white_mask = cv2.inRange(hsv_image, (0, 0, 230), (255, 0, 255))
            white_mask = cv2.blur(white_mask, (3, 3))
            white_mask[white_mask != 0] = 255
            
            # Определение центров линий
            M_yellow = cv2.moments(yellow_mask, binaryImage = True)
            M_white = cv2.moments(white_mask, binaryImage = True)

            yellow_center_x = 0 if M_yellow['m00'] == 0 else M_yellow['m10'] // M_yellow['m00']
            white_center_x = hsv_image.shape[1] if M_white['m00'] == 0 else M_white['m10'] // M_white['m00']

            # Пропускать те случаи, когда желтая не левее белой
            if white_center_x < yellow_center_x:
                white_center_x = image.shape[1]

            centers_x.append(yellow_center_x)
            centers_x.append(white_center_x)

        # Публикация центра между линиями
        self.center_lane_pub.publish(Float64(data = np.sum(centers_x) / len(centers_x)))

    def get_parking(self, msg):     
        self.is_parking = msg.data


def main():
    rclpy.init()

    node = LineDetector()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()