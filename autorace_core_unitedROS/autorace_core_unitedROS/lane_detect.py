import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge

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
        
        self.white_weight_sub = self.create_subscription(
            Float64,
            '/white_weight',
            self.set_white_weight,
            1)
        
        self.yellow_weight_sub = self.create_subscription(
            Float64,
            '/yellow_weight',
            self.set_yellow_weight,
            1)

        self.cv_bridge = CvBridge()

        self.white_weight = 1.0
        self.yellow_weight = 1.0
        
    def image_processing(self, msg):
        
        # Считывание изображения и перевод в HSV
        image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Маска желтой линии
        yellow_mask = cv2.inRange(hsv_image, (20, 100, 100), (30, 255, 255))
        yellow_mask = cv2.blur(yellow_mask, (3, 3))
        yellow_mask[yellow_mask != 0] = 255

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
            white_center_x = hsv_image.shape[1]

        # Публикация центра между линиями
        self.center_lane_pub.publish(Float64(data = (self.yellow_weight * yellow_center_x + self.white_weight * white_center_x) / 2))

    def set_white_weight(self, msg):
        self.white_weight = msg.data
    
    def set_yellow_weight(self, msg):
        self.yellow_weight = msg.data

def main():
    rclpy.init()

    node = LineDetector()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()