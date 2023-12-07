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

        self.pub = self.create_publisher(
            Float64,
            '/center_lane',
            1)
        
        self.sub = self.create_subscription(
            Image,
            '/color/image_projected',
            self.subscription_callback,
            1)

        self.cv_bridge = CvBridge()

    def subscription_callback(self, msg):
        
        # Считывание изображения и перевод в HSV
        image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Маска для линий
        yellow_mask = cv2.inRange(image, (20, 100, 100), (30, 255, 255))
        white_mask = cv2.inRange(image, (0, 0, 255), (255, 0, 255))

        # Определение центров линий
        M_yellow = cv2.moments(yellow_mask, binaryImage = True)
        M_white = cv2.moments(white_mask, binaryImage = True)

        yellow_center_x = 0 if M_yellow['m00'] == 0 else M_yellow['m10'] // M_yellow['m00']
        white_center_x = image.shape[1] if M_white['m00'] == 0 else M_white['m10'] // M_white['m00']

        # Пропускать те случаи, когда желтая не левее белой
        if white_center_x < yellow_center_x:
            white_center_x = image.shape[1]

        # Публикация центра между линиями
        self.pub.publish(Float64(data = (yellow_center_x + white_center_x) / 2))


def main():
    rclpy.init()

    node = LineDetector()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()