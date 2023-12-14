import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

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
        
        self.image_sub = self.create_subscription(
            Image,
            '/depth/image',
            self.image_callback,
            1)
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1)
        
        self.sign_flag = False
        self.distance = 0
        self.stop = False 
        self.cv_bridge = CvBridge()

    def handle_sign(self, msg):

        if msg.data == 'pedestrian_crossing_sign' and not self.sign_flag:
            self.sign_flag = True

    def image_callback(self, msg):

        # Определяем расстояние от машинки до препятствия 
        image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        image = image[int(msg.height * 0.35) : -int(msg.height * 0.35), 
                      int(msg.width * 0.35)  : -int(msg.width * 0.35)]
        
        self.distance = np.min(image)
        if self.sign_flag:

            if self.distance < 0.3 and not self.stop:
                self.enable_following_pub.publish(Bool(data = False))
                self.cmd_vel_pub.publish(Twist())
                self.stop = True

            if self.stop and self.distance > 0.3:
                self.enable_following_pub.publish(Bool(data = True))
                rclpy.shutdown()

def main():
    rclpy.init()

    node = Pedestrian_Crossing_Handler()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()