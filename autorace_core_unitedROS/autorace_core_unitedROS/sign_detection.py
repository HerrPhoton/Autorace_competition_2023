import os

import torch

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class Sign_detection(Node):

    """"Детекция знаков с помощью YOLOv5.
    Модель выдает только один знак и только тогда, когда площадь bbox превышает заданный порог (знак близко к роботу)."""

    def __init__(self):
        super().__init__('Sign_detection')

        self.class_pub = self.create_publisher(
            String,
            '/sign',
            1)
        
        self.image_pub = self.create_publisher(
            Image,
            '/color/detect',
            1)
        
        self.sub = self.create_subscription(
            Image,
            '/color/image',
            self.subscription_callback,
            1)
        
        self.model_path = self.declare_parameter('model_path', 'model').get_parameter_value().string_value 
        self.model = torch.hub.load(os.path.join(self.model_path, 'yolov5'), 'custom', path = os.path.join(self.model_path, 'best.pt'), source = 'local') 

        self.classes = self.model.names

        self.min_square = 10000 # Минимальная площадь (в пикселях) bbox
        self.cvBridge = CvBridge()

    def subscription_callback(self, image_msg):

        image = self.cvBridge.imgmsg_to_cv2(image_msg, image_msg.encoding)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Прогоняем изображение через модель
        result = self.model(image)    
        bbox = result.pandas().xyxy[0]

        # Если есть детекции, то публикуем найденный класс
        if len(bbox['class'].values) != 0:
            sqr = (bbox['xmax'].values[0] - bbox['xmin'].values[0]) * (bbox['ymax'].values[0] - bbox['ymin'].values[0]) 
            
            if sqr > self.min_square:
                image = result.render()[0]
                self.class_pub.publish(String(data = self.classes[result.pandas().xyxy[0]['class'].values[0]]))

        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) 
        self.image_pub.publish(self.cvBridge.cv2_to_imgmsg(image,  image_msg.encoding))
                

def main():
    rclpy.init()

    node = Sign_detection()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()