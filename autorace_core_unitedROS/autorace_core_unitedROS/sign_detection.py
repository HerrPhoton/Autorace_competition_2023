import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import torch
import cv2


class Sign_detection(Node):

    """"Детекция знаков с помощью YOLOv5.
    Модель выдает только один знак и только тогда, когда площадь bbox превышает заданный порог (знак близко к роботу).
    """

    def __init__(self):
        super().__init__('Sign_detection')

        # Publishers
        self.class_pub = self.create_publisher(String, '/sign', 1)
        self.image_pub = self.create_publisher(Image, '/color/detect', 1)
        
        # Subscribers
        self.image_sub = self.create_subscription(Image, '/color/image', self.subscription_callback, 1)
        
        self.model_path = self.declare_parameter('model_path', 'model').get_parameter_value().string_value 
        self.model = torch.hub.load(os.path.join(self.model_path, 'yolov5'), 'custom', path = os.path.join(self.model_path, 'best.pt'), source = 'local') 

        self.classes = self.model.names

        # Минимальная площадь bbox для детекции знака
        self.min_square = self.declare_parameter('min_square', 0).get_parameter_value().integer_value   

        # Количество детекций для усреднения ответа
        self.detects_cnt = self.declare_parameter('detects_cnt', 0).get_parameter_value().integer_value   
        self.detects = [] # Список детектированных знаков

        self.cvBridge = CvBridge()

    def subscription_callback(self, image_msg):

        image = self.cvBridge.imgmsg_to_cv2(image_msg, image_msg.encoding)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Прогоняем изображение через модель
        result = self.model(image)    
        bbox = result.pandas().xyxy[0]

        # Проверка на детекцию
        if len(bbox['class'].values) != 0:
            sqr = (bbox['xmax'].values[0] - bbox['xmin'].values[0]) * (bbox['ymax'].values[0] - bbox['ymin'].values[0]) 
            
            if sqr > self.min_square:
                image = result.render()[0]
                cur_sign = self.classes[result.pandas().xyxy[0]['class'].values[0]]

                # Усредням результат по заданному количеству детекций
                if len(self.detects) != self.detects_cnt:
                    self.detects.append(cur_sign)
                else:
                    # Находим самый встречаемый класс
                    unique, pos = np.unique(self.detects, return_inverse = True)
                    cur_sign = unique[np.bincount(pos).argmax()] 

                    self.detects = []
                    self.class_pub.publish(String(data = cur_sign))


        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) 
        self.image_pub.publish(self.cvBridge.cv2_to_imgmsg(image,  image_msg.encoding))
                

def main():
    rclpy.init()

    node = Sign_detection()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()