import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from numpy import min

class Movement_publisher(Node):

    def __init__(self):
        super().__init__('Movement_publisher')

        self.pub = self.create_publisher(
            Twist,
            '/robot/cmd_vel',
            10)
        
        self.sub = self.create_subscription(
            Image,
            '/depth/image',
            self.subscription_callback,
            10
        )

        self.cv_bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.publisher_callback)
        self.is_obstacle = False

    def subscription_callback(self, msg):
        image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)

        image = image[int(msg.height * 0.4) : -int(msg.height * 0.4), 
                      int(msg.width * 0.4)  : -int(msg.width * 0.4)]
        
        distance = min(image)
        self.is_obstacle = distance < 2.0

    def publisher_callback(self):
        twist = Twist()
        
        if not self.is_obstacle:
            twist.linear.x = 1.0

        self.pub.publish(twist)

def main():
    rclpy.init()

    node = Movement_publisher()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()