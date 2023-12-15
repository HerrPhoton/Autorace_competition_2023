import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from cv_bridge import CvBridge


class Finish_Handler(Node):
    """ Останавливает робота после пересечения финишной линии."""

    def __init__(self):
        super().__init__('Finish_Handler')

        self.enable_following_pub = self.create_publisher(
            Bool,
            '/enable_following',
            1)
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1)
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.get_odom,
            1)
        
        self.finish_enable_sub = self.create_subscription(
            Odometry,
            '/finish_enable',
            self.finish_state,
            1)

        self.finish_enable = False

    def get_odom(self, msg):

        if self.finish_enable:

            pose_x = msg.pose.pose.position.x
            pose_y = msg.pose.pose.position.y

            if pose_x >= 0 and pose_y >= 0:
                self.enable_following_pub.publish(Bool(data = False))
                self.cmd_vel_pub.publish(Twist())

    def finish_state(self, msg):
        self.finish_enable = msg.data


def main():
    rclpy.init()

    node = Finish_Handler()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()