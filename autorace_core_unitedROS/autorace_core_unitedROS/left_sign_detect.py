import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist


class LeftSignDetector(Node):
    """ Поворот налево на перекрестке.
    Принимает распознанный знак и, если этот знак - поворот налево,
    то поворачивает робота влево, после чего он должен снова следовать по полосе."""

    def __init__(self):
        super().__init__('LeftSignDetector')

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1)

        self.enable_following_pub = self.create_publisher(
            Bool,
            '/enable_following',
            1)

        self.sign_sub = self.create_subscription(
            String,
            '/sign',
            self.handle_sign,
            1)
        
        self.timer = self.create_timer(0.1, self.rotate_robot)

        self.cnt = 0
        self.enable_rotate = False

    def handle_sign(self, msg):
        
        cur_sign = msg.data

        if cur_sign == 'turn_left_sign' and not self.enable_rotate:

            self.enable_following_pub.publish(Bool(data = False))
            self.enable_rotate = True

    def rotate_robot(self):

        if self.enable_rotate:

            twist = Twist()
            twist.linear.x = 0.2  
            twist.angular.z = 3.0

            self.cmd_vel_pub.publish(twist)

            self.cnt += 1

            if self.cnt == 15:
                self.enable_following_pub.publish(Bool(data = True))
                rclpy.shutdown()

        
def main():
    rclpy.init()

    node = LeftSignDetector()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()