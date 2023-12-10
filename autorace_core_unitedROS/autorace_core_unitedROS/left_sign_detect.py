import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64
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
        
        self.sign_sub = self.create_subscription(
            String,
            '/sign',
            self.rotate_robot,
            1)
        
    def rotate_robot(self, msg):
        
        cur_sign = msg.data

        if cur_sign == 'turn_left_sign' or cur_sign == 'turn_right_sign':

            for _ in range(50): 
            
                twist = Twist()
                twist.linear.x = 0.20   
                twist.angular.z = 2.0

                self.cmd_vel_pub.publish(twist)

            rclpy.shutdown()


def main():
    rclpy.init()

    node = LeftSignDetector()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()