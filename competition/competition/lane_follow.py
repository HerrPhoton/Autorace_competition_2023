import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class LaneFollower(Node):
    """ Программа следования по полосе.
    Принимает координату центра между белой и желтой линиями, после чего
    производит корректировку направления движения, чтобы робот двигался по цетру полосы."""

    def __init__(self):
        super().__init__('LaneFollower')

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        self.sub = self.create_subscription(
            Float64,
            '/lane/center',
            self.subscription_callback,
            10
        )

        self.last_error = 0
        self.max_vel = 0.12

    def subscription_callback(self, msg):
        center = msg.data
        error = center

        Kp = 0.0025
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        twist = Twist()   
        twist.linear.x = min(self.max_vel * ((1 - abs(error) / 500) ** 2.2), 0.2)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)

        self.pub.publish(twist)


def main():
    rclpy.init()

    node = LaneFollower()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()