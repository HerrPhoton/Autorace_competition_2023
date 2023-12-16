import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float64, Int8
from nav_msgs.msg import Odometry
from robot_rotate_interface.msg import Rotate


class Intersection_Handler(Node):
    """ Обработка перекрестка.
    При знаке "поворот налево" или "поворот направо" принудительно поворачивает робота в соответствующую сторону,
    после чего робот снова следует вдоль полосы по PID. """

    def __init__(self):
        super().__init__('Intersection_Handler')

        self.enable_following_pub = self.create_publisher(
            Bool,
            '/enable_following',
            1)
        
        self.enable_detection_pub = self.create_publisher(
            Bool,
            '/enable_detection',
            1)

        self.max_vel_pub = self.create_publisher(
            Float64,
            '/max_vel',
            1)

        self.offset_pub = self.create_publisher(
            Float64,
            '/offset',
            1)
        
        self.rotate_pub = self.create_publisher(
            Rotate,
            '/rotate',
            1)
        
        self.rotate_done_sub = self.create_subscription(
            Int8,
            '/rotate_done',
            self.set_rotate_done,
            1)
        
        self.sign_sub = self.create_subscription(
            String,
            '/sign',
            self.handle_sign,
            1)
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.get_odom,
            1)
        
        self.ID = 0

        self.rotated = False

        # Скорости движения по перекрестку
        self.interIn_speed = self.declare_parameter('interIn_speed', 0.0).get_parameter_value().double_value
        self.inter_speed = self.declare_parameter('inter_speed', 0.0).get_parameter_value().double_value
        self.interOut_speed = self.declare_parameter('interOut_speed', 0.0).get_parameter_value().double_value

        # Смещения
        self.interIn_offset = self.declare_parameter('interIn_offset', 0.0).get_parameter_value().double_value
        self.interL_offset = self.declare_parameter('interL_offset', 0.0).get_parameter_value().double_value
        self.interR_offset = self.declare_parameter('interR_offset', 0.0).get_parameter_value().double_value
        self.interOut_offset = self.declare_parameter('interOut_offset', 0.0).get_parameter_value().double_value       

    def handle_sign(self, msg):
        
        cur_sign = msg.data

        if not self.rotated:

            # Действия при проезде знака перекрестка
            if cur_sign == 'intersection_sign':
                self.max_vel_pub.publish(Float64(data = self.interIn_speed))
                self.offset_pub.publish(Float64(data = self.interIn_offset))

            # Действия при повороте налево или направо
            if (cur_sign == 'turn_left_sign' or cur_sign == 'turn_right_sign'):

                self.max_vel_pub.publish(Float64(data = self.inter_speed))
                self.enable_detection_pub.publish(Bool(data = False))

                self.rotated = True

                if cur_sign == 'turn_left_sign':
                    self.enable_following_pub.publish(Bool(data = False))
                    self.offset_pub.publish(Float64(data = self.interL_offset))
                    self.rotate_pub.publish(Rotate(angle = 80.0, linear_x = 0.2, angular_z = 1.0, id = self.ID))

                elif cur_sign == 'turn_right_sign':
                    self.offset_pub.publish(Float64(data = self.interR_offset))

    def get_odom(self, msg):

        pose_x = msg.pose.pose.position.x

        # Изменить скорость и смещение при выезде из перекрестка
        if pose_x <= -0.1:
            self.offset_pub.publish(Float64(data = self.interOut_offset))
            self.max_vel_pub.publish(Float64(data = self.interOut_speed))
            self.enable_detection_pub.publish(Bool(data = True))
            rclpy.shutdown()

    def set_rotate_done(self, msg):
        if msg.data == self.ID:
            self.enable_following_pub.publish(Bool(data = True))


def main():
    rclpy.init()

    node = Intersection_Handler()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()