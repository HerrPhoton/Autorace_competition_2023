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
        
        self.ID = 0 # Идентификатор ноды

        self.rotated = False # Был ли робот повернут принудительно
        self.enable_detection = False # Разрешена ли детекция знаков поворота

        # Скорости движения по перекрестку
        self.in_speed = self.declare_parameter('in_speed', 0.0).get_parameter_value().double_value
        self.speed_L = self.declare_parameter('speed_L', 0.0).get_parameter_value().double_value
        self.speed_R = self.declare_parameter('speed_R', 0.0).get_parameter_value().double_value
        self.out_speed = self.declare_parameter('out_speed', 0.0).get_parameter_value().double_value

        # Смещения
        self.in_offset = self.declare_parameter('in_offset', 0.0).get_parameter_value().double_value
        self.L_offset = self.declare_parameter('L_offset', 0.0).get_parameter_value().double_value
        self.R_offset = self.declare_parameter('R_offset', 0.0).get_parameter_value().double_value
        self.out_offset = self.declare_parameter('out_offset', 0.0).get_parameter_value().double_value       

        # Углы поворота
        self.angle_L = self.declare_parameter('angle_L', 0.0).get_parameter_value().double_value
        self.angle_R = self.declare_parameter('angle_R', 0.0).get_parameter_value().double_value

        # Линейные скорости при повороте
        self.linear_x_L = self.declare_parameter('linear_x_L', 0.0).get_parameter_value().double_value
        self.linear_x_R = self.declare_parameter('linear_x_R', 0.0).get_parameter_value().double_value

        # Угловые скорости при повороте
        self.angular_z_L = self.declare_parameter('angular_z_L', 0.0).get_parameter_value().double_value
        self.angular_z_R = self.declare_parameter('angular_z_R', 0.0).get_parameter_value().double_value         

    def handle_sign(self, msg):
        
        cur_sign = msg.data

        if not self.rotated:

            # Действия при проезде знака перекрестка
            if cur_sign == 'intersection_sign':
                self.max_vel_pub.publish(Float64(data = self.in_speed))
                self.offset_pub.publish(Float64(data = self.in_offset))

            # Действия при повороте налево или направо
            if (cur_sign == 'turn_left_sign' or cur_sign == 'turn_right_sign') and self.enable_detection:

                self.enable_detection_pub.publish(Bool(data = False))
                self.enable_following_pub.publish(Bool(data = False))

                self.rotated = True

                if cur_sign == 'turn_left_sign':
                    self.max_vel_pub.publish(Float64(data = self.speed_L))
                    self.offset_pub.publish(Float64(data = self.L_offset))
                    self.rotate_pub.publish(Rotate(angle = self.angle_L, linear_x = self.linear_x_L, angular_z = self.angular_z_L, id = self.ID))

                elif cur_sign == 'turn_right_sign':
                    self.max_vel_pub.publish(Float64(data = self.speed_R))
                    self.offset_pub.publish(Float64(data = self.R_offset))
                    self.rotate_pub.publish(Rotate(angle = self.angle_R, linear_x = self.linear_x_R, angular_z = self.angular_z_R, id = self.ID))

    def get_odom(self, msg):

        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y

        if pose_x <= 0.93 and pose_y >= 0.86:
            self.enable_detection = True

        # Изменить скорость и смещение при выезде из перекрестка
        if pose_x <= -0.55:
            self.offset_pub.publish(Float64(data = self.out_offset))
            self.max_vel_pub.publish(Float64(data = self.out_speed))
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