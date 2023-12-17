import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class MoveToGoalNode(Node):

    def __init__(self):
        super().__init__('move_to_goal')
        self.sign_sub = self.create_subscription(
            String,
            '/sign',
            self.handle_sign,
            1)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            1)
        self.timer = self.create_timer(0.1, self.move_to_goal)
        self.current_pose = None
        self.target_x = 0.0 # -2.0 or -4.8 Тут нужно указать целевую координату x
        self.target_y = 3.0 # 0.0 Тут y
        self.target_theta = 0.0
        self.reached_goal = False # Флаг того, что мы добрались до нужной точки
        self.obstacle_detected = False  # Флаг для обнаружения препятствия
        self.obstacle_avoidance_in_progress = False #Флаг для определения начали мы объезд препятствия или нет
        self.buffered_target_x = None
        self.buffered_target_y = None
        self.sign_flag = False
    
    def handle_sign(self, msg):
        if msg.data == 'tunnel_sign' and not self.sign_flag:
            self.sign_flag = True
            
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def lidar_callback(self, msg):
    	#Тут мы уменьшаем угол видимости Лидара, если можно так сказать
        dop_forward = msg.ranges[-5::] + msg.ranges[:5:]
        min_distance = min(dop_forward)
        if min_distance < 2.0:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False


    def move_to_goal(self):
        if self.sign_flag:
            linear_vel = 0.0  
            angular_vel = 0.0
            if self.current_pose is not None and not self.reached_goal:
                distance_to_goal = math.sqrt((self.target_x - self.current_pose.position.x) ** 2 + (self.target_y - self.current_pose.position.y) ** 2)
                #Тут можно вывести логи, чтобы глянуть как изменяется позиция робота, как меняются целевые углы, ну и сколько еще нам осталось ехать 
                #self.get_logger().info(f"\n Coordinates: {self.current_pose}")
                #self.get_logger().info(f"\n Target: x = {self.target_x}, y = {self.target_y}!!!!! ")
                #self.get_logger().info(f"Distance to goal: {distance_to_goal}")
            
                if self.obstacle_detected and not self.obstacle_avoidance_in_progress:
                    # Сохраняем буферные целевые координаты и создаем новые для обхода препятствия
                    if self.target_x is not None and self.target_y is not None:
                        self.buffered_target_x = self.target_x
                        self.buffered_target_y = self.target_y
                        self.target_x += -1.0  # Тут зависит от величины оригинальных координат чем больше координата, тем больше сдвиг 
                        self.target_y += 2.0  # Тут так же
                        self.obstacle_avoidance_in_progress = True
                        # Устанавливаем таймер на 3 секунды для возврата к оригинальным координатам
                        self.create_timer(3.0, self.return_to_original_coordinates)
                    return

                elif not self.obstacle_detected and self.obstacle_avoidance_in_progress:
                    # Тут мы кароче выполняем обьезд препятствия по сдвинутым координатам, ну и больше сюда не вохвращаемся, если на пути предвидится одно препятствие.
                    linear_vel = 0.5 * distance_to_goal
                    angle_to_buffered_goal = math.atan2(self.buffered_target_y - self.current_pose.position.y, self.buffered_target_x - self.current_pose.position.x)
                    angular_vel = 2.0 * (angle_to_buffered_goal - self.current_pose.orientation.z)
                    self.obstacle_avoidance_in_progress = False
                    return
                     
                else:
                    # Тут мы просто движемся к нашим координатам
                    angle_to_goal = math.atan2(self.target_y - self.current_pose.position.y, self.target_x - self.current_pose.position.x)
                    linear_vel = 0.5 * distance_to_goal
                    angular_vel = 2.0 * (angle_to_goal - self.current_pose.orientation.z)

                    if distance_to_goal < 0.1:
                        self.reached_goal = True
                        linear_vel = 0.0
                        angular_vel = 0.0
                        #self.get_logger().info("Reached the goal")
                        return
                
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = linear_vel
                cmd_vel_msg.angular.z = angular_vel

                self.cmd_vel_publisher.publish(cmd_vel_msg)

    def return_to_original_coordinates(self):
        if self.buffered_target_x is not None and self.buffered_target_y is not None:
        # Возврат к оригинальным целевым координатам
            self.target_x = self.buffered_target_x
            self.target_y = self.buffered_target_y
            self.buffered_target_x = None
            self.buffered_target_y = None
            self.obstacle_avoidance_in_progress = False
            
def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

