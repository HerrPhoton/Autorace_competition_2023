import os

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_autorace_core = get_package_share_directory('autorace_core_unitedROS')
    pkg_autorace_camera = get_package_share_directory('autorace_camera')

    autorace_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_bringup, 'launch', 'autorace_2023.launch.py')))
    
    camera_calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_autorace_camera, 'launch', 'extrinsic_camera_calibration.launch.py')))
    
    base_config = os.path.join(pkg_autorace_core, 'config', 'base.yaml')
    intersection_config = os.path.join(pkg_autorace_core, 'config', 'intersection.yaml')
    obstacles_config = os.path.join(pkg_autorace_core, 'config', 'obstacles.yaml')
    parking_config = os.path.join(pkg_autorace_core, 'config', 'parking.yaml')
    pedestrian_crossing_config = os.path.join(pkg_autorace_core, 'config', 'pedestrian_crossing.yaml')
    tunnel_config = os.path.join(pkg_autorace_core, 'config', 'tunnel.yaml')

    sign_detection = Node(
        package = 'autorace_core_unitedROS',
        executable = 'sign_detection',
        name = 'sign_detection',
        parameters=[
            {'model_path': os.path.join(pkg_autorace_core, 'model')},
            base_config])
    
    lane_detect = Node(
        package = 'autorace_core_unitedROS',
        executable = 'lane_detect',
        name = 'lane_detect',
        parameters = [base_config])
    
    lane_follow = Node(
        package = 'autorace_core_unitedROS',
        executable = 'lane_follow',
        name = 'lane_follow',
        parameters = [base_config])
    
    robot_rotator = Node(
        package = 'autorace_core_unitedROS',
        executable = 'robot_rotator',
        name = 'robot_rotator')
    
    traffic_light = Node(
        package = 'autorace_core_unitedROS',
        executable = 'traffic_light',
        name = 'traffic_light')
    
    intersection = Node(
        package = 'autorace_core_unitedROS',
        executable = 'intersection',
        name = 'intersection',
        parameters = [intersection_config])
    
    obstacles = Node(
        package = 'autorace_core_unitedROS',
        executable = 'obstacles',
        name = 'obstacles',
        parameters = [obstacles_config])
    
    parking = Node(
        package = 'autorace_core_unitedROS',
        executable = 'parking',
        name = 'parking',
        parameters = [parking_config])
    
    pedestrian_crossing = Node(
        package = 'autorace_core_unitedROS',
        executable = 'pedestrian_crossing',
        name = 'pedestrian_crossing',
        parameters = [pedestrian_crossing_config])
    
    tunnel = Node(
        package = 'autorace_core_unitedROS',
        executable = 'tunnel',
        name = 'tunnel',
        parameters = [tunnel_config])
    
    finish = Node(
        package = 'autorace_core_unitedROS',
        executable = 'finish',
        name = 'finish')

    return LaunchDescription([
        autorace_sim,
        camera_calibration,

        # Постоянно работающие ноды
        sign_detection,
        lane_detect,
        lane_follow,
        robot_rotator,

        # Ноды испытаний
        traffic_light,
        intersection,
        obstacles,
        parking,
        pedestrian_crossing,
        tunnel,
        finish,

        TimerAction(
            period = 5.0,
            actions = [Node(
                package = 'referee_console',
                executable = 'mission_autorace_2023_referee',
                name = 'mission_autorace_2023_referee')]),
    ])
