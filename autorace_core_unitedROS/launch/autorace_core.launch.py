import os

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
    
    params = os.path.join(pkg_autorace_core, 'config', 'params.yaml')

    sign_detection = Node(
        package = 'autorace_core_unitedROS',
        executable = 'sign_detection',
        name = 'sign_detection',
        parameters=[
            {'model_path': os.path.join(pkg_autorace_core, 'model')},
            params])
    
    lane_detect = Node(
        package = 'autorace_core_unitedROS',
        executable = 'lane_detect',
        name = 'lane_detect',
        parameters = [params])
    
    lane_follow = Node(
        package = 'autorace_core_unitedROS',
        executable = 'lane_follow',
        name = 'lane_follow',
        parameters = [params])
    
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
        parameters = [params])
    
    avoid_obstacles = Node(
        package = 'autorace_core_unitedROS',
        executable = 'avoid_obstacles',
        name = 'avoid_obstacles',
        parameters = [params])
    
    parking = Node(
        package = 'autorace_core_unitedROS',
        executable = 'parking',
        name = 'parking')
    
    pedestrian_crossing = Node(
        package = 'autorace_core_unitedROS',
        executable = 'pedestrian_crossing',
        name = 'pedestrian_crossing')
      
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
        avoid_obstacles,
        parking,
        pedestrian_crossing,
        finish,

        TimerAction(
            period = 5.0,
            actions = [Node(
                package = 'referee_console',
                executable = 'mission_autorace_2023_referee',
                name = 'mission_autorace_2023_referee')]),
    ])
