import os

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
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

    return LaunchDescription([
        autorace_sim, # УБРАТЬ В ФИНАЛЬНОЙ ВЕРСИИ!!!!
        camera_calibration,

        Node(
            package = 'autorace_core_unitedROS',
            executable = 'sign_detection',
            name = 'sign_detection',
            parameters=[
                {'model_path': os.path.join(pkg_autorace_core, 'model')}]),

        Node(
            package = 'autorace_core_unitedROS',
            executable = 'traffic_light_detect',
            name = 'traffic_light_detect'),
         
        Node(
            package = 'autorace_core_unitedROS',
            executable = 'lane_detect',
            name = 'lane_detect'),

        Node(
            package = 'autorace_core_unitedROS',
            executable = 'lane_follow',
            name = 'lane_follow'),

        Node(
            package = 'autorace_core_unitedROS',
            executable = 'left_sign_detect',
            name = 'left_sign_detect'),

        # Node(
        #     package = 'autorace_core_unitedROS',
        #     executable = 'avoid_obstacles',
        #     name = 'avoid_obstacles'),  
        
        # УБРАТЬ В ФИНАЛЬНОЙ ВЕРСИИ!!!!
        Node(
            package = 'referee_console',
            executable = 'mission_autorace_2023_referee',
            name = 'mission_autorace_2023_referee'),

    ])
