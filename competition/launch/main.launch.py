import os

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_autorace_camera = get_package_share_directory('autorace_camera')

    autorace_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_bringup, 'launch', 'autorace_2023.launch.py')))
    
    camera_calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_autorace_camera, 'launch', 'extrinsic_camera_calibration.launch.py')))

    return LaunchDescription([
        autorace_sim,
        camera_calibration,

        TimerAction(
            period = 7.0,
            actions = [
                Node(
                    package = 'autorace_camera',
                    executable = 'core_node_mission',
                    name = 'core_node_mission')
                    ])
    ])
