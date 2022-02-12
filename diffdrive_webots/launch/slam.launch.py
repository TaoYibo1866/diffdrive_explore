import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node

def generate_launch_description():
    diffdrive_webots_share = get_package_share_directory('diffdrive_webots')
    diffdrive_navigation_share = get_package_share_directory('diffdrive_navigation')
    ground_control_share = get_package_share_directory('ground_control')

    webots_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(diffdrive_webots_share, 'launch'), '/webots.launch.py']),
                launch_arguments={'world': 'diffdrive_navigation.wbt'}.items(),
            )
        ]
    )

    odometry_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(diffdrive_navigation_share, 'launch'), '/odometry.launch.py']),
                launch_arguments={'use_sim_time': 'true'}.items(),
            )
        ]
    )

    mapping_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(diffdrive_navigation_share, 'launch'), '/mapping.launch.py']),
                launch_arguments={'use_sim_time': 'true'}.items(),
            )
        ]
    )

    ground_control_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(ground_control_share, 'launch'), '/ground_control.launch.py'])
            )
        ]
    )

    return LaunchDescription([
        webots_launch,
        odometry_launch,
        mapping_launch,
        # ground_control_launch
    ])
