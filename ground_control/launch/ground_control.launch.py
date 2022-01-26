import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    diffdrive_description_share = get_package_share_directory('diffdrive_description')
    ground_control_share = get_package_share_directory('ground_control')

    load_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(diffdrive_description_share, 'launch'), '/load_description.launch.py']),
        launch_arguments={'model': os.path.join(diffdrive_description_share, 'urdf', 'diffdrive.urdf')}.items(),
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
    )

    joy_control_node = Node(
        package='ground_control',
        executable='joy_control',
        name='joy_control',
        output='screen',
    )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', os.path.join(ground_control_share, 'rviz', 'default.rviz')],
    # )

    return LaunchDescription([
        load_description_launch,
        # rviz_node,
        joy_node,
        joy_control_node
    ])
