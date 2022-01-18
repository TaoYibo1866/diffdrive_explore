import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    diffdrive_description_share = get_package_share_directory('diffdrive_description')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model',
                              default_value=os.path.join(diffdrive_description_share, 'urdf', 'diffdrive.urdf'),
                              description='urdf file path'),
        robot_state_publisher_node,
    ])