import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    diffdrive_description_share = get_package_share_directory('diffdrive_description')

    load_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(diffdrive_description_share, 'launch'), '/load_description.launch.py']),
        launch_arguments={'model': os.path.join(diffdrive_description_share, 'urdf', 'diffdrive.urdf')}.items(),
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(diffdrive_description_share, 'rviz', 'default.rviz')],
    )

    return LaunchDescription([
        load_description_launch,
        joint_state_publisher_gui_node,
        rviz_node
    ])