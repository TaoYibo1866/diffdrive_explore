from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    include_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/webots.launch.py']
        ),
        launch_arguments={'world': 'diffdrive_navigation.wbt'}.items(),
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )

    joy_control = Node(
        package='diffdrive_webots',
        executable='joy_control',
        name='joy_control'
    )

    return LaunchDescription([
        include_robot_launch,
        joy_node,
        joy_control
    ])
