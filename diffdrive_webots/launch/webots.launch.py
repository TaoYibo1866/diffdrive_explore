import os
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    diffdrive_webots_share = get_package_share_directory('diffdrive_webots')
    
    world = LaunchConfiguration('world')
    declare_world_argument = DeclareLaunchArgument(
            'world',
            default_value='diffdrive_navigation.wbt',
            description='Choose one of the world files from `/diffdrive_webots/worlds` directory'
    )

    robot_description_file = LaunchConfiguration('robot_description_file')
    declare_robot_description_file_argument = DeclareLaunchArgument(
            'robot_description_file',
            default_value=os.path.join(diffdrive_webots_share, 'resource', 'diffdrive_webots.urdf'),
            description='Choose one of the urdf files from `/diffdrive_webots/resource` directory'
    )

    webots = WebotsLauncher(
        world=PathJoinSubstitution([diffdrive_webots_share, 'worlds', world]),
        gui='true',
        mode='realtime'
    )

    robot_driver_node = Node(
        package='webots_ros2_driver',
        executable='driver',
        name='webots_ros2_driver',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro ', robot_description_file])},
            {'use_sim_time': True}
        ]
    )

    static_transform_publisher_node1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher1',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        arguments=['0', '0', '0.1', '0', '-0.1', '0', 'base_link', 'camera']
    )

    static_transform_publisher_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher2',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser'],
    )

    return LaunchDescription([
        declare_world_argument,
        declare_robot_description_file_argument,
        webots,
        robot_driver_node,
        static_transform_publisher_node1,
        static_transform_publisher_node2
    ])