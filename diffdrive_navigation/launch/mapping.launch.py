import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    diffdrive_navigation_share = get_package_share_directory('diffdrive_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')

    params_file = LaunchConfiguration('params_file')
    declare_params_file_argument = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(diffdrive_navigation_share,
                                       'config', 'slam_toolbox.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    mapping_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
          params_file,
          {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_params_file_argument,
        mapping_node
    ])
