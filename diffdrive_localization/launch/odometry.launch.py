import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('diffdrive_localization')

    wheel_odometry_node = Node(
        package='diffdrive_localization',
        executable='wheel_odometry',
        name='wheel_odometry',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='True',
            description='Flag to enable use_sim_time'),
        
        wheel_odometry_node,
        robot_localization_node
    ])