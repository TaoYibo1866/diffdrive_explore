import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    diffdrive_navigation_share = get_package_share_directory('diffdrive_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_argument = DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation clock')

    params_file = LaunchConfiguration('params_file')
    declare_params_file_argument = DeclareLaunchArgument(
            name='params_file', 
            default_value=os.path.join(diffdrive_navigation_share, 'config', 'robot_localization.yaml'),
            description='Full path to the ROS2 parameters file to use for the robot_localization node')

    wheel_odometry_node = Node(
        package='diffdrive_navigation',
        executable='wheel_odometry',
        name='wheel_odometry',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_params_file_argument,
        wheel_odometry_node,
        robot_localization_node
    ])