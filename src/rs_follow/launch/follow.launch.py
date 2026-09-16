from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    active = LaunchConfiguration('active')
    with_lidar = LaunchConfiguration('with_lidar')

    # Optional: start the RoboSense SDK driver (no RViz) in the same launch.
    lidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        condition=IfCondition(with_lidar),
    )

    rs_follow_node = Node(
        package='rs_follow',
        executable='rs_follow_node',
        name='rs_follow_node',
        output='screen',
        parameters=[params_file, {'active': active}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rs_follow'), 'config', 'follow_params.yaml'
            ]),
            description='rs_follow parameter file'),
        DeclareLaunchArgument(
            'active', default_value='false',
            description='enable the follow controller at startup'),
        DeclareLaunchArgument(
            'with_lidar', default_value='false',
            description='also launch the RoboSense rslidar_sdk driver'),
        lidar_node,
        rs_follow_node,
    ])
