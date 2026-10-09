"""One controller, one selected robot adapter, and optional Web UI."""
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from dog_adapters.stack_config import ADAPTERS, load_stack_config


def launch_stack(context):
    adapter = LaunchConfiguration('adapter').perform(context)
    robot_config = LaunchConfiguration('robot_config').perform(context)
    web_value = LaunchConfiguration('with_web').perform(context).lower()
    if web_value not in ('true', 'false'):
        raise ValueError('with_web must be true or false')
    follow, selected, web = load_stack_config(robot_config, adapter, web_value == 'true')
    package, executable, name = ADAPTERS[adapter]
    if adapter == 'm20':
        try:
            get_package_share_directory(package)
        except PackageNotFoundError as exc:
            raise RuntimeError('adapter=m20 requires the optional m20_bridge package; build/source it first') from exc
    nodes = [
        Node(package='rs_follow', executable='rs_follow_node', name='rs_follow_node',
             namespace='/', output='screen', parameters=[follow]),
        Node(package=package, executable=executable, name=name,
             namespace='/', output='screen', parameters=[selected]),
    ]
    if web is not None:
        nodes.append(Node(package='rs_follow', executable='web_ui.py', name='web_ui',
                          namespace='/', output='screen', parameters=[web]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('adapter', choices=list(ADAPTERS),
                              description='Exactly one adapter: m20, twist, unitree_sport'),
        DeclareLaunchArgument('robot_config', description='Required existing deployment YAML'),
        DeclareLaunchArgument('with_web', default_value='true', choices=['true', 'false'],
                              description='Start Web UI using the same deployment configuration'),
        OpaqueFunction(function=launch_stack),
    ])
