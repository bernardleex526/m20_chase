import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    cfg = share('champ_config')
    description = os.path.join(os.path.dirname(__file__), 'champ_lidar.urdf.xacro')
    def include(pkg, filename, args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share(pkg), 'launch', filename)), launch_arguments=args.items())
    return LaunchDescription([
        include('champ_bringup', 'bringup.launch.py', {
            'use_sim_time': 'true', 'gazebo': 'true', 'rviz': 'false',
            'joints_map_path': cfg + '/config/joints/joints.yaml',
            'robot_name': 'champ',
            'description_path': description,
            'links_map_path': cfg + '/config/links/links.yaml',
            'gait_config_path': cfg + '/config/gait/gait.yaml',
            'publish_joint_states': 'false', 'publish_foot_contacts': 'false',
            'joint_controller_topic': '/joint_group_effort_controller/joint_trajectory',
        }),
        include('champ_gazebo', 'gazebo.launch.py', {
            'use_sim_time': 'true', 'headless': 'True', 'gui': 'False',
            'world': os.path.join(os.path.dirname(__file__), 'follow.world'),
            'world_init_heading': '0.0',
            'robot_name': 'champ',
            'description_path': description,
        }),
    ])
