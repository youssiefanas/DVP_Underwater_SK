import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('dv_slam'),
        'config'
    )
    dv_slam_config = os.path.join(config_dir, 'dv_slam.yaml')
    dataset_config = PathJoinSubstitution(
        [config_dir, LaunchConfiguration('dataset_config')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'dataset_config',
            default_value='datasets/aquaslam.yaml',
            description='Dataset parameter file path relative to config/'
        ),
        Node(
            package='dv_slam',
            executable='vo_node',
            name='visual_odom_node',
            output='screen',
            parameters=[dv_slam_config, dataset_config, {'use_sim_time': True}]
        )
    ])
