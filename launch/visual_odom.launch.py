import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    visual_odom_config = os.path.join(
        get_package_share_directory('dv_slam'),
        'config',
        'visual_odom.yaml'
    )
    
    orb_params_config = os.path.join(
        get_package_share_directory('dv_slam'),
        'config',
        'orb_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='dv_slam',
            executable='vo_node',
            name='visual_odom_node',
            output='screen',
            parameters=[visual_odom_config, orb_params_config]
        )
    ])
