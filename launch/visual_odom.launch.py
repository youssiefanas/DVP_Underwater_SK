import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Topic for the image input'
    )

    return LaunchDescription([
        image_topic_arg,
        Node(
            package='dv_slam',
            executable='vo_node',
            name='visual_odom_node',
            output='screen',
            parameters=[
                visual_odom_config, 
                orb_params_config,
                {'image_topic': LaunchConfiguration('image_topic')}
            ]
        )
    ])
