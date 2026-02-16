import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    dv_slam_config = os.path.join(
        get_package_share_directory('dv_slam'),
        'config',
        'dv_slam.yaml'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/rgb/image_color',
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
                dv_slam_config,
                {'image_topic': LaunchConfiguration('image_topic')}
            ]
        )
    ])
