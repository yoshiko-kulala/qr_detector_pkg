import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('qr_detector_pkg')
    params_file = os.path.join(package_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='qr_detector_pkg',
            executable='qr_detector',
            name='qr_detector',
            parameters=[params_file],
            output='screen',
        )
    ])
