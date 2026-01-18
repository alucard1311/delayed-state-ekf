# Copyright 2026 Vinay
# SPDX-License-Identifier: MIT

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for EKF node."""
    pkg_dir = get_package_share_directory('auv_ekf')
    params_file = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')

    return LaunchDescription([
        Node(
            package='auv_ekf',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('/auv/imu', '/auv/imu'),
                ('/auv/dvl', '/auv/dvl'),
                ('/auv/pressure', '/auv/pressure'),
            ]
        )
    ])
