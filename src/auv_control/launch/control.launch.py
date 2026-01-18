from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('auv_control')
    params_file = os.path.join(pkg_dir, 'config', 'control_params.yaml')

    return LaunchDescription([
        Node(
            package='auv_control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[params_file],
        )
    ])
