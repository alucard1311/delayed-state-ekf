"""Launch file for AUV mission planner."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for planner node."""

    pkg_dir = get_package_share_directory('auv_planner')
    default_mission = os.path.join(pkg_dir, 'config', 'mission.yaml')

    # Launch arguments
    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=default_mission,
        description='Path to mission YAML file'
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Auto-start mission when EKF data received'
    )

    # Planner node
    planner_node = Node(
        package='auv_planner',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[{
            'mission_file': LaunchConfiguration('mission_file'),
            'arrival_radius': 1.0,
            'depth_tolerance': 0.5,
            'cruise_velocity': 0.3,
            'cruise_depth': 3.0,
            'surface_depth': 0.5,
            'control_rate': 10.0,
            'auto_start': LaunchConfiguration('auto_start'),
        }]
    )

    return LaunchDescription([
        mission_file_arg,
        auto_start_arg,
        planner_node,
    ])
