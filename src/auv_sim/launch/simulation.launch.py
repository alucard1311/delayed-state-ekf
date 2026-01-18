"""
ROS2 Launch file for AUV Stonefish simulation.

Launches the Stonefish simulator with the AUV scenario.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for AUV simulation."""

    # Declare launch arguments
    simulation_rate_arg = DeclareLaunchArgument(
        'simulation_rate',
        default_value='100.0',
        description='Simulation rate in Hz'
    )

    rendering_quality_arg = DeclareLaunchArgument(
        'rendering_quality',
        default_value='medium',
        description='Rendering quality: low, medium, or high'
    )

    window_res_x_arg = DeclareLaunchArgument(
        'window_res_x',
        default_value='1280',
        description='Window width in pixels'
    )

    window_res_y_arg = DeclareLaunchArgument(
        'window_res_y',
        default_value='720',
        description='Window height in pixels'
    )

    # Paths
    auv_sim_share = FindPackageShare('auv_sim')
    stonefish_ros2_share = FindPackageShare('stonefish_ros2')

    scenario_desc = PathJoinSubstitution([
        auv_sim_share, 'description', 'auv.xml'
    ])

    simulation_data = PathJoinSubstitution([
        stonefish_ros2_share, 'data'
    ])

    # Include stonefish_ros2 simulator launch file
    stonefish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                stonefish_ros2_share,
                'launch',
                'stonefish_simulator.launch.py'
            ])
        ]),
        launch_arguments={
            'scenario_desc': scenario_desc,
            'simulation_data': simulation_data,
            'simulation_rate': LaunchConfiguration('simulation_rate'),
            'rendering_quality': LaunchConfiguration('rendering_quality'),
            'window_res_x': LaunchConfiguration('window_res_x'),
            'window_res_y': LaunchConfiguration('window_res_y'),
        }.items()
    )

    return LaunchDescription([
        simulation_rate_arg,
        rendering_quality_arg,
        window_res_x_arg,
        window_res_y_arg,
        stonefish_launch,
    ])
