"""
ROS2 Launch file for complete AUV simulation stack.

Launches:
- Stonefish simulator with AUV scenario
- EKF state estimator
- Control node with PID controllers
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for full AUV simulation stack."""

    # Declare launch arguments for simulation
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

    # Package share directories
    auv_sim_share = FindPackageShare('auv_sim')
    stonefish_ros2_share = FindPackageShare('stonefish_ros2')

    # Paths for simulation
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

    # EKF node with parameters
    ekf_pkg_dir = get_package_share_directory('auv_ekf')
    ekf_params_file = os.path.join(ekf_pkg_dir, 'config', 'ekf_params.yaml')

    ekf_node = Node(
        package='auv_ekf',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[
            ('/auv/imu', '/auv/imu'),
            ('/auv/dvl', '/auv/dvl'),
            ('/auv/pressure', '/auv/pressure'),
        ]
    )

    # Control node with parameters
    control_pkg_dir = get_package_share_directory('auv_control')
    control_params_file = os.path.join(control_pkg_dir, 'config', 'control_params.yaml')

    control_node = Node(
        package='auv_control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[control_params_file],
    )

    # Delay EKF and control nodes to let simulation start first
    delayed_ekf = TimerAction(
        period=1.0,
        actions=[ekf_node]
    )

    delayed_control = TimerAction(
        period=1.5,
        actions=[control_node]
    )

    # Planner node
    planner_pkg_dir = get_package_share_directory('auv_planner')
    planner_mission_file = os.path.join(planner_pkg_dir, 'config', 'mission.yaml')

    planner_node = Node(
        package='auv_planner',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[{
            'mission_file': planner_mission_file,
            'arrival_radius': 1.0,
            'depth_tolerance': 0.5,
            'cruise_velocity': 0.3,
            'cruise_depth': 3.0,
            'surface_depth': 0.5,
            'control_rate': 10.0,
            'auto_start': True,
        }]
    )

    # Delay planner to let control stabilize
    delayed_planner = TimerAction(
        period=2.0,
        actions=[planner_node]
    )

    return LaunchDescription([
        # Launch arguments
        simulation_rate_arg,
        rendering_quality_arg,
        window_res_x_arg,
        window_res_y_arg,
        # Simulation first
        stonefish_launch,
        # EKF after 1 second
        delayed_ekf,
        # Control after 1.5 seconds
        delayed_control,
        # Planner after 2 seconds
        delayed_planner,
    ])
