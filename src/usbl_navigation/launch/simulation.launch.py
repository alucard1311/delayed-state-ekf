"""
ROS2 Launch file for USBL Navigation simulation.

Launches the sensor simulators and navigation filter for USBL navigation testing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for USBL navigation simulation."""

    # Declare launch arguments
    enable_canyon_dropout_arg = DeclareLaunchArgument(
        'enable_canyon_dropout',
        default_value='true',
        description='Enable DVL dropout during canyon scenario'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp',
        description='Directory for output files (metrics, logs)'
    )

    # Package paths
    usbl_nav_share = FindPackageShare('usbl_navigation')

    # Parameter files
    simulation_params = PathJoinSubstitution([
        usbl_nav_share, 'config', 'simulation_params.yaml'
    ])

    sensor_noise_params = PathJoinSubstitution([
        usbl_nav_share, 'config', 'sensor_noise.yaml'
    ])

    # Truth generator node
    truth_generator_node = Node(
        package='usbl_navigation',
        executable='truth_generator_node',
        name='truth_generator',
        output='screen',
        parameters=[simulation_params],
    )

    # IMU simulator node
    imu_simulator_node = Node(
        package='usbl_navigation',
        executable='imu_simulator_node',
        name='imu_simulator',
        output='screen',
        parameters=[simulation_params, sensor_noise_params],
    )

    # DVL simulator node
    dvl_simulator_node = Node(
        package='usbl_navigation',
        executable='dvl_simulator_node',
        name='dvl_simulator',
        output='screen',
        parameters=[
            simulation_params,
            sensor_noise_params,
            {'enable_canyon_dropout': LaunchConfiguration('enable_canyon_dropout')},
        ],
    )

    # USBL simulator node
    usbl_simulator_node = Node(
        package='usbl_navigation',
        executable='usbl_simulator_node',
        name='usbl_simulator',
        output='screen',
        parameters=[simulation_params, sensor_noise_params],
    )

    # EKF parameters
    ekf_params = PathJoinSubstitution([
        usbl_nav_share, 'config', 'ekf_params.yaml'
    ])

    # Navigation node (delayed-state EKF)
    navigation_node = Node(
        package='usbl_navigation',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[
            ekf_params,
            sensor_noise_params,
        ],
    )

    # Future nodes (commented out until implemented):
    # --------------------------------------------------------------------------

    # Metrics logger node (Phase 8)
    # metrics_logger_node = Node(
    #     package='usbl_navigation',
    #     executable='metrics_logger_node',
    #     name='metrics_logger',
    #     output='screen',
    #     parameters=[
    #         {'output_dir': LaunchConfiguration('output_dir')},
    #     ],
    # )

    return LaunchDescription([
        # Arguments
        enable_canyon_dropout_arg,
        output_dir_arg,
        # Sensor simulators
        truth_generator_node,
        imu_simulator_node,
        dvl_simulator_node,
        usbl_simulator_node,
        # Navigation filter
        navigation_node,
        # metrics_logger_node,     # Phase 8
    ])
