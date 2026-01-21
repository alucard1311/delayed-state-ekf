"""
ROS2 Launch file for USBL Navigation simulation.

Launches the sensor simulators, navigation filter, path publishers, and optionally RViz.

Scenarios:
- nominal: Standard conditions (5% USBL outliers, no DVL dropout)
- canyon_dropout: DVL dropout during canyon crossing (120-150s)
- high_outlier: High USBL outlier rate (20%)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Set up launch nodes with scenario-based parameter overrides."""

    # Get launch configurations
    scenario = LaunchConfiguration('scenario').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)

    # Package paths
    usbl_nav_share = FindPackageShare('usbl_navigation')

    # Parameter files
    simulation_params = PathJoinSubstitution([
        usbl_nav_share, 'config', 'simulation_params.yaml'
    ])

    sensor_noise_params = PathJoinSubstitution([
        usbl_nav_share, 'config', 'sensor_noise.yaml'
    ])

    ekf_params = PathJoinSubstitution([
        usbl_nav_share, 'config', 'ekf_params.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        usbl_nav_share, 'config', 'rviz_config.rviz'
    ])

    # Scenario-based parameter overrides
    if scenario == 'canyon_dropout':
        enable_canyon_dropout = True
        outlier_probability = 0.05
    elif scenario == 'high_outlier':
        enable_canyon_dropout = False
        outlier_probability = 0.20
    else:  # nominal
        enable_canyon_dropout = False
        outlier_probability = 0.05

    nodes = []

    # Truth generator node
    nodes.append(Node(
        package='usbl_navigation',
        executable='truth_generator_node',
        name='truth_generator',
        output='screen',
        parameters=[simulation_params],
    ))

    # IMU simulator node
    nodes.append(Node(
        package='usbl_navigation',
        executable='imu_simulator_node',
        name='imu_simulator',
        output='screen',
        parameters=[simulation_params, sensor_noise_params],
    ))

    # DVL simulator node with scenario-based canyon dropout
    nodes.append(Node(
        package='usbl_navigation',
        executable='dvl_simulator_node',
        name='dvl_simulator',
        output='screen',
        parameters=[
            simulation_params,
            sensor_noise_params,
            {'enable_canyon_dropout': enable_canyon_dropout},
        ],
    ))

    # USBL simulator node with scenario-based outlier rate
    nodes.append(Node(
        package='usbl_navigation',
        executable='usbl_simulator_node',
        name='usbl_simulator',
        output='screen',
        parameters=[
            simulation_params,
            sensor_noise_params,
            {'usbl.outlier_probability': outlier_probability},
        ],
    ))

    # Navigation node (delayed-state EKF)
    nodes.append(Node(
        package='usbl_navigation',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[
            ekf_params,
            sensor_noise_params,
        ],
    ))

    # Path publisher for truth trajectory
    nodes.append(Node(
        package='usbl_navigation',
        executable='path_publisher_node',
        name='truth_path_publisher',
        output='screen',
        parameters=[{
            'input_topic': '/truth',
            'output_topic': '/truth_path',
            'frame_id': 'world',
            'max_points': 1000,
            'publish_rate': 2.0,
        }],
    ))

    # Path publisher for estimate trajectory
    nodes.append(Node(
        package='usbl_navigation',
        executable='path_publisher_node',
        name='estimate_path_publisher',
        output='screen',
        parameters=[{
            'input_topic': '/navigation/odometry',
            'output_topic': '/estimate_path',
            'frame_id': 'world',
            'max_points': 1000,
            'publish_rate': 2.0,
        }],
    ))

    # Metrics logger node (CSV output for plotting)
    metrics_output_file = output_dir + '/navigation_metrics.csv'
    nodes.append(Node(
        package='usbl_navigation',
        executable='metrics_logger_node',
        name='metrics_logger',
        output='screen',
        parameters=[{
            'output_file': metrics_output_file,
        }],
    ))

    # RViz (optional)
    if rviz.lower() == 'true':
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ))

    return nodes


def generate_launch_description():
    """Generate launch description for USBL navigation simulation."""

    # Declare launch arguments
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='nominal',
        description='Simulation scenario: nominal, canyon_dropout, or high_outlier',
        choices=['nominal', 'canyon_dropout', 'high_outlier']
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp',
        description='Directory for output files (metrics, logs)'
    )

    return LaunchDescription([
        # Arguments
        scenario_arg,
        rviz_arg,
        output_dir_arg,
        # Nodes via OpaqueFunction for scenario-based configuration
        OpaqueFunction(function=launch_setup),
    ])
