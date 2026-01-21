"""ROS2 Mission Planner Node for AUV."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
import os
import math

from .state_machine import MissionStateMachine, MissionState
from .waypoint_manager import WaypointManager
from .navigation_utils import compute_heading, compute_distance, is_at_waypoint, normalize_angle


class PlannerNode(Node):
    """Mission planner node with state machine control."""

    def __init__(self):
        super().__init__('planner_node')

        # Parameters
        self.declare_parameter('mission_file', '')
        self.declare_parameter('arrival_radius', 1.0)
        self.declare_parameter('depth_tolerance', 0.5)
        self.declare_parameter('cruise_velocity', 0.3)
        self.declare_parameter('cruise_depth', 3.0)
        self.declare_parameter('surface_depth', 0.5)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('auto_start', True)

        # Get parameters
        mission_file = self.get_parameter('mission_file').get_parameter_value().string_value
        if not mission_file:
            pkg_dir = get_package_share_directory('auv_planner')
            mission_file = os.path.join(pkg_dir, 'config', 'mission.yaml')

        self._arrival_radius = self.get_parameter('arrival_radius').get_parameter_value().double_value
        self._depth_tolerance = self.get_parameter('depth_tolerance').get_parameter_value().double_value
        self._cruise_velocity = self.get_parameter('cruise_velocity').get_parameter_value().double_value
        self._cruise_depth = self.get_parameter('cruise_depth').get_parameter_value().double_value
        self._surface_depth = self.get_parameter('surface_depth').get_parameter_value().double_value
        self._auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value

        # State machine
        self._state_machine = MissionStateMachine(on_state_change=self._on_state_change)

        # Waypoint manager
        self._waypoints = WaypointManager()
        if not self._waypoints.load_from_yaml(mission_file):
            self.get_logger().error(f'Failed to load mission from {mission_file}')

        # Current state from EKF
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_depth = 0.0
        self._current_heading = 0.0
        self._ekf_received = False

        # Publishers - commands to control system
        self._depth_pub = self.create_publisher(Float64, '/auv/cmd/depth', 10)
        self._heading_pub = self.create_publisher(Float64, '/auv/cmd/heading', 10)
        self._velocity_pub = self.create_publisher(Float64, '/auv/cmd/velocity', 10)
        self._state_pub = self.create_publisher(String, '/auv/mission/state', 10)

        # Subscriber - state from EKF
        self._ekf_sub = self.create_subscription(
            Odometry, '/auv/ekf/pose', self._ekf_callback, 10)

        # Control timer
        control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / control_rate, self._control_loop)

        self.get_logger().info(f'Planner initialized with {self._waypoints.total_waypoints} waypoints')
        self.get_logger().info(f'Cruise depth: {self._cruise_depth}m, velocity: {self._cruise_velocity}m/s')

    def _on_state_change(self, old_state: MissionState, new_state: MissionState):
        """Callback when state changes."""
        self.get_logger().info(f'State: {old_state.name} -> {new_state.name}')
        self._publish_state()

    def _publish_state(self):
        """Publish current state for monitoring."""
        msg = String()
        state = self._state_machine.state
        wp = self._waypoints.get_current_waypoint()
        wp_info = f" (WP{self._waypoints.current_index + 1}/{self._waypoints.total_waypoints})" if wp else ""
        msg.data = f"{state.name}{wp_info}"
        self._state_pub.publish(msg)

    def _ekf_callback(self, msg: Odometry):
        """Process EKF state estimate."""
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        self._current_depth = msg.pose.pose.position.z  # Positive = underwater (NED)

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_heading = math.atan2(siny_cosp, cosy_cosp)

        if not self._ekf_received:
            self._ekf_received = True
            self.get_logger().info('EKF data received, planner ready')
            if self._auto_start:
                self._state_machine.start_mission()

    def _control_loop(self):
        """Main control loop - runs at control_rate Hz."""
        if not self._ekf_received:
            return

        state = self._state_machine.state

        if state == MissionState.IDLE:
            self._handle_idle()
        elif state == MissionState.DIVING:
            self._handle_diving()
        elif state == MissionState.NAVIGATING:
            self._handle_navigating()
        elif state == MissionState.SURFACING:
            self._handle_surfacing()
        elif state == MissionState.ERROR:
            self._handle_error()
        # COMPLETE: do nothing

        # Periodic state publish
        self._publish_state()

    def _handle_idle(self):
        """IDLE: Stop and wait."""
        self._send_commands(depth=0.0, heading=self._current_heading, velocity=0.0)

    def _handle_diving(self):
        """DIVING: Descend to cruise depth."""
        self._send_commands(
            depth=self._cruise_depth,
            heading=self._current_heading,  # Hold current heading
            velocity=0.0  # No forward motion while diving
        )

        # Check if depth reached
        if abs(self._current_depth - self._cruise_depth) <= self._depth_tolerance:
            self.get_logger().info(f'Cruise depth reached: {self._current_depth:.2f}m')
            self._state_machine.depth_reached()

    def _handle_navigating(self):
        """NAVIGATING: Follow waypoint sequence."""
        wp = self._waypoints.get_current_waypoint()

        if wp is None:
            # All waypoints complete
            self.get_logger().info('All waypoints complete')
            self._state_machine.waypoints_complete()
            return

        # Check if at current waypoint
        if is_at_waypoint(
            self._current_x, self._current_y, self._current_depth,
            wp.x, wp.y, wp.depth,
            self._arrival_radius, self._depth_tolerance
        ):
            self.get_logger().info(f'Reached waypoint {wp.name} at ({wp.x}, {wp.y}, {wp.depth})')
            if not self._waypoints.advance_waypoint():
                # No more waypoints
                self._state_machine.waypoints_complete()
                return
            wp = self._waypoints.get_current_waypoint()
            if wp:
                self.get_logger().info(f'Next waypoint: {wp.name} at ({wp.x}, {wp.y}, {wp.depth})')

        if wp:
            # Compute navigation commands
            target_heading = compute_heading(
                self._current_x, self._current_y, wp.x, wp.y)
            distance = compute_distance(
                self._current_x, self._current_y, wp.x, wp.y)

            self.get_logger().debug(
                f'Nav: heading={math.degrees(target_heading):.1f}deg, dist={distance:.2f}m')

            self._send_commands(
                depth=wp.depth,
                heading=target_heading,
                velocity=self._cruise_velocity
            )

    def _handle_surfacing(self):
        """SURFACING: Return to surface."""
        self._send_commands(
            depth=self._surface_depth,
            heading=self._current_heading,  # Hold current heading
            velocity=0.0  # No forward motion while surfacing
        )

        # Check if surface reached
        if self._current_depth <= self._surface_depth + self._depth_tolerance:
            self.get_logger().info(f'Surface reached: {self._current_depth:.2f}m')
            self._state_machine.surface_reached()

    def _handle_error(self):
        """ERROR: Safe surfacing."""
        self.get_logger().warn(f'ERROR state: {self._state_machine.error_message}, surfacing')
        self._send_commands(
            depth=self._surface_depth,
            heading=self._current_heading,
            velocity=0.0
        )

        # Transition to COMPLETE when surface reached
        if self._current_depth <= self._surface_depth + self._depth_tolerance:
            self._state_machine.surface_reached()

    def _send_commands(self, depth: float, heading: float, velocity: float):
        """Send commands to control system."""
        depth_msg = Float64()
        depth_msg.data = depth
        self._depth_pub.publish(depth_msg)

        heading_msg = Float64()
        heading_msg.data = heading
        self._heading_pub.publish(heading_msg)

        velocity_msg = Float64()
        velocity_msg.data = velocity
        self._velocity_pub.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
