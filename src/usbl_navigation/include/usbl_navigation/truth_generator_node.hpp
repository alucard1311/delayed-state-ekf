#ifndef USBL_NAVIGATION__TRUTH_GENERATOR_NODE_HPP_
#define USBL_NAVIGATION__TRUTH_GENERATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <string>

namespace usbl_navigation
{

/**
 * @brief Trajectory segment types for lawnmower pattern
 */
enum class TrajectorySegment
{
  STRAIGHT,
  TURNING
};

/**
 * @brief Truth generator node that publishes ground truth trajectory
 *
 * Generates a lawnmower survey pattern at configurable depth with smooth
 * circular turns. Publishes odometry at 100Hz and path at 1Hz.
 */
class TruthGeneratorNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Truth Generator Node
   */
  explicit TruthGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Timer callback for trajectory update
   */
  void trajectoryCallback();

  /**
   * @brief Timer callback for path publishing
   */
  void pathCallback();

  /**
   * @brief Update position along straight segment
   * @param dt Time step in seconds
   */
  void updateStraight(double dt);

  /**
   * @brief Update position along turning segment
   * @param dt Time step in seconds
   */
  void updateTurning(double dt);

  /**
   * @brief Initialize turn parameters when entering turn
   */
  void initializeTurn();

  /**
   * @brief Compute quaternion from heading angle
   * @param heading Heading angle in radians (0 = North, positive = clockwise)
   * @return Quaternion representing orientation
   */
  Eigen::Quaterniond headingToQuaternion(double heading) const;

  /**
   * @brief Publish odometry message
   */
  void publishOdometry();

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr trajectory_timer_;
  rclcpp::TimerBase::SharedPtr path_timer_;

  // Parameters
  double vehicle_speed_;      // m/s
  double survey_depth_;       // meters (NED: positive down)
  double line_length_;        // meters
  double line_spacing_;       // meters
  int num_lines_;
  double turn_radius_;        // meters
  double publish_rate_;       // Hz

  // Trajectory state
  Eigen::Vector3d position_;        // NED position [x, y, z]
  Eigen::Vector3d velocity_;        // Body-frame velocity [vx, vy, vz]
  Eigen::Quaterniond orientation_;  // Orientation quaternion
  double heading_;                  // Current heading (radians)
  double angular_velocity_;         // Current angular velocity (rad/s)

  // Mission state
  TrajectorySegment current_segment_;
  int current_line_;                // Current survey line (0-indexed)
  double segment_progress_;         // Progress along current segment (meters or radians)
  bool moving_positive_x_;          // Direction along survey line
  int turn_phase_;                  // 0-3 for four quarter turns in full turn

  // Turn state
  Eigen::Vector3d turn_center_;     // Center of turn arc
  double turn_start_heading_;       // Heading at start of turn
  double turn_angle_progress_;      // Accumulated turn angle (radians)
  double target_turn_angle_;        // Total angle to turn (radians)

  // Path accumulator
  std::vector<geometry_msgs::msg::PoseStamped> path_poses_;

  // Timing
  rclcpp::Time last_update_time_;
  bool mission_complete_;
};

}  // namespace usbl_navigation

#endif  // USBL_NAVIGATION__TRUTH_GENERATOR_NODE_HPP_
