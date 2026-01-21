#include "usbl_navigation/truth_generator_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>

namespace usbl_navigation
{

TruthGeneratorNode::TruthGeneratorNode(const rclcpp::NodeOptions & options)
: Node("truth_generator_node", options),
  position_(Eigen::Vector3d::Zero()),
  velocity_(Eigen::Vector3d::Zero()),
  orientation_(Eigen::Quaterniond::Identity()),
  heading_(0.0),
  angular_velocity_(0.0),
  current_segment_(TrajectorySegment::STRAIGHT),
  current_line_(0),
  segment_progress_(0.0),
  moving_positive_x_(true),
  turn_phase_(0),
  turn_center_(Eigen::Vector3d::Zero()),
  turn_start_heading_(0.0),
  turn_angle_progress_(0.0),
  target_turn_angle_(0.0),
  mission_complete_(false)
{
  // Declare and get parameters
  this->declare_parameter("vehicle_speed", 1.5);
  this->declare_parameter("survey_depth", 50.0);
  this->declare_parameter("line_length", 100.0);
  this->declare_parameter("line_spacing", 10.0);
  this->declare_parameter("num_lines", 5);
  this->declare_parameter("turn_radius", 5.0);
  this->declare_parameter("publish_rate", 100.0);

  vehicle_speed_ = this->get_parameter("vehicle_speed").as_double();
  survey_depth_ = this->get_parameter("survey_depth").as_double();
  line_length_ = this->get_parameter("line_length").as_double();
  line_spacing_ = this->get_parameter("line_spacing").as_double();
  num_lines_ = this->get_parameter("num_lines").as_int();
  turn_radius_ = this->get_parameter("turn_radius").as_double();
  publish_rate_ = this->get_parameter("publish_rate").as_double();

  // Initialize position at survey depth (NED: +Z is down)
  position_ = Eigen::Vector3d(0.0, 0.0, survey_depth_);

  // Initialize velocity (moving along +X initially)
  velocity_ = Eigen::Vector3d(vehicle_speed_, 0.0, 0.0);

  // Initialize orientation (heading North = +X)
  heading_ = 0.0;
  orientation_ = headingToQuaternion(heading_);

  // Create publishers
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/truth/odometry", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/truth/path", 10);

  // Create timers
  auto trajectory_period = std::chrono::duration<double>(1.0 / publish_rate_);
  trajectory_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(trajectory_period),
    std::bind(&TruthGeneratorNode::trajectoryCallback, this));

  path_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&TruthGeneratorNode::pathCallback, this));

  last_update_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Truth generator initialized");
  RCLCPP_INFO(this->get_logger(), "  Vehicle speed: %.2f m/s", vehicle_speed_);
  RCLCPP_INFO(this->get_logger(), "  Survey depth: %.1f m", survey_depth_);
  RCLCPP_INFO(this->get_logger(), "  Line length: %.1f m", line_length_);
  RCLCPP_INFO(this->get_logger(), "  Line spacing: %.1f m", line_spacing_);
  RCLCPP_INFO(this->get_logger(), "  Num lines: %d", num_lines_);
  RCLCPP_INFO(this->get_logger(), "  Turn radius: %.1f m", turn_radius_);
  RCLCPP_INFO(this->get_logger(), "  Publish rate: %.1f Hz", publish_rate_);
}

void TruthGeneratorNode::trajectoryCallback()
{
  if (mission_complete_) {
    // Keep publishing final position
    publishOdometry();
    return;
  }

  // Calculate time step
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();
  last_update_time_ = current_time;

  // Clamp dt to avoid large jumps
  if (dt > 0.1) {
    dt = 0.1;
  }
  if (dt <= 0.0) {
    dt = 1.0 / publish_rate_;
  }

  // Update trajectory based on current segment
  switch (current_segment_) {
    case TrajectorySegment::STRAIGHT:
      updateStraight(dt);
      break;
    case TrajectorySegment::TURNING:
      updateTurning(dt);
      break;
  }

  // Publish odometry
  publishOdometry();
}

void TruthGeneratorNode::updateStraight(double dt)
{
  // Move forward at vehicle speed
  double distance = vehicle_speed_ * dt;
  segment_progress_ += distance;

  // Update position based on heading
  double dx = distance * std::cos(heading_);
  double dy = distance * std::sin(heading_);
  position_.x() += dx;
  position_.y() += dy;

  // Body-frame velocity (forward motion only)
  velocity_ = Eigen::Vector3d(vehicle_speed_, 0.0, 0.0);
  angular_velocity_ = 0.0;

  // Check if we've completed the line
  if (segment_progress_ >= line_length_) {
    // Check if mission is complete
    if (current_line_ >= num_lines_ - 1) {
      mission_complete_ = true;
      velocity_ = Eigen::Vector3d::Zero();
      RCLCPP_INFO(this->get_logger(), "Mission complete! Final position: [%.2f, %.2f, %.2f]",
        position_.x(), position_.y(), position_.z());
      return;
    }

    // Start turning to next line
    current_segment_ = TrajectorySegment::TURNING;
    turn_phase_ = 0;
    initializeTurn();

    RCLCPP_DEBUG(this->get_logger(), "Completed line %d, starting turn at [%.2f, %.2f]",
      current_line_, position_.x(), position_.y());
  }
}

void TruthGeneratorNode::updateTurning(double dt)
{
  // Calculate angular velocity for smooth turn
  // omega = v / r
  double omega = vehicle_speed_ / turn_radius_;
  angular_velocity_ = moving_positive_x_ ? omega : -omega;  // Turn direction depends on line direction

  // Update turn progress
  double angle_increment = omega * dt;
  turn_angle_progress_ += angle_increment;

  // Update heading
  heading_ += angular_velocity_ * dt;

  // Normalize heading to [-pi, pi]
  while (heading_ > M_PI) heading_ -= 2.0 * M_PI;
  while (heading_ < -M_PI) heading_ += 2.0 * M_PI;

  // Update position along circular arc
  // Position relative to turn center
  double arc_angle = turn_start_heading_ + (moving_positive_x_ ? 1.0 : -1.0) * turn_angle_progress_;
  if (turn_phase_ == 0) {
    // First quarter turn (90 degrees)
    position_.x() = turn_center_.x() + turn_radius_ * std::cos(arc_angle - M_PI_2 * (moving_positive_x_ ? 1.0 : -1.0));
    position_.y() = turn_center_.y() + turn_radius_ * std::sin(arc_angle - M_PI_2 * (moving_positive_x_ ? 1.0 : -1.0));
  } else {
    // Second quarter turn (90 degrees) - moving laterally
    position_.x() = turn_center_.x() + turn_radius_ * std::cos(arc_angle - M_PI_2 * (moving_positive_x_ ? 1.0 : -1.0));
    position_.y() = turn_center_.y() + turn_radius_ * std::sin(arc_angle - M_PI_2 * (moving_positive_x_ ? 1.0 : -1.0));
  }

  // Update orientation
  orientation_ = headingToQuaternion(heading_);

  // Body-frame velocity (still moving forward)
  velocity_ = Eigen::Vector3d(vehicle_speed_, 0.0, 0.0);

  // Check if quarter turn is complete
  if (turn_angle_progress_ >= M_PI_2) {
    turn_phase_++;
    turn_angle_progress_ = 0.0;

    if (turn_phase_ == 1) {
      // First quarter turn done, start lateral segment (straight line for line_spacing)
      // Actually for smoother turns, we'll do two quarter turns
      turn_start_heading_ = heading_;

      // Update turn center for second quarter turn
      double offset_angle = heading_ + M_PI_2 * (moving_positive_x_ ? 1.0 : -1.0);
      turn_center_.x() = position_.x() + turn_radius_ * std::cos(offset_angle);
      turn_center_.y() = position_.y() + turn_radius_ * std::sin(offset_angle);

      RCLCPP_DEBUG(this->get_logger(), "First quarter turn complete, heading: %.2f deg",
        heading_ * 180.0 / M_PI);
    } else if (turn_phase_ >= 2) {
      // Both quarter turns done - completed 180 degree turn
      current_segment_ = TrajectorySegment::STRAIGHT;
      segment_progress_ = 0.0;
      current_line_++;
      moving_positive_x_ = !moving_positive_x_;

      // Snap heading to clean value
      heading_ = moving_positive_x_ ? 0.0 : M_PI;
      orientation_ = headingToQuaternion(heading_);

      // Adjust Y position to be exactly on the new line
      position_.y() = current_line_ * line_spacing_;

      RCLCPP_INFO(this->get_logger(), "Turn complete, starting line %d at [%.2f, %.2f], heading: %.0f deg",
        current_line_, position_.x(), position_.y(), heading_ * 180.0 / M_PI);
    }
  }
}

void TruthGeneratorNode::initializeTurn()
{
  turn_start_heading_ = heading_;
  turn_angle_progress_ = 0.0;
  target_turn_angle_ = M_PI_2;  // 90 degrees per quarter turn

  // Calculate turn center (perpendicular to heading, in direction of turn)
  // For lawnmower pattern, turn towards +Y
  double offset_angle = heading_ + M_PI_2 * (moving_positive_x_ ? 1.0 : -1.0);
  turn_center_.x() = position_.x() + turn_radius_ * std::cos(offset_angle);
  turn_center_.y() = position_.y() + turn_radius_ * std::sin(offset_angle);
  turn_center_.z() = position_.z();

  RCLCPP_DEBUG(this->get_logger(), "Initialized turn: center=[%.2f, %.2f], start_heading=%.2f",
    turn_center_.x(), turn_center_.y(), turn_start_heading_ * 180.0 / M_PI);
}

Eigen::Quaterniond TruthGeneratorNode::headingToQuaternion(double heading) const
{
  // NED convention: heading is rotation about Z-down axis
  // Heading 0 = +X (North), positive heading = clockwise when viewed from above
  Eigen::AngleAxisd rotation(heading, Eigen::Vector3d::UnitZ());
  return Eigen::Quaterniond(rotation);
}

void TruthGeneratorNode::publishOdometry()
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = "world";
  odom_msg.child_frame_id = "base_link";

  // Position (NED)
  odom_msg.pose.pose.position.x = position_.x();
  odom_msg.pose.pose.position.y = position_.y();
  odom_msg.pose.pose.position.z = position_.z();

  // Orientation
  odom_msg.pose.pose.orientation.x = orientation_.x();
  odom_msg.pose.pose.orientation.y = orientation_.y();
  odom_msg.pose.pose.orientation.z = orientation_.z();
  odom_msg.pose.pose.orientation.w = orientation_.w();

  // Linear velocity (body frame)
  odom_msg.twist.twist.linear.x = velocity_.x();
  odom_msg.twist.twist.linear.y = velocity_.y();
  odom_msg.twist.twist.linear.z = velocity_.z();

  // Angular velocity (body frame)
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = angular_velocity_;

  odom_pub_->publish(odom_msg);

  // Add pose to path
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = odom_msg.header;
  pose_stamped.pose = odom_msg.pose.pose;
  path_poses_.push_back(pose_stamped);
}

void TruthGeneratorNode::pathCallback()
{
  if (path_poses_.empty()) {
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "world";
  path_msg.poses = path_poses_;

  path_pub_->publish(path_msg);
}

}  // namespace usbl_navigation
