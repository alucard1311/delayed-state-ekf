#include "usbl_navigation/usbl_simulator_node.hpp"

#include <chrono>
#include <cmath>

namespace usbl_navigation
{

UsblSimulatorNode::UsblSimulatorNode(const rclcpp::NodeOptions & options)
: Node("usbl_simulator", options),
  noise_dist_(0.0, 1.0),
  uniform_dist_(0.0, 1.0),
  total_measurements_(0),
  dropout_count_(0),
  outlier_count_(0)
{
  // Declare and get parameters
  range_noise_percent_ = this->declare_parameter("usbl.range_noise_percent", 0.02);
  min_noise_ = this->declare_parameter("usbl.min_noise", 0.3);
  processing_delay_ = this->declare_parameter("usbl.processing_delay", 0.2);
  dropout_probability_ = this->declare_parameter("usbl.dropout_probability", 0.10);
  outlier_probability_ = this->declare_parameter("usbl.outlier_probability", 0.05);
  outlier_offset_ = this->declare_parameter("usbl.outlier_offset", 5.0);
  publish_rate_ = this->declare_parameter("usbl.publish_rate", 0.2);

  // Ship position parameter (vector)
  std::vector<double> ship_pos_default = {50.0, 25.0, 0.0};
  std::vector<double> ship_pos = this->declare_parameter("usbl.ship_position", ship_pos_default);
  if (ship_pos.size() != 3) {
    RCLCPP_WARN(get_logger(), "Invalid ship_position size, using default");
    ship_pos = ship_pos_default;
  }
  ship_position_ = Eigen::Vector3d(ship_pos[0], ship_pos[1], ship_pos[2]);

  // Initialize RNG with random device seed
  std::random_device rd;
  rng_.seed(rd());

  // Create subscriber to truth odometry
  truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/truth/odometry", 10,
    std::bind(&UsblSimulatorNode::odometryCallback, this, std::placeholders::_1));

  // Create publishers
  position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/usbl/position", 10);
  valid_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/usbl/valid", 10);

  // Create timer for publishing (convert Hz to period)
  double period_sec = 1.0 / publish_rate_;
  auto period = std::chrono::duration<double>(period_sec);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&UsblSimulatorNode::publishTimer, this));

  RCLCPP_INFO(get_logger(),
    "USBL simulator initialized: rate=%.2fHz, delay=%.2fs, noise=%.1f%%, "
    "dropout=%.0f%%, outlier=%.0f%% (%.1fm offset)",
    publish_rate_, processing_delay_, range_noise_percent_ * 100,
    dropout_probability_ * 100, outlier_probability_ * 100, outlier_offset_);
  RCLCPP_INFO(get_logger(),
    "Ship position: [%.1f, %.1f, %.1f]",
    ship_position_.x(), ship_position_.y(), ship_position_.z());
}

void UsblSimulatorNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Extract position from odometry
  Eigen::Vector3d position(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z);

  // Store in buffer
  position_buffer_.emplace_back(msg->header.stamp, position);

  // Maintain buffer size (remove oldest entries)
  while (position_buffer_.size() > MAX_BUFFER_SIZE) {
    position_buffer_.pop_front();
  }
}

void UsblSimulatorNode::publishTimer()
{
  total_measurements_++;

  // 1. Determine measurement time (NOT current time!)
  rclcpp::Time current_time = now();
  rclcpp::Time measurement_time = current_time - rclcpp::Duration::from_seconds(processing_delay_);

  // 2. Look up position at measurement_time from buffer
  Eigen::Vector3d auv_position;
  if (!getPositionAtTime(measurement_time, auv_position)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "No buffered position for USBL measurement (delay=%.2fs)", processing_delay_);
    return;
  }

  // 3. Check for dropout
  if (uniform_dist_(rng_) < dropout_probability_) {
    dropout_count_++;
    std_msgs::msg::Bool valid_msg;
    valid_msg.data = false;
    valid_pub_->publish(valid_msg);
    RCLCPP_INFO(get_logger(), "USBL dropout (#%lu, %.1f%% rate)",
      dropout_count_, 100.0 * dropout_count_ / total_measurements_);
    return;
  }

  // 4. Compute range and noise
  double range = computeRange(auv_position);
  double noise_std = std::max(range_noise_percent_ * range, min_noise_);

  // 5. Add noise to position
  Eigen::Vector3d noisy_position = addNoise(auv_position, range);

  // 6. Inject outlier (5% probability)
  bool is_outlier = false;
  if (uniform_dist_(rng_) < outlier_probability_) {
    is_outlier = true;
    outlier_count_++;
    noisy_position = injectOutlier(noisy_position);
    RCLCPP_WARN(get_logger(), "USBL outlier injected! (#%lu, %.1f%% rate)",
      outlier_count_, 100.0 * outlier_count_ / total_measurements_);
  }

  // 7. Publish with MEASUREMENT timestamp (not current!)
  geometry_msgs::msg::PointStamped usbl_msg;
  usbl_msg.header.stamp = measurement_time;  // CRITICAL: delayed timestamp!
  usbl_msg.header.frame_id = "world";
  usbl_msg.point.x = noisy_position.x();
  usbl_msg.point.y = noisy_position.y();
  usbl_msg.point.z = noisy_position.z();
  position_pub_->publish(usbl_msg);

  std_msgs::msg::Bool valid_msg;
  valid_msg.data = true;
  valid_pub_->publish(valid_msg);

  RCLCPP_INFO(get_logger(),
    "USBL: pos=[%.2f, %.2f, %.2f], range=%.1fm, noise_std=%.3fm, outlier=%s, delay=%.2fs",
    noisy_position.x(), noisy_position.y(), noisy_position.z(),
    range, noise_std, is_outlier ? "YES" : "no", processing_delay_);
}

bool UsblSimulatorNode::getPositionAtTime(
  const rclcpp::Time & time,
  Eigen::Vector3d & position_out)
{
  if (position_buffer_.empty()) {
    return false;
  }

  // Check if time is before buffer start
  if (time < position_buffer_.front().first) {
    return false;
  }

  // Check if time is after buffer end
  if (time >= position_buffer_.back().first) {
    // Use most recent position if requested time is recent
    position_out = position_buffer_.back().second;
    return true;
  }

  // Find bracketing entries for interpolation
  for (size_t i = 1; i < position_buffer_.size(); ++i) {
    if (position_buffer_[i].first >= time) {
      // Interpolate between i-1 and i
      const auto & prev = position_buffer_[i - 1];
      const auto & next = position_buffer_[i];

      double dt = (next.first - prev.first).seconds();
      if (dt <= 0.0) {
        position_out = prev.second;
        return true;
      }

      double alpha = (time - prev.first).seconds() / dt;
      alpha = std::clamp(alpha, 0.0, 1.0);

      position_out = (1.0 - alpha) * prev.second + alpha * next.second;
      return true;
    }
  }

  // Should not reach here, but return last position as fallback
  position_out = position_buffer_.back().second;
  return true;
}

double UsblSimulatorNode::computeRange(const Eigen::Vector3d & auv_position) const
{
  return (auv_position - ship_position_).norm();
}

Eigen::Vector3d UsblSimulatorNode::addNoise(
  const Eigen::Vector3d & position,
  double range)
{
  // Compute noise standard deviation: max(2% * range, 0.3m)
  double noise_std = std::max(range_noise_percent_ * range, min_noise_);

  // Add independent Gaussian noise to each axis
  Eigen::Vector3d noisy_position = position;
  noisy_position.x() += noise_std * noise_dist_(rng_);
  noisy_position.y() += noise_std * noise_dist_(rng_);
  noisy_position.z() += noise_std * noise_dist_(rng_);

  return noisy_position;
}

Eigen::Vector3d UsblSimulatorNode::injectOutlier(const Eigen::Vector3d & position)
{
  // Generate random unit vector for outlier direction
  Eigen::Vector3d random_direction;
  random_direction.x() = noise_dist_(rng_);
  random_direction.y() = noise_dist_(rng_);
  random_direction.z() = noise_dist_(rng_);

  // Normalize and scale by outlier offset
  if (random_direction.norm() < 1e-6) {
    random_direction = Eigen::Vector3d::UnitX();  // Fallback if near-zero
  }
  random_direction.normalize();

  return position + outlier_offset_ * random_direction;
}

}  // namespace usbl_navigation
