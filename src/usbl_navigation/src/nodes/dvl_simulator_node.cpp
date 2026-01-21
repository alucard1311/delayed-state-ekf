#include "usbl_navigation/dvl_simulator_node.hpp"

#include <chrono>
#include <cmath>

namespace usbl_navigation
{

DvlSimulatorNode::DvlSimulatorNode(const rclcpp::NodeOptions & options)
: Node("dvl_simulator_node", options),
  orientation_(Eigen::Quaterniond::Identity()),
  linear_velocity_world_(Eigen::Vector3d::Zero()),
  have_truth_(false),
  mission_started_(false),
  rng_(std::random_device{}()),
  normal_dist_(0.0, 1.0),
  uniform_dist_(0.0, 1.0)
{
  // Declare and get DVL noise parameters
  this->declare_parameter("dvl.velocity_noise_std", 0.02);
  this->declare_parameter("dvl.scale_factor_error", 0.01);
  this->declare_parameter("dvl.dropout_probability", 0.05);
  this->declare_parameter("dvl.canyon_dropout_start", 120.0);
  this->declare_parameter("dvl.canyon_dropout_end", 150.0);
  this->declare_parameter("dvl.publish_rate", 5.0);
  this->declare_parameter("enable_canyon_dropout", true);

  velocity_noise_std_ = this->get_parameter("dvl.velocity_noise_std").as_double();
  scale_factor_error_ = this->get_parameter("dvl.scale_factor_error").as_double();
  dropout_probability_ = this->get_parameter("dvl.dropout_probability").as_double();
  canyon_dropout_start_ = this->get_parameter("dvl.canyon_dropout_start").as_double();
  canyon_dropout_end_ = this->get_parameter("dvl.canyon_dropout_end").as_double();
  publish_rate_ = this->get_parameter("dvl.publish_rate").as_double();
  enable_canyon_dropout_ = this->get_parameter("enable_canyon_dropout").as_bool();

  // Create subscriber
  truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/truth/odometry", 10,
    std::bind(&DvlSimulatorNode::truthCallback, this, std::placeholders::_1));

  // Create publishers
  dvl_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/dvl/twist", 10);
  bottom_lock_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/dvl/bottom_lock", 10);

  // Create timer for DVL publishing
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&DvlSimulatorNode::publishCallback, this));

  RCLCPP_INFO(this->get_logger(), "DVL simulator initialized");
  RCLCPP_INFO(this->get_logger(), "  Velocity noise std: %.3f m/s", velocity_noise_std_);
  RCLCPP_INFO(this->get_logger(), "  Scale factor error: %.1f%%", scale_factor_error_ * 100.0);
  RCLCPP_INFO(this->get_logger(), "  Random dropout probability: %.1f%%", dropout_probability_ * 100.0);
  RCLCPP_INFO(this->get_logger(), "  Canyon dropout: %.0fs - %.0fs (%s)",
    canyon_dropout_start_, canyon_dropout_end_,
    enable_canyon_dropout_ ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), "  Publish rate: %.1f Hz", publish_rate_);
}

void DvlSimulatorNode::truthCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Initialize mission start time on first truth message
  if (!mission_started_) {
    mission_start_time_ = this->now();
    mission_started_ = true;
    RCLCPP_INFO(this->get_logger(), "Mission started - DVL simulation active");
  }

  // Extract orientation
  orientation_ = Eigen::Quaterniond(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);

  // Extract linear velocity (world frame from truth)
  // Note: Truth odometry twist is in body frame, but position derivative would be in world frame
  // Based on truth_generator, twist.linear is in body frame
  // We need to convert to world frame first, then to body frame with DVL orientation
  // Actually, looking at truth_generator: velocity_ is body frame and directly published
  // So twist.linear is already body frame - but we want to simulate DVL which transforms world velocity
  // Let's assume truth publishes world-frame velocity (NED) in twist.linear for this simulation
  // Reading truth_generator more carefully: velocity_ = Eigen::Vector3d(vehicle_speed_, 0.0, 0.0) is body frame
  // But DVL measures velocity relative to ground, which is world frame transformed to body
  // For simulation purposes, let's transform the world-frame velocity (position derivative)

  // Extract world-frame velocity by rotating body velocity to world frame
  Eigen::Vector3d body_velocity(
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z);

  // Transform body to world frame
  linear_velocity_world_ = orientation_ * body_velocity;

  have_truth_ = true;
}

void DvlSimulatorNode::publishCallback()
{
  if (!have_truth_) {
    return;  // Wait for truth data
  }

  // Check bottom lock status
  bool bottom_lock = checkBottomLock();

  // Always publish bottom lock status
  std_msgs::msg::Bool lock_msg;
  lock_msg.data = bottom_lock;
  bottom_lock_pub_->publish(lock_msg);

  // Only publish velocity when we have bottom lock
  if (bottom_lock) {
    // Transform velocity from world to body frame
    Eigen::Vector3d v_body = transformToBodyFrame(linear_velocity_world_);

    // Add noise and scale factor error
    Eigen::Vector3d v_measured = addNoise(v_body);

    // Create twist message
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "dvl_link";

    // Linear velocity (body frame)
    twist_msg.twist.twist.linear.x = v_measured.x();
    twist_msg.twist.twist.linear.y = v_measured.y();
    twist_msg.twist.twist.linear.z = v_measured.z();

    // Angular velocity not measured by DVL
    twist_msg.twist.twist.angular.x = 0.0;
    twist_msg.twist.twist.angular.y = 0.0;
    twist_msg.twist.twist.angular.z = 0.0;

    // Populate covariance (diagonal for linear velocity, zeros for angular)
    double vel_var = velocity_noise_std_ * velocity_noise_std_;
    // Covariance matrix layout: [vx, vy, vz, wx, wy, wz]
    // Only diagonal elements for linear velocity
    twist_msg.twist.covariance[0] = vel_var;   // vx variance
    twist_msg.twist.covariance[7] = vel_var;   // vy variance
    twist_msg.twist.covariance[14] = vel_var;  // vz variance
    // Angular velocity covariance set to -1 to indicate unknown/not measured
    twist_msg.twist.covariance[21] = -1.0;
    twist_msg.twist.covariance[28] = -1.0;
    twist_msg.twist.covariance[35] = -1.0;

    dvl_pub_->publish(twist_msg);
  }
}

bool DvlSimulatorNode::checkBottomLock()
{
  if (!mission_started_) {
    return false;
  }

  double mission_time = (this->now() - mission_start_time_).seconds();

  // Canyon dropout check (deterministic based on time)
  if (enable_canyon_dropout_ &&
      mission_time >= canyon_dropout_start_ &&
      mission_time <= canyon_dropout_end_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "DVL bottom lock lost (canyon scenario at t=%.1fs)", mission_time);
    return false;
  }

  // Random dropout check
  if (uniform_dist_(rng_) < dropout_probability_) {
    RCLCPP_DEBUG(this->get_logger(), "DVL random dropout at t=%.1fs", mission_time);
    return false;
  }

  return true;
}

Eigen::Vector3d DvlSimulatorNode::transformToBodyFrame(const Eigen::Vector3d & v_world) const
{
  // DVL measures velocity in body frame
  // v_body = R^(-1) * v_world where R is body-to-world rotation
  // Quaternion q represents body-to-world, so q.inverse() is world-to-body
  return orientation_.inverse() * v_world;
}

Eigen::Vector3d DvlSimulatorNode::addNoise(const Eigen::Vector3d & v_body)
{
  // Apply scale factor error: v_measured = v_true * (1 + scale_error)
  Eigen::Vector3d v_scaled = v_body * (1.0 + scale_factor_error_);

  // Add white noise
  Eigen::Vector3d noise(
    normal_dist_(rng_) * velocity_noise_std_,
    normal_dist_(rng_) * velocity_noise_std_,
    normal_dist_(rng_) * velocity_noise_std_);

  return v_scaled + noise;
}

}  // namespace usbl_navigation
