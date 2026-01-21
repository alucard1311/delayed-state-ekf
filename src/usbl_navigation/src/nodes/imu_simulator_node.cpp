#include "usbl_navigation/imu_simulator_node.hpp"

#include <chrono>
#include <cmath>

namespace usbl_navigation
{

ImuSimulatorNode::ImuSimulatorNode(const rclcpp::NodeOptions & options)
: Node("imu_simulator_node", options),
  gyro_bias_(Eigen::Vector3d::Zero()),
  accel_bias_(Eigen::Vector3d::Zero()),
  orientation_(Eigen::Quaterniond::Identity()),
  angular_velocity_body_(Eigen::Vector3d::Zero()),
  linear_velocity_world_(Eigen::Vector3d::Zero()),
  prev_linear_velocity_world_(Eigen::Vector3d::Zero()),
  have_truth_(false),
  rng_(std::random_device{}()),
  normal_dist_(0.0, 1.0),
  bias_log_interval_(10.0)
{
  // Declare and get IMU noise parameters
  this->declare_parameter("imu.gyro_noise_density", 0.001);
  this->declare_parameter("imu.gyro_bias_instability", 0.0001);
  this->declare_parameter("imu.accel_noise_density", 0.01);
  this->declare_parameter("imu.accel_bias_instability", 0.001);
  this->declare_parameter("imu.publish_rate", 100.0);

  gyro_noise_density_ = this->get_parameter("imu.gyro_noise_density").as_double();
  gyro_bias_instability_ = this->get_parameter("imu.gyro_bias_instability").as_double();
  accel_noise_density_ = this->get_parameter("imu.accel_noise_density").as_double();
  accel_bias_instability_ = this->get_parameter("imu.accel_bias_instability").as_double();
  publish_rate_ = this->get_parameter("imu.publish_rate").as_double();

  // Create subscriber
  truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/truth/odometry", 10,
    std::bind(&ImuSimulatorNode::truthCallback, this, std::placeholders::_1));

  // Create publisher
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

  // Create timer for IMU publishing
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ImuSimulatorNode::publishCallback, this));

  last_truth_time_ = this->now();
  last_bias_log_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "IMU simulator initialized");
  RCLCPP_INFO(this->get_logger(), "  Gyro noise density: %.4f rad/s/sqrt(Hz)", gyro_noise_density_);
  RCLCPP_INFO(this->get_logger(), "  Gyro bias instability: %.5f rad/s", gyro_bias_instability_);
  RCLCPP_INFO(this->get_logger(), "  Accel noise density: %.3f m/s^2/sqrt(Hz)", accel_noise_density_);
  RCLCPP_INFO(this->get_logger(), "  Accel bias instability: %.4f m/s^2", accel_bias_instability_);
  RCLCPP_INFO(this->get_logger(), "  Publish rate: %.1f Hz", publish_rate_);
}

void ImuSimulatorNode::truthCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Extract orientation
  orientation_ = Eigen::Quaterniond(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);

  // Extract angular velocity (body frame from truth)
  angular_velocity_body_ = Eigen::Vector3d(
    msg->twist.twist.angular.x,
    msg->twist.twist.angular.y,
    msg->twist.twist.angular.z);

  // Store previous velocity for acceleration computation
  prev_linear_velocity_world_ = linear_velocity_world_;

  // Extract linear velocity (world frame from truth)
  linear_velocity_world_ = Eigen::Vector3d(
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z);

  last_truth_time_ = this->now();
  have_truth_ = true;
}

void ImuSimulatorNode::publishCallback()
{
  if (!have_truth_) {
    return;  // Wait for truth data
  }

  double dt = 1.0 / publish_rate_;

  // Update biases with random walk
  updateBias(dt);

  // Compute true acceleration from velocity change
  Eigen::Vector3d accel_world = computeAcceleration(dt);

  // Transform acceleration to body frame
  Eigen::Vector3d accel_body = orientation_.inverse() * accel_world;

  // Add gravity (accelerometer measures specific force)
  Eigen::Vector3d specific_force = addGravity(accel_body);

  // Compute noise standard deviations for this timestep
  // Noise density is in units/sqrt(Hz), so std_dev = density * sqrt(rate)
  double gyro_noise_std = gyro_noise_density_ * std::sqrt(publish_rate_);
  double accel_noise_std = accel_noise_density_ * std::sqrt(publish_rate_);

  // Add noise and bias to measurements
  Eigen::Vector3d gyro_measured = angular_velocity_body_ + gyro_bias_ + generateNoise(gyro_noise_std);
  Eigen::Vector3d accel_measured = specific_force + accel_bias_ + generateNoise(accel_noise_std);

  // Create IMU message
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = this->now();
  imu_msg.header.frame_id = "imu_link";

  // Orientation from truth (some IMUs provide this)
  imu_msg.orientation.x = orientation_.x();
  imu_msg.orientation.y = orientation_.y();
  imu_msg.orientation.z = orientation_.z();
  imu_msg.orientation.w = orientation_.w();

  // Angular velocity (with noise and bias)
  imu_msg.angular_velocity.x = gyro_measured.x();
  imu_msg.angular_velocity.y = gyro_measured.y();
  imu_msg.angular_velocity.z = gyro_measured.z();

  // Linear acceleration (specific force with noise and bias)
  imu_msg.linear_acceleration.x = accel_measured.x();
  imu_msg.linear_acceleration.y = accel_measured.y();
  imu_msg.linear_acceleration.z = accel_measured.z();

  // Populate covariance matrices (diagonal)
  // Covariance is noise variance
  double gyro_var = gyro_noise_std * gyro_noise_std;
  double accel_var = accel_noise_std * accel_noise_std;

  // Orientation covariance (small since from truth)
  imu_msg.orientation_covariance[0] = 0.001;
  imu_msg.orientation_covariance[4] = 0.001;
  imu_msg.orientation_covariance[8] = 0.001;

  // Angular velocity covariance
  imu_msg.angular_velocity_covariance[0] = gyro_var;
  imu_msg.angular_velocity_covariance[4] = gyro_var;
  imu_msg.angular_velocity_covariance[8] = gyro_var;

  // Linear acceleration covariance
  imu_msg.linear_acceleration_covariance[0] = accel_var;
  imu_msg.linear_acceleration_covariance[4] = accel_var;
  imu_msg.linear_acceleration_covariance[8] = accel_var;

  imu_pub_->publish(imu_msg);

  // Log biases periodically for debugging
  rclcpp::Time current_time = this->now();
  if ((current_time - last_bias_log_time_).seconds() >= bias_log_interval_) {
    RCLCPP_INFO(this->get_logger(),
      "IMU bias: gyro=[%.4f, %.4f, %.4f] rad/s, accel=[%.4f, %.4f, %.4f] m/s^2",
      gyro_bias_.x(), gyro_bias_.y(), gyro_bias_.z(),
      accel_bias_.x(), accel_bias_.y(), accel_bias_.z());
    last_bias_log_time_ = current_time;
  }
}

void ImuSimulatorNode::updateBias(double dt)
{
  // Random walk for bias drift
  // bias += random_sample * bias_instability * sqrt(dt)
  double sqrt_dt = std::sqrt(dt);

  gyro_bias_.x() += normal_dist_(rng_) * gyro_bias_instability_ * sqrt_dt;
  gyro_bias_.y() += normal_dist_(rng_) * gyro_bias_instability_ * sqrt_dt;
  gyro_bias_.z() += normal_dist_(rng_) * gyro_bias_instability_ * sqrt_dt;

  accel_bias_.x() += normal_dist_(rng_) * accel_bias_instability_ * sqrt_dt;
  accel_bias_.y() += normal_dist_(rng_) * accel_bias_instability_ * sqrt_dt;
  accel_bias_.z() += normal_dist_(rng_) * accel_bias_instability_ * sqrt_dt;
}

Eigen::Vector3d ImuSimulatorNode::computeAcceleration(double dt)
{
  // Acceleration from velocity derivative
  // a = dv/dt
  if (dt <= 0.0) {
    return Eigen::Vector3d::Zero();
  }

  return (linear_velocity_world_ - prev_linear_velocity_world_) / dt;
}

Eigen::Vector3d ImuSimulatorNode::addGravity(const Eigen::Vector3d & accel_body) const
{
  // In NED frame, gravity is [0, 0, +g] (pointing down)
  Eigen::Vector3d gravity_world(0.0, 0.0, GRAVITY);

  // Transform gravity to body frame
  // Accelerometer measures: a_measured = a_true - R^T * g
  // where R is world-to-body rotation (inverse of orientation)
  // This is because accelerometer measures specific force (thrust minus gravity)
  Eigen::Vector3d gravity_body = orientation_.inverse() * gravity_world;

  // Specific force = kinematic acceleration - gravity (but accelerometer sees +gravity when stationary)
  // When stationary in NED: accel_body = 0, but accelerometer reads +g in body Z (down)
  return accel_body + gravity_body;
}

Eigen::Vector3d ImuSimulatorNode::generateNoise(double std_dev)
{
  return Eigen::Vector3d(
    normal_dist_(rng_) * std_dev,
    normal_dist_(rng_) * std_dev,
    normal_dist_(rng_) * std_dev);
}

}  // namespace usbl_navigation
