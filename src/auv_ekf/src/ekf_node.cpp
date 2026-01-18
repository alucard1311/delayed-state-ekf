// Copyright 2026 Vinay
// SPDX-License-Identifier: MIT

#include "auv_ekf/ekf_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace auv_ekf {

EkfNode::EkfNode()
: Node("ekf_node"),
  state_(Eigen::VectorXd::Zero(STATE_SIZE)),
  covariance_(Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE)),
  process_noise_(Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE)),
  initialized_(false)
{
  // Declare parameters with defaults
  this->declare_parameter("initial_covariance.position", 1.0);
  this->declare_parameter("initial_covariance.velocity", 0.1);
  this->declare_parameter("initial_covariance.orientation", 0.1);
  this->declare_parameter("initial_covariance.angular_velocity", 0.01);
  this->declare_parameter("process_noise.position", 0.01);
  this->declare_parameter("process_noise.velocity", 0.1);
  this->declare_parameter("process_noise.orientation", 0.01);
  this->declare_parameter("process_noise.angular_velocity", 0.1);
  this->declare_parameter("prediction_rate", 50.0);

  // Get parameters
  initial_covariance_position_ = this->get_parameter("initial_covariance.position").as_double();
  initial_covariance_velocity_ = this->get_parameter("initial_covariance.velocity").as_double();
  initial_covariance_orientation_ = this->get_parameter("initial_covariance.orientation").as_double();
  initial_covariance_angular_vel_ = this->get_parameter("initial_covariance.angular_velocity").as_double();
  process_noise_position_ = this->get_parameter("process_noise.position").as_double();
  process_noise_velocity_ = this->get_parameter("process_noise.velocity").as_double();
  process_noise_orientation_ = this->get_parameter("process_noise.orientation").as_double();
  process_noise_angular_vel_ = this->get_parameter("process_noise.angular_velocity").as_double();
  prediction_rate_ = this->get_parameter("prediction_rate").as_double();

  // Initialize process noise matrix Q (diagonal)
  process_noise_.diagonal() <<
    process_noise_position_, process_noise_position_, process_noise_position_,
    process_noise_velocity_, process_noise_velocity_, process_noise_velocity_,
    process_noise_orientation_, process_noise_orientation_, process_noise_orientation_,
    process_noise_angular_vel_, process_noise_angular_vel_, process_noise_angular_vel_;

  // Create publisher for pose estimate
  pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/auv/ekf/pose", 10);

  // Create prediction timer
  auto period = std::chrono::duration<double>(1.0 / prediction_rate_);
  predict_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&EkfNode::predict, this));

  RCLCPP_INFO(this->get_logger(), "EKF node initialized with prediction rate: %.1f Hz", prediction_rate_);
}

void EkfNode::initializeState()
{
  // Initialize state to zeros (will be updated by first sensor readings)
  state_.setZero();

  // Initialize covariance matrix P (diagonal with initial uncertainties)
  covariance_.setZero();

  // Position uncertainty (x, y, z)
  covariance_(X, X) = initial_covariance_position_;
  covariance_(Y, Y) = initial_covariance_position_;
  covariance_(Z, Z) = initial_covariance_position_;

  // Velocity uncertainty (vx, vy, vz)
  covariance_(VX, VX) = initial_covariance_velocity_;
  covariance_(VY, VY) = initial_covariance_velocity_;
  covariance_(VZ, VZ) = initial_covariance_velocity_;

  // Orientation uncertainty (roll, pitch, yaw)
  covariance_(ROLL, ROLL) = initial_covariance_orientation_;
  covariance_(PITCH, PITCH) = initial_covariance_orientation_;
  covariance_(YAW, YAW) = initial_covariance_orientation_;

  // Angular velocity uncertainty (wx, wy, wz)
  covariance_(WX, WX) = initial_covariance_angular_vel_;
  covariance_(WY, WY) = initial_covariance_angular_vel_;
  covariance_(WZ, WZ) = initial_covariance_angular_vel_;

  last_predict_time_ = this->now();
  initialized_ = true;

  RCLCPP_INFO(this->get_logger(), "EKF state initialized");
}

void EkfNode::predict()
{
  // Initialize on first call
  if (!initialized_) {
    initializeState();
    return;
  }

  // Calculate time delta
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_predict_time_).seconds();

  // Guard against invalid dt (first call after init, clock jumps, etc.)
  if (dt <= 0.0 || dt > 1.0) {
    last_predict_time_ = current_time;
    return;
  }

  // Extract current state
  double x = state_(X);
  double y = state_(Y);
  double z = state_(Z);
  double vx = state_(VX);
  double vy = state_(VY);
  double vz = state_(VZ);
  double roll = state_(ROLL);
  double pitch = state_(PITCH);
  double yaw = state_(YAW);
  double wx = state_(WX);
  double wy = state_(WY);
  double wz = state_(WZ);

  // Get rotation matrix from body to NED frame
  Eigen::Matrix3d R = eulerToRotationMatrix(roll, pitch, yaw);

  // Body velocity vector
  Eigen::Vector3d v_body(vx, vy, vz);

  // Transform velocity to NED frame
  Eigen::Vector3d v_ned = R * v_body;

  // State prediction (constant velocity model)
  // Position updates (integrate velocity in NED frame)
  state_(X) = x + v_ned(0) * dt;
  state_(Y) = y + v_ned(1) * dt;
  state_(Z) = z + v_ned(2) * dt;

  // Velocity unchanged in constant velocity model
  // state_(VX), state_(VY), state_(VZ) remain the same

  // Orientation updates (integrate angular velocity)
  state_(ROLL) = normalizeAngle(roll + wx * dt);
  state_(PITCH) = normalizeAngle(pitch + wy * dt);
  state_(YAW) = normalizeAngle(yaw + wz * dt);

  // Angular velocity unchanged in constant velocity model
  // state_(WX), state_(WY), state_(WZ) remain the same

  // Build state transition Jacobian F (12x12)
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

  // Position depends on velocity (through rotation)
  // dPos/dVel = R * dt (simplified - full Jacobian would include rotation derivatives)
  F.block<3, 3>(X, VX) = R * dt;

  // Position depends on orientation (through rotation of velocity)
  // Simplified: we use numerical approximation or ignore for now
  // Full implementation would compute dR/d(roll,pitch,yaw)

  // Orientation depends on angular velocity
  F(ROLL, WX) = dt;
  F(PITCH, WY) = dt;
  F(YAW, WZ) = dt;

  // Covariance prediction: P = F * P * F^T + Q
  covariance_ = F * covariance_ * F.transpose() + process_noise_ * dt;

  // Update time
  last_predict_time_ = current_time;

  // Publish state estimate
  publishState();
}

void EkfNode::publishState()
{
  auto msg = nav_msgs::msg::Odometry();

  // Header
  msg.header.stamp = this->now();
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  // Position
  msg.pose.pose.position.x = state_(X);
  msg.pose.pose.position.y = state_(Y);
  msg.pose.pose.position.z = state_(Z);

  // Orientation (convert Euler to quaternion)
  tf2::Quaternion q;
  q.setRPY(state_(ROLL), state_(PITCH), state_(YAW));
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  // Pose covariance (6x6 subset: x, y, z, roll, pitch, yaw)
  // Row-major order for ROS message
  msg.pose.covariance[0] = covariance_(X, X);      // x-x
  msg.pose.covariance[7] = covariance_(Y, Y);      // y-y
  msg.pose.covariance[14] = covariance_(Z, Z);     // z-z
  msg.pose.covariance[21] = covariance_(ROLL, ROLL);   // roll-roll
  msg.pose.covariance[28] = covariance_(PITCH, PITCH); // pitch-pitch
  msg.pose.covariance[35] = covariance_(YAW, YAW);     // yaw-yaw

  // Linear velocity (body frame)
  msg.twist.twist.linear.x = state_(VX);
  msg.twist.twist.linear.y = state_(VY);
  msg.twist.twist.linear.z = state_(VZ);

  // Angular velocity (body frame)
  msg.twist.twist.angular.x = state_(WX);
  msg.twist.twist.angular.y = state_(WY);
  msg.twist.twist.angular.z = state_(WZ);

  // Twist covariance
  msg.twist.covariance[0] = covariance_(VX, VX);
  msg.twist.covariance[7] = covariance_(VY, VY);
  msg.twist.covariance[14] = covariance_(VZ, VZ);
  msg.twist.covariance[21] = covariance_(WX, WX);
  msg.twist.covariance[28] = covariance_(WY, WY);
  msg.twist.covariance[35] = covariance_(WZ, WZ);

  pose_pub_->publish(msg);
}

Eigen::Matrix3d EkfNode::eulerToRotationMatrix(double roll, double pitch, double yaw)
{
  // ZYX (yaw-pitch-roll) rotation matrix
  // This transforms from body frame to NED frame
  double cr = std::cos(roll);
  double sr = std::sin(roll);
  double cp = std::cos(pitch);
  double sp = std::sin(pitch);
  double cy = std::cos(yaw);
  double sy = std::sin(yaw);

  Eigen::Matrix3d R;
  R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
       sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
       -sp,     cp * sr,                cp * cr;

  return R;
}

double EkfNode::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace auv_ekf
