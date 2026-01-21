// Copyright 2026 Vinay
// SPDX-License-Identifier: MIT

#include "usbl_navigation/navigation_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace usbl_navigation {

NavigationNode::NavigationNode(const rclcpp::NodeOptions& options)
    : Node("navigation_node", options),
      dvl_valid_(false),
      usbl_valid_(false),
      initialized_(false),
      publish_rate_(50.0) {
  // Load parameters
  loadParameters();

  // Create EKF
  ekf_ = std::make_unique<DelayedStateEKF>();

  // Configure EKF from parameters
  double pos_noise = this->declare_parameter("ekf.process_noise.position", 0.01);
  double vel_noise = this->declare_parameter("ekf.process_noise.velocity", 0.1);
  double ori_noise = this->declare_parameter("ekf.process_noise.orientation", 0.001);
  double gyro_bias_noise = this->declare_parameter("ekf.process_noise.gyro_bias", 0.0001);
  double accel_bias_noise = this->declare_parameter("ekf.process_noise.accel_bias", 0.001);
  ekf_->setProcessNoise(pos_noise, vel_noise, ori_noise, gyro_bias_noise, accel_bias_noise);

  double mahal_threshold = this->declare_parameter("ekf.mahalanobis_threshold", 9.21);
  ekf_->setMahalanobisThreshold(mahal_threshold);

  int buffer_size = this->declare_parameter("ekf.state_buffer_size", 500);
  ekf_->setBufferSize(static_cast<size_t>(buffer_size));

  // Setup measurement noise covariances
  double usbl_noise = this->declare_parameter("ekf.measurement_noise.usbl_position", 1.0);
  R_usbl_ = Eigen::Matrix3d::Identity() * usbl_noise * usbl_noise;

  double dvl_noise = this->declare_parameter("ekf.measurement_noise.dvl_velocity", 0.05);
  R_dvl_ = Eigen::Matrix3d::Identity() * dvl_noise * dvl_noise;

  // Create subscribers
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&NavigationNode::imuCallback, this, std::placeholders::_1));

  dvl_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/dvl/twist", 10,
      std::bind(&NavigationNode::dvlCallback, this, std::placeholders::_1));

  dvl_valid_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/dvl/bottom_lock", 10,
      std::bind(&NavigationNode::dvlValidCallback, this, std::placeholders::_1));

  usbl_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/usbl/position", 10,
      std::bind(&NavigationNode::usblCallback, this, std::placeholders::_1));

  usbl_valid_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/usbl/valid", 10,
      std::bind(&NavigationNode::usblValidCallback, this, std::placeholders::_1));

  // Create publisher
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/navigation/odometry", 10);

  // Create TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create publish timer (50Hz)
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&NavigationNode::publishTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Navigation node initialized");
  RCLCPP_INFO(this->get_logger(), "  Publish rate: %.1f Hz", publish_rate_);
  RCLCPP_INFO(this->get_logger(), "  USBL noise: %.2f m", usbl_noise);
  RCLCPP_INFO(this->get_logger(), "  DVL noise: %.3f m/s", dvl_noise);
  RCLCPP_INFO(this->get_logger(), "  Mahalanobis threshold: %.2f", mahal_threshold);
}

void NavigationNode::loadParameters() {
  // Parameters are loaded in constructor via declare_parameter
  // This method can be extended for additional parameter loading
}

void NavigationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Create IMU measurement
  ImuMeasurement imu;
  imu.timestamp = msg->header.stamp;
  imu.gyro = Eigen::Vector3d(msg->angular_velocity.x,
                              msg->angular_velocity.y,
                              msg->angular_velocity.z);
  imu.accel = Eigen::Vector3d(msg->linear_acceleration.x,
                               msg->linear_acceleration.y,
                               msg->linear_acceleration.z);

  // Calculate dt
  if (!initialized_) {
    last_imu_time_ = imu.timestamp;
    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Navigation filter initialized with first IMU");
    return;
  }

  double dt = (imu.timestamp - last_imu_time_).seconds();
  if (dt <= 0 || dt > 0.1) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Invalid IMU dt: %.4f s, skipping", dt);
    last_imu_time_ = imu.timestamp;
    return;  // Skip invalid dt
  }

  // Run EKF prediction
  ekf_->predict(imu, dt);
  last_imu_time_ = imu.timestamp;
}

void NavigationNode::dvlCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
  if (!initialized_ || !dvl_valid_) {
    return;
  }

  Eigen::Vector3d v_body(msg->twist.twist.linear.x,
                          msg->twist.twist.linear.y,
                          msg->twist.twist.linear.z);

  ekf_->updateDvl(v_body, R_dvl_);

  RCLCPP_DEBUG(this->get_logger(), "DVL update: v_body=[%.2f, %.2f, %.2f] m/s",
               v_body.x(), v_body.y(), v_body.z());
}

void NavigationNode::dvlValidCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  bool was_valid = dvl_valid_;
  dvl_valid_ = msg->data;

  if (dvl_valid_ && !was_valid) {
    RCLCPP_INFO(this->get_logger(), "DVL bottom lock acquired");
  } else if (!dvl_valid_ && was_valid) {
    RCLCPP_WARN(this->get_logger(), "DVL bottom lock lost");
  }
}

void NavigationNode::usblCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (!initialized_ || !usbl_valid_) {
    return;
  }

  Eigen::Vector3d position(msg->point.x, msg->point.y, msg->point.z);

  // Calculate delay from message timestamp (which is the delayed measurement time)
  double delay = (this->now() - msg->header.stamp).seconds();

  // Use message timestamp (which is the delayed measurement time)
  bool accepted = ekf_->updateUsblDelayed(position, msg->header.stamp, R_usbl_);

  if (accepted) {
    RCLCPP_INFO(this->get_logger(), "USBL fix applied: delay=%.2fs, pos=[%.1f, %.1f, %.1f]",
                delay, position.x(), position.y(), position.z());
  } else {
    RCLCPP_WARN(this->get_logger(), "USBL fix rejected: delay=%.2fs, pos=[%.1f, %.1f, %.1f]",
                delay, position.x(), position.y(), position.z());
  }
}

void NavigationNode::usblValidCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  bool was_valid = usbl_valid_;
  usbl_valid_ = msg->data;

  if (usbl_valid_ && !was_valid) {
    RCLCPP_INFO(this->get_logger(), "USBL signal acquired");
  } else if (!usbl_valid_ && was_valid) {
    RCLCPP_WARN(this->get_logger(), "USBL signal lost");
  }
}

void NavigationNode::publishTimerCallback() {
  if (!initialized_ || !ekf_->isInitialized()) {
    return;
  }

  publishOdometry();
  broadcastTf();
}

void NavigationNode::publishOdometry() {
  // Get state from EKF
  Eigen::Vector3d position = ekf_->getPosition();
  Eigen::Vector3d velocity = ekf_->getVelocity();
  Eigen::Quaterniond orientation = ekf_->getQuaternion();
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> covariance = ekf_->getCovariance();

  // Build Odometry message
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = "world";
  odom_msg.child_frame_id = "base_link";

  // Pose
  odom_msg.pose.pose.position.x = position.x();
  odom_msg.pose.pose.position.y = position.y();
  odom_msg.pose.pose.position.z = position.z();
  odom_msg.pose.pose.orientation.w = orientation.w();
  odom_msg.pose.pose.orientation.x = orientation.x();
  odom_msg.pose.pose.orientation.y = orientation.y();
  odom_msg.pose.pose.orientation.z = orientation.z();

  // Pose covariance (6x6: x, y, z, roll, pitch, yaw)
  // Map from EKF covariance (position and orientation blocks)
  // Row-major order: [cov_xx, cov_xy, cov_xz, cov_xroll, cov_xpitch, cov_xyaw, ...]
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      int src_i = (i < 3) ? PX + i : QX + (i - 3);  // Map to EKF indices
      int src_j = (j < 3) ? PX + j : QX + (j - 3);
      odom_msg.pose.covariance[i * 6 + j] = covariance(src_i, src_j);
    }
  }

  // Twist (velocity in body frame for standard odometry)
  // Rotate NED velocity to body frame
  Eigen::Vector3d vel_body = orientation.inverse() * velocity;
  odom_msg.twist.twist.linear.x = vel_body.x();
  odom_msg.twist.twist.linear.y = vel_body.y();
  odom_msg.twist.twist.linear.z = vel_body.z();

  // Angular velocity is zero (we don't estimate it in this EKF version)
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;

  // Twist covariance (velocity block from EKF)
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom_msg.twist.covariance[i * 6 + j] = covariance(VX + i, VX + j);
    }
  }

  odom_pub_->publish(odom_msg);
}

void NavigationNode::broadcastTf() {
  // Get state from EKF
  Eigen::Vector3d position = ekf_->getPosition();
  Eigen::Quaterniond orientation = ekf_->getQuaternion();

  // Build transform message
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "base_link";

  transform.transform.translation.x = position.x();
  transform.transform.translation.y = position.y();
  transform.transform.translation.z = position.z();

  transform.transform.rotation.w = orientation.w();
  transform.transform.rotation.x = orientation.x();
  transform.transform.rotation.y = orientation.y();
  transform.transform.rotation.z = orientation.z();

  tf_broadcaster_->sendTransform(transform);
}

}  // namespace usbl_navigation
