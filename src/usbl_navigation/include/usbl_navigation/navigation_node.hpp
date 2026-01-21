// Copyright 2026 Vinay
// SPDX-License-Identifier: MIT

#ifndef USBL_NAVIGATION__NAVIGATION_NODE_HPP_
#define USBL_NAVIGATION__NAVIGATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "usbl_navigation/delayed_state_ekf.hpp"

namespace usbl_navigation {

/**
 * @brief ROS2 node for USBL-aided navigation using delayed-state EKF
 *
 * This node fuses IMU, DVL, and delayed USBL measurements to provide
 * real-time navigation output. The EKF handles USBL latency properly
 * using state buffering and repropagation.
 *
 * Subscriptions:
 * - /imu/data (sensor_msgs/Imu, 100Hz) - IMU angular velocity and acceleration
 * - /dvl/twist (geometry_msgs/TwistStamped, 5Hz) - DVL body-frame velocity
 * - /dvl/bottom_lock (std_msgs/Bool, 5Hz) - DVL validity flag
 * - /usbl/position (geometry_msgs/PointStamped, 0.2Hz) - USBL position fix
 * - /usbl/valid (std_msgs/Bool, 0.2Hz) - USBL validity flag
 *
 * Publications:
 * - /navigation/odometry (nav_msgs/Odometry, 50Hz) - Fused navigation state
 *
 * TF:
 * - Broadcasts world -> base_link transform
 */
class NavigationNode : public rclcpp::Node {
public:
  explicit NavigationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Sensor callbacks
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void dvlCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void dvlValidCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void usblCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void usblValidCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // Timer callback for publishing
  void publishTimerCallback();

  // Helper methods
  void publishOdometry();
  void broadcastTf();
  void loadParameters();

  // EKF
  std::unique_ptr<DelayedStateEKF> ekf_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dvl_valid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr usbl_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr usbl_valid_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer for publishing at fixed rate
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // State tracking
  rclcpp::Time last_imu_time_;
  bool dvl_valid_;
  bool usbl_valid_;
  bool initialized_;

  // Parameters
  double publish_rate_;
  Eigen::Matrix3d R_dvl_;   // DVL measurement noise
  Eigen::Matrix3d R_usbl_;  // USBL measurement noise
};

}  // namespace usbl_navigation

#endif  // USBL_NAVIGATION__NAVIGATION_NODE_HPP_
