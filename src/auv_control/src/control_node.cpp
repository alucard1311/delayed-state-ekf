#include "auv_control/control_node.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace auv_control {

ControlNode::ControlNode()
    : Node("control_node"),
      current_depth_(0.0),
      current_heading_(0.0),
      current_velocity_(0.0),
      last_ekf_time_(this->now()),
      ekf_received_(false),
      target_depth_(0.0),
      target_heading_(0.0),
      target_velocity_(0.0) {
  // EKF state subscriber
  ekf_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/auv/ekf/pose", 10,
      std::bind(&ControlNode::ekfCallback, this, std::placeholders::_1));

  // Setpoint subscribers
  depth_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/auv/cmd/depth", 10,
      std::bind(&ControlNode::depthCmdCallback, this, std::placeholders::_1));

  heading_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/auv/cmd/heading", 10,
      std::bind(&ControlNode::headingCmdCallback, this, std::placeholders::_1));

  velocity_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/auv/cmd/velocity", 10,
      std::bind(&ControlNode::velocityCmdCallback, this, std::placeholders::_1));

  // Thruster publishers
  surge_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/auv/thruster/surge", 10);
  sway_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/auv/thruster/sway", 10);
  heave_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/auv/thruster/heave", 10);
  yaw_bow_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/auv/thruster/yaw_bow", 10);
  yaw_stern_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/auv/thruster/yaw_stern", 10);

  // Control timer at 50Hz (20ms period)
  control_timer_ = this->create_wall_timer(
      20ms, std::bind(&ControlNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Control node initialized");
  RCLCPP_INFO(this->get_logger(), "Subscribing to EKF: /auv/ekf/pose");
  RCLCPP_INFO(this->get_logger(), "Setpoint topics: /auv/cmd/{depth,heading,velocity}");
}

void ControlNode::ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Extract depth from z position (positive z is up, depth is positive down)
  current_depth_ = -msg->pose.pose.position.z;

  // Extract heading (yaw) from quaternion
  current_heading_ = quaternionToYaw(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

  // Extract forward velocity from twist
  current_velocity_ = msg->twist.twist.linear.x;

  // Update timestamp and flag
  last_ekf_time_ = this->now();
  ekf_received_ = true;
}

void ControlNode::depthCmdCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  target_depth_ = msg->data;
  RCLCPP_DEBUG(this->get_logger(), "New depth target: %.2f m", target_depth_);
}

void ControlNode::headingCmdCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  target_heading_ = msg->data;
  RCLCPP_DEBUG(this->get_logger(), "New heading target: %.2f rad", target_heading_);
}

void ControlNode::velocityCmdCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  target_velocity_ = msg->data;
  RCLCPP_DEBUG(this->get_logger(), "New velocity target: %.2f m/s", target_velocity_);
}

void ControlNode::controlLoop() {
  // Early return if no EKF data received yet
  if (!ekf_received_) {
    return;
  }

  // STUB: Controllers will be implemented in Plan 03-02
  // For now, just log state periodically for debugging
  static int log_counter = 0;
  if (++log_counter >= 250) {  // Log every 5 seconds at 50Hz
    RCLCPP_INFO(this->get_logger(),
        "State: depth=%.2f m, heading=%.2f rad, velocity=%.2f m/s",
        current_depth_, current_heading_, current_velocity_);
    RCLCPP_INFO(this->get_logger(),
        "Target: depth=%.2f m, heading=%.2f rad, velocity=%.2f m/s",
        target_depth_, target_heading_, target_velocity_);
    log_counter = 0;
  }

  // TODO (Plan 03-02): Implement depth, heading, and velocity controllers
  // TODO (Plan 03-02): Publish thruster commands
}

double ControlNode::quaternionToYaw(double x, double y, double z, double w) {
  // Standard quaternion to yaw (ZYX Euler convention)
  // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace auv_control
