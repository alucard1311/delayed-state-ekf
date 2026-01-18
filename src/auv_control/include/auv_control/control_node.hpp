#ifndef AUV_CONTROL__CONTROL_NODE_HPP_
#define AUV_CONTROL__CONTROL_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include "auv_control/pid.hpp"

namespace auv_control {

class ControlNode : public rclcpp::Node {
public:
  ControlNode();

private:
  // EKF state subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;

  // Setpoint subscribers (for external commands)
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_cmd_sub_;

  // Thruster publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr surge_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sway_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heave_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_bow_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_stern_pub_;

  // Control timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // PID controllers
  std::unique_ptr<PID> depth_pid_;
  std::unique_ptr<PID> heading_pid_;
  std::unique_ptr<PID> velocity_pid_;

  // Control enable flags
  bool depth_control_enabled_;
  bool heading_control_enabled_;
  bool velocity_control_enabled_;

  // Last control time for dt calculation
  rclcpp::Time last_control_time_;

  // Current state from EKF
  double current_depth_;
  double current_heading_;
  double current_velocity_;
  rclcpp::Time last_ekf_time_;
  bool ekf_received_;

  // Setpoints
  double target_depth_;
  double target_heading_;
  double target_velocity_;

  // Callbacks
  void ekfCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void depthCmdCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void headingCmdCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void velocityCmdCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void controlLoop();

  // Helper to extract yaw from quaternion
  double quaternionToYaw(double x, double y, double z, double w);

  // Helper to wrap angle to [-pi, pi]
  double wrapAngle(double angle);
};

}  // namespace auv_control

#endif  // AUV_CONTROL__CONTROL_NODE_HPP_
