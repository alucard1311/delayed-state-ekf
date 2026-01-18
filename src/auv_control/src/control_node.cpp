#include "auv_control/control_node.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace auv_control {

ControlNode::ControlNode()
    : Node("control_node"),
      depth_control_enabled_(true),
      heading_control_enabled_(true),
      velocity_control_enabled_(true),
      last_control_time_(this->now()),
      current_depth_(0.0),
      current_heading_(0.0),
      current_velocity_(0.0),
      last_ekf_time_(this->now()),
      ekf_received_(false),
      target_depth_(0.0),
      target_heading_(0.0),
      target_velocity_(0.0) {
  // ===== Declare PID parameters with defaults =====

  // Depth PID gains
  this->declare_parameter("depth_kp", 20.0);
  this->declare_parameter("depth_ki", 2.0);
  this->declare_parameter("depth_kd", 10.0);

  // Heading PID gains
  this->declare_parameter("heading_kp", 15.0);
  this->declare_parameter("heading_ki", 1.0);
  this->declare_parameter("heading_kd", 5.0);

  // Velocity PID gains
  this->declare_parameter("velocity_kp", 50.0);
  this->declare_parameter("velocity_ki", 5.0);
  this->declare_parameter("velocity_kd", 10.0);

  // Control rate
  this->declare_parameter("control_rate", 50.0);

  // ===== Get parameters =====
  double depth_kp = this->get_parameter("depth_kp").as_double();
  double depth_ki = this->get_parameter("depth_ki").as_double();
  double depth_kd = this->get_parameter("depth_kd").as_double();

  double heading_kp = this->get_parameter("heading_kp").as_double();
  double heading_ki = this->get_parameter("heading_ki").as_double();
  double heading_kd = this->get_parameter("heading_kd").as_double();

  double velocity_kp = this->get_parameter("velocity_kp").as_double();
  double velocity_ki = this->get_parameter("velocity_ki").as_double();
  double velocity_kd = this->get_parameter("velocity_kd").as_double();

  // ===== Create PID controllers =====
  // Max outputs match thruster limits, max integral is half for anti-windup
  // Heave: ±50 N, Yaw: ±30 N, Surge: ±100 N
  depth_pid_ = std::make_unique<PID>(depth_kp, depth_ki, depth_kd, 50.0, 25.0);
  heading_pid_ = std::make_unique<PID>(heading_kp, heading_ki, heading_kd, 30.0, 15.0);
  velocity_pid_ = std::make_unique<PID>(velocity_kp, velocity_ki, velocity_kd, 100.0, 50.0);

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
  RCLCPP_INFO(this->get_logger(), "Depth PID: kp=%.1f, ki=%.1f, kd=%.1f",
              depth_kp, depth_ki, depth_kd);
  RCLCPP_INFO(this->get_logger(), "Heading PID: kp=%.1f, ki=%.1f, kd=%.1f",
              heading_kp, heading_ki, heading_kd);
  RCLCPP_INFO(this->get_logger(), "Velocity PID: kp=%.1f, ki=%.1f, kd=%.1f",
              velocity_kp, velocity_ki, velocity_kd);
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
  // Skip if no EKF data yet
  if (!ekf_received_) {
    return;
  }

  // Calculate dt
  auto now = this->now();
  double dt = (now - last_control_time_).seconds();
  last_control_time_ = now;

  // Skip on first run or if dt is invalid
  if (dt <= 0.0 || dt > 1.0) {
    return;
  }

  std_msgs::msg::Float64 msg;

  // ===== DEPTH CONTROL (Z -> heave) =====
  if (depth_control_enabled_) {
    // Error: positive when we need to go deeper
    // In NED: positive Z is down, positive heave force pushes down
    double depth_error = target_depth_ - current_depth_;
    double heave_cmd = depth_pid_->compute(depth_error, dt);

    msg.data = heave_cmd;
    heave_pub_->publish(msg);
  }

  // ===== HEADING CONTROL (yaw -> yaw_bow, yaw_stern) =====
  if (heading_control_enabled_) {
    // Error with angle wrapping
    double heading_error = wrapAngle(target_heading_ - current_heading_);
    double yaw_cmd = heading_pid_->compute(heading_error, dt);

    // Bow and stern thrusters work together for pure yaw
    // Bow pushes one way, stern pushes opposite (stern is inverted in XML)
    // So we send same command to both
    msg.data = yaw_cmd;
    yaw_bow_pub_->publish(msg);
    yaw_stern_pub_->publish(msg);
  }

  // ===== VELOCITY CONTROL (vx -> surge) =====
  if (velocity_control_enabled_) {
    double velocity_error = target_velocity_ - current_velocity_;
    double surge_cmd = velocity_pid_->compute(velocity_error, dt);

    msg.data = surge_cmd;
    surge_pub_->publish(msg);
  }

  // Log state periodically (every 50 cycles = 1 second)
  static int log_counter = 0;
  if (++log_counter >= 50) {
    log_counter = 0;
    RCLCPP_INFO(this->get_logger(),
        "State: depth=%.2f (target=%.2f), heading=%.2f (target=%.2f), vel=%.2f (target=%.2f)",
        current_depth_, target_depth_,
        current_heading_, target_heading_,
        current_velocity_, target_velocity_);
  }
}

double ControlNode::quaternionToYaw(double x, double y, double z, double w) {
  // Standard quaternion to yaw (ZYX Euler convention)
  // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double ControlNode::wrapAngle(double angle) {
  // Normalize angle to [-pi, pi]
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace auv_control
