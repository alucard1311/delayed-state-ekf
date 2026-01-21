// Copyright 2026 Vinay
// SPDX-License-Identifier: MIT

#include "usbl_navigation/path_publisher_node.hpp"

#include <chrono>

namespace usbl_navigation {

PathPublisherNode::PathPublisherNode(const rclcpp::NodeOptions& options)
    : Node("path_publisher", options) {
  // Declare parameters
  this->declare_parameter<std::string>("input_topic", "/odometry");
  this->declare_parameter<std::string>("output_topic", "/path");
  this->declare_parameter<std::string>("frame_id", "world");
  this->declare_parameter<int>("max_points", 1000);
  this->declare_parameter<double>("publish_rate", 2.0);

  // Get parameters
  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  max_points_ = this->get_parameter("max_points").as_int();
  publish_rate_ = this->get_parameter("publish_rate").as_double();

  // Initialize path header
  path_.header.frame_id = frame_id_;

  // Create subscriber
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, 10,
      std::bind(&PathPublisherNode::odometryCallback, this, std::placeholders::_1));

  // Create publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(output_topic_, 10);

  // Create timer for periodic publishing (publish_rate Hz)
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PathPublisherNode::publishTimerCallback, this));

  RCLCPP_INFO(this->get_logger(),
              "PathPublisher initialized: %s -> %s (frame: %s, max: %d points)",
              input_topic_.c_str(), output_topic_.c_str(),
              frame_id_.c_str(), max_points_);
}

void PathPublisherNode::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Create pose from odometry
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = frame_id_;  // Use configured frame
  pose.pose = msg->pose.pose;

  // Add to path
  path_.poses.push_back(pose);

  // Trim to max_points (remove oldest)
  if (static_cast<int>(path_.poses.size()) > max_points_) {
    path_.poses.erase(path_.poses.begin());
  }
}

void PathPublisherNode::publishTimerCallback() {
  // Update path header timestamp
  path_.header.stamp = this->now();

  // Publish path
  path_pub_->publish(path_);
}

}  // namespace usbl_navigation
