// Copyright 2026 Vinay
// SPDX-License-Identifier: MIT

#ifndef USBL_NAVIGATION__PATH_PUBLISHER_NODE_HPP_
#define USBL_NAVIGATION__PATH_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>

namespace usbl_navigation {

/**
 * @brief ROS2 node that accumulates odometry poses into a path for visualization
 *
 * This node subscribes to an Odometry topic, extracts poses, and publishes
 * an accumulated path. Useful for visualizing truth vs estimate trajectories
 * in RViz.
 *
 * Parameters:
 * - input_topic: source odometry topic (string)
 * - output_topic: path topic to publish (string)
 * - frame_id: frame for path header (string, default: "world")
 * - max_points: maximum path length (int, default: 1000)
 * - publish_rate: rate to publish path in Hz (double, default: 2.0)
 */
class PathPublisherNode : public rclcpp::Node {
public:
  explicit PathPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Callback for odometry messages
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Timer callback for publishing path
  void publishTimerCallback();

  // Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Timer for periodic publishing
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Accumulated path
  nav_msgs::msg::Path path_;

  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  int max_points_;
  double publish_rate_;
};

}  // namespace usbl_navigation

#endif  // USBL_NAVIGATION__PATH_PUBLISHER_NODE_HPP_
