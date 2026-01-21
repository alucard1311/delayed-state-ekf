// Copyright 2026 Vinay
// SPDX-License-Identifier: MIT

#ifndef USBL_NAVIGATION__METRICS_LOGGER_NODE_HPP_
#define USBL_NAVIGATION__METRICS_LOGGER_NODE_HPP_

#include <fstream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>

namespace usbl_navigation {

/**
 * @brief ROS2 node for logging navigation metrics to CSV
 *
 * This node synchronizes truth and estimated odometry messages and logs
 * position error, covariance, and other metrics to a CSV file for
 * offline analysis and plotting.
 *
 * Subscriptions:
 * - /truth/odometry (nav_msgs/Odometry) - Ground truth from truth generator
 * - /navigation/odometry (nav_msgs/Odometry) - EKF navigation estimate
 *
 * Output:
 * - CSV file with columns: timestamp, truth_xyz, est_xyz, error_3d, sigma_xyz
 */
class MetricsLoggerNode : public rclcpp::Node {
public:
  explicit MetricsLoggerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MetricsLoggerNode() override;

private:
  // Message synchronization types
  using OdometryMsg = nav_msgs::msg::Odometry;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<OdometryMsg, OdometryMsg>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  // Synchronized callback
  void synchronizedCallback(
    const OdometryMsg::ConstSharedPtr& truth_msg,
    const OdometryMsg::ConstSharedPtr& estimate_msg);

  // Helper methods
  void loadParameters();
  void openOutputFile();
  void writeHeader();

  // Message filter subscribers
  std::shared_ptr<message_filters::Subscriber<OdometryMsg>> truth_sub_;
  std::shared_ptr<message_filters::Subscriber<OdometryMsg>> estimate_sub_;
  std::shared_ptr<Synchronizer> sync_;

  // Output file
  std::ofstream output_file_;
  std::string output_path_;

  // State
  bool header_written_;
  rclcpp::Time start_time_;
  bool start_time_set_;
};

}  // namespace usbl_navigation

#endif  // USBL_NAVIGATION__METRICS_LOGGER_NODE_HPP_
