// Copyright 2026 Vinay
// SPDX-License-Identifier: MIT

#include "usbl_navigation/metrics_logger_node.hpp"

#include <cmath>
#include <iomanip>

namespace usbl_navigation {

MetricsLoggerNode::MetricsLoggerNode(const rclcpp::NodeOptions& options)
  : Node("metrics_logger", options),
    header_written_(false),
    start_time_set_(false)
{
  loadParameters();
  openOutputFile();

  // Create message filter subscribers
  truth_sub_ = std::make_shared<message_filters::Subscriber<OdometryMsg>>(
    this, "/truth/odometry", rmw_qos_profile_sensor_data);

  estimate_sub_ = std::make_shared<message_filters::Subscriber<OdometryMsg>>(
    this, "/navigation/odometry", rmw_qos_profile_sensor_data);

  // Create synchronizer with 10ms tolerance (100 messages queue size)
  sync_ = std::make_shared<Synchronizer>(
    SyncPolicy(100), *truth_sub_, *estimate_sub_);

  // Set max interval between messages (10ms = 0.01s)
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.01));

  // Register callback
  sync_->registerCallback(
    std::bind(&MetricsLoggerNode::synchronizedCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "MetricsLoggerNode initialized, writing to: %s",
              output_path_.c_str());
}

MetricsLoggerNode::~MetricsLoggerNode()
{
  if (output_file_.is_open()) {
    output_file_.flush();
    output_file_.close();
    RCLCPP_INFO(get_logger(), "Closed metrics output file");
  }
}

void MetricsLoggerNode::loadParameters()
{
  // Declare and get output file parameter
  declare_parameter("output_file", "/tmp/navigation_metrics.csv");
  output_path_ = get_parameter("output_file").as_string();
}

void MetricsLoggerNode::openOutputFile()
{
  output_file_.open(output_path_, std::ios::out | std::ios::trunc);
  if (!output_file_.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open output file: %s", output_path_.c_str());
    throw std::runtime_error("Failed to open metrics output file");
  }

  writeHeader();
}

void MetricsLoggerNode::writeHeader()
{
  output_file_ << "timestamp,"
               << "truth_x,truth_y,truth_z,"
               << "est_x,est_y,est_z,"
               << "error_3d,"
               << "sigma_x,sigma_y,sigma_z"
               << std::endl;
  output_file_.flush();
  header_written_ = true;
}

void MetricsLoggerNode::synchronizedCallback(
  const OdometryMsg::ConstSharedPtr& truth_msg,
  const OdometryMsg::ConstSharedPtr& estimate_msg)
{
  // Set start time on first message
  if (!start_time_set_) {
    start_time_ = truth_msg->header.stamp;
    start_time_set_ = true;
  }

  // Calculate relative timestamp in seconds
  double timestamp = (rclcpp::Time(truth_msg->header.stamp) - start_time_).seconds();

  // Extract truth position
  double truth_x = truth_msg->pose.pose.position.x;
  double truth_y = truth_msg->pose.pose.position.y;
  double truth_z = truth_msg->pose.pose.position.z;

  // Extract estimate position
  double est_x = estimate_msg->pose.pose.position.x;
  double est_y = estimate_msg->pose.pose.position.y;
  double est_z = estimate_msg->pose.pose.position.z;

  // Compute position error components
  double err_x = est_x - truth_x;
  double err_y = est_y - truth_y;
  double err_z = est_z - truth_z;

  // Compute 3D position error (Euclidean distance)
  double error_3d = std::sqrt(err_x * err_x + err_y * err_y + err_z * err_z);

  // Extract covariance diagonal elements (0, 7, 14 for x, y, z variance)
  // pose_covariance is a 36-element array (6x6 matrix in row-major order)
  const auto& cov = estimate_msg->pose.covariance;
  double sigma_x = std::sqrt(std::max(0.0, cov[0]));   // sqrt(var_x)
  double sigma_y = std::sqrt(std::max(0.0, cov[7]));   // sqrt(var_y)
  double sigma_z = std::sqrt(std::max(0.0, cov[14]));  // sqrt(var_z)

  // Write CSV row with fixed precision
  output_file_ << std::fixed << std::setprecision(6)
               << timestamp << ","
               << truth_x << "," << truth_y << "," << truth_z << ","
               << est_x << "," << est_y << "," << est_z << ","
               << error_3d << ","
               << sigma_x << "," << sigma_y << "," << sigma_z
               << std::endl;

  // Flush after each write to prevent data loss on shutdown
  output_file_.flush();
}

}  // namespace usbl_navigation
