// Copyright 2026 Vinay
// SPDX-License-Identifier: MIT

#include <rclcpp/rclcpp.hpp>
#include "auv_ekf/ekf_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<auv_ekf::EkfNode>();

  RCLCPP_INFO(node->get_logger(), "Starting EKF node...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
