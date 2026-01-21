// Copyright 2026 Vinay
// SPDX-License-Identifier: MIT

#include "usbl_navigation/path_publisher_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<usbl_navigation::PathPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
