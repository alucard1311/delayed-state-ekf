#include "usbl_navigation/truth_generator_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<usbl_navigation::TruthGeneratorNode>();

  RCLCPP_INFO(node->get_logger(), "Starting truth generator node...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
