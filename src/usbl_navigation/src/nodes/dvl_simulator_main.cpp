#include "usbl_navigation/dvl_simulator_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<usbl_navigation::DvlSimulatorNode>();

  RCLCPP_INFO(node->get_logger(), "Starting DVL simulator node...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
