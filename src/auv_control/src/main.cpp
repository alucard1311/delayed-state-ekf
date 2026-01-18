#include <rclcpp/rclcpp.hpp>
#include "auv_control/control_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auv_control::ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
