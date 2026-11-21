#include "tf_explorer/patrol_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tf_explorer::PatrolNode>());
  rclcpp::shutdown();
  return 0;
}
