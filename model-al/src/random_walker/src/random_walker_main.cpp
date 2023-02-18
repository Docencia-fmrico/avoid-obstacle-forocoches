#include "random_walker/RandomWalkerNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RandomWalkerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}