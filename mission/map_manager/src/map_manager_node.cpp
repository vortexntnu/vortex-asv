#include <map_manager/map_manager_ros.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<map_manager::MapManagerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}