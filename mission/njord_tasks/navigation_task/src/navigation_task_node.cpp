#include <navigation_task/navigation_task_ros.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<navigation_task::NavigationTaskNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}