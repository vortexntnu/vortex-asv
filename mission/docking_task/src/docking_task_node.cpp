#include <docking_task/docking_task_ros.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<docking_task::DockingTaskNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
