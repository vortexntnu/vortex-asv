#include <maneuvering_task/maneuvering_task_ros.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<maneuvering_task::ManeuveringTaskNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}