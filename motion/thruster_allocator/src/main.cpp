#include "rclcpp/rclcpp.hpp"
#include "../include/thruster_allocator/allocator_ros.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Allocator>());
  rclcpp::shutdown();
  return 0;
}