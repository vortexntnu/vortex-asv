#include "../include/thruster_allocator/allocator_ros.hpp"
#include "../include/thruster_allocator/allocator_utils.hpp"

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto allocator = std::make_shared<Allocator>();
  rclcpp::Rate loop_rate(10);

  while (rclcpp::ok()) {
    rclcpp::spin_some(allocator);
    loop_rate.sleep();
  }

  return 0;
}
