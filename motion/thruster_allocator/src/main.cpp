#include "thruster_allocator/allocator_ros.hpp"
#include "thruster_allocator/allocator_utils.hpp"

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto allocator = std::make_shared<Allocator>();
  rclcpp::Rate loop_rate(10);
  RCLCPP_INFO(allocator->get_logger(), "Thruster allocator initiated");

  while (rclcpp::ok()) {
    allocator->spinOnce();
    rclcpp::spin_some(allocator);
    loop_rate.sleep();
  }

  return 0;
}
