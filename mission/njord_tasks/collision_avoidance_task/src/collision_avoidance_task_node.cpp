#include <collision_avoidance_task/collision_avoidance_task_ros.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node =
      std::make_shared<collision_avoidance_task::CollisionAvoidanceTaskNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}