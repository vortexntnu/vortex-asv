#include <collision_avoidance_task/collision_avoidance_task_ros.hpp>

namespace collision_avoidance_task {

CollisionAvoidanceTaskNode::CollisionAvoidanceTaskNode(
    const rclcpp::NodeOptions &options)
    : NjordTaskBaseNode("collision_avoidance_task_node", options) {

  std::thread(&CollisionAvoidanceTaskNode::main_task, this).detach();
}

void CollisionAvoidanceTaskNode::main_task() {
  // Sleep for 3 seconds to allow system to initialize and tracks to be aquired
  RCLCPP_INFO(
      this->get_logger(),
      "Waiting 3 seconds for system to initialize before starting main task");
  rclcpp::sleep_for(std::chrono::seconds(3));

  while (true) {
    
  }
}

} // namespace collision_avoidance_task