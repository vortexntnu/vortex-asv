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
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    std::unique_lock<std::mutex> setup_lock(navigation_mutex_);
    if (!(this->get_parameter("map_origin_set").as_bool())) {
      RCLCPP_INFO(this->get_logger(), "Map origin not set, sleeping for 100ms");
      setup_lock.unlock();
      continue;
    }
    if (!(this->get_parameter("gps_frame_coords_set").as_bool())) {
      get_map_odom_tf();
      set_gps_odom_points();
      setup_lock.unlock();
      break;
    }
    setup_lock.unlock();
  }
}

} // namespace collision_avoidance_task