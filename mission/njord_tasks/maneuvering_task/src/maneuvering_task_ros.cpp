#include <maneuvering_task/maneuvering_task_ros.hpp>

namespace maneuvering_task {

ManeuveringTaskNode::ManeuveringTaskNode(const rclcpp::NodeOptions &options)
    : NjordTaskBaseNode("maneuvering_task_node", options) {

  std::thread(&ManeuveringTaskNode::main_task, this).detach();
}

void ManeuveringTaskNode::main_task() {
  // Sleep for 5 seconds to allow system to initialize and tracks to be aquired
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
      set_gps_odom_points();
      get_map_odom_tf();
      setup_lock.unlock();
      break;
    }
    setup_lock.unlock();
  }
}

Eigen::Array<double, 2, 2> ManeuveringTaskNode::predict_first_buoy_pair() {
  // Predict the positions of the first two buoys
  geometry_msgs::msg::PoseStamped buoy_0_base_link_frame;
  geometry_msgs::msg::PoseStamped buoy_1_base_link_frame;
  buoy_0_base_link_frame.header.frame_id = "base_link";
  buoy_1_base_link_frame.header.frame_id = "base_link";

  double distance_to_first_buoy_pair =
      this->get_parameter("distance_to_first_buoy_pair").as_double();

  buoy_0_base_link_frame.pose.position.x = distance_to_first_buoy_pair;
  buoy_0_base_link_frame.pose.position.y = -2.5;
  buoy_0_base_link_frame.pose.position.z = 0.0;
  buoy_1_base_link_frame.pose.position.x = distance_to_first_buoy_pair;
  buoy_1_base_link_frame.pose.position.y = 2.5;
  buoy_1_base_link_frame.pose.position.z = 0.0;

  geometry_msgs::msg::PoseStamped buoy_0_odom_frame;
  geometry_msgs::msg::PoseStamped buoy_1_odom_frame;

  try {
    auto transform = tf_buffer_->lookupTransform(
        "odom", "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
    tf2::doTransform(buoy_0_base_link_frame, buoy_0_base_link_frame, transform);
    tf2::doTransform(buoy_1_base_link_frame, buoy_1_base_link_frame, transform);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
  }

  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) = buoy_0_odom_frame.pose.position.x;
  predicted_positions(1, 0) = buoy_0_odom_frame.pose.position.y;
  predicted_positions(0, 1) = buoy_1_odom_frame.pose.position.x;
  predicted_positions(1, 1) = buoy_1_odom_frame.pose.position.y;

  return predicted_positions;
}

} // namespace maneuvering_task