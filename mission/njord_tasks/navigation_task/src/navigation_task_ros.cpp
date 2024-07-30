#include <navigation_task/navigation_task_ros.hpp>

namespace navigation_task {

NavigationTaskNode::NavigationTaskNode(const rclcpp::NodeOptions &options)
    : NjordTaskBaseNode("navigation_task_node", options) {

  declare_parameter<double>("distance_to_first_buoy_pair", 2.0);

  std::thread(&NavigationTaskNode::main_task, this).detach();
}

void NavigationTaskNode::main_task() {
  // Sleep for 3 seconds to allow system to initialize and tracks to be aquired
  RCLCPP_INFO(
      this->get_logger(),
      "Waiting 3 seconds for system to initialize before starting main task");
  rclcpp::sleep_for(std::chrono::seconds(3));

  while (true) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    std::unique_lock<std::mutex> setup_lock(setup_mutex_);
    if (!(this->get_parameter("map_origin_set").as_bool())) {
      RCLCPP_INFO(this->get_logger(), "Map origin not set, sleeping for 100ms");
      setup_lock.unlock();
      continue;
    }
    if (!(this->get_parameter("gps_frame_coords_set").as_bool())) {
      setup_map_odom_tf_and_subs();
      set_gps_odom_points();
      setup_lock.unlock();
      break;
    }
    setup_lock.unlock();
  }
  // First pair of buoys
  Eigen::Array22d predicted_first_buoy_pair = predict_first_buoy_pair();
  std::vector<LandmarkPoseID> buoy_landmarks_0_to_1 =
      get_buoy_landmarks(predicted_first_buoy_pair);
  if (buoy_landmarks_0_to_1.size() != 2) {
    RCLCPP_ERROR(this->get_logger(), "Could not find two buoys");
  }
  geometry_msgs::msg::Point waypoint_first_pair;
  waypoint_first_pair.x =
      (buoy_landmarks_0_to_1[0].pose_odom_frame.position.x +
       buoy_landmarks_0_to_1[1].pose_odom_frame.position.x) /
      2;
  waypoint_first_pair.y =
      (buoy_landmarks_0_to_1[0].pose_odom_frame.position.y +
       buoy_landmarks_0_to_1[1].pose_odom_frame.position.y) /
      2;
  waypoint_first_pair.z = 0.0;
  send_waypoint(waypoint_first_pair);
  reach_waypoint(1.0);

  // Second pair of buoys
  Eigen::Array<double, 2, 4> predicted_first_and_second_pair =
      predict_first_and_second_buoy_pair(
          buoy_landmarks_0_to_1[0].pose_odom_frame.position,
          buoy_landmarks_0_to_1[1].pose_odom_frame.position);
  std::vector<LandmarkPoseID> buoy_landmarks_0_to_3 =
      get_buoy_landmarks(predicted_first_and_second_pair);
  if (buoy_landmarks_0_to_3.size() != 4) {
    RCLCPP_ERROR(this->get_logger(), "Could not find four buoys");
  }
  geometry_msgs::msg::Point waypoint_second_pair;
  waypoint_second_pair.x =
      (buoy_landmarks_0_to_3[2].pose_odom_frame.position.x +
       buoy_landmarks_0_to_3[3].pose_odom_frame.position.x) /
      2;
  waypoint_second_pair.y =
      (buoy_landmarks_0_to_3[2].pose_odom_frame.position.y +
       buoy_landmarks_0_to_3[3].pose_odom_frame.position.y) /
      2;
  waypoint_second_pair.z = 0.0;
  send_waypoint(waypoint_second_pair);
  reach_waypoint(1.0);

  // West buoy
}

Eigen::Array<double, 2, 2> NavigationTaskNode::predict_first_buoy_pair() {
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

Eigen::Array<double, 2, 4>
NavigationTaskNode::predict_first_and_second_buoy_pair(
    const geometry_msgs::msg::Point &buoy_0,
    const geometry_msgs::msg::Point &buoy_1) {
  Eigen::Vector2d direction_vector;
  direction_vector << previous_waypoint_odom_frame_.x -
                          this->get_parameter("gps_start_x").as_double(),
      previous_waypoint_odom_frame_.y -
          this->get_parameter("gps_start_y").as_double();
  direction_vector.normalize();

  Eigen::Array<double, 2, 4> predicted_positions;
  predicted_positions(0, 0) = buoy_0.x;
  predicted_positions(1, 0) = buoy_0.y;
  predicted_positions(0, 1) = buoy_1.x;
  predicted_positions(1, 1) = buoy_1.y;
  predicted_positions(0, 2) = buoy_0.x + direction_vector(0) * 5;
  predicted_positions(1, 2) = buoy_0.y + direction_vector(1) * 5;
  predicted_positions(0, 3) = buoy_1.x + direction_vector(0) * 5;
  predicted_positions(1, 3) = buoy_1.y + direction_vector(1) * 5;

  return predicted_positions;
}

} // namespace navigation_task