#include <docking_task/docking_task_ros.hpp>

namespace docking_task {

DockingTaskNode::DockingTaskNode(const rclcpp::NodeOptions &options)
    : NjordTaskBaseNode("dock_localization_node", options) {

  declare_parameter<double>("distance_to_first_buoy_pair", 2.0);
  declare_parameter<double>("distance_between_buoys_in_pair", 5.0);
  declare_parameter<std::string>("grid_topic", "grid");
  declare_parameter<double>("dock_width", 0.0);
  declare_parameter<double>("dock_width_tolerance", 0.0);
  declare_parameter<double>("dock_length", 0.0);
  declare_parameter<double>("dock_length_tolerance", 0.0);
  declare_parameter<double>("dock_edge_width", 0.0);
  declare_parameter<double>("dock_edge_width_tolerance", 0.0);
  declare_parameter<double>("dock_search_offset", 0.0);
  declare_parameter<int>("task_nr", 0.0);
  declare_parameter<double>("models.dynmod_stddev", 0.0);
  declare_parameter<double>("models.sen_stddev", 0.0);

  initialize_grid_sub();

  std::thread(&DockingTaskNode::main_task, this).detach();
}

void DockingTaskNode::main_task() {
  navigation_ready();
  // Starting docking task
  Eigen::Array<double, 2, 2> predicted_first_pair = predict_first_buoy_pair();
  double distance_to_first_buoy_pair =
      this->get_parameter("distance_to_first_buoy_pair").as_double();
  if (distance_to_first_buoy_pair > 6.0){
    geometry_msgs::msg::Point waypoint_to_approach_first_pair_base_link;
    waypoint_to_approach_first_pair_base_link.x = distance_to_first_buoy_pair-4.0;
    waypoint_to_approach_first_pair_base_link.y = 0.0;
    waypoint_to_approach_first_pair_base_link.z = 0.0;
    try {
      auto transform = tf_buffer_->lookupTransform(
          "odom", "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
      geometry_msgs::msg::Point waypoint_to_approach_first_pair_odom;
      tf2::doTransform(waypoint_to_approach_first_pair_base_link,
                       waypoint_to_approach_first_pair_odom, transform);
      send_waypoint(waypoint_to_approach_first_pair_odom);
      reach_waypoint(1.0);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
  }
  std::vector<LandmarkPoseID> buoy_landmarks_0_to_1 = get_buoy_landmarks(predicted_first_pair);
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
      predict_second_buoy_pair(
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

  // Third pair of buoys
  Eigen::Array<double, 2, 4> predicted_second_and_third_pair =
      predict_third_buoy_pair(
          buoy_landmarks_0_to_1[0].pose_odom_frame.position,
          buoy_landmarks_0_to_1[1].pose_odom_frame.position,
          buoy_landmarks_0_to_3[2].pose_odom_frame.position,
          buoy_landmarks_0_to_3[3].pose_odom_frame.position);
  std::vector<LandmarkPoseID> buoy_landmarks_2_to_5 =
      get_buoy_landmarks(predicted_second_and_third_pair);
  if (buoy_landmarks_2_to_5.size() != 4) {
    RCLCPP_ERROR(this->get_logger(), "Could not find four buoys");
  }
  geometry_msgs::msg::Point waypoint_third_pair;
  waypoint_third_pair.x =
      (buoy_landmarks_2_to_5[2].pose_odom_frame.position.x +
       buoy_landmarks_2_to_5[3].pose_odom_frame.position.x) /
      2;
  waypoint_third_pair.y = (buoy_landmarks_2_to_5[2].pose_odom_frame.position.y +
                           buoy_landmarks_2_to_5[3].pose_odom_frame.position.y) /
                          2;
  waypoint_third_pair.z = 0.0;
  send_waypoint(waypoint_third_pair);
  reach_waypoint(1.0);

}

Eigen::Array<double, 2, 2> DockingTaskNode::predict_first_buoy_pair() {
  // Predict the positions of the first two buoys
  geometry_msgs::msg::PoseStamped buoy_0_base_link_frame;
  geometry_msgs::msg::PoseStamped buoy_1_base_link_frame;
  buoy_0_base_link_frame.header.frame_id = "base_link";
  buoy_1_base_link_frame.header.frame_id = "base_link";

  double distance_to_first_buoy_pair =
      this->get_parameter("distance_to_first_buoy_pair").as_double();
  double distance_between_buoys_in_pair =
      this->get_parameter("distance_between_buoys_in_pair").as_double();

  buoy_0_base_link_frame.pose.position.x = distance_to_first_buoy_pair;
  buoy_0_base_link_frame.pose.position.y = -distance_between_buoys_in_pair / 2;
  buoy_0_base_link_frame.pose.position.z = 0.0;
  buoy_1_base_link_frame.pose.position.x = distance_to_first_buoy_pair;
  buoy_1_base_link_frame.pose.position.y = distance_between_buoys_in_pair / 2;
  buoy_1_base_link_frame.pose.position.z = 0.0;

  geometry_msgs::msg::PoseStamped buoy_0_odom_frame;
  geometry_msgs::msg::PoseStamped buoy_1_odom_frame;

  bool transform_success = false;
  while (!transform_success) {
    try {
      auto transform = tf_buffer_->lookupTransform(
          "odom", "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
      tf2::doTransform(buoy_0_base_link_frame, buoy_0_odom_frame,
                       transform);
      tf2::doTransform(buoy_1_base_link_frame, buoy_1_odom_frame,
                       transform);
      transform_success = true;
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
  }

  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) = buoy_0_odom_frame.pose.position.x;
  predicted_positions(1, 0) = buoy_0_odom_frame.pose.position.y;
  predicted_positions(0, 1) = buoy_1_odom_frame.pose.position.x;
  predicted_positions(1, 1) = buoy_1_odom_frame.pose.position.y;

  return predicted_positions;
}

Eigen::Array<double, 2, 4>
DockingTaskNode::predict_second_buoy_pair(
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

Eigen::Array<double, 2, 4> DockingTaskNode::predict_third_buoy_pair(
    const geometry_msgs::msg::Point &buoy_0,
    const geometry_msgs::msg::Point &buoy_1,
    const geometry_msgs::msg::Point &buoy_2,
    const geometry_msgs::msg::Point &buoy_3) {
  Eigen::Vector2d direction_vector_first_to_second_pair;
  direction_vector_first_to_second_pair << (buoy_2.x + buoy_3.x) / 2 -
                                               (buoy_0.x + buoy_1.x) / 2,
      (buoy_2.y + buoy_3.y) / 2 - (buoy_0.y + buoy_1.y) / 2;
  direction_vector_first_to_second_pair.normalize();

  Eigen::Array<double, 2, 4> predicted_positions;
  predicted_positions(0, 0) = buoy_2.x;
  predicted_positions(1, 0) = buoy_2.y;
  predicted_positions(0, 1) = buoy_3.x;
  predicted_positions(1, 1) = buoy_3.y;
  predicted_positions(0, 2) = buoy_2.x + direction_vector_first_to_second_pair(0) * 5;
  predicted_positions(1, 2) = buoy_2.y + direction_vector_first_to_second_pair(1) * 5;
  predicted_positions(0, 3) = buoy_3.x + direction_vector_first_to_second_pair(0) * 5;
  predicted_positions(1, 3) = buoy_3.y + direction_vector_first_to_second_pair(1) * 5;

  return predicted_positions;
}

void DockingTaskNode::initialize_grid_sub() {
  rmw_qos_profile_t qos_profile_sensor_data = rmw_qos_profile_sensor_data;
  auto qos_sensor_data =
      rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sensor_data.history, 1),
                  qos_profile_sensor_data);
  std::string grid_topic = this->get_parameter("grid_topic").as_string();
  grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      grid_topic, qos_sensor_data, std::bind(&DockingTaskNode::grid_callback, this, std::placeholders::_1));
}

void DockingTaskNode::grid_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(grid_mutex_);
  grid_msg_ = msg;
  new_grid_msg_ = true;
  lock.unlock();
  grid_cond_var_.notify_one();
}

std::shared_ptr<nav_msgs::msg::OccupancyGrid> DockingTaskNode::get_grid() {
  std::unique_lock<std::mutex> lock(grid_mutex_);
  grid_cond_var_.wait(lock, [this] { return new_grid_msg_; });
  new_grid_msg_ = false;
  lock.unlock();
  return grid_msg_;
}

} // namespace docking_task