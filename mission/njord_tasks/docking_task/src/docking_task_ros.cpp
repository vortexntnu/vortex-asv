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

  std::thread(&DockingTaskNode::main_task, this).detach();
}

void DockingTaskNode::main_task() {
  navigation_ready();
  // Starting docking task
  odom_start_point_ = get_odom()->pose.pose.position;
  Eigen::Array<double, 2, 2> predicted_first_pair = predict_first_buoy_pair();

  sensor_msgs::msg::PointCloud2 buoy_vis_msg;
  pcl::PointCloud<pcl::PointXYZRGB> buoy_vis;
  pcl::PointXYZRGB buoy_red_0;
  buoy_red_0.x = predicted_first_pair(0, 0);
  buoy_red_0.y = predicted_first_pair(1, 0);
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  pcl::PointXYZRGB buoy_green_1;
  buoy_green_1.x = predicted_first_pair(0, 1);
  buoy_green_1.y = predicted_first_pair(1, 1);
  buoy_green_1.z = 0.0;
  buoy_green_1.r = 0;
  buoy_green_1.g = 255;
  buoy_green_1.b = 0;
  buoy_vis.push_back(buoy_green_1);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  double distance_to_first_buoy_pair =
      this->get_parameter("distance_to_first_buoy_pair").as_double();
  if (distance_to_first_buoy_pair > 6.0) {
    geometry_msgs::msg::Point waypoint_to_approach_first_pair_base_link;
    waypoint_to_approach_first_pair_base_link.x =
        distance_to_first_buoy_pair - 4.0;
    waypoint_to_approach_first_pair_base_link.y = 0.0;
    waypoint_to_approach_first_pair_base_link.z = 0.0;
    try {
      auto transform =
          tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
      geometry_msgs::msg::Point waypoint_to_approach_first_pair_odom;
      tf2::doTransform(waypoint_to_approach_first_pair_base_link,
                       waypoint_to_approach_first_pair_odom, transform);
      send_waypoint(waypoint_to_approach_first_pair_odom);
      set_desired_heading(odom_start_point_,
                          waypoint_to_approach_first_pair_odom);
      reach_waypoint(1.0);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
  }
  std::vector<LandmarkPoseID> buoy_landmarks_0_to_1 =
      get_buoy_landmarks(predicted_first_pair);
  if (buoy_landmarks_0_to_1.size() != 2) {
    RCLCPP_ERROR(this->get_logger(), "Could not find two buoys");
  }

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = buoy_landmarks_0_to_1[0].pose_odom_frame.position.x;
  buoy_red_0.y = buoy_landmarks_0_to_1[0].pose_odom_frame.position.y;
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = buoy_landmarks_0_to_1[1].pose_odom_frame.position.x;
  buoy_green_1.y = buoy_landmarks_0_to_1[1].pose_odom_frame.position.y;
  buoy_green_1.z = 0.0;
  buoy_green_1.r = 0;
  buoy_green_1.g = 255;
  buoy_green_1.b = 0;
  buoy_vis.push_back(buoy_green_1);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

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
  set_desired_heading(odom_start_point_, waypoint_first_pair);
  reach_waypoint(1.0);

  // Second pair of buoys
  Eigen::Array<double, 2, 2> predicted_first_and_second_pair =
      predict_second_buoy_pair(
          buoy_landmarks_0_to_1[0].pose_odom_frame.position,
          buoy_landmarks_0_to_1[1].pose_odom_frame.position);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = predicted_first_and_second_pair(0, 0);
  buoy_red_0.y = predicted_first_and_second_pair(1, 0);
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = predicted_first_and_second_pair(0, 1);
  buoy_green_1.y = predicted_first_and_second_pair(1, 1);
  buoy_green_1.z = 0.0;
  buoy_green_1.r = 0;
  buoy_green_1.g = 255;
  buoy_green_1.b = 0;
  buoy_vis.push_back(buoy_green_1);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  std::vector<LandmarkPoseID> buoy_landmarks_2_to_3 =
      get_buoy_landmarks(predicted_first_and_second_pair);
  if (buoy_landmarks_2_to_3.size() != 2) {
    RCLCPP_ERROR(this->get_logger(), "Could not find four buoys");
  }

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = buoy_landmarks_2_to_3[0].pose_odom_frame.position.x;
  buoy_red_0.y = buoy_landmarks_2_to_3[0].pose_odom_frame.position.y;
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = buoy_landmarks_2_to_3[1].pose_odom_frame.position.x;
  buoy_green_1.y = buoy_landmarks_2_to_3[1].pose_odom_frame.position.y;
  buoy_green_1.z = 0.0;
  buoy_green_1.r = 0;
  buoy_green_1.g = 255;
  buoy_green_1.b = 0;
  buoy_vis.push_back(buoy_green_1);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  geometry_msgs::msg::Point waypoint_second_pair;
  waypoint_second_pair.x =
      (buoy_landmarks_2_to_3[0].pose_odom_frame.position.x +
       buoy_landmarks_2_to_3[1].pose_odom_frame.position.x) /
      2;
  waypoint_second_pair.y =
      (buoy_landmarks_2_to_3[0].pose_odom_frame.position.y +
       buoy_landmarks_2_to_3[1].pose_odom_frame.position.y) /
      2;
  waypoint_second_pair.z = 0.0;
  send_waypoint(waypoint_second_pair);
  set_desired_heading(odom_start_point_, waypoint_second_pair);
  reach_waypoint(1.0);

  // Third pair of buoys
  Eigen::Array<double, 2, 2> predicted_third_pair = predict_third_buoy_pair(
      buoy_landmarks_0_to_1[0].pose_odom_frame.position,
      buoy_landmarks_0_to_1[1].pose_odom_frame.position,
      buoy_landmarks_2_to_3[2].pose_odom_frame.position,
      buoy_landmarks_2_to_3[3].pose_odom_frame.position);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = predicted_third_pair(0, 0);
  buoy_red_0.y = predicted_third_pair(1, 0);
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = predicted_third_pair(0, 1);
  buoy_green_1.y = predicted_third_pair(1, 1);
  buoy_green_1.z = 0.0;
  buoy_green_1.r = 0;
  buoy_green_1.g = 255;
  buoy_green_1.b = 0;
  buoy_vis.push_back(buoy_green_1);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  std::vector<LandmarkPoseID> buoy_landmarks_4_to_5 =
      get_buoy_landmarks(predicted_third_pair);
  if (buoy_landmarks_4_to_5.size() != 2) {
    RCLCPP_ERROR(this->get_logger(), "Could not find four buoys");
  }

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = buoy_landmarks_4_to_5[0].pose_odom_frame.position.x;
  buoy_red_0.y = buoy_landmarks_4_to_5[0].pose_odom_frame.position.y;
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = buoy_landmarks_4_to_5[1].pose_odom_frame.position.x;
  buoy_green_1.y = buoy_landmarks_4_to_5[1].pose_odom_frame.position.y;
  buoy_green_1.z = 0.0;
  buoy_green_1.r = 0;
  buoy_green_1.g = 255;
  buoy_green_1.b = 0;
  buoy_vis.push_back(buoy_green_1);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  Eigen::Vector2d direction_vector_up;
  direction_vector_up << (buoy_landmarks_4_to_5[0].pose_odom_frame.position.x +
       buoy_landmarks_4_to_5[1].pose_odom_frame.position.x) /
      2 - odom_start_point_.x,
      (buoy_landmarks_4_to_5[0].pose_odom_frame.position.y +
       buoy_landmarks_4_to_5[1].pose_odom_frame.position.y) /
      2 - odom_start_point_.y;
  direction_vector_up.normalize();

  geometry_msgs::msg::Point waypoint_third_pair;
  waypoint_third_pair.x =
      (buoy_landmarks_4_to_5[0].pose_odom_frame.position.x +
       buoy_landmarks_4_to_5[1].pose_odom_frame.position.x) /
      2 + direction_vector_up(0) * 2;
  waypoint_third_pair.y =
      (buoy_landmarks_4_to_5[0].pose_odom_frame.position.y +
       buoy_landmarks_4_to_5[1].pose_odom_frame.position.y) /
      2 + direction_vector_up(1) * 2;
  waypoint_third_pair.z = 0.0;
  send_waypoint(waypoint_third_pair);
  set_desired_heading(odom_start_point_, waypoint_third_pair);
  reach_waypoint(1.0);

  std::thread(&DockingTaskNode::initialize_grid_sub, this).detach();
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
      tf2::doTransform(buoy_0_base_link_frame, buoy_0_odom_frame, transform);
      tf2::doTransform(buoy_1_base_link_frame, buoy_1_odom_frame, transform);
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

Eigen::Array<double, 2, 2> DockingTaskNode::predict_second_buoy_pair(
    const geometry_msgs::msg::Point &buoy_0,
    const geometry_msgs::msg::Point &buoy_1) {
  Eigen::Vector2d direction_vector;
  direction_vector << previous_waypoint_odom_frame_.x - odom_start_point_.x,
      previous_waypoint_odom_frame_.y - odom_start_point_.y;
  direction_vector.normalize();

  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) = buoy_0.x + direction_vector(0) * 5;
  predicted_positions(1, 0) = buoy_0.y + direction_vector(1) * 5;
  predicted_positions(0, 1) = buoy_1.x + direction_vector(0) * 5;
  predicted_positions(1, 1) = buoy_1.y + direction_vector(1) * 5;

  return predicted_positions;
}

Eigen::Array<double, 2, 2> DockingTaskNode::predict_third_buoy_pair(
    const geometry_msgs::msg::Point &buoy_0,
    const geometry_msgs::msg::Point &buoy_1,
    const geometry_msgs::msg::Point &buoy_2,
    const geometry_msgs::msg::Point &buoy_3) {
  Eigen::Vector2d direction_vector_first_to_second_pair;
  direction_vector_first_to_second_pair
      << (buoy_2.x + buoy_3.x) / 2 - (buoy_0.x + buoy_1.x) / 2,
      (buoy_2.y + buoy_3.y) / 2 - (buoy_0.y + buoy_1.y) / 2;
  direction_vector_first_to_second_pair.normalize();

  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) =
      buoy_2.x + direction_vector_first_to_second_pair(0) * 5;
  predicted_positions(1, 0) =
      buoy_2.y + direction_vector_first_to_second_pair(1) * 5;
  predicted_positions(0, 1) =
      buoy_3.x + direction_vector_first_to_second_pair(0) * 5;
  predicted_positions(1, 1) =
      buoy_3.y + direction_vector_first_to_second_pair(1) * 5;

  return predicted_positions;
}

void DockingTaskNode::initialize_grid_sub() {
  rmw_qos_profile_t qos_profile_sensor_data = rmw_qos_profile_sensor_data;
  auto qos_sensor_data =
      rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sensor_data.history, 1),
                  qos_profile_sensor_data);
  std::string grid_topic = this->get_parameter("grid_topic").as_string();
  grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      grid_topic, qos_sensor_data,
      std::bind(&DockingTaskNode::grid_callback, this, std::placeholders::_1));
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

std::vector<bool> DockingTaskNode::search_line(const nav_msgs::msg::OccupancyGrid &grid, double dx0, double dy0,
                                               double dx1, double dy1) {
  int x0 = x0 / grid.info.resolution + grid.info.width / 2;
  int y0 = y0 / grid.info.resolution + grid.info.height / 2;
  int x1 = x1 / grid.info.resolution + grid.info.width / 2;
  int y1 = y1 / grid.info.resolution + grid.info.height / 2;

  std::vector<bool> occupied_cells;
  bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
  if (steep) {
    occupied_cells.reserve(std::abs(y1 - y0));
  } else {
    occupied_cells.reserve(std::abs(x1 - x0));
  }
  if (steep) {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  bool swap = x0 > x1;
  if (swap) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }
  int dx = x1 - x0;
  int dy = std::abs(y1 - y0);
  int error = 2 * dy - dx;
  int ystep = (y0 < y1) ? 1 : -1;
  int xbounds = steep ? grid.info.height : grid.info.width;
  int ybounds = steep ? grid.info.width : grid.info.height;
  int y = y0;

  for (int x = x0; x <= x1; x++) {
    if (steep) {
      if (x >= 0 && x < xbounds && y >= 0 && y < ybounds) {
        occupied_cells.push_back(grid.data[y + x * grid.info.width] > 0);
      }
    } else {
      if (y >= 0 && y < ybounds && x >= 0 && x < xbounds) {
        occupied_cells.push_back(grid.data[y * grid.info.width + x] > 0);
      }
    }
    if (error > 0) {
      y += ystep;
      error -= 2 * (dx - dy);
    } else {
      error += 2 * dy;
    }
  }
  if (swap) {
    std::reverse(occupied_cells.begin(), occupied_cells.end());
  }
  return occupied_cells;

}

void DockingTaskNode::find_dock_structure_edges(){

}

} // namespace docking_task