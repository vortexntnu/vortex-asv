#include <docking_task/docking_task_ros.hpp>

namespace docking_task {

DockingTaskNode::DockingTaskNode(const rclcpp::NodeOptions &options)
    : NjordTaskBaseNode("dock_localization_node", options) {

  declare_parameter<double>("distance_to_first_buoy_pair", 2.0);
  declare_parameter<double>("distance_between_buoys_in_pair", 5.0);
  declare_parameter<std::string>("grid_topic", "grid");
  declare_parameter<double>("dock_structure_absolute_width", 0.0);
  declare_parameter<double>("dock_structure_absolute_width_tolerance", 0.0);
  declare_parameter<double>("dock_edge_width", 0.0);
  declare_parameter<double>("line_search_distance", 0.0);
  declare_parameter<double>("dock_search_offset", 0.0);

  declare_parameter<double>("dock_width", 0.0);
  declare_parameter<double>("dock_width_tolerance", 0.0);
  declare_parameter<double>("dock_length", 0.0);
  declare_parameter<double>("dock_length_tolerance", 0.0);
  declare_parameter<double>("dock_edge_width_tolerance", 0.0);
  declare_parameter<int>("task_nr", 0.0);
  declare_parameter<double>("models.dynmod_stddev", 0.0);
  declare_parameter<double>("models.sen_stddev", 0.0);

  std::thread(&DockingTaskNode::main_task, this).detach();
}

void DockingTaskNode::main_task() {
  navigation_ready();
  // Starting docking task
  odom_start_point_ = get_odom()->pose.pose.position;
  // Eigen::Array<double, 2, 2> predicted_first_pair =
  // predict_first_buoy_pair();

  // sensor_msgs::msg::PointCloud2 buoy_vis_msg;
  // pcl::PointCloud<pcl::PointXYZRGB> buoy_vis;
  // pcl::PointXYZRGB buoy_red_0;
  // buoy_red_0.x = predicted_first_pair(0, 0);
  // buoy_red_0.y = predicted_first_pair(1, 0);
  // buoy_red_0.z = 0.0;
  // buoy_red_0.r = 255;
  // buoy_red_0.g = 0;
  // buoy_red_0.b = 0;
  // buoy_vis.push_back(buoy_red_0);
  // pcl::PointXYZRGB buoy_green_1;
  // buoy_green_1.x = predicted_first_pair(0, 1);
  // buoy_green_1.y = predicted_first_pair(1, 1);
  // buoy_green_1.z = 0.0;
  // buoy_green_1.r = 0;
  // buoy_green_1.g = 255;
  // buoy_green_1.b = 0;
  // buoy_vis.push_back(buoy_green_1);
  // pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  // buoy_vis_msg.header.frame_id = "odom";
  // buoy_vis_msg.header.stamp = this->now();
  // buoy_visualization_pub_->publish(buoy_vis_msg);

  // double distance_to_first_buoy_pair =
  //     this->get_parameter("distance_to_first_buoy_pair").as_double();
  // if (distance_to_first_buoy_pair > 6.0) {
  //   geometry_msgs::msg::Point waypoint_to_approach_first_pair_base_link;
  //   waypoint_to_approach_first_pair_base_link.x =
  //       distance_to_first_buoy_pair - 4.0;
  //   waypoint_to_approach_first_pair_base_link.y = 0.0;
  //   waypoint_to_approach_first_pair_base_link.z = 0.0;
  //   try {
  //     auto transform =
  //         tf_buffer_->lookupTransform("odom", "base_link",
  //         tf2::TimePointZero);
  //     geometry_msgs::msg::Point waypoint_to_approach_first_pair_odom;
  //     tf2::doTransform(waypoint_to_approach_first_pair_base_link,
  //                      waypoint_to_approach_first_pair_odom, transform);
  //     send_waypoint(waypoint_to_approach_first_pair_odom);
  //     set_desired_heading(odom_start_point_,
  //                         waypoint_to_approach_first_pair_odom);
  //     reach_waypoint(1.0);
  //   } catch (tf2::TransformException &ex) {
  //     RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
  //   }
  // }
  // std::vector<LandmarkPoseID> buoy_landmarks_0_to_1 =
  //     get_buoy_landmarks(predicted_first_pair);
  // if (buoy_landmarks_0_to_1.size() != 2) {
  //   RCLCPP_ERROR(this->get_logger(), "Could not find two buoys");
  // }

  // buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  // buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  // buoy_red_0 = pcl::PointXYZRGB();
  // buoy_green_1 = pcl::PointXYZRGB();
  // buoy_red_0.x = buoy_landmarks_0_to_1[0].pose_odom_frame.position.x;
  // buoy_red_0.y = buoy_landmarks_0_to_1[0].pose_odom_frame.position.y;
  // buoy_red_0.z = 0.0;
  // buoy_red_0.r = 255;
  // buoy_red_0.g = 0;
  // buoy_red_0.b = 0;
  // buoy_vis.push_back(buoy_red_0);
  // buoy_green_1.x = buoy_landmarks_0_to_1[1].pose_odom_frame.position.x;
  // buoy_green_1.y = buoy_landmarks_0_to_1[1].pose_odom_frame.position.y;
  // buoy_green_1.z = 0.0;
  // buoy_green_1.r = 0;
  // buoy_green_1.g = 255;
  // buoy_green_1.b = 0;
  // buoy_vis.push_back(buoy_green_1);
  // pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  // buoy_vis_msg.header.frame_id = "odom";
  // buoy_vis_msg.header.stamp = this->now();
  // buoy_visualization_pub_->publish(buoy_vis_msg);

  // geometry_msgs::msg::Point waypoint_first_pair;
  // waypoint_first_pair.x =
  //     (buoy_landmarks_0_to_1[0].pose_odom_frame.position.x +
  //      buoy_landmarks_0_to_1[1].pose_odom_frame.position.x) /
  //     2;
  // waypoint_first_pair.y =
  //     (buoy_landmarks_0_to_1[0].pose_odom_frame.position.y +
  //      buoy_landmarks_0_to_1[1].pose_odom_frame.position.y) /
  //     2;
  // waypoint_first_pair.z = 0.0;
  // send_waypoint(waypoint_first_pair);
  // set_desired_heading(odom_start_point_, waypoint_first_pair);
  // reach_waypoint(1.0);

  // // Second pair of buoys
  // Eigen::Array<double, 2, 2> predicted_first_and_second_pair =
  //     predict_second_buoy_pair(
  //         buoy_landmarks_0_to_1[0].pose_odom_frame.position,
  //         buoy_landmarks_0_to_1[1].pose_odom_frame.position);

  // buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  // buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  // buoy_red_0 = pcl::PointXYZRGB();
  // buoy_green_1 = pcl::PointXYZRGB();
  // buoy_red_0.x = predicted_first_and_second_pair(0, 0);
  // buoy_red_0.y = predicted_first_and_second_pair(1, 0);
  // buoy_red_0.z = 0.0;
  // buoy_red_0.r = 255;
  // buoy_red_0.g = 0;
  // buoy_red_0.b = 0;
  // buoy_vis.push_back(buoy_red_0);
  // buoy_green_1.x = predicted_first_and_second_pair(0, 1);
  // buoy_green_1.y = predicted_first_and_second_pair(1, 1);
  // buoy_green_1.z = 0.0;
  // buoy_green_1.r = 0;
  // buoy_green_1.g = 255;
  // buoy_green_1.b = 0;
  // buoy_vis.push_back(buoy_green_1);
  // pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  // buoy_vis_msg.header.frame_id = "odom";
  // buoy_vis_msg.header.stamp = this->now();
  // buoy_visualization_pub_->publish(buoy_vis_msg);

  // std::vector<LandmarkPoseID> buoy_landmarks_2_to_3 =
  //     get_buoy_landmarks(predicted_first_and_second_pair);
  // if (buoy_landmarks_2_to_3.size() != 2) {
  //   RCLCPP_ERROR(this->get_logger(), "Could not find four buoys");
  // }

  // buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  // buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  // buoy_red_0 = pcl::PointXYZRGB();
  // buoy_green_1 = pcl::PointXYZRGB();
  // buoy_red_0.x = buoy_landmarks_2_to_3[0].pose_odom_frame.position.x;
  // buoy_red_0.y = buoy_landmarks_2_to_3[0].pose_odom_frame.position.y;
  // buoy_red_0.z = 0.0;
  // buoy_red_0.r = 255;
  // buoy_red_0.g = 0;
  // buoy_red_0.b = 0;
  // buoy_vis.push_back(buoy_red_0);
  // buoy_green_1.x = buoy_landmarks_2_to_3[1].pose_odom_frame.position.x;
  // buoy_green_1.y = buoy_landmarks_2_to_3[1].pose_odom_frame.position.y;
  // buoy_green_1.z = 0.0;
  // buoy_green_1.r = 0;
  // buoy_green_1.g = 255;
  // buoy_green_1.b = 0;
  // buoy_vis.push_back(buoy_green_1);
  // pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  // buoy_vis_msg.header.frame_id = "odom";
  // buoy_vis_msg.header.stamp = this->now();
  // buoy_visualization_pub_->publish(buoy_vis_msg);

  // geometry_msgs::msg::Point waypoint_second_pair;
  // waypoint_second_pair.x =
  //     (buoy_landmarks_2_to_3[0].pose_odom_frame.position.x +
  //      buoy_landmarks_2_to_3[1].pose_odom_frame.position.x) /
  //     2;
  // waypoint_second_pair.y =
  //     (buoy_landmarks_2_to_3[0].pose_odom_frame.position.y +
  //      buoy_landmarks_2_to_3[1].pose_odom_frame.position.y) /
  //     2;
  // waypoint_second_pair.z = 0.0;
  // send_waypoint(waypoint_second_pair);
  // set_desired_heading(odom_start_point_, waypoint_second_pair);
  // reach_waypoint(1.0);

  // // Third pair of buoys
  // Eigen::Array<double, 2, 2> predicted_third_pair = predict_third_buoy_pair(
  //     buoy_landmarks_0_to_1[0].pose_odom_frame.position,
  //     buoy_landmarks_0_to_1[1].pose_odom_frame.position,
  //     buoy_landmarks_2_to_3[2].pose_odom_frame.position,
  //     buoy_landmarks_2_to_3[3].pose_odom_frame.position);

  // buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  // buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  // buoy_red_0 = pcl::PointXYZRGB();
  // buoy_green_1 = pcl::PointXYZRGB();
  // buoy_red_0.x = predicted_third_pair(0, 0);
  // buoy_red_0.y = predicted_third_pair(1, 0);
  // buoy_red_0.z = 0.0;
  // buoy_red_0.r = 255;
  // buoy_red_0.g = 0;
  // buoy_red_0.b = 0;
  // buoy_vis.push_back(buoy_red_0);
  // buoy_green_1.x = predicted_third_pair(0, 1);
  // buoy_green_1.y = predicted_third_pair(1, 1);
  // buoy_green_1.z = 0.0;
  // buoy_green_1.r = 0;
  // buoy_green_1.g = 255;
  // buoy_green_1.b = 0;
  // buoy_vis.push_back(buoy_green_1);
  // pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  // buoy_vis_msg.header.frame_id = "odom";
  // buoy_vis_msg.header.stamp = this->now();
  // buoy_visualization_pub_->publish(buoy_vis_msg);

  // std::vector<LandmarkPoseID> buoy_landmarks_4_to_5 =
  //     get_buoy_landmarks(predicted_third_pair);
  // if (buoy_landmarks_4_to_5.size() != 2) {
  //   RCLCPP_ERROR(this->get_logger(), "Could not find four buoys");
  // }

  // buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  // buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  // buoy_red_0 = pcl::PointXYZRGB();
  // buoy_green_1 = pcl::PointXYZRGB();
  // buoy_red_0.x = buoy_landmarks_4_to_5[0].pose_odom_frame.position.x;
  // buoy_red_0.y = buoy_landmarks_4_to_5[0].pose_odom_frame.position.y;
  // buoy_red_0.z = 0.0;
  // buoy_red_0.r = 255;
  // buoy_red_0.g = 0;
  // buoy_red_0.b = 0;
  // buoy_vis.push_back(buoy_red_0);
  // buoy_green_1.x = buoy_landmarks_4_to_5[1].pose_odom_frame.position.x;
  // buoy_green_1.y = buoy_landmarks_4_to_5[1].pose_odom_frame.position.y;
  // buoy_green_1.z = 0.0;
  // buoy_green_1.r = 0;
  // buoy_green_1.g = 255;
  // buoy_green_1.b = 0;
  // buoy_vis.push_back(buoy_green_1);
  // pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  // buoy_vis_msg.header.frame_id = "odom";
  // buoy_vis_msg.header.stamp = this->now();
  // buoy_visualization_pub_->publish(buoy_vis_msg);

  // Eigen::Vector2d direction_vector_up;
  // direction_vector_up << (buoy_landmarks_4_to_5[0].pose_odom_frame.position.x
  // +
  //      buoy_landmarks_4_to_5[1].pose_odom_frame.position.x) /
  //     2 - odom_start_point_.x,
  //     (buoy_landmarks_4_to_5[0].pose_odom_frame.position.y +
  //      buoy_landmarks_4_to_5[1].pose_odom_frame.position.y) /
  //     2 - odom_start_point_.y;
  // direction_vector_up.normalize();

  // geometry_msgs::msg::Point waypoint_third_pair;
  // waypoint_third_pair.x =
  //     (buoy_landmarks_4_to_5[0].pose_odom_frame.position.x +
  //      buoy_landmarks_4_to_5[1].pose_odom_frame.position.x) /
  //     2;
  // waypoint_third_pair.y =
  //     (buoy_landmarks_4_to_5[0].pose_odom_frame.position.y +
  //      buoy_landmarks_4_to_5[1].pose_odom_frame.position.y) /
  //     2;
  // waypoint_third_pair.z = 0.0;
  // send_waypoint(waypoint_third_pair);
  // set_desired_heading(odom_start_point_, waypoint_third_pair);

  // geometry_msgs::msg::Point waypoint_through_third_pair;
  // waypoint_through_third_pair.x =
  //     (buoy_landmarks_4_to_5[0].pose_odom_frame.position.x +
  //      buoy_landmarks_4_to_5[1].pose_odom_frame.position.x) /
  //     2 + direction_vector_up(0) * 2;
  // waypoint_through_third_pair.y =
  //     (buoy_landmarks_4_to_5[0].pose_odom_frame.position.y +
  //      buoy_landmarks_4_to_5[1].pose_odom_frame.position.y) /
  //     2 + direction_vector_up(1) * 2;
  // waypoint_through_third_pair.z = 0.0;
  // send_waypoint(waypoint_through_third_pair);
  // set_desired_heading(odom_start_point_, waypoint_through_third_pair);
  // reach_waypoint(1.0);

  geometry_msgs::msg::Point direction_vector_up_base_link;
  direction_vector_up_base_link.x = 1.0;
  direction_vector_up_base_link.y = 0.0;
  direction_vector_up_base_link.z = 0.0;
  Eigen::Vector2d direction_vector_up_test;
  try {
    auto transform =
        tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    geometry_msgs::msg::Point direction_vector_up_odom;
    tf2::doTransform(direction_vector_up_base_link, direction_vector_up_odom,
                     transform);
    direction_vector_up_test << direction_vector_up_odom.x,
        direction_vector_up_odom.y;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
  }

  std::thread(&DockingTaskNode::initialize_grid_sub, this).detach();
  Eigen::Vector2d edge1, edge2;
  // std::tie(edge1, edge2) = find_dock_structure_edges(waypoint_third_pair,
  // direction_vector_up);

  std::tie(edge1, edge2) =
      find_dock_structure_edges(odom_start_point_, direction_vector_up_test);
  RCLCPP_INFO(this->get_logger(), "Edge 1: %f, %f", edge1(0), edge1(1));
  RCLCPP_INFO(this->get_logger(), "Edge 2: %f, %f", edge2(0), edge2(1));
  sensor_msgs::msg::PointCloud2 buoy_vis_msg;
  pcl::PointCloud<pcl::PointXYZRGB> buoy_vis;
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  pcl::PointXYZRGB edge1_point;
  edge1_point.x = edge1(0);
  edge1_point.y = edge1(1);
  edge1_point.z = 0.0;
  edge1_point.r = 255;
  edge1_point.g = 0;
  edge1_point.b = 0;
  pcl::PointXYZRGB edge2_point;
  edge2_point.x = edge2(0);
  edge2_point.y = edge2(1);
  edge2_point.z = 0.0;
  edge2_point.r = 0;
  edge2_point.g = 255;
  edge2_point.b = 0;
  buoy_vis.push_back(edge1_point);
  buoy_vis.push_back(edge2_point);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);
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

std::vector<int8_t>
DockingTaskNode::search_line(const nav_msgs::msg::OccupancyGrid &grid,
                             double dx0, double dy0, double dx1, double dy1) {
  int x0 = dx0 / grid.info.resolution + grid.info.width / 2;
  int y0 = dy0 / grid.info.resolution + grid.info.height / 2;
  int x1 = dx1 / grid.info.resolution + grid.info.width / 2;
  int y1 = dy1 / grid.info.resolution + grid.info.height / 2;

  std::vector<int8_t> occupied_cells;
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
        occupied_cells.push_back(grid.data[y + x * grid.info.width]);
      }
    } else {
      if (y >= 0 && y < ybounds && x >= 0 && x < xbounds) {
        occupied_cells.push_back(grid.data[y * grid.info.width + x]);
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

std::pair<Eigen::Vector2d, Eigen::Vector2d>
DockingTaskNode::find_dock_structure_edges(
    const geometry_msgs::msg::Point &waypoint_third_pair,
    Eigen::Vector2d &direction_vector_up) {
  double dock_width_structure_absolute_width =
      this->get_parameter("dock_structure_absolute_width").as_double();
  double dock_width_structure_absolute_width_tolerance =
      this->get_parameter("dock_structure_absolute_width_tolerance")
          .as_double();
  double dock_search_offset =
      this->get_parameter("dock_search_offset").as_double();
  Eigen::Vector2d direction_vector_right;
  // Rotate direction vector 90 degrees to the right
  direction_vector_right << direction_vector_up(1), -direction_vector_up(0);
  geometry_msgs::msg::Point waypoint_third_pair_map;
  bool transform_success = false;
  while (!transform_success) {
    try {
      auto transform =
          tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
      tf2::doTransform(waypoint_third_pair, waypoint_third_pair_map, transform);
      transform_success = true;
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
  }
  Eigen::Vector2d line_start, line_end;
  line_start << waypoint_third_pair_map.x -
                    (direction_vector_right(0) *
                     (dock_width_structure_absolute_width / 2 +
                      dock_width_structure_absolute_width_tolerance)) +
                    (direction_vector_up(0) * dock_search_offset),
      waypoint_third_pair_map.y -
          (direction_vector_right(1) *
           (dock_width_structure_absolute_width / 2 +
            dock_width_structure_absolute_width_tolerance)) +
          (direction_vector_up(1) * dock_search_offset);
  line_end << waypoint_third_pair_map.x +
                  (direction_vector_right(0) *
                   (dock_width_structure_absolute_width / 2 +
                    dock_width_structure_absolute_width_tolerance)) +
                  (direction_vector_up(0) * dock_search_offset),
      waypoint_third_pair_map.y +
          (direction_vector_right(1) *
           (dock_width_structure_absolute_width / 2 +
            dock_width_structure_absolute_width_tolerance)) +
          (direction_vector_up(1) * dock_search_offset);
  // double line_length = std::sqrt(std::pow(line_end(0) - line_start(0), 2) +
  // std::pow(line_end(1) - line_start(1), 2));
  double dock_edge_width = this->get_parameter("dock_edge_width").as_double();
  double dock_width = dock_width_structure_absolute_width - 2 * dock_edge_width;
  nav_msgs::msg::MapMetaData grid_info = get_grid()->info;
  double line_search_distance =
      this->get_parameter("line_search_distance").as_double();
  int search_iterations =
      static_cast<int>(line_search_distance / grid_info.resolution);
  int result_index = -1;
  // int line_vector_size = -1;
  while (true) {
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid = get_grid();
    std::vector<int8_t> occupied_cells = search_line(
        *grid, line_start(0), line_start(1), line_end(0), line_end(1));
    int max_result = std::numeric_limits<int>::min();
    int max_index = -1;
    for (int i = 0; i < search_iterations; i++) {
      int left_edge_left = i;
      int left_edge_right =
          left_edge_left +
          static_cast<int>(dock_edge_width / grid->info.resolution);
      int right_edge_left =
          left_edge_right +
          static_cast<int>(dock_width / grid->info.resolution);
      int right_edge_right =
          right_edge_left +
          static_cast<int>(dock_edge_width / grid->info.resolution);
      int left_edge_occupied_cells =
          std::accumulate(occupied_cells.begin() + left_edge_left,
                          occupied_cells.begin() + left_edge_right, 0);
      int right_edge_occupied_cells =
          std::accumulate(occupied_cells.begin() + right_edge_left,
                          occupied_cells.begin() + right_edge_right, 0);
      if (left_edge_occupied_cells == 0 || right_edge_occupied_cells == 0) {
        continue;
      }
      if (left_edge_occupied_cells + right_edge_occupied_cells > max_result) {
        max_result = left_edge_occupied_cells + right_edge_occupied_cells;
        max_index = i;
      }
    }
    if (max_result > 300) {
      result_index = max_index;
      // line_vector_size = occupied_cells.size();
    }
    line_start(0) =
        line_start(0) + direction_vector_up(0) * grid_info.resolution;
    line_start(1) =
        line_start(1) + direction_vector_up(1) * grid_info.resolution;
    line_end(0) = line_end(0) + direction_vector_up(0) * grid_info.resolution;
    line_end(1) = line_end(1) + direction_vector_up(1) * grid_info.resolution;
  }
  int right_edge_left_index =
      result_index + static_cast<int>(dock_edge_width / grid_info.resolution) +
      static_cast<int>(dock_width / grid_info.resolution);

  Eigen::Vector2d dock_edge_left, dock_edge_right;
  dock_edge_left << line_start(0) +
                        (direction_vector_right(0) * result_index *
                         grid_info.resolution) +
                        dock_edge_width / 2,
      line_start(1) +
          (direction_vector_right(1) * result_index * grid_info.resolution) +
          dock_edge_width / 2;
  dock_edge_right << line_start(0) +
                         (direction_vector_right(0) * right_edge_left_index *
                          grid_info.resolution) +
                         dock_edge_width / 2,
      line_start(1) +
          (direction_vector_right(1) * right_edge_left_index *
           grid_info.resolution) +
          dock_edge_width / 2;

  return std::make_pair(dock_edge_left, dock_edge_right);
}

} // namespace docking_task