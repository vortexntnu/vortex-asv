#include <navigation_task/navigation_task_ros.hpp>

namespace navigation_task {

NavigationTaskNode::NavigationTaskNode(const rclcpp::NodeOptions &options)
    : NjordTaskBaseNode("navigation_task_node", options) {

  declare_parameter<double>("distance_to_first_buoy_pair", 2.0);

  std::thread(&NavigationTaskNode::main_task, this).detach();
}

void NavigationTaskNode::main_task() {

  navigation_ready();
  RCLCPP_INFO(this->get_logger(), "Navigation task started");
  auto odom_msg = get_odom();
  odom_start_point_ = odom_msg->pose.pose.position;
  // First pair of buoys
  Eigen::Array22d predicted_first_buoy_pair = predict_first_buoy_pair();
  sensor_msgs::msg::PointCloud2 buoy_vis_msg;
  pcl::PointCloud<pcl::PointXYZRGB> buoy_vis;

  pcl::PointXYZRGB buoy_red_0;
  buoy_red_0.x = predicted_first_buoy_pair(0, 0);
  buoy_red_0.y = predicted_first_buoy_pair(1, 0);
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  pcl::PointXYZRGB buoy_green_1;
  buoy_green_1.x = predicted_first_buoy_pair(0, 1);
  buoy_green_1.y = predicted_first_buoy_pair(1, 1);
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
      auto transform = tf_buffer_->lookupTransform(
          "odom", "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
      geometry_msgs::msg::Point waypoint_to_approach_first_pair_odom;
      tf2::doTransform(waypoint_to_approach_first_pair_base_link,
                       waypoint_to_approach_first_pair_odom, transform);
      send_waypoint(waypoint_to_approach_first_pair_odom);
      set_desired_heading(odom_start_point_, waypoint_to_approach_first_pair_odom);
      reach_waypoint(1.0);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
  }
  std::vector<LandmarkPoseID> buoy_landmarks_0_to_1 =
      get_buoy_landmarks(predicted_first_buoy_pair);
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
  Eigen::Array<double, 2, 2> predicted_second_pair =
      predict_second_buoy_pair(
          buoy_landmarks_0_to_1[0].pose_odom_frame.position,
          buoy_landmarks_0_to_1[1].pose_odom_frame.position);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  pcl::PointXYZRGB buoy_red_2;
  pcl::PointXYZRGB buoy_green_3;
  buoy_red_2.x = predicted_second_pair(0, 0);
  buoy_red_2.y = predicted_second_pair(1, 0);
  buoy_red_2.z = 0.0;
  buoy_red_2.r = 255;
  buoy_red_2.g = 0;
  buoy_red_2.b = 0;
  buoy_vis.push_back(buoy_red_2);
  buoy_green_3.x = predicted_second_pair(0, 1);
  buoy_green_3.y = predicted_second_pair(1, 1);
  buoy_green_3.z = 0.0;
  buoy_green_3.r = 0;
  buoy_green_3.g = 255;
  buoy_green_3.b = 0;
  buoy_vis.push_back(buoy_green_3);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  std::vector<LandmarkPoseID> buoy_landmarks_2_to_3 =
      get_buoy_landmarks(predicted_second_pair);
  if (buoy_landmarks_2_to_3.size() != 2) {
    RCLCPP_ERROR(this->get_logger(), "Could not find four buoys");
  }

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_2 = pcl::PointXYZRGB();
  buoy_green_3 = pcl::PointXYZRGB();
  buoy_red_2.x = buoy_landmarks_2_to_3[0].pose_odom_frame.position.x;
  buoy_red_2.y = buoy_landmarks_2_to_3[0].pose_odom_frame.position.y;
  buoy_red_2.z = 0.0;
  buoy_red_2.r = 255;
  buoy_red_2.g = 0;
  buoy_red_2.b = 0;
  buoy_vis.push_back(buoy_red_2);
  buoy_green_3.x = buoy_landmarks_2_to_3[1].pose_odom_frame.position.x;
  buoy_green_3.y = buoy_landmarks_2_to_3[1].pose_odom_frame.position.y;
  buoy_green_3.z = 0.0;
  buoy_green_3.r = 0;
  buoy_green_3.g = 255;
  buoy_green_3.b = 0;
  buoy_vis.push_back(buoy_green_3);
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
  set_desired_heading(waypoint_first_pair, waypoint_second_pair);
  reach_waypoint(1.0);

  // West buoy
  Eigen::Array<double, 2, 1> predicted_west_buoy =
      predict_west_buoy(buoy_landmarks_0_to_1[0].pose_odom_frame.position,
                        buoy_landmarks_0_to_1[1].pose_odom_frame.position,
                        buoy_landmarks_2_to_3[0].pose_odom_frame.position,
                        buoy_landmarks_2_to_3[1].pose_odom_frame.position);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  pcl::PointXYZRGB buoy_west;
  buoy_west.x = predicted_west_buoy(0, 0);
  buoy_west.y = predicted_west_buoy(1, 0);
  buoy_west.z = 0.0;
  buoy_west.r = 255;
  buoy_west.g = 255;
  buoy_west.b = 0;
  buoy_vis.push_back(buoy_west);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);
  
  std::vector<LandmarkPoseID> buoy_landmarks_4 =
      get_buoy_landmarks(predicted_west_buoy);
  if (buoy_landmarks_4.size() != 1) {
    RCLCPP_ERROR(this->get_logger(), "Could not find west buoy");
  }
  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_west = pcl::PointXYZRGB();
  buoy_west.x = buoy_landmarks_4[0].pose_odom_frame.position.x;
  buoy_west.y = buoy_landmarks_4[0].pose_odom_frame.position.y;
  buoy_west.z = 0.0;
  buoy_west.r = 255;
  buoy_west.g = 255;
  buoy_west.b = 0;
  buoy_vis.push_back(buoy_west);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);
  
  Eigen::Vector2d direction_vector_upwards;
  direction_vector_upwards
      << buoy_landmarks_2_to_3[0].pose_odom_frame.position.x -
             buoy_landmarks_2_to_3[1].pose_odom_frame.position.x,
      buoy_landmarks_2_to_3[0].pose_odom_frame.position.y -
          buoy_landmarks_2_to_3[1].pose_odom_frame.position.y;
  direction_vector_upwards.normalize();

  // Set waypoint two meters above west buoy
  geometry_msgs::msg::Point waypoint_west_buoy;
  waypoint_west_buoy.x = buoy_landmarks_4[0].pose_odom_frame.position.x +
                         direction_vector_upwards(0) * 3;
  waypoint_west_buoy.y = buoy_landmarks_4[0].pose_odom_frame.position.y +
                         direction_vector_upwards(1) * 3;
  waypoint_west_buoy.z = 0.0;

  send_waypoint(waypoint_west_buoy);
  set_desired_heading(waypoint_second_pair, waypoint_west_buoy);
  reach_waypoint(1.0);

  // Third pair of buoys
  Eigen::Array<double, 2, 2> predicted_third_buoy_pair =
      predict_third_buoy_pair(
          buoy_landmarks_0_to_1[0].pose_odom_frame.position,
          buoy_landmarks_0_to_1[1].pose_odom_frame.position,
          buoy_landmarks_2_to_3[0].pose_odom_frame.position,
          buoy_landmarks_2_to_3[1].pose_odom_frame.position);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  pcl::PointXYZRGB buoy_red_5;
  pcl::PointXYZRGB buoy_green_6;
  buoy_red_5.x = predicted_third_buoy_pair(0, 0);
  buoy_red_5.y = predicted_third_buoy_pair(1, 0);
  buoy_red_5.z = 0.0;
  buoy_red_5.r = 255;
  buoy_red_5.g = 0;
  buoy_red_5.b = 0;
  buoy_vis.push_back(buoy_red_5);
  buoy_green_6.x = predicted_third_buoy_pair(0, 1);
  buoy_green_6.y = predicted_third_buoy_pair(1, 1);
  buoy_green_6.z = 0.0;
  buoy_green_6.r = 0;
  buoy_green_6.g = 255;
  buoy_green_6.b = 0;
  buoy_vis.push_back(buoy_green_6);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  std::vector<LandmarkPoseID> buoy_landmarks_5_to_6 =
      get_buoy_landmarks(predicted_third_buoy_pair);
  if (buoy_landmarks_5_to_6.size() != 2) {
    RCLCPP_ERROR(this->get_logger(), "Could not find third buoy pair");
  }
  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_5 = pcl::PointXYZRGB();
  buoy_green_6 = pcl::PointXYZRGB();
  buoy_red_5.x = buoy_landmarks_5_to_6[0].pose_odom_frame.position.x;
  buoy_red_5.y = buoy_landmarks_5_to_6[0].pose_odom_frame.position.y;
  buoy_red_5.z = 0.0;
  buoy_red_5.r = 255;
  buoy_red_5.g = 0;
  buoy_red_5.b = 0;
  buoy_vis.push_back(buoy_red_5);
  buoy_green_6.x = buoy_landmarks_5_to_6[1].pose_odom_frame.position.x;
  buoy_green_6.y = buoy_landmarks_5_to_6[1].pose_odom_frame.position.y;
  buoy_green_6.z = 0.0;
  buoy_green_6.r = 0;
  buoy_green_6.g = 255;
  buoy_green_6.b = 0;
  buoy_vis.push_back(buoy_green_6);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  geometry_msgs::msg::Point waypoint_third_pair;
  waypoint_third_pair.x =
      (buoy_landmarks_5_to_6[0].pose_odom_frame.position.x +
       buoy_landmarks_5_to_6[1].pose_odom_frame.position.x) /
      2;
  waypoint_third_pair.y =
      (buoy_landmarks_5_to_6[0].pose_odom_frame.position.y +
       buoy_landmarks_5_to_6[1].pose_odom_frame.position.y) /
      2;
  waypoint_third_pair.z = 0.0;

  send_waypoint(waypoint_third_pair);
  set_desired_heading(waypoint_west_buoy, waypoint_third_pair);
  Eigen::Vector2d direction_vector_second_to_third_pair;
  direction_vector_second_to_third_pair
      << waypoint_third_pair.x - waypoint_second_pair.x,
      waypoint_third_pair.y - waypoint_second_pair.y;
  direction_vector_second_to_third_pair.normalize();
  geometry_msgs::msg::Point waypoint_through_third_pair;
  waypoint_through_third_pair.x =
      waypoint_third_pair.x + direction_vector_second_to_third_pair(0) * 2;
  waypoint_through_third_pair.y =
      waypoint_third_pair.y + direction_vector_second_to_third_pair(1) * 2;
  waypoint_through_third_pair.z = 0.0;
  send_waypoint(waypoint_through_third_pair);
  set_desired_heading(waypoint_third_pair, waypoint_through_third_pair);
  reach_waypoint(1.0);

  // North buoy
  Eigen::Array<double, 2, 1> predicted_north_buoy =
      predict_north_buoy(buoy_landmarks_5_to_6[0].pose_odom_frame.position,
                         buoy_landmarks_5_to_6[1].pose_odom_frame.position,
                         direction_vector_second_to_third_pair);
  std::vector<LandmarkPoseID> buoy_landmarks_7 =
      get_buoy_landmarks(predicted_north_buoy);
  if (buoy_landmarks_7.size() != 1) {
    RCLCPP_ERROR(this->get_logger(), "Could not find north buoy");
  }
  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  pcl::PointXYZRGB buoy_north;
  buoy_north.x = buoy_landmarks_7[0].pose_odom_frame.position.x;
  buoy_north.y = buoy_landmarks_7[0].pose_odom_frame.position.y;
  buoy_north.z = 0.0;
  buoy_north.r = 255;
  buoy_north.g = 255;
  buoy_north.b = 0;
  buoy_vis.push_back(buoy_north);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  geometry_msgs::msg::Point waypoint_north_buoy;
  waypoint_north_buoy.x = buoy_landmarks_7[0].pose_odom_frame.position.x +
                          direction_vector_second_to_third_pair(0) * 2.5;
  waypoint_north_buoy.y = buoy_landmarks_7[0].pose_odom_frame.position.y +
                          direction_vector_second_to_third_pair(1) * 2.5;
  waypoint_north_buoy.z = 0.0;
  send_waypoint(waypoint_north_buoy);
  set_desired_heading(waypoint_through_third_pair, waypoint_north_buoy);
  reach_waypoint(1.0);

  // South buoy
  Eigen::Array<double, 2, 1> predicted_south_buoy =
      predict_south_buoy(buoy_landmarks_5_to_6[0].pose_odom_frame.position,
                         buoy_landmarks_5_to_6[1].pose_odom_frame.position,
                         direction_vector_second_to_third_pair);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  pcl::PointXYZRGB buoy_south;
  buoy_south.x = predicted_south_buoy(0, 0);
  buoy_south.y = predicted_south_buoy(1, 0);
  buoy_south.z = 0.0;
  buoy_south.r = 255;
  buoy_south.g = 255;
  buoy_south.b = 0;
  buoy_vis.push_back(buoy_south);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  std::vector<LandmarkPoseID> buoy_landmarks_8 =
      get_buoy_landmarks(predicted_south_buoy);
  if (buoy_landmarks_8.size() != 1) {
    RCLCPP_ERROR(this->get_logger(), "Could not find south buoy");
  }
  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_south = pcl::PointXYZRGB();
  buoy_south.x = buoy_landmarks_8[0].pose_odom_frame.position.x;
  buoy_south.y = buoy_landmarks_8[0].pose_odom_frame.position.y;
  buoy_south.z = 0.0;
  buoy_south.r = 255;
  buoy_south.g = 255;
  buoy_south.b = 0;
  buoy_vis.push_back(buoy_south);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  Eigen::Vector2d direction_vector_downwards;
  direction_vector_downwards
      << buoy_landmarks_5_to_6[1].pose_odom_frame.position.x -
             buoy_landmarks_5_to_6[10].pose_odom_frame.position.x,
      buoy_landmarks_5_to_6[1].pose_odom_frame.position.y -
          buoy_landmarks_5_to_6[0].pose_odom_frame.position.y;
  direction_vector_downwards.normalize();

  geometry_msgs::msg::Point waypoint_south_buoy;
  waypoint_south_buoy.x = buoy_landmarks_8[0].pose_odom_frame.position.x -
                          direction_vector_second_to_third_pair(0) * 3
                          + direction_vector_downwards(0) * 1;
  waypoint_south_buoy.y = buoy_landmarks_8[0].pose_odom_frame.position.y -
                          direction_vector_second_to_third_pair(1) * 3
                          + direction_vector_downwards(1) * 1;
  waypoint_south_buoy.z = 0.0;
  send_waypoint(waypoint_south_buoy);
  set_desired_heading(waypoint_north_buoy, waypoint_south_buoy);
  reach_waypoint(0.2);

  // Fourth pair of buoys
  Eigen::Array<double, 2, 2> predicted_fourth_buoy_pair =
      predict_fourth_buoy_pair(
          buoy_landmarks_5_to_6[0].pose_odom_frame.position,
          buoy_landmarks_5_to_6[1].pose_odom_frame.position);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  pcl::PointXYZRGB buoy_red_9;
  pcl::PointXYZRGB buoy_green_10;
  buoy_red_9.x = predicted_fourth_buoy_pair(0, 0);
  buoy_red_9.y = predicted_fourth_buoy_pair(1, 0);
  buoy_red_9.z = 0.0;
  buoy_red_9.r = 255;
  buoy_red_9.g = 0;
  buoy_red_9.b = 0;
  buoy_vis.push_back(buoy_red_9);
  buoy_green_10.x = predicted_fourth_buoy_pair(0, 1);
  buoy_green_10.y = predicted_fourth_buoy_pair(1, 1);
  buoy_green_10.z = 0.0;
  buoy_green_10.r = 0;
  buoy_green_10.g = 255;
  buoy_green_10.b = 0;
  buoy_vis.push_back(buoy_green_10);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  std::vector<LandmarkPoseID> buoy_landmarks_9_to_10 =
      get_buoy_landmarks(predicted_fourth_buoy_pair);
  if (buoy_landmarks_9_to_10.size() != 2) {
    RCLCPP_ERROR(this->get_logger(), "Could not find fourth buoy pair");
  }

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_9 = pcl::PointXYZRGB();
  buoy_green_10 = pcl::PointXYZRGB();
  buoy_red_9.x = buoy_landmarks_9_to_10[0].pose_odom_frame.position.x;
  buoy_red_9.y = buoy_landmarks_9_to_10[0].pose_odom_frame.position.y;
  buoy_red_9.z = 0.0;
  buoy_red_9.r = 255;
  buoy_red_9.g = 0;
  buoy_red_9.b = 0;
  buoy_vis.push_back(buoy_red_9);
  buoy_green_10.x = buoy_landmarks_9_to_10[1].pose_odom_frame.position.x;
  buoy_green_10.y = buoy_landmarks_9_to_10[1].pose_odom_frame.position.y;
  buoy_green_10.z = 0.0;
  buoy_green_10.r = 0;
  buoy_green_10.g = 255;
  buoy_green_10.b = 0;
  buoy_vis.push_back(buoy_green_10);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  geometry_msgs::msg::Point waypoint_fourth_pair;
  waypoint_fourth_pair.x =
      (buoy_landmarks_9_to_10[0].pose_odom_frame.position.x +
       buoy_landmarks_9_to_10[1].pose_odom_frame.position.x) /
      2;
  waypoint_fourth_pair.y =
      (buoy_landmarks_9_to_10[0].pose_odom_frame.position.y +
       buoy_landmarks_9_to_10[1].pose_odom_frame.position.y) /
      2;
  waypoint_fourth_pair.z = 0.0;
  send_waypoint(waypoint_fourth_pair);
  set_desired_heading(waypoint_south_buoy, waypoint_fourth_pair);
  reach_waypoint(1.0);

  // East buoy
  Eigen::Array<double, 2, 1> predicted_east_buoy =
      predict_east_buoy(buoy_landmarks_9_to_10[0].pose_odom_frame.position,
                        buoy_landmarks_9_to_10[1].pose_odom_frame.position,
                        direction_vector_second_to_third_pair);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  pcl::PointXYZRGB buoy_east;
  buoy_east.x = predicted_east_buoy(0, 0);
  buoy_east.y = predicted_east_buoy(1, 0);
  buoy_east.z = 0.0;
  buoy_east.r = 255;
  buoy_east.g = 255;
  buoy_east.b = 0;
  buoy_vis.push_back(buoy_east);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  std::vector<LandmarkPoseID> buoy_landmarks_11 =
      get_buoy_landmarks(predicted_east_buoy);
  if (buoy_landmarks_11.size() != 1) {
    RCLCPP_ERROR(this->get_logger(), "Could not find east buoy");
  }

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_east = pcl::PointXYZRGB();
  buoy_east.x = buoy_landmarks_11[0].pose_odom_frame.position.x;
  buoy_east.y = buoy_landmarks_11[0].pose_odom_frame.position.y;
  buoy_east.z = 0.0;
  buoy_east.r = 255;
  buoy_east.g = 255;
  buoy_east.b = 0;
  buoy_vis.push_back(buoy_east);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  Eigen::Vector2d direction_vector_9_to_10;
  direction_vector_9_to_10
      << buoy_landmarks_9_to_10[1].pose_odom_frame.position.x -
             buoy_landmarks_9_to_10[0].pose_odom_frame.position.x,
      buoy_landmarks_9_to_10[1].pose_odom_frame.position.y -
          buoy_landmarks_9_to_10[0].pose_odom_frame.position.y;
  direction_vector_9_to_10.normalize();
  geometry_msgs::msg::Point waypoint_east_buoy;
  waypoint_east_buoy.x = buoy_landmarks_11[0].pose_odom_frame.position.x +
                         direction_vector_9_to_10(0) * 3;
  waypoint_east_buoy.y = buoy_landmarks_11[0].pose_odom_frame.position.y +
                         direction_vector_9_to_10(1) * 3;
  waypoint_east_buoy.z = 0.0;
  send_waypoint(waypoint_east_buoy);
  set_desired_heading(waypoint_fourth_pair, waypoint_east_buoy);
  reach_waypoint(1.0);

  // Fifth pair of buoys
  Eigen::Array<double, 2, 2> predicted_fifth_buoy_pair =
      predict_fifth_buoy_pair(
          buoy_landmarks_9_to_10[0].pose_odom_frame.position,
          buoy_landmarks_9_to_10[1].pose_odom_frame.position,
          direction_vector_second_to_third_pair);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  pcl::PointXYZRGB buoy_red_12;
  pcl::PointXYZRGB buoy_green_13;
  buoy_red_12.x = predicted_fifth_buoy_pair(0, 0);
  buoy_red_12.y = predicted_fifth_buoy_pair(1, 0);
  buoy_red_12.z = 0.0;
  buoy_red_12.r = 255;
  buoy_red_12.g = 0;
  buoy_red_12.b = 0;
  buoy_vis.push_back(buoy_red_12);
  buoy_green_13.x = predicted_fifth_buoy_pair(0, 1);
  buoy_green_13.y = predicted_fifth_buoy_pair(1, 1);
  buoy_green_13.z = 0.0;
  buoy_green_13.r = 0;
  buoy_green_13.g = 255;
  buoy_green_13.b = 0;
  buoy_vis.push_back(buoy_green_13);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  std::vector<LandmarkPoseID> buoy_landmarks_12_to_13 =
      get_buoy_landmarks(predicted_fifth_buoy_pair);
  if (buoy_landmarks_12_to_13.size() != 2) {
    RCLCPP_ERROR(this->get_logger(), "Could not find fifth buoy pair");
  }
  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_12 = pcl::PointXYZRGB();
  buoy_green_13 = pcl::PointXYZRGB();
  buoy_red_12.x = buoy_landmarks_12_to_13[0].pose_odom_frame.position.x;
  buoy_red_12.y = buoy_landmarks_12_to_13[0].pose_odom_frame.position.y;
  buoy_red_12.z = 0.0;
  buoy_red_12.r = 255;
  buoy_red_12.g = 0;
  buoy_red_12.b = 0;
  buoy_vis.push_back(buoy_red_12);
  buoy_green_13.x = buoy_landmarks_12_to_13[1].pose_odom_frame.position.x;
  buoy_green_13.y = buoy_landmarks_12_to_13[1].pose_odom_frame.position.y;
  buoy_green_13.z = 0.0;
  buoy_green_13.r = 0;
  buoy_green_13.g = 255;
  buoy_green_13.b = 0;
  buoy_vis.push_back(buoy_green_13);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  geometry_msgs::msg::Point waypoint_fifth_pair;
  waypoint_fifth_pair.x =
      (buoy_landmarks_12_to_13[0].pose_odom_frame.position.x +
       buoy_landmarks_12_to_13[1].pose_odom_frame.position.x) /
      2;
  waypoint_fifth_pair.y =
      (buoy_landmarks_12_to_13[0].pose_odom_frame.position.y +
       buoy_landmarks_12_to_13[1].pose_odom_frame.position.y) /
      2;
  waypoint_fifth_pair.z = 0.0;
  send_waypoint(waypoint_fifth_pair);
  set_desired_heading(waypoint_east_buoy, waypoint_fifth_pair);
  reach_waypoint(1.0);

  // Sixth pair of buoys
  Eigen::Array<double, 2, 2> predicted_sixth_buoy_pair =
      predict_sixth_buoy_pair(
          buoy_landmarks_9_to_10[0].pose_odom_frame.position,
          buoy_landmarks_9_to_10[1].pose_odom_frame.position,
          buoy_landmarks_12_to_13[0].pose_odom_frame.position,
          buoy_landmarks_12_to_13[1].pose_odom_frame.position);
  
  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  pcl::PointXYZRGB buoy_red_14;
  pcl::PointXYZRGB buoy_green_15;
  buoy_red_14.x = predicted_sixth_buoy_pair(0, 0);
  buoy_red_14.y = predicted_sixth_buoy_pair(1, 0);
  buoy_red_14.z = 0.0;
  buoy_red_14.r = 255;
  buoy_red_14.g = 0;
  buoy_red_14.b = 0;
  buoy_vis.push_back(buoy_red_14);
  buoy_green_15.x = predicted_sixth_buoy_pair(0, 1);
  buoy_green_15.y = predicted_sixth_buoy_pair(1, 1);
  buoy_green_15.z = 0.0;
  buoy_green_15.r = 0;
  buoy_green_15.g = 255;
  buoy_green_15.b = 0;
  buoy_vis.push_back(buoy_green_15);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  std::vector<LandmarkPoseID> buoy_landmarks_14_to_15 =
      get_buoy_landmarks(predicted_sixth_buoy_pair);
  if (buoy_landmarks_14_to_15.size() != 2) {
    RCLCPP_ERROR(this->get_logger(), "Could not find sixth buoy pair");
  }

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_14 = pcl::PointXYZRGB();
  buoy_green_15 = pcl::PointXYZRGB();
  buoy_red_14.x = buoy_landmarks_14_to_15[0].pose_odom_frame.position.x;
  buoy_red_14.y = buoy_landmarks_14_to_15[0].pose_odom_frame.position.y;
  buoy_red_14.z = 0.0;
  buoy_red_14.r = 255;
  buoy_red_14.g = 0;
  buoy_red_14.b = 0;
  buoy_vis.push_back(buoy_red_14);
  buoy_green_15.x = buoy_landmarks_14_to_15[1].pose_odom_frame.position.x;
  buoy_green_15.y = buoy_landmarks_14_to_15[1].pose_odom_frame.position.y;
  buoy_green_15.z = 0.0;
  buoy_green_15.r = 0;
  buoy_green_15.g = 255;
  buoy_green_15.b = 0;
  buoy_vis.push_back(buoy_green_15);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  geometry_msgs::msg::Point waypoint_sixth_pair;
  waypoint_sixth_pair.x =
      (buoy_landmarks_14_to_15[0].pose_odom_frame.position.x +
       buoy_landmarks_14_to_15[1].pose_odom_frame.position.x) /
      2;
  waypoint_sixth_pair.y =
      (buoy_landmarks_14_to_15[0].pose_odom_frame.position.y +
       buoy_landmarks_14_to_15[1].pose_odom_frame.position.y) /
      2;
  waypoint_sixth_pair.z = 0.0;
  send_waypoint(waypoint_sixth_pair);
  set_desired_heading(waypoint_fifth_pair, waypoint_sixth_pair);
  reach_waypoint(1.0);

  // Gps end goal
  geometry_msgs::msg::Point gps_end_goal;
  gps_end_goal.x = this->get_parameter("gps_end_x").as_double();
  gps_end_goal.y = this->get_parameter("gps_end_y").as_double();
  gps_end_goal.z = 0.0;
  send_waypoint(gps_end_goal);
  set_desired_heading(waypoint_sixth_pair, gps_end_goal);
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

Eigen::Array<double, 2, 2>
NavigationTaskNode::predict_second_buoy_pair(
    const geometry_msgs::msg::Point &buoy_0,
    const geometry_msgs::msg::Point &buoy_1) {
  Eigen::Vector2d direction_vector;
  direction_vector << previous_waypoint_odom_frame_.x -
                          odom_start_point_.x,
      previous_waypoint_odom_frame_.y -
          odom_start_point_.y;
  direction_vector.normalize();

  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) = buoy_0.x + direction_vector(0) * 5;
  predicted_positions(1, 0) = buoy_0.y + direction_vector(1) * 5;
  predicted_positions(0, 1) = buoy_1.x + direction_vector(0) * 5;
  predicted_positions(1, 1) = buoy_1.y + direction_vector(1) * 5;

  return predicted_positions;
}

Eigen::Array<double, 2, 1>
NavigationTaskNode::predict_west_buoy(const geometry_msgs::msg::Point &buoy_0,
                                      const geometry_msgs::msg::Point &buoy_1,
                                      const geometry_msgs::msg::Point &buoy_2,
                                      const geometry_msgs::msg::Point &buoy_3) {
  Eigen::Vector2d direction_vector_right;
  direction_vector_right << (buoy_3.x + buoy_2.x) / 2 -
                                (buoy_1.x + buoy_0.x) / 2,
      (buoy_3.y + buoy_2.y) / 2 - (buoy_1.y + buoy_0.y) / 2;
  direction_vector_right.normalize();

  Eigen::Vector2d direction_vector_up;
  direction_vector_up << (buoy_0.x + buoy_2.x) / 2 - (buoy_1.x + buoy_3.x) / 2,
      (buoy_0.y + buoy_2.y) / 2 - (buoy_1.y + buoy_3.y) / 2;
  direction_vector_up.normalize();

  Eigen::Array<double, 2, 1> predicted_positions;
  predicted_positions(0, 0) = buoy_3.x +
                              direction_vector_right(0) * 12.5 * 1 / 2 +
                              direction_vector_up(0) * 5 * 2 / 5;
  predicted_positions(1, 0) = buoy_3.y +
                              direction_vector_right(1) * 12.5 * 1 / 2 +
                              direction_vector_up(1) * 5 * 2 / 5;

  return predicted_positions;
}

Eigen::Array<double, 2, 2> NavigationTaskNode::predict_third_buoy_pair(
    const geometry_msgs::msg::Point &buoy_0,
    const geometry_msgs::msg::Point &buoy_1,
    const geometry_msgs::msg::Point &buoy_2,
    const geometry_msgs::msg::Point &buoy_3) {
  Eigen::Vector2d direction_vector_right;
  direction_vector_right << (buoy_3.x + buoy_2.x) / 2 -
                                (buoy_1.x + buoy_0.x) / 2,
      (buoy_3.y + buoy_2.y) / 2 - (buoy_1.y + buoy_0.y) / 2;
  direction_vector_right.normalize();

  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) = buoy_2.x + direction_vector_right(0) * 12.5;
  predicted_positions(1, 0) = buoy_2.y + direction_vector_right(1) * 12.5;
  predicted_positions(0, 1) = buoy_3.x + direction_vector_right(0) * 12.5;
  predicted_positions(1, 1) = buoy_3.y + direction_vector_right(1) * 12.5;

  return predicted_positions;
}

Eigen::Array<double, 2, 1> NavigationTaskNode::predict_north_buoy(
    const geometry_msgs::msg::Point &buoy_5,
    const geometry_msgs::msg::Point &buoy_6,
    const Eigen::Vector2d &direction_vector_second_to_third_pair) {
  Eigen::Vector2d direction_5_to_6;
  direction_5_to_6 << buoy_6.x - buoy_5.x, buoy_6.y - buoy_5.y;
  direction_5_to_6.normalize();

  Eigen::Array<double, 2, 1> predicted_positions;
  predicted_positions(0, 0) = buoy_6.x + direction_5_to_6(0) * 12.5 * 0.5 +
                              direction_vector_second_to_third_pair(0) * 2;
  predicted_positions(1, 0) = buoy_6.y + direction_5_to_6(1) * 12.5 * 0.5 +
                              direction_vector_second_to_third_pair(1) * 2;

  return predicted_positions;
}

Eigen::Array<double, 2, 1> NavigationTaskNode::predict_south_buoy(
    const geometry_msgs::msg::Point &buoy_5,
    const geometry_msgs::msg::Point &buoy_6,
    const Eigen::Vector2d &direction_vector_second_to_third_pair) {
  Eigen::Vector2d direction_5_to_6;
  direction_5_to_6 << buoy_6.x - buoy_5.x, buoy_6.y - buoy_5.y;
  direction_5_to_6.normalize();

  Eigen::Array<double, 2, 1> predicted_positions;
  predicted_positions(0, 0) = buoy_6.x + direction_5_to_6(0) * 12.5 +
                              direction_vector_second_to_third_pair(0) * 8;
  predicted_positions(1, 0) = buoy_6.y + direction_5_to_6(1) * 12.5 +
                              direction_vector_second_to_third_pair(1) * 8;

  return predicted_positions;
}

Eigen::Array<double, 2, 2> NavigationTaskNode::predict_fourth_buoy_pair(
    const geometry_msgs::msg::Point &buoy_5,
    const geometry_msgs::msg::Point &buoy_6) {
  Eigen::Vector2d direction_5_to_6;
  direction_5_to_6 << buoy_6.x - buoy_5.x, buoy_6.y - buoy_5.y;
  direction_5_to_6.normalize();

  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) = buoy_6.x + direction_5_to_6(0) * 12.5;
  predicted_positions(1, 0) = buoy_6.y + direction_5_to_6(1) * 12.5;
  predicted_positions(0, 1) =
      buoy_6.x + direction_5_to_6(0) * 12.5 + direction_5_to_6(0) * 5;
  predicted_positions(1, 1) =
      buoy_6.y + direction_5_to_6(1) * 12.5 + direction_5_to_6(1) * 5;

  return predicted_positions;
}

Eigen::Array<double, 2, 1> NavigationTaskNode::predict_east_buoy(
    const geometry_msgs::msg::Point &buoy_9,
    const geometry_msgs::msg::Point &buoy_10,
    const Eigen::Vector2d &direction_vector_second_to_third_pair) {
  Eigen::Vector2d direction_9_to_10;
  direction_9_to_10 << buoy_10.x - buoy_9.x, buoy_10.y - buoy_9.y;
  direction_9_to_10.normalize();

  Eigen::Array<double, 2, 1> predicted_positions;
  predicted_positions(0, 0) = buoy_9.x + direction_9_to_10(0) * 2.5 -
                              direction_vector_second_to_third_pair(0) * 7.5;
  predicted_positions(1, 0) = buoy_9.y + direction_9_to_10(1) * 2.5 -
                              direction_vector_second_to_third_pair(1) * 7.5;

  return predicted_positions;
}

Eigen::Array<double, 2, 2> NavigationTaskNode::predict_fifth_buoy_pair(
    const geometry_msgs::msg::Point &buoy_9,
    const geometry_msgs::msg::Point &buoy_10,
    const Eigen::Vector2d &direction_vector_second_to_third_pair) {

  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) =
      buoy_9.x - direction_vector_second_to_third_pair(0) * 12.5;
  predicted_positions(1, 0) =
      buoy_9.y - direction_vector_second_to_third_pair(1) * 12.5;
  predicted_positions(0, 1) =
      buoy_10.x - direction_vector_second_to_third_pair(0) * 12.5;
  predicted_positions(1, 1) =
      buoy_10.y - direction_vector_second_to_third_pair(1) * 12.5;

  return predicted_positions;
}

Eigen::Array<double, 2, 2> NavigationTaskNode::predict_sixth_buoy_pair(
    const geometry_msgs::msg::Point &buoy_9,
    const geometry_msgs::msg::Point &buoy_10,
    const geometry_msgs::msg::Point &buoy_12,
    const geometry_msgs::msg::Point &buoy_13) {
  Eigen::Vector2d direction_fourth_to_fifth_pair;
  direction_fourth_to_fifth_pair
      << (buoy_13.x + buoy_12.x) / 2 - (buoy_10.x + buoy_9.x) / 2,
      (buoy_13.y + buoy_12.y) / 2 - (buoy_10.y + buoy_9.y) / 2;
  direction_fourth_to_fifth_pair.normalize();

  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) = buoy_12.x + direction_fourth_to_fifth_pair(0) * 5;
  predicted_positions(1, 0) = buoy_12.y + direction_fourth_to_fifth_pair(1) * 5;
  predicted_positions(0, 1) = buoy_13.x + direction_fourth_to_fifth_pair(0) * 5;
  predicted_positions(1, 1) = buoy_13.y + direction_fourth_to_fifth_pair(1) * 5;

  return predicted_positions;
}

} // namespace navigation_task