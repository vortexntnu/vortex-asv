#include <maneuvering_task/maneuvering_task_ros.hpp>

namespace maneuvering_task {

ManeuveringTaskNode::ManeuveringTaskNode(const rclcpp::NodeOptions &options)
    : NjordTaskBaseNode("maneuvering_task_node", options) {

  declare_parameter<double>("distance_to_first_buoy_pair", 2.0);
  declare_parameter<double>("initial_offset", 1.0);
  declare_parameter<int>("nr_of_pair_in_formation", 11);

  std::thread(&ManeuveringTaskNode::main_task, this).detach();
}

void ManeuveringTaskNode::main_task() {
  navigation_ready();
  RCLCPP_INFO(this->get_logger(), "Maneuvering task started");
  odom_start_point_ = get_odom()->pose.pose.position;
  // First buoy pair
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

  // First first buoy pair is far away, should be closer before trying to
  // measure it.
  double gps_end_x = this->get_parameter("gps_end_x").as_double();
  double gps_end_y = this->get_parameter("gps_end_y").as_double();
  Eigen::Vector2d direction_vector_to_end;
  direction_vector_to_end << gps_end_x - odom_start_point_.x,
      gps_end_y - odom_start_point_.y;
  direction_vector_to_end.normalize();

  double distance =
      this->get_parameter("distance_to_first_buoy_pair").as_double();
  if (distance > 5.0) {
    auto odom = get_odom();
    geometry_msgs::msg::Point waypoint_toward_start;
    waypoint_toward_start.x =
        odom->pose.pose.position.x + direction_vector_to_end(0) * distance - 3;
    waypoint_toward_start.y =
        odom->pose.pose.position.y + direction_vector_to_end(1) * distance - 3;
    waypoint_toward_start.z = 0.0;
    send_waypoint(waypoint_toward_start);
    reach_waypoint(2.0);
  }

  std::vector<LandmarkPoseID> measured_first_buoy_pair =
      get_buoy_landmarks(predicted_first_buoy_pair);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = measured_first_buoy_pair[0].pose_odom_frame.position.x;
  buoy_red_0.y = measured_first_buoy_pair[0].pose_odom_frame.position.y;
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = measured_first_buoy_pair[1].pose_odom_frame.position.x;
  buoy_green_1.y = measured_first_buoy_pair[1].pose_odom_frame.position.y;
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
      (measured_first_buoy_pair[0].pose_odom_frame.position.x +
       measured_first_buoy_pair[1].pose_odom_frame.position.x) /
      2;
  waypoint_first_pair.y =
      (measured_first_buoy_pair[0].pose_odom_frame.position.y +
       measured_first_buoy_pair[1].pose_odom_frame.position.y) /
      2;
  waypoint_first_pair.z = 0.0;
  send_waypoint(waypoint_first_pair);
  set_desired_heading(odom_start_point_, waypoint_first_pair);
  reach_waypoint(0.5);

  // Second buoy pair is FAR away, should be closer before trying to measure it.
  geometry_msgs::msg::Point waypoint_approach_formation;
  waypoint_approach_formation.x =
      waypoint_first_pair.x + direction_vector_to_end(0) * 15;
  waypoint_approach_formation.y =
      waypoint_first_pair.y + direction_vector_to_end(1) * 15;
  waypoint_approach_formation.z = 0.0;
  send_waypoint(waypoint_approach_formation);
  set_desired_heading(waypoint_first_pair, waypoint_approach_formation);
  reach_waypoint(1.0);

  // Second buoy pair
  Eigen::Array<double, 2, 2> predicted_second_buoy_pair =
      predict_second_buoy_pair(
          measured_first_buoy_pair[0].pose_odom_frame.position,
          measured_first_buoy_pair[1].pose_odom_frame.position);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = predicted_second_buoy_pair(0, 0);
  buoy_red_0.y = predicted_second_buoy_pair(1, 0);
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = predicted_second_buoy_pair(0, 1);
  buoy_green_1.y = predicted_second_buoy_pair(1, 1);
  buoy_green_1.z = 0.0;
  buoy_green_1.r = 0;
  buoy_green_1.g = 255;
  buoy_green_1.b = 0;
  buoy_vis.push_back(buoy_green_1);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  std::vector<LandmarkPoseID> measured_buoy_2_to_3 =
      get_buoy_landmarks(predicted_second_buoy_pair);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = measured_buoy_2_to_3[0].pose_odom_frame.position.x;
  buoy_red_0.y = measured_buoy_2_to_3[0].pose_odom_frame.position.y;
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = measured_buoy_2_to_3[1].pose_odom_frame.position.x;
  buoy_green_1.y = measured_buoy_2_to_3[1].pose_odom_frame.position.y;
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
      (measured_buoy_2_to_3[0].pose_odom_frame.position.x +
       measured_buoy_2_to_3[1].pose_odom_frame.position.x) /
      2;
  waypoint_second_pair.y =
      (measured_buoy_2_to_3[0].pose_odom_frame.position.y +
       measured_buoy_2_to_3[1].pose_odom_frame.position.y) /
      2;
  waypoint_second_pair.z = 0.0;
  send_waypoint(waypoint_second_pair);
  set_desired_heading(waypoint_approach_formation, waypoint_second_pair);
  reach_waypoint(0.5);

  // Setup variable to navigate formation
  Eigen::Vector2d direction_vector_forwards;
  direction_vector_forwards << (waypoint_second_pair.x - waypoint_first_pair.x),
      (waypoint_second_pair.y - waypoint_first_pair.y);
  direction_vector_forwards.normalize();

  Eigen::Vector2d direction_vector_downwards;
  direction_vector_downwards
      << measured_buoy_2_to_3[1].pose_odom_frame.position.x -
             measured_buoy_2_to_3[0].pose_odom_frame.position.x,
      measured_buoy_2_to_3[1].pose_odom_frame.position.y -
          measured_buoy_2_to_3[0].pose_odom_frame.position.y;
  direction_vector_downwards.normalize();

  Eigen::Vector2d vector_to_next_pair;
  vector_to_next_pair
      << direction_vector_forwards(0) * 5 +
             direction_vector_downwards(0) *
                 this->get_parameter("initial_offset").as_double(),
      direction_vector_forwards(1) * 5 +
          direction_vector_downwards(1) *
              this->get_parameter("initial_offset").as_double();

  geometry_msgs::msg::Point red_buoy =
      measured_buoy_2_to_3[0].pose_odom_frame.position;
  geometry_msgs::msg::Point green_buoy =
      measured_buoy_2_to_3[1].pose_odom_frame.position;
  geometry_msgs::msg::Point waypoint_prev_pair = waypoint_second_pair;
  // ASV is at first buoy pair in formation.
  // Run loop for the rest of the formation excluding the first pair.
  int num_iterations =
      this->get_parameter("nr_of_pair_in_formation").as_int() - 1;

  for (int _ = 0; _ < num_iterations; ++_) {
    Eigen::Array<double, 2, 2> predicted_next_pair =
        predict_next_pair_in_formation(red_buoy, green_buoy,
                                       vector_to_next_pair);

    buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
    buoy_vis_msg = sensor_msgs::msg::PointCloud2();
    buoy_red_0 = pcl::PointXYZRGB();
    buoy_green_1 = pcl::PointXYZRGB();
    buoy_red_0.x = predicted_next_pair(0, 0);
    buoy_red_0.y = predicted_next_pair(1, 0);
    buoy_red_0.z = 0.0;
    buoy_red_0.r = 255;
    buoy_red_0.g = 0;
    buoy_red_0.b = 0;
    buoy_vis.push_back(buoy_red_0);
    buoy_green_1.x = predicted_next_pair(0, 1);
    buoy_green_1.y = predicted_next_pair(1, 1);
    buoy_green_1.z = 0.0;
    buoy_green_1.r = 0;
    buoy_green_1.g = 255;
    buoy_green_1.b = 0;
    buoy_vis.push_back(buoy_green_1);
    pcl::toROSMsg(buoy_vis, buoy_vis_msg);
    buoy_vis_msg.header.frame_id = "odom";
    buoy_vis_msg.header.stamp = this->now();
    buoy_visualization_pub_->publish(buoy_vis_msg);

    std::vector<LandmarkPoseID> measured_next_pair =
        get_buoy_landmarks(predicted_next_pair);

    buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
    buoy_vis_msg = sensor_msgs::msg::PointCloud2();
    buoy_red_0 = pcl::PointXYZRGB();
    buoy_green_1 = pcl::PointXYZRGB();
    buoy_red_0.x = measured_next_pair[0].pose_odom_frame.position.x;
    buoy_red_0.y = measured_next_pair[0].pose_odom_frame.position.y;
    buoy_red_0.z = 0.0;
    buoy_red_0.r = 255;
    buoy_red_0.g = 0;
    buoy_red_0.b = 0;
    buoy_vis.push_back(buoy_red_0);
    buoy_green_1.x = measured_next_pair[1].pose_odom_frame.position.x;
    buoy_green_1.y = measured_next_pair[1].pose_odom_frame.position.y;
    buoy_green_1.z = 0.0;
    buoy_green_1.r = 0;
    buoy_green_1.g = 255;
    buoy_green_1.b = 0;
    buoy_vis.push_back(buoy_green_1);
    pcl::toROSMsg(buoy_vis, buoy_vis_msg);
    buoy_vis_msg.header.frame_id = "odom";
    buoy_vis_msg.header.stamp = this->now();
    buoy_visualization_pub_->publish(buoy_vis_msg);

    geometry_msgs::msg::Point waypoint_next_pair;
    waypoint_next_pair.x = (measured_next_pair[0].pose_odom_frame.position.x +
                            measured_next_pair[1].pose_odom_frame.position.x) /
                           2;
    waypoint_next_pair.y = (measured_next_pair[0].pose_odom_frame.position.y +
                            measured_next_pair[1].pose_odom_frame.position.y) /
                           2;
    waypoint_next_pair.z = 0.0;
    send_waypoint(waypoint_next_pair);
    set_desired_heading(waypoint_prev_pair, waypoint_next_pair);
    reach_waypoint(1.0);
    red_buoy = measured_next_pair[0].pose_odom_frame.position;
    green_buoy = measured_next_pair[1].pose_odom_frame.position;
    vector_to_next_pair << waypoint_next_pair.x - waypoint_prev_pair.x,
        waypoint_next_pair.y - waypoint_prev_pair.y;
    waypoint_prev_pair = waypoint_next_pair;
  }

  // ASV is now at the last pair of the buoy formation
  // Should move close to end before we try to measure the last pair
  auto odom = get_odom();
  double distance_to_end =
      std::sqrt(std::pow(odom->pose.pose.position.x - gps_end_x, 2) +
                std::pow(odom->pose.pose.position.y - gps_end_y, 2));
  if (distance_to_end > 7.0) {
    geometry_msgs::msg::Point waypoint_toward_end;
    waypoint_toward_end.x =
        odom->pose.pose.position.x +
        direction_vector_to_end(0) * (distance_to_end - 4.0);
    waypoint_toward_end.y =
        odom->pose.pose.position.y +
        direction_vector_to_end(1) * (distance_to_end - 4.0);
    waypoint_toward_end.z = 0.0;
    send_waypoint(waypoint_toward_end);
    set_desired_heading(waypoint_prev_pair, waypoint_toward_end);
    reach_waypoint(2.0);
  }
  Eigen::Array<double, 2, 2> predicted_last_buoy_pair;
  predicted_last_buoy_pair(0, 0) =
      gps_end_x - direction_vector_downwards(0) * 2.5;
  predicted_last_buoy_pair(1, 0) =
      gps_end_y - direction_vector_downwards(1) * 2.5;
  predicted_last_buoy_pair(0, 1) =
      gps_end_x + direction_vector_downwards(0) * 2.5;
  predicted_last_buoy_pair(1, 1) =
      gps_end_y + direction_vector_downwards(1) * 2.5;

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = predicted_last_buoy_pair(0, 0);
  buoy_red_0.y = predicted_last_buoy_pair(1, 0);
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = predicted_last_buoy_pair(0, 1);
  buoy_green_1.y = predicted_last_buoy_pair(1, 1);
  buoy_green_1.z = 0.0;
  buoy_green_1.r = 0;
  buoy_green_1.g = 255;
  buoy_green_1.b = 0;
  buoy_vis.push_back(buoy_green_1);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  std::vector<LandmarkPoseID> measured_last_buoy_pair =
      get_buoy_landmarks(predicted_last_buoy_pair);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = measured_last_buoy_pair[0].pose_odom_frame.position.x;
  buoy_red_0.y = measured_last_buoy_pair[0].pose_odom_frame.position.y;
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = measured_last_buoy_pair[1].pose_odom_frame.position.x;
  buoy_green_1.y = measured_last_buoy_pair[1].pose_odom_frame.position.y;
  buoy_green_1.z = 0.0;
  buoy_green_1.r = 0;
  buoy_green_1.g = 255;
  buoy_green_1.b = 0;
  buoy_vis.push_back(buoy_green_1);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  geometry_msgs::msg::Point waypoint_last_pair;
  waypoint_last_pair.x =
      (measured_last_buoy_pair[0].pose_odom_frame.position.x +
       measured_last_buoy_pair[1].pose_odom_frame.position.x) /
      2;
  waypoint_last_pair.y =
      (measured_last_buoy_pair[0].pose_odom_frame.position.y +
       measured_last_buoy_pair[1].pose_odom_frame.position.y) /
      2;
  waypoint_last_pair.z = 0.0;
  send_waypoint(waypoint_last_pair);
  set_desired_heading(waypoint_prev_pair, waypoint_last_pair);
  reach_waypoint(1.0);
  // Also send waypoint 2m through the last pair
  geometry_msgs::msg::Point waypoint_through_last_pair;
  waypoint_through_last_pair.x =
      waypoint_last_pair.x + direction_vector_to_end(0) * 2;
  waypoint_through_last_pair.y =
      waypoint_last_pair.y + direction_vector_to_end(1) * 2;
  waypoint_through_last_pair.z = 0.0;
  send_waypoint(waypoint_through_last_pair);
  set_desired_heading(waypoint_last_pair, waypoint_through_last_pair);
  // Task is now finished
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

Eigen::Array<double, 2, 2> ManeuveringTaskNode::predict_second_buoy_pair(
    const geometry_msgs::msg::Point &buoy_0,
    const geometry_msgs::msg::Point &buoy_1) {
  Eigen::Vector2d direction_vector;
  direction_vector << previous_waypoint_odom_frame_.x - odom_start_point_.x,
      previous_waypoint_odom_frame_.y - odom_start_point_.y;
  direction_vector.normalize();

  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) = buoy_0.x + direction_vector(0) * 20;
  predicted_positions(1, 0) = buoy_0.y + direction_vector(1) * 20;
  predicted_positions(0, 1) = buoy_1.x + direction_vector(0) * 20;
  predicted_positions(1, 1) = buoy_1.y + direction_vector(1) * 20;

  return predicted_positions;
}

Eigen::Array<double, 2, 2> ManeuveringTaskNode::predict_next_pair_in_formation(
    const geometry_msgs::msg::Point &buoy_red,
    const geometry_msgs::msg::Point &buoy_green,
    Eigen::Vector2d vector_to_next_pair) {
  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) = buoy_red.x + vector_to_next_pair(0);
  predicted_positions(1, 0) = buoy_red.y + vector_to_next_pair(1);
  predicted_positions(0, 1) = buoy_green.x + vector_to_next_pair(0);
  predicted_positions(1, 1) = buoy_green.y + vector_to_next_pair(1);

  return predicted_positions;
}

} // namespace maneuvering_task