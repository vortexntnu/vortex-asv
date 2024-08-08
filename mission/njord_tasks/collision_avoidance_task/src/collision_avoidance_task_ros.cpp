#include <collision_avoidance_task/collision_avoidance_task_ros.hpp>

namespace collision_avoidance_task {

CollisionAvoidanceTaskNode::CollisionAvoidanceTaskNode(
    const rclcpp::NodeOptions &options)
    : NjordTaskBaseNode("collision_avoidance_task_node", options) {

  declare_parameter<double>("distance_to_first_buoy_pair", 2.0);
  declare_parameter<double>("distance_between_buoy_pairs", 5.0);
  declare_parameter<int>("vessel_assignment_confidence", 5);
  
  std::thread(&CollisionAvoidanceTaskNode::main_task, this).detach();
}

void CollisionAvoidanceTaskNode::main_task() {
  navigation_ready();
  RCLCPP_INFO(this->get_logger(), "Collision avoidance task started");
  odom_start_point_ = get_odom()->pose.pose.position;
  Eigen::Array<double, 2, 2> predicted_first_buoy_pair = predict_first_buoy_pair();

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
        odom->pose.pose.position.x + direction_vector_to_end(0) * (distance - 3);
    waypoint_toward_start.y =
        odom->pose.pose.position.y + direction_vector_to_end(1) * (distance - 3);
    waypoint_toward_start.z = 0.0;
    send_waypoint(waypoint_toward_start);
    set_desired_heading(odom_start_point_, waypoint_toward_start);
    reach_waypoint(3.0);
  }

  std::vector<LandmarkPoseID> measured_first_buoy_pair =
      get_buoy_landmarks(predicted_first_buoy_pair);
  if (measured_first_buoy_pair.size() != 2) {
    RCLCPP_ERROR(this->get_logger(), "Could not find two buoys");
  }

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
  reach_waypoint(3.0);

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

  std::vector<LandmarkPoseID> measured_second_buoy_pair =
      get_buoy_landmarks(predicted_second_buoy_pair);
  if (measured_second_buoy_pair.size() != 2) {
    RCLCPP_ERROR(this->get_logger(), "Could not find two buoys");
  }

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>(); 
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = measured_second_buoy_pair[0].pose_odom_frame.position.x;
  buoy_red_0.y = measured_second_buoy_pair[0].pose_odom_frame.position.y;
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = measured_second_buoy_pair[1].pose_odom_frame.position.x;
  buoy_green_1.y = measured_second_buoy_pair[1].pose_odom_frame.position.y;
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
      (measured_second_buoy_pair[0].pose_odom_frame.position.x +
       measured_second_buoy_pair[1].pose_odom_frame.position.x) /
      2;
  waypoint_second_pair.y =
      (measured_second_buoy_pair[0].pose_odom_frame.position.y +
       measured_second_buoy_pair[1].pose_odom_frame.position.y) /
      2;
  waypoint_second_pair.z = 0.0;
  send_waypoint(waypoint_second_pair);
  set_desired_heading(odom_start_point_, waypoint_second_pair);
  reach_waypoint(3.0);
  
  Eigen::Vector2d direction_vector_forwards;
  Eigen::Vector2d direction_vector_downwards;
  direction_vector_forwards << waypoint_second_pair.x - waypoint_first_pair.x,
      waypoint_second_pair.y - waypoint_first_pair.y;
  direction_vector_forwards.normalize();
  direction_vector_downwards << (measured_first_buoy_pair[1].pose_odom_frame.position.x + measured_second_buoy_pair[1].pose_odom_frame.position.x)/2
  - (measured_first_buoy_pair[0].pose_odom_frame.position.x + measured_second_buoy_pair[0].pose_odom_frame.position.x)/2,
      (measured_first_buoy_pair[1].pose_odom_frame.position.y + measured_second_buoy_pair[1].pose_odom_frame.position.y)/2
  - (measured_first_buoy_pair[0].pose_odom_frame.position.y + measured_second_buoy_pair[0].pose_odom_frame.position.y)/2;
  direction_vector_downwards.normalize();

  geometry_msgs::msg::Point waypoint_midpoint;
  waypoint_midpoint.x = waypoint_second_pair.x + direction_vector_forwards(0) * 10;
  waypoint_midpoint.y = waypoint_second_pair.y + direction_vector_forwards(1) * 10;
  waypoint_midpoint.z = 0.0;
  send_waypoint(waypoint_midpoint);
  set_desired_heading(waypoint_second_pair, waypoint_midpoint);

  std::thread(std::bind(&CollisionAvoidanceTaskNode::track_vessel, this, direction_vector_forwards, direction_vector_downwards, waypoint_second_pair)).detach();
  RCLCPP_INFO(this->get_logger(), "Tracking vessel");
  reach_waypoint(9.0);
  // Run until the angle between the twist vectors are between 60 and 120 degrees
  vessel_collision_heading();
  auto vessel_odom = get_vessel_odom();
  auto freya_odom = get_odom();

  Eigen::Vector2d direction_vector_freya_to_vessel;
  direction_vector_freya_to_vessel << vessel_odom.pose.pose.position.x - freya_odom->pose.pose.position.x,
      vessel_odom.pose.pose.position.y - freya_odom->pose.pose.position.y;
  // Project the vector do find "height" difference of vessels
  double downwards_diff = direction_vector_freya_to_vessel.dot(direction_vector_downwards);
  geometry_msgs::msg::Point first_avoidance_waypoint;
  if (std::abs(downwards_diff) > 5) {
    first_avoidance_waypoint.x = vessel_odom.pose.pose.position.x - direction_vector_forwards(0) * 3;
    first_avoidance_waypoint.y = vessel_odom.pose.pose.position.y - direction_vector_forwards(1) * 3;
    first_avoidance_waypoint.z = 0.0;
    send_waypoint(first_avoidance_waypoint);
    set_desired_heading(freya_odom->pose.pose.position, first_avoidance_waypoint);
  } else {
    first_avoidance_waypoint.x = vessel_odom.pose.pose.position.x - direction_vector_forwards(0) * 3 + direction_vector_downwards(0) * 2;
    first_avoidance_waypoint.y = vessel_odom.pose.pose.position.y - direction_vector_forwards(1) * 3 + direction_vector_downwards(1) * 2;
    first_avoidance_waypoint.z = 0.0;
    send_waypoint(first_avoidance_waypoint);
    set_desired_heading(freya_odom->pose.pose.position, first_avoidance_waypoint);
  }

  while(!has_vessel_passed_freya(direction_vector_downwards)){
    RCLCPP_INFO(this->get_logger(), "Vessel has not passed Freya yet");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  freya_odom = get_odom();
  geometry_msgs::msg::Point second_avoidance_waypoint;
  second_avoidance_waypoint.x = freya_odom->pose.pose.position.x + direction_vector_forwards(0) * 5;
  second_avoidance_waypoint.y = freya_odom->pose.pose.position.y + direction_vector_forwards(1) * 5;
  second_avoidance_waypoint.z = 0.0;

  // reach_waypoint(1.0);

  geometry_msgs::msg::Point back_on_track_waypoint;
  back_on_track_waypoint.x = waypoint_second_pair.x + direction_vector_forwards(0) * 15 + direction_vector_downwards(0) * 0;
  back_on_track_waypoint.y = waypoint_second_pair.y + direction_vector_forwards(1) * 15 + direction_vector_downwards(1) * 0;
  back_on_track_waypoint.z = 0.0;

   auto request = std::make_shared<vortex_msgs::srv::Waypoint::Request>();
  request->waypoint.push_back(second_avoidance_waypoint);
  request->waypoint.push_back(back_on_track_waypoint);
  auto result_future = waypoint_client_->async_send_request(request);
 
  // Check if the service was successful
  auto odom = get_odom();
  double dx = back_on_track_waypoint.x - odom->pose.pose.position.x;
  double dy = back_on_track_waypoint.y - odom->pose.pose.position.y;
  double desired_heading = std::atan2(dy, dx);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, desired_heading);

  geometry_msgs::msg::PoseStamped waypoint_vis;
  waypoint_vis.header.frame_id = "odom";
  waypoint_vis.pose.position.x = back_on_track_waypoint.x;
  waypoint_vis.pose.position.y = back_on_track_waypoint.y;
  waypoint_vis.pose.orientation = tf2::toMsg(q);
  waypoint_visualization_pub_->publish(waypoint_vis);
  auto status = result_future.wait_for(std::chrono::seconds(5));
  while (status == std::future_status::timeout) {
    RCLCPP_INFO(this->get_logger(), "Waypoint service timed out");
    status = result_future.wait_for(std::chrono::seconds(5));
  }
  if (!result_future.get()->success) {
    RCLCPP_INFO(this->get_logger(), "Waypoint service failed");
  }

  previous_waypoint_odom_frame_ = back_on_track_waypoint;

  set_desired_heading(second_avoidance_waypoint, back_on_track_waypoint);
  reach_waypoint(3.0);

  Eigen::Array<double, 2, 2> predicted_third_buoy_pair =
      predict_third_buoy_pair(
          measured_first_buoy_pair[0].pose_odom_frame.position,
          measured_first_buoy_pair[1].pose_odom_frame.position);
  
  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = predicted_third_buoy_pair(0, 0);
  buoy_red_0.y = predicted_third_buoy_pair(1, 0);
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = predicted_third_buoy_pair(0, 1);
  buoy_green_1.y = predicted_third_buoy_pair(1, 1);
  buoy_green_1.z = 0.0;
  buoy_green_1.r = 0;
  buoy_green_1.g = 255;
  buoy_green_1.b = 0;
  buoy_vis.push_back(buoy_green_1);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  std::vector<LandmarkPoseID> measured_third_buoy_pair =
      get_buoy_landmarks(predicted_third_buoy_pair);
  if (measured_third_buoy_pair.size() != 2) {
    RCLCPP_ERROR(this->get_logger(), "Could not find two buoys");
  }

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = measured_third_buoy_pair[0].pose_odom_frame.position.x;
  buoy_red_0.y = measured_third_buoy_pair[0].pose_odom_frame.position.y;
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = measured_third_buoy_pair[1].pose_odom_frame.position.x;
  buoy_green_1.y = measured_third_buoy_pair[1].pose_odom_frame.position.y;
  buoy_green_1.z = 0.0;
  buoy_green_1.r = 0;
  buoy_green_1.g = 255;
  buoy_green_1.b = 0;
  buoy_vis.push_back(buoy_green_1);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  geometry_msgs::msg::Point waypoint_third_pair;
  waypoint_third_pair.x =
      (measured_third_buoy_pair[0].pose_odom_frame.position.x +
       measured_third_buoy_pair[1].pose_odom_frame.position.x) /
      2;
  waypoint_third_pair.y =
      (measured_third_buoy_pair[0].pose_odom_frame.position.y +
       measured_third_buoy_pair[1].pose_odom_frame.position.y) /
      2; 
  waypoint_third_pair.z = 0.0;
  send_waypoint(waypoint_third_pair);
  set_desired_heading(back_on_track_waypoint, waypoint_third_pair);
  reach_waypoint(3.0);

  Eigen::Array<double, 2, 2> predicted_fourth_buoy_pair =
      predict_fourth_buoy_pair(
          measured_third_buoy_pair[0].pose_odom_frame.position,
          measured_third_buoy_pair[1].pose_odom_frame.position);

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = predicted_fourth_buoy_pair(0, 0);
  buoy_red_0.y = predicted_fourth_buoy_pair(1, 0);
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = predicted_fourth_buoy_pair(0, 1);
  buoy_green_1.y = predicted_fourth_buoy_pair(1, 1);
  buoy_green_1.z = 0.0;
  buoy_green_1.r = 0;
  buoy_green_1.g = 255;
  buoy_green_1.b = 0;
  buoy_vis.push_back(buoy_green_1);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  std::vector<LandmarkPoseID> measured_fourth_buoy_pair =
      get_buoy_landmarks(predicted_fourth_buoy_pair);
  if (measured_fourth_buoy_pair.size() != 2) {
    RCLCPP_ERROR(this->get_logger(), "Could not find two buoys");
  }

  buoy_vis = pcl::PointCloud<pcl::PointXYZRGB>();
  buoy_vis_msg = sensor_msgs::msg::PointCloud2();
  buoy_red_0 = pcl::PointXYZRGB();
  buoy_green_1 = pcl::PointXYZRGB();
  buoy_red_0.x = measured_fourth_buoy_pair[0].pose_odom_frame.position.x;
  buoy_red_0.y = measured_fourth_buoy_pair[0].pose_odom_frame.position.y;
  buoy_red_0.z = 0.0;
  buoy_red_0.r = 255;
  buoy_red_0.g = 0;
  buoy_red_0.b = 0;
  buoy_vis.push_back(buoy_red_0);
  buoy_green_1.x = measured_fourth_buoy_pair[1].pose_odom_frame.position.x;
  buoy_green_1.y = measured_fourth_buoy_pair[1].pose_odom_frame.position.y;
  buoy_green_1.z = 0.0;
  buoy_green_1.r = 0;
  buoy_green_1.g = 255;
  buoy_green_1.b = 0;
  buoy_vis.push_back(buoy_green_1);
  pcl::toROSMsg(buoy_vis, buoy_vis_msg);
  buoy_vis_msg.header.frame_id = "odom";
  buoy_vis_msg.header.stamp = this->now();
  buoy_visualization_pub_->publish(buoy_vis_msg);

  geometry_msgs::msg::Point waypoint_fourth_pair;
  waypoint_fourth_pair.x =
      (measured_fourth_buoy_pair[0].pose_odom_frame.position.x +
       measured_fourth_buoy_pair[1].pose_odom_frame.position.x) /
      2;
  waypoint_fourth_pair.y =
      (measured_fourth_buoy_pair[0].pose_odom_frame.position.y +
       measured_fourth_buoy_pair[1].pose_odom_frame.position.y) /
      2;
  waypoint_fourth_pair.z = 0.0;
  send_waypoint(waypoint_fourth_pair);
  set_desired_heading(waypoint_third_pair, waypoint_fourth_pair);
  reach_waypoint(3.0);

  geometry_msgs::msg::Point waypoint_end;
  waypoint_end.x = this->get_parameter("gps_end_x").as_double();
  waypoint_end.y = this->get_parameter("gps_end_y").as_double();
  waypoint_end.z = 0.0;
  send_waypoint(waypoint_end);
  set_desired_heading(waypoint_fourth_pair, waypoint_end);

}

Eigen::Array<double, 2, 2> CollisionAvoidanceTaskNode::predict_first_buoy_pair() {
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

Eigen::Array<double, 2, 2> CollisionAvoidanceTaskNode::predict_second_buoy_pair(
    const geometry_msgs::msg::Point &buoy_0,
    const geometry_msgs::msg::Point &buoy_1) {
  Eigen::Vector2d direction_vector;
  direction_vector << previous_waypoint_odom_frame_.x - odom_start_point_.x,
      previous_waypoint_odom_frame_.y - odom_start_point_.y;
  direction_vector.normalize();

  double distance_between_buoy_pairs =
      this->get_parameter("distance_between_buoy_pairs").as_double();

  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) = buoy_0.x + direction_vector(0) * distance_between_buoy_pairs;
  predicted_positions(1, 0) = buoy_0.y + direction_vector(1) * distance_between_buoy_pairs;
  predicted_positions(0, 1) = buoy_1.x + direction_vector(0) * distance_between_buoy_pairs;
  predicted_positions(1, 1) = buoy_1.y + direction_vector(1) * distance_between_buoy_pairs;

  return predicted_positions;
}

void CollisionAvoidanceTaskNode::track_vessel(const Eigen::Vector2d& direction_vector_downwards,
      const Eigen::Vector2d& direction_vector_forwards,
       const geometry_msgs::msg::Point& waypoint_second_buoy_pair) {
  std::shared_ptr<vortex_msgs::msg::LandmarkArray> landmark_msg;
  int32_t prev_vessel_id = -1;
  int vessel_assignment_confidence = this->get_parameter("vessel_assignment_confidence").as_int();
  int current_confidence = 0;
  while (true){
      landmark_msg = get_landmarks_odom_frame();
      LandmarkOdomID vessel = filter_landmarks(*landmark_msg, direction_vector_downwards, direction_vector_forwards, waypoint_second_buoy_pair);

      if (vessel.id == -1){
        current_confidence = 0;
        continue;
      }
      if (current_confidence == 0){
        prev_vessel_id = vessel.id;
        current_confidence++;
        continue;
      }
      if (vessel.id != prev_vessel_id){
        current_confidence = 0;
        continue;
      }
      if (vessel.id == prev_vessel_id){
        if(current_confidence < vessel_assignment_confidence){
          current_confidence++;
          continue;
        }
        current_confidence++;
        std::unique_lock<std::mutex> lock(vessel_odom_mutex_);
        vessel_odom_ = vessel.odom;
        new_vessel_odom_msg_ = true;
        lock.unlock();
        vessel_odom_cv_.notify_one();
        continue;
      }
    }
}

LandmarkOdomID CollisionAvoidanceTaskNode::filter_landmarks(
    const vortex_msgs::msg::LandmarkArray &landmarks,
    const Eigen::Vector2d& direction_vector_downwards,
      const Eigen::Vector2d& direction_vector_forwards,
       const geometry_msgs::msg::Point& waypoint_second_buoy_pair) {
  double max_velocity = std::numeric_limits<double>::min();
  LandmarkOdomID filtered_landmark;
  filtered_landmark.id = -1;
  for (auto landmark : landmarks.landmarks) {
    double velocity = std::hypot(landmark.odom.twist.twist.linear.x,
                                 landmark.odom.twist.twist.linear.y);
    geometry_msgs::msg::Point landmark_pose = landmark.odom.pose.pose.position;
    Eigen::Vector2d vector_waypoint_to_landmark;
    vector_waypoint_to_landmark << landmark_pose.x - waypoint_second_buoy_pair.x,
        landmark_pose.y - waypoint_second_buoy_pair.y;
    double projection_down = vector_waypoint_to_landmark.dot(direction_vector_downwards);
    double projection_forwards = vector_waypoint_to_landmark.dot(direction_vector_forwards);

    if (projection_down < -1 || projection_down > 20){
      continue;
    }
    if (projection_forwards < -1 || projection_forwards > 20){
      continue;
    }
    if (velocity > max_velocity && velocity > 0.1){ 
      max_velocity = velocity;
      filtered_landmark.odom = landmark.odom;
      filtered_landmark.id = landmark.id;
    }
  }
  return filtered_landmark;
}

nav_msgs::msg::Odometry CollisionAvoidanceTaskNode::get_vessel_odom() {
  std::unique_lock<std::mutex> lock(vessel_odom_mutex_);
  vessel_odom_cv_.wait(lock, [this] { return new_vessel_odom_msg_; });
  new_vessel_odom_msg_ = false;
  lock.unlock();
  return vessel_odom_;
}

void CollisionAvoidanceTaskNode::vessel_collision_heading() {
  RCLCPP_INFO(this->get_logger(), "Checking vessel collision heading");
  while (true) {
    auto freya_odom = get_odom();
    bool transform_success = false;
    geometry_msgs::msg::TransformStamped transform;
    while (!transform_success) {
      try {
        transform = tf_buffer_->lookupTransform(
            "odom", "base_link", tf2::TimePointZero);
        transform_success = true;
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      }
    }
    geometry_msgs::msg::Vector3 freya_odom_twist;
    tf2::doTransform(freya_odom->twist.twist.linear, freya_odom_twist, transform);
    auto vessel_odom = get_vessel_odom();
    double collision_angle = calculate_angle(freya_odom_twist, vessel_odom.twist.twist.linear);
    if (collision_angle > 60 || collision_angle < 120) {
        return;
    }
    continue;
  }
  
}

// Function to calculate the angle between two vectors
double CollisionAvoidanceTaskNode::calculate_angle(const geometry_msgs::msg::Vector3& twist1, const geometry_msgs::msg::Vector3& twist2) {
    // Extract linear velocities
    double Ax = twist1.x;
    double Ay = twist1.y;
    double Bx = twist2.x;
    double By = twist2.y;

    // Calculate dot product
    double dot_product = Ax * Bx + Ay * By;

    // Calculate magnitudes
    double magnitude_A = std::sqrt(Ax * Ax + Ay * Ay);
    double magnitude_B = std::sqrt(Bx * Bx + By * By);

    // Calculate angle in radians
    double angle_radians = std::acos(dot_product / (magnitude_A * magnitude_B));

    // Convert to degrees if needed
    double angle_degrees = angle_radians * (180.0 / M_PI);

    return angle_degrees;
}

bool CollisionAvoidanceTaskNode::has_vessel_passed_freya(const Eigen::Vector2d& direction_vector_downwards) {
    // Calculate the relative vector from freya to the vessel
    auto freya_pose = get_odom();
    auto vessel_pose = get_vessel_odom();
    Eigen::Vector2d direction_vector_freya_to_vessel;
    direction_vector_freya_to_vessel << vessel_pose.pose.pose.position.x - freya_pose->pose.pose.position.x,
        vessel_pose.pose.pose.position.y - freya_pose->pose.pose.position.y;

    // Project the relative vector onto the direction_vector_downwards
    double projection = direction_vector_freya_to_vessel.dot(direction_vector_downwards);

    // Check the sign of the projection
    if (projection < 0) {
        // The vessel has passed freya
        return true;
    } else {
        // The vessel has not passed freya yet
        return false;
    }
}

Eigen::Array<double, 2, 2> CollisionAvoidanceTaskNode::predict_third_buoy_pair(
    const geometry_msgs::msg::Point &buoy_0,
    const geometry_msgs::msg::Point &buoy_1) {
  Eigen::Vector2d direction_vector;
  direction_vector << this->get_parameter("gps_end_x").as_double() - odom_start_point_.x,
      this->get_parameter("gps_end_y").as_double() - odom_start_point_.y;
  direction_vector.normalize();
  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) = buoy_0.x + direction_vector(0) * 20;
  predicted_positions(1, 0) = buoy_0.y + direction_vector(1) * 20;
  predicted_positions(0, 1) = buoy_1.x + direction_vector(0) * 20;
  predicted_positions(1, 1) = buoy_1.y + direction_vector(1) * 20;

  return predicted_positions;
}

Eigen::Array<double, 2, 2> CollisionAvoidanceTaskNode::predict_fourth_buoy_pair(
    const geometry_msgs::msg::Point &buoy_red,
    const geometry_msgs::msg::Point &buoy_green) {
  Eigen::Vector2d direction_vector;
  direction_vector << this->get_parameter("gps_end_x").as_double() - odom_start_point_.x,
      this->get_parameter("gps_end_y").as_double() - odom_start_point_.y;
  direction_vector.normalize();
  Eigen::Array<double, 2, 2> predicted_positions;
  predicted_positions(0, 0) = buoy_red.x + direction_vector(0) * 5;
  predicted_positions(1, 0) = buoy_red.y + direction_vector(1) * 5;
  predicted_positions(0, 1) = buoy_green.x + direction_vector(0) * 5;
  predicted_positions(1, 1) = buoy_green.y + direction_vector(1) * 5;

  return predicted_positions;
}

} // namespace collision_avoidance_task