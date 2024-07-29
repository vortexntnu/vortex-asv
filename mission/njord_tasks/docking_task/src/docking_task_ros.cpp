#include <docking_task/docking_task_ros.hpp>

namespace docking_task {

DockingTaskNode::DockingTaskNode(const rclcpp::NodeOptions &options)
    : NjordTaskBaseNode("dock_localization_node", options) {

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
  declare_parameter<std::string>("grid_topic", "grid");

  std::thread(&DockingTaskNode::main_task, this).detach();
}

void DockingTaskNode::main_task() {
  // Sleep for 5 seconds to allow system to initialize and tracks to be aquired
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
      set_gps_frame_coords();
      setup_map_odom_tf_and_subs();
      setup_lock.unlock();
      break;
    }
    setup_lock.unlock();
  }

  // Set initial waypoint between first buoy pair
  auto [landmark1, landmark2] = initial_waypoint();
  RCLCPP_INFO(this->get_logger(),
              "Initial waypoint set between landmarks %d and %d", landmark1.id,
              landmark2.id);

  reach_waypoint(1.0);

  auto formation = predict_buoy_formation(landmark1, landmark2);

  auto [buoy_left_id, buoy_right_id] = navigate_formation(formation);
}

std::pair<LandmarkWithID, LandmarkWithID> DockingTaskNode::initial_waypoint() {
  RCLCPP_INFO(this->get_logger(), "Initial waypoint running");

  while (true) {

    // std::unique_lock<std::mutex> odom_lock(odom_mutex_);
    // if (odom_msg_ == nullptr) {
    //   RCLCPP_INFO(this->get_logger(), "Odometry message not received, exiting
    //   initial waypoint"); odom_lock.unlock(); continue;
    // }
    // double heading = get_freya_heading(odom_msg_->pose.pose.orientation);
    // odom_lock.unlock();

    // double gps_start_x = this->get_parameter("gps_start_x").as_double();
    // double gps_start_y = this->get_parameter("gps_start_y").as_double();
    // double gps_end_x = this->get_parameter("gps_end_x").as_double();
    // double gps_end_y = this->get_parameter("gps_end_y").as_double();

    // // Calculate the direction vector from the start point
    // double dir_x = cos(heading);
    // double dir_y = sin(heading);

    // // The point P should lie on the line from the start point in the
    // direction of the heading
    // // Find the intersection point where vector from P to end point is
    // orthogonal to the heading direction vector

    // // Set up the equation for the line in the heading direction from start
    // point
    // // P = (P_x, P_y)
    // // Line equation: P_x = gps_start_x + t * dir_x, P_y = gps_start_y + t *
    // dir_y

    // // We need the vector (gps_end_x - P_x, gps_end_y - P_y) to be orthogonal
    // to the direction vector (dir_x, dir_y)
    // // (gps_end_x - (gps_start_x + t * dir_x)) * dir_x + (gps_end_y -
    // (gps_start_y + t * dir_y)) * dir_y = 0
    // // Solving for t

    // double t = ((gps_end_x - gps_start_x) * dir_x + (gps_end_y - gps_start_y)
    // * dir_y) / (dir_x * dir_x + dir_y * dir_y);

    // // Calculate intersection point
    // double intersection_x = gps_start_x + t * dir_x;
    // double intersection_y = gps_start_y + t * dir_y;

    // RCLCPP_INFO(this->get_logger(), "Intersection point: %f, %f",
    // intersection_x, intersection_y); geometry_msgs::msg::PoseStamped
    // waypoint; waypoint.header.frame_id = "map"; waypoint.pose.position.x =
    // intersection_x; waypoint.pose.position.y = intersection_y;
    // waypoint_visualization_pub_->publish(waypoint);

    if (landmarks_msg_ == nullptr) {
      RCLCPP_INFO(this->get_logger(),
                  "Landmark pose array not received, exiting initial waypoint");
      continue;
    }

    int result = 0;
    double x_waypoint = 0.0;
    double y_waypoint = 0.0;
    std::pair<int32_t, int32_t> id_pair;

    std::pair<LandmarkWithID, LandmarkWithID> landmark_return_pair;

    while (result < 10) {

      // Transform landmark poses to base_link frame
      std::vector<LandmarkWithID> landmarks_with_ids;
      auto landmark_msg = get_landmarks_odom_frame();
      for (const auto &landmark : landmark_msg->landmarks) {
        LandmarkWithID lwid;
        lwid.id = landmark.id;
        lwid.pose_odom_frame = landmark.odom.pose.pose;
        landmarks_with_ids.push_back(lwid);
      }

      try {
        auto transform =
            tf_buffer_->lookupTransform("base_link", "odom", rclcpp::Time(0));
        for (size_t i = 0; i < landmarks_with_ids.size(); ++i) {
          tf2::doTransform(landmarks_with_ids.at(i).pose_odom_frame,
                           landmarks_with_ids.at(i).pose_base_link_frame,
                           transform);
        }
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
      }

      // Remove landmarks behind the drone
      std::remove_if(landmarks_with_ids.begin(), landmarks_with_ids.end(),
                     [](const LandmarkWithID &l) {
                       return l.pose_base_link_frame.position.x < 0.0;
                     });

      // Sort landmarks based on distance
      std::sort(landmarks_with_ids.begin(), landmarks_with_ids.end(),
                [](const LandmarkWithID &a, const LandmarkWithID &b) {
                  double dist_a =
                      sqrt(pow(a.pose_base_link_frame.position.x, 2) +
                           pow(a.pose_base_link_frame.position.y, 2));
                  double dist_b =
                      sqrt(pow(b.pose_base_link_frame.position.x, 2) +
                           pow(b.pose_base_link_frame.position.y, 2));
                  return dist_a < dist_b;
                });

      // Find a pair of landmarks that satisfy the conditions
      for (size_t i = 0; i < landmarks_with_ids.size() - 1; ++i) {
        const auto &landmark1 = landmarks_with_ids[i];
        const auto &landmark2 = landmarks_with_ids[i + 1];

        // Calculate the distance between the landmarks
        double distance =
            sqrt(pow(landmark2.pose_base_link_frame.position.x -
                         landmark1.pose_base_link_frame.position.x,
                     2) +
                 pow(landmark2.pose_base_link_frame.position.y -
                         landmark1.pose_base_link_frame.position.y,
                     2));

        // Check if the landmarks are on opposite sides of the drone and within
        // the desired distance range.
        // Base link is NED frame, x is forward, y is right
        if ((landmark1.pose_base_link_frame.position.x > 0 &&
             landmark2.pose_base_link_frame.position.x > 0) &&
            (distance >= 3.0 && distance <= 8.0) &&
            ((landmark1.pose_base_link_frame.position.y < 0) !=
             (landmark2.pose_base_link_frame.position.y < 0))) {
          // Calculate the midpoint between the two landmarks in map frame
          double x_waypoint_sample = (landmark1.pose_odom_frame.position.x +
                                      landmark2.pose_odom_frame.position.x) /
                                     2;
          double y_waypoint_sample = (landmark1.pose_odom_frame.position.y +
                                      landmark2.pose_odom_frame.position.y) /
                                     2;
          std::pair<int32_t, int32_t> id_pair_sample = {landmark1.id,
                                                        landmark2.id};
          // Check if this is the first valid pair of landmarks
          if (result == 0) {
            x_waypoint = x_waypoint_sample;
            y_waypoint = y_waypoint_sample;
            id_pair = id_pair_sample;
            result++;
            break;
          }

          // Check if the new waypoint is further away from the current waypoint
          // or if the new pair of landmarks is the same as the current pair
          if ((sqrt(pow(x_waypoint_sample - x_waypoint, 2) +
                    pow(y_waypoint_sample - y_waypoint, 2)) > 1.0) ||
              !((id_pair_sample.first == id_pair.first &&
                 id_pair_sample.second == id_pair.second) ||
                (id_pair_sample.first == id_pair.second &&
                 id_pair_sample.second == id_pair.first))) {
            result = 0;
            break;
          }

          // Update the waypoint and the pair of landmarks
          id_pair = id_pair_sample;
          x_waypoint += x_waypoint_sample;
          y_waypoint += y_waypoint_sample;
          x_waypoint /= 2;
          y_waypoint /= 2;
          result++;
          landmark_return_pair.first = landmark1;
          landmark_return_pair.second = landmark2;
          break;
        }
      }
    }
    // send waypoint
    if (!waypoint_client_->service_is_ready()) {
      RCLCPP_INFO(this->get_logger(), "Waypoint client not ready");
      continue;
    }
    geometry_msgs::msg::Point waypoint_odom_frame;
    waypoint_odom_frame.x = x_waypoint;
    waypoint_odom_frame.y = y_waypoint;
    waypoint_odom_frame.z = 0.0;

    auto request = std::make_shared<vortex_msgs::srv::Waypoint::Request>();
    request->waypoint.push_back(waypoint_odom_frame);
    auto result_future = waypoint_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Waypoint(odom frame) sent: %f, %f",
                waypoint_odom_frame.x, waypoint_odom_frame.y);
    // Check if the service was successful

    auto status = result_future.wait_for(std::chrono::seconds(5));
    if (status == std::future_status::timeout) {
      RCLCPP_INFO(this->get_logger(), "Waypoint service timed out");
      continue;
    }
    if (!result_future.get()->success) {
      RCLCPP_INFO(this->get_logger(), "Waypoint service failed");
    }

    geometry_msgs::msg::PoseStamped waypoint_vis;
    waypoint_vis.header.frame_id = "odom";
    waypoint_vis.pose.position.x = waypoint_odom_frame.x;
    waypoint_vis.pose.position.y = waypoint_odom_frame.y;
    waypoint_visualization_pub_->publish(waypoint_vis);

    previous_waypoint_odom_frame_ = waypoint_odom_frame;
    return landmark_return_pair;
  }
}

void DockingTaskNode::reach_waypoint(const double distance_threshold) {
  RCLCPP_INFO(this->get_logger(), "Reach waypoint running");
  auto odom_msg = get_odom();
  double x = odom_msg->pose.pose.position.x;
  double y = odom_msg->pose.pose.position.y;
  while (sqrt(pow(x - previous_waypoint_odom_frame_.x, 2) +
              pow(y - previous_waypoint_odom_frame_.y, 2)) >
         distance_threshold) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    auto odom_msg_ = get_odom();
    x = odom_msg->pose.pose.position.x;
    y = odom_msg->pose.pose.position.y;
  }
  RCLCPP_INFO(this->get_logger(), "Reached waypoint");
  return;
}

Eigen::Array<double, 2, 6>
DockingTaskNode::predict_buoy_formation(const LandmarkWithID &buoy1,
                                        const LandmarkWithID &buoy2) {
  RCLCPP_INFO(this->get_logger(), "Predict buoy formation running");
  geometry_msgs::msg::Point previous_waypoint_map_frame;
  geometry_msgs::msg::TransformStamped odom_map_tf;
  try {
    // Compute the inverse transform from the stored map_odom_tf_
    tf2::Transform tf2_transform;
    tf2::fromMsg(map_odom_tf_.transform, tf2_transform);
    tf2::Transform tf2_inverse_transform = tf2_transform.inverse();
    odom_map_tf.transform = tf2::toMsg(tf2_inverse_transform);

    // Use the inverse transform
    tf2::doTransform(previous_waypoint_odom_frame_, previous_waypoint_map_frame,
                     odom_map_tf);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
  }
  Eigen::Vector2d direction_vector(
      previous_waypoint_map_frame.x -
          this->get_parameter("gps_start_x").as_double(),
      previous_waypoint_map_frame.y -
          this->get_parameter("gps_start_y").as_double());
  direction_vector.normalize();
  // Sanity check that first buoy pair is ish correct
  Eigen::Vector2d first_left, first_right;

  size_t found = 0;
  auto landmarks_msg = get_landmarks_odom_frame();
  for (const auto &landmark : landmarks_msg->landmarks) {
    if (landmark.id == buoy1.id &&
        std::sqrt(std::pow(landmark.odom.pose.pose.position.x -
                               buoy1.pose_odom_frame.position.x,
                           2) +
                  std::pow(landmark.odom.pose.pose.position.y -
                               buoy1.pose_odom_frame.position.y,
                           2)) < 0.5) {
      found++;
      if (buoy1.pose_base_link_frame.position.y < 0) {
        first_left = Eigen::Vector2d(landmark.odom.pose.pose.position.x,
                                     landmark.odom.pose.pose.position.y);
      } else {
        first_right = Eigen::Vector2d(landmark.odom.pose.pose.position.x,
                                      landmark.odom.pose.pose.position.y);
      }
    }
    if (landmark.id == buoy2.id &&
        std::sqrt(std::pow(landmark.odom.pose.pose.position.x -
                               buoy2.pose_odom_frame.position.x,
                           2) +
                  std::pow(landmark.odom.pose.pose.position.y -
                               buoy2.pose_odom_frame.position.y,
                           2)) < 0.5) {
      found++;
      if (buoy2.pose_base_link_frame.position.y < 0) {
        first_left = Eigen::Vector2d(landmark.odom.pose.pose.position.x,
                                     landmark.odom.pose.pose.position.y);
      } else {
        first_right = Eigen::Vector2d(landmark.odom.pose.pose.position.x,
                                      landmark.odom.pose.pose.position.y);
      }
    }
  }
  if (found != 2) {
    RCLCPP_WARN(this->get_logger(), "Buoy pair not found in landmark array");
  }
  Eigen::Array<double, 2, 6> formation;
  // Create formation with index
  //  dock
  //<< 0 1
  //<< 2 3
  //<< 4 5
  //  start
  Eigen::Vector2d second_left, second_right, third_left, third_right;
  second_left = first_left + direction_vector * 5.0;
  second_right = first_right + direction_vector * 5.0;
  third_left = first_left + direction_vector * 10.0;
  third_right = first_right + direction_vector * 10.0;
  formation << third_left.x(), third_right.x(), second_left.x(),
      second_right.x(), first_left.x(), first_right.x(), third_left.y(),
      third_right.y(), second_left.y(), second_right.y(), first_left.y(),
      first_right.y();
  return formation;
}

std::pair<int32_t, int32_t> DockingTaskNode::navigate_formation(
    Eigen::Array<double, 2, 6> &predicted_positions) {
  RCLCPP_INFO(this->get_logger(), "Navigating though formation");
  std::vector<int32_t> prev_assignment;
  std::vector<int32_t> landmark_ids;
  bool first_half = true;
  // Buoy formation with index
  //  dock
  //<< 0 1
  //<< 2 3
  //<< 4 5
  //  start
  int result = 0;
  while (true) {
    landmark_ids.clear();
    geometry_msgs::msg::PoseArray landmark_poses_odom_frame;
    auto landmark_msg = get_landmarks_odom_frame();
    for (const auto &landmark : landmark_msg->landmarks) {

      landmark_ids.push_back(landmark.id);
      landmark_poses_odom_frame.poses.push_back(landmark.odom.pose.pose);
    }

    Eigen::MatrixXd measured_positions(2, landmark_ids.size());
    for (size_t i = 0; i < landmark_ids.size(); ++i) {
      measured_positions(0, i) =
          landmark_poses_odom_frame.poses.at(i).position.x;
      measured_positions(1, i) =
          landmark_poses_odom_frame.poses.at(i).position.y;
    }

    Eigen::VectorXi assignment =
        assign_landmarks(predicted_positions, measured_positions);

    if (result == 0) {
      for (Eigen::Index i = 0; i < assignment.size(); i++) {
        prev_assignment.push_back(landmark_ids.at(assignment(i)));
      }
      result++;
      continue;
    }
    bool valied = true;
    // Check that the assigned landmarks matches the previous assignment by id
    if (first_half) {
      if (assignment(2) == -1 || assignment(3) == -1 || assignment(4) == -1 ||
          assignment(5) == -1) {
        valied = false;
      }

      // Check index 2,3,4,5
      if (landmark_ids.at(assignment(2)) != prev_assignment.at(2) ||
          landmark_ids.at(assignment(3)) != prev_assignment.at(3) ||
          landmark_ids.at(assignment(4)) != prev_assignment.at(4) ||
          landmark_ids.at(assignment(5)) != prev_assignment.at(5)) {
        valied = false;
      }
    } else {
      if (assignment(0) == -1 || assignment(1) == -1 || assignment(2) == -1 ||
          assignment(3) == -1) {
        valied = false;
      }
      // Check index 0,1,2,3
      if (landmark_ids.at(assignment(0)) != prev_assignment.at(0) ||
          landmark_ids.at(assignment(1)) != prev_assignment.at(1) ||
          landmark_ids.at(assignment(2)) != prev_assignment.at(2) ||
          landmark_ids.at(assignment(3)) != prev_assignment.at(3)) {
        valied = false;
      }
    }

    if (!valied) {
      result = 0;
      prev_assignment.clear();
      continue;
    } else {
      result++;
    }
    if (result > 10) {
      geometry_msgs::msg::Point waypoint_odom_frame;
      // Calculate the waypoint between the relevant buoy pair
      double buoy_left_x;
      double buoy_left_y;
      double buoy_right_x;
      double buoy_right_y;
      if (first_half) {
        buoy_left_x =
            landmark_poses_odom_frame.poses.at(assignment(2)).position.x;
        buoy_left_y =
            landmark_poses_odom_frame.poses.at(assignment(2)).position.y;
        buoy_right_x =
            landmark_poses_odom_frame.poses.at(assignment(3)).position.x;
        buoy_right_y =
            landmark_poses_odom_frame.poses.at(assignment(3)).position.y;
        Eigen::Vector2d direction_vector(
            (buoy_left_x -
             landmark_poses_odom_frame.poses.at(assignment(4)).position.x +
             buoy_right_x -
             landmark_poses_odom_frame.poses.at(assignment(5)).position.x) /
                2,
            (buoy_left_y -
             landmark_poses_odom_frame.poses.at(assignment(4)).position.y +
             buoy_right_y -
             landmark_poses_odom_frame.poses.at(assignment(5)).position.y) /
                2);
        direction_vector.normalize();
        // Update the predicted positions
        predicted_positions << buoy_left_x + direction_vector.x() * 5,
            buoy_right_x + direction_vector.x() * 5, buoy_left_x, buoy_right_x,
            buoy_left_x - direction_vector.x() * 5,
            buoy_right_x - direction_vector.x() * 5,
            buoy_left_y + direction_vector.y() * 5,
            buoy_right_y + direction_vector.y() * 5, buoy_left_y, buoy_right_y,
            buoy_left_y - direction_vector.y() * 5,
            buoy_right_y - direction_vector.y() * 5;
      } else {
        buoy_left_x =
            landmark_poses_odom_frame.poses.at(assignment(0)).position.x;
        buoy_left_y =
            landmark_poses_odom_frame.poses.at(assignment(0)).position.y;
        buoy_right_x =
            landmark_poses_odom_frame.poses.at(assignment(1)).position.x;
        buoy_right_y =
            landmark_poses_odom_frame.poses.at(assignment(1)).position.y;
      }

      waypoint_odom_frame.x = (buoy_left_x + buoy_right_x) / 2;
      waypoint_odom_frame.y = (buoy_left_y + buoy_right_y) / 2;

      auto request = std::make_shared<vortex_msgs::srv::Waypoint::Request>();
      request->waypoint.push_back(waypoint_odom_frame);
      auto result_future = waypoint_client_->async_send_request(request);
      RCLCPP_INFO(this->get_logger(), "Waypoint sent: %f, %f",
                  waypoint_odom_frame.x, waypoint_odom_frame.y);
      // Check if the service was successful

      auto status = result_future.wait_for(std::chrono::seconds(5));
      if (status == std::future_status::timeout) {
        RCLCPP_INFO(this->get_logger(), "Waypoint service timed out");
        continue;
      }
      if (!result_future.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Waypoint service failed");
      }
      geometry_msgs::msg::PoseStamped waypoint_vis;
      waypoint_vis.header.frame_id = "odom";
      waypoint_vis.pose.position.x = waypoint_odom_frame.x;
      waypoint_vis.pose.position.y = waypoint_odom_frame.y;
      waypoint_visualization_pub_->publish(waypoint_vis);

      previous_waypoint_odom_frame_ = waypoint_odom_frame;

      if (first_half) {
        first_half = false;
        result = 0;
        prev_assignment.clear();

        // Wait for the ASV to reach the waypoint and then find waypoint for the
        // second half of the formation
        reach_waypoint(1.0);
        continue;
      } else {
        // Return from function when the asv is close to the second waypoint
        reach_waypoint(0.2);
        return {landmark_ids.at(assignment(0)), landmark_ids.at(assignment(1))};
      }
    }
  }
}

} // namespace docking_task