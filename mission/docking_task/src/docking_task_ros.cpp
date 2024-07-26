#include <docking_task/docking_task_ros.hpp>


namespace docking_task {

DockingTaskNode::DockingTaskNode(const rclcpp::NodeOptions &options)
    : Node("dock_localization_node", options) {

  declare_parameter<double>("map_origin_lat", 0.0);
  declare_parameter<double>("map_origin_lon", 0.0);
  declare_parameter<bool>("map_origin_set", false);
  declare_parameter<double>("gps_start_lat", 0.0);
  declare_parameter<double>("gps_start_lon", 0.0);
  declare_parameter<double>("gps_end_lat", 0.0);
  declare_parameter<double>("gps_end_lon", 0.0);
  declare_parameter<double>("gps_start_x", 0.0);
  declare_parameter<double>("gps_start_y", 0.0);
  declare_parameter<double>("gps_end_x", 0.0);
  declare_parameter<double>("gps_end_y", 0.0);
  declare_parameter<bool>("gps_frame_coords_set", false);
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

  declare_parameter<std::string>("map_origin_topic", "/map/origin");
  declare_parameter<std::string>("grid_topic", "grid");
  declare_parameter<std::string>("odom_topic", "/seapath/odom/ned");
  declare_parameter<std::string>("landmark_pose_topic", "/landmark/pose");
 
  // Sensor data QoS profile
  rmw_qos_profile_t qos_profile_sensor_data = rmw_qos_profile_sensor_data;
  auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sensor_data.history, 1), qos_profile_sensor_data);
  
  // Transient local QoS profile
  rmw_qos_profile_t qos_profile_transient_local = rmw_qos_profile_parameters;
  qos_profile_transient_local.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  auto qos_transient_local = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_transient_local.history, 1), qos_profile_transient_local);

  gps_map_coord_visualization_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "/gps_map_coord_visualization", qos_sensor_data);

  map_origin_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    get_parameter("map_origin_topic").as_string(),
      qos_transient_local, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      this->set_parameter(rclcpp::Parameter("map_origin_lat", msg->latitude));
      this->set_parameter(rclcpp::Parameter("map_origin_lon", msg->longitude));
      this->set_parameter(rclcpp::Parameter("map_origin_set", true));
      RCLCPP_INFO(this->get_logger(), "Map origin set to: %f, %f", msg->latitude, msg->longitude);

      // Set GPS frame coordinates
      set_gps_frame_coords();
      
      map_origin_sub_.reset();
    });

  grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    get_parameter("grid_topic").as_string(),
      qos_sensor_data, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received grid message");
      std::unique_lock<std::mutex> lock(grid_mutex_);
      grid_msg_ = msg;
      lock.unlock();
    });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    get_parameter("odom_topic").as_string(),
      qos_sensor_data, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received odometry message");
      std::unique_lock<std::mutex> lock(odom_mutex_);
      odom_msg_ = msg;
      lock.unlock();
    });

  landmarks_sub_ = this->create_subscription<vortex_msgs::msg::LandmarkArray>(
    get_parameter("landmark_pose_topic").as_string(),
      qos_sensor_data, [this](const vortex_msgs::msg::LandmarkArray::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received landmark pose array");
      std::unique_lock<std::mutex> lock(landmark_mutex_);
      landmarks_msg_ = msg;
      lock.unlock();
    });

  waypoint_visualization_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/waypoint_visualization", qos_sensor_data);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  waypoint_client_ = this->create_client<vortex_msgs::srv::Waypoint>("/waypoint");

  std::thread(&DockingTaskNode::main_task, this).detach();
}


void DockingTaskNode::main_task() {
  // Sleep for 5 seconds to allow system to initialize and tracks to be aquired
  RCLCPP_INFO(this->get_logger(), "Waiting for system to initialize before starting main task");
  rclcpp::sleep_for(std::chrono::seconds(5));
  RCLCPP_INFO(this->get_logger(), "System initialized, starting main task");

  // Set initial waypoint between first buoy pair
  auto [landmark1, landmark2] = initial_waypoint();
  RCLCPP_INFO(this->get_logger(), "Initial waypoint set between landmarks %d and %d", landmark1.id, landmark2.id);

  reach_waypoint(1.0);
  
  auto formation = predict_buoy_formation(landmark1, landmark2);

  navigate_formation(formation);

}

std::pair<LandmarkWithID, LandmarkWithID> DockingTaskNode::initial_waypoint() {
  RCLCPP_INFO(this->get_logger(), "Initial waypoint running");

  while (true) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    if (!(this->get_parameter("map_origin_set").as_bool())) {
      RCLCPP_INFO(this->get_logger(), "Map origin not set, exiting initial waypoint");
      continue;
    }
    if (!(this->get_parameter("gps_frame_coords_set").as_bool())) {
      set_gps_frame_coords();
    }
    // std::unique_lock<std::mutex> odom_lock(odom_mutex_);
    // if (odom_msg_ == nullptr) {
    //   RCLCPP_INFO(this->get_logger(), "Odometry message not received, exiting initial waypoint");
    //   odom_lock.unlock();
    //   continue;
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

    // // The point P should lie on the line from the start point in the direction of the heading
    // // Find the intersection point where vector from P to end point is orthogonal to the heading direction vector

    // // Set up the equation for the line in the heading direction from start point
    // // P = (P_x, P_y)
    // // Line equation: P_x = gps_start_x + t * dir_x, P_y = gps_start_y + t * dir_y

    // // We need the vector (gps_end_x - P_x, gps_end_y - P_y) to be orthogonal to the direction vector (dir_x, dir_y)
    // // (gps_end_x - (gps_start_x + t * dir_x)) * dir_x + (gps_end_y - (gps_start_y + t * dir_y)) * dir_y = 0
    // // Solving for t

    // double t = ((gps_end_x - gps_start_x) * dir_x + (gps_end_y - gps_start_y) * dir_y) / (dir_x * dir_x + dir_y * dir_y);

    // // Calculate intersection point
    // double intersection_x = gps_start_x + t * dir_x;
    // double intersection_y = gps_start_y + t * dir_y;

    // RCLCPP_INFO(this->get_logger(), "Intersection point: %f, %f", intersection_x, intersection_y);
    // geometry_msgs::msg::PoseStamped waypoint;
    // waypoint.header.frame_id = "map";
    // waypoint.pose.position.x = intersection_x;
    // waypoint.pose.position.y = intersection_y;
    // waypoint_visualization_pub_->publish(waypoint);

    if (landmarks_msg_ == nullptr) {
      RCLCPP_INFO(this->get_logger(), "Landmark pose array not received, exiting initial waypoint");
      continue;
    }

    int result = 0;
    double x_waypoint = 0.0;
    double y_waypoint = 0.0;
    std::pair<uint32_t, u_int32_t> id_pair;

    std::pair<LandmarkWithID, LandmarkWithID> landmark_return_pair;

    while (result < 10){
    
      // Transform landmark poses to base_link frame
      geometry_msgs::msg::PoseArray landmark_poses_map_frame;
      geometry_msgs::msg::PoseArray landmark_poses_base_link_frame;
      std::vector<LandmarkWithID> landmarks_with_ids;
      std::unique_lock<std::mutex> landmark_lock(landmark_mutex_);
      for (const auto& landmark : landmarks_msg_->landmarks) {
        LandmarkWithID lwid;
        lwid.id = landmark.id;
        lwid.pose_map_frame = landmark.odom.pose.pose;
        landmark_poses_map_frame.poses.push_back(landmark.odom.pose.pose);
        landmarks_with_ids.push_back(lwid);
      }

      landmark_lock.unlock();

      try {
        auto transform = tf_buffer_->lookupTransform("base_link", "map", rclcpp::Time(0));
        for (size_t i = 0; i < landmark_poses_base_link_frame.poses.size(); ++i) {
          tf2::doTransform(landmark_poses_map_frame.poses.at(i), landmark_poses_base_link_frame.poses.at(i), transform);
          landmarks_with_ids[i].pose_base_link_frame = landmark_poses_base_link_frame.poses[i];
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
                  double dist_a = sqrt(pow(a.pose_base_link_frame.position.x, 2) + pow(a.pose_base_link_frame.position.y, 2));
                  double dist_b = sqrt(pow(b.pose_base_link_frame.position.x, 2) + pow(b.pose_base_link_frame.position.y, 2));
                  return dist_a < dist_b;
                });

      // Find a pair of landmarks that satisfy the conditions
      for (size_t i = 0; i < landmarks_with_ids.size() - 1; ++i) {
        const auto& landmark1 = landmarks_with_ids[i];
        const auto& landmark2 = landmarks_with_ids[i + 1];

        // Calculate the distance between the landmarks
        double distance = sqrt(pow(landmark2.pose_base_link_frame.position.x - landmark1.pose_base_link_frame.position.x, 2) +
                              pow(landmark2.pose_base_link_frame.position.y - landmark1.pose_base_link_frame.position.y, 2));

        // Check if the landmarks are on opposite sides of the drone and within the desired distance range
        // Base link is NED frame, x is forward, y is right
        if ((landmark1.pose_base_link_frame.position.x > 0 && landmark2.pose_base_link_frame.position.x > 0) 
        && (distance >= 3.0 && distance <= 8.0)
        && ((landmark1.pose_base_link_frame.position.y < 0) != (landmark2.pose_base_link_frame.position.y < 0))) {
          // Calculate the midpoint between the two landmarks in map frame
          double x_waypoint_sample = (landmark1.pose_map_frame.position.x + landmark2.pose_map_frame.position.x) / 2;
          double y_waypoint_sample = (landmark1.pose_map_frame.position.y + landmark2.pose_map_frame.position.y) / 2;
          std::pair<uint32_t, uint32_t> id_pair_sample = {landmark1.id, landmark2.id};
          // Check if this is the first valid pair of landmarks
          if (result == 0){
            x_waypoint = x_waypoint_sample;
            y_waypoint = y_waypoint_sample;
            id_pair = id_pair_sample;
            result ++;
            break;
          }
        
          // Check if the new waypoint is further away from the current waypoint
          // or if the new pair of landmarks is the same as the current pair
          if ((sqrt(pow(x_waypoint_sample - x_waypoint, 2) + pow(y_waypoint_sample - y_waypoint, 2)) > 1.0) ||
          !((id_pair_sample.first == id_pair.first && id_pair_sample.second == id_pair.second) ||
          (id_pair_sample.first == id_pair.second && id_pair_sample.second == id_pair.first)))
          {
            result = 0;
            break;
          }

          // Update the waypoint and the pair of landmarks
          id_pair = id_pair_sample;
          x_waypoint += x_waypoint_sample;
          y_waypoint += y_waypoint_sample;
          x_waypoint /= 2;
          y_waypoint /= 2;
          result ++;
          landmark_return_pair.first = landmark1;
          landmark_return_pair.second = landmark2;
          break;

        }
      }

    }
    //send waypoint
    if(!waypoint_client_->service_is_ready()){
      RCLCPP_INFO(this->get_logger(), "Waypoint client not ready");
      continue;
    }
    geometry_msgs::msg::Point waypoint_map_frame;
    geometry_msgs::msg::Point waypoint_odom_frame;
    waypoint_map_frame.x = x_waypoint;
    waypoint_map_frame.y = y_waypoint;
    waypoint_map_frame.z = 0.0;
    try {
      auto transform = tf_buffer_->lookupTransform("odom", "map", rclcpp::Time(0));
      tf2::doTransform(waypoint_map_frame, waypoint_odom_frame, transform);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    }
    auto request = std::make_shared<vortex_msgs::srv::Waypoint::Request>();
    request->waypoint.push_back(waypoint_odom_frame);
    auto result_future = waypoint_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Waypoint sent: %f, %f", waypoint_odom_frame.x, waypoint_odom_frame.y);
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
  std::unique_lock<std::mutex> odom_lock(odom_mutex_);
  double x = odom_msg_->pose.pose.position.x;
  double y = odom_msg_->pose.pose.position.y;
  odom_lock.unlock();
  while (sqrt(pow(x - previous_waypoint_odom_frame_.x, 2) + pow(y - previous_waypoint_odom_frame_.y, 2)) > distance_threshold) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    odom_lock.lock();
    x = odom_msg_->pose.pose.position.x;
    y = odom_msg_->pose.pose.position.y;
    odom_lock.unlock();
  }
  RCLCPP_INFO(this->get_logger(), "Reached waypoint");
  return;
}

Eigen::Array<double, 2, 6> DockingTaskNode::predict_buoy_formation(const LandmarkWithID &buoy1, const LandmarkWithID &buoy2) const {
  RCLCPP_INFO(this->get_logger(), "Predict buoy formation running");
  try {
    geometry_msgs::msg::Point previous_waypoint_map_frame;
    auto transform = tf_buffer_->lookupTransform("map", "odom", rclcpp::Time(0));
    tf2::doTransform(previous_waypoint_odom_frame_, previous_waypoint_map_frame, transform);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
  } 
  Eigen::Vector2d direction_vector(previous_waypoint_odom_frame_.x - this->get_parameter("gps_start_x").as_double(),
                                   previous_waypoint_odom_frame_.y - this->get_parameter("gps_start_y").as_double());
  direction_vector.normalize();
  // Sanity check that first buoy pair is ish correct
  Eigen::Vector2d first_left, first_right;

  size_t found = 0;
  std::unique_lock<std::mutex> landmark_lock(landmark_mutex_);
  for (const auto& landmark : landmarks_msg_->landmarks) {
    if (landmark.id == buoy1.id && 
    std::sqrt(std::pow(landmark.odom.pose.pose.position.x - buoy1.pose_map_frame.position.x, 2) +
    std::pow(landmark.odom.pose.pose.position.y - buoy1.pose_map_frame.position.y, 2)) < 0.5) {
      found++;
      if(buoy1.pose_base_link_frame.position.y < 0){
        first_left = Eigen::Vector2d(landmark.odom.pose.pose.position.x, landmark.odom.pose.pose.position.y);
      } else {
        first_right = Eigen::Vector2d(landmark.odom.pose.pose.position.x, landmark.odom.pose.pose.position.y);
      }
    }
    if (landmark.id == buoy2.id &&
    std::sqrt(std::pow(landmark.odom.pose.pose.position.x - buoy2.pose_map_frame.position.x, 2) +
    std::pow(landmark.odom.pose.pose.position.y - buoy2.pose_map_frame.position.y, 2)) < 0.5) {
      found++;
      if(buoy2.pose_base_link_frame.position.y < 0){
        first_left = Eigen::Vector2d(landmark.odom.pose.pose.position.x, landmark.odom.pose.pose.position.y);
      } else {
        first_right = Eigen::Vector2d(landmark.odom.pose.pose.position.x, landmark.odom.pose.pose.position.y);
      }
    }
  }
  landmark_lock.unlock();
  if(found != 2){
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
  formation << third_left.x(), third_right.x(), second_left.x(), second_right.x(), first_left.x(), first_right.x(),
               third_left.y(), third_right.y(), second_left.y(), second_right.y(), first_left.y(), first_right.y();
  return formation;
}

Eigen::MatrixXd DockingTaskNode::generate_reward_matrix(const Eigen::Array<double, 2, 6>& predicted_positions, const Eigen::MatrixXd& measured_positions) {
    int num_predicted = predicted_positions.cols(); // Should be 6
    int num_measured = measured_positions.cols();
    Eigen::MatrixXd reward_matrix(num_measured, num_predicted);
    
    // Initialize the reward matrix
    for (int i = 0; i < num_measured; ++i) {
        for (int j = 0; j < num_predicted; ++j) {
            double dx = measured_positions(0, i) - predicted_positions(0, j);
            double dy = measured_positions(1, i) - predicted_positions(1, j);
            double distance = std::sqrt(dx * dx + dy * dy);
            reward_matrix(i, j) = -distance; // We use negative distance as reward (shorter distance = higher reward)
        }
    }
    return reward_matrix;
}

Eigen::VectorXi DockingTaskNode::auction_algorithm(const Eigen::MatrixXd &reward_matrix) {
  int num_items              = reward_matrix.rows();
  int num_customers          = reward_matrix.cols();
  Eigen::VectorXi assignment = Eigen::VectorXi::Constant(num_customers, -1);
  Eigen::VectorXd prices     = Eigen::VectorXd::Zero(num_items);

  std::vector<int> unassigned;
  for (int i = 0; i < num_customers; ++i) {
    unassigned.push_back(i);
  }

  double epsilon = 1.0 / (num_items + 1);

  while (!unassigned.empty()) {
    int customer = unassigned.back();
    unassigned.pop_back();

    double max_value = std::numeric_limits<double>::lowest();
    int max_item = -1;
    for (int item = 0; item < reward_matrix.rows(); ++item) {
      double value = reward_matrix.coeff(item, customer) - prices[item];
      if (value > max_value) {
        max_value = value;
        max_item  = item;
      }
    }

    // Find the current owner of max item
    int current_owner = -1;
    for (int i = 0; i < num_customers; ++i) {
      if (assignment[i] == max_item) {
        current_owner = i;
        break;
      }
    }
    if (current_owner != -1) {
      unassigned.push_back(current_owner);
    }

    assignment[customer] = max_item;
    prices[max_item] += max_value + epsilon;
  }

  return assignment;
}

void DockingTaskNode::navigate_formation(const Eigen::Array<double, 2, 6>& predicted_positions) {
  RCLCPP_INFO(this->get_logger(), "Navigating though formation");
  std::vector<uint32_t> prev_assignment;
  std::vector<uint32_t> landmark_ids;
  bool first_half = true;
  // Buoy formation with index 
  //  dock
  //<< 0 1
  //<< 2 3
  //<< 4 5
  //  start
  int result = 0;
  while (true){
    landmark_ids.clear();
    geometry_msgs::msg::PoseArray landmark_poses_map_frame;
    std::unique_lock<std::mutex> landmark_lock(landmark_mutex_);
    for (const auto& landmark : landmarks_msg_->landmarks) {
     
      landmark_ids.push_back(landmark.id);
      landmark_poses_map_frame.poses.push_back(landmark.odom.pose.pose);
    }

    landmark_lock.unlock();

    // Transform landmark poses to odom_frame
    geometry_msgs::msg::PoseArray landmark_poses_odom_frame;
    try {
      auto transform = tf_buffer_->lookupTransform("odom", "map", rclcpp::Time(0));
      for (size_t i = 0; i < landmark_poses_map_frame.poses.size(); ++i) {
        tf2::doTransform(landmark_poses_map_frame.poses.at(i), landmark_poses_odom_frame.poses.at(i), transform);
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    }

    Eigen::MatrixXd measured_positions(2, landmark_ids.size());
    for (size_t i = 0; i < landmark_ids.size(); ++i) {
      measured_positions(0, i) = landmark_poses_odom_frame.poses.at(i).position.x;
      measured_positions(1, i) = landmark_poses_odom_frame.poses.at(i).position.y;
    }
  
    Eigen::MatrixXd reward_matrix = generate_reward_matrix(predicted_positions, measured_positions);

    Eigen::VectorXi assignment = auction_algorithm(reward_matrix);


      if (result == 0) {
        for(Eigen::Index i = 0; i < assignment.size(); i++){
          prev_assignment.push_back(landmark_ids.at(assignment(i)));
        }
        result++;
        continue;
      }
      bool valied = true;
      // Check that the assigned landmarks matches the previous assignment by id
      if(first_half){
        // Check index 2,3,4,5 
        if (landmark_ids.at(assignment(2)) != prev_assignment.at(2) || landmark_ids.at(assignment(3)) != prev_assignment.at(3)
        || landmark_ids.at(assignment(4)) != prev_assignment.at(4) || landmark_ids.at(assignment(5)) != prev_assignment.at(5)){
          valied = false;
        }
      } else {
        // Check index 0,1,2,3
        if (landmark_ids.at(assignment(0)) != prev_assignment.at(0) || landmark_ids.at(assignment(1)) != prev_assignment.at(1)
        || landmark_ids.at(assignment(2)) != prev_assignment.at(2) || landmark_ids.at(assignment(3)) != prev_assignment.at(3)){
          valied = false;
        }
      }

      if(!valied){
        result = 0;
        prev_assignment.clear();
        continue;
      }
      else{
        result++; 
        }
      if (result > 10){
        geometry_msgs::msg::Point waypoint_odom_frame;
        // Calculate the waypoint between the relevant buoy pair
        double buoy_left_x;
        double buoy_left_y;
        double buoy_right_x;
        double buoy_right_y;
        if(first_half){
          buoy_left_x = landmark_poses_odom_frame.poses.at(assignment(2)).position.x;
          buoy_left_y = landmark_poses_odom_frame.poses.at(assignment(2)).position.y;
          buoy_right_x = landmark_poses_odom_frame.poses.at(assignment(3)).position.x;
          buoy_right_y = landmark_poses_odom_frame.poses.at(assignment(3)).position.y;
        } else {
          buoy_left_x = landmark_poses_odom_frame.poses.at(assignment(0)).position.x;
          buoy_left_y = landmark_poses_odom_frame.poses.at(assignment(0)).position.y;
          buoy_right_x = landmark_poses_odom_frame.poses.at(assignment(1)).position.x;
          buoy_right_y = landmark_poses_odom_frame.poses.at(assignment(1)).position.y;
        }

        waypoint_odom_frame.x = (buoy_left_x + buoy_right_x) / 2;
        waypoint_odom_frame.y = (buoy_left_y + buoy_right_y) / 2;

        auto request = std::make_shared<vortex_msgs::srv::Waypoint::Request>();
        request->waypoint.push_back(waypoint_odom_frame);
        auto result_future = waypoint_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Waypoint sent: %f, %f", waypoint_odom_frame.x, waypoint_odom_frame.y);
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

        if(first_half){
          first_half = false;
          result = 0;
          prev_assignment.clear();

          // Wait for the ASV to reach the waypoint and then find waypoint for the second half of the formation
          reach_waypoint(1.0);
          continue;
        }
        else{
          // Return from function when the asv is close to the second waypoint
          reach_waypoint(0.2);
          return;
        }

      
      }
   
  }
}

void DockingTaskNode::set_gps_frame_coords() {
  auto [gps_start_x, gps_start_y] = lla2flat(this->get_parameter("gps_start_lat").as_double(), this->get_parameter("gps_start_lon").as_double());
  auto [gps_end_x, gps_end_y] = lla2flat(this->get_parameter("gps_end_lat").as_double(), this->get_parameter("gps_end_lon").as_double());
  this->set_parameter(rclcpp::Parameter("gps_start_x", gps_start_x));
  this->set_parameter(rclcpp::Parameter("gps_start_y", gps_start_y));
  this->set_parameter(rclcpp::Parameter("gps_end_x", gps_end_x));
  this->set_parameter(rclcpp::Parameter("gps_end_y", gps_end_y));
  this->set_parameter(rclcpp::Parameter("gps_frame_coords_set", true));
  RCLCPP_INFO(this->get_logger(), "GPS map frame coordinates set to: %f, %f, %f, %f", gps_start_x, gps_start_y, gps_end_x, gps_end_y);

  geometry_msgs::msg::PoseArray gps_points;
  gps_points.header.frame_id = "map";

  // Convert GPS points to geometry poses
  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = gps_start_x;
  start_pose.position.y = gps_start_y;
  gps_points.poses.push_back(start_pose);

  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = gps_end_x;
  end_pose.position.y = gps_end_y;
  gps_points.poses.push_back(end_pose);

  gps_map_coord_visualization_pub_->publish(gps_points);
}

std::pair<double, double> DockingTaskNode::lla2flat(double lat, double lon) const {
  const double R = 6378137.0;           // WGS-84 Earth semimajor axis (meters)
  const double f = 1.0 / 298.257223563; // Flattening of the earth
  const double psi_rad = 0.0; 

  // Angular direction of the flat Earth x-axis, specified as a scalar.
  // The angular direction is the degrees clockwise from north,
  // which is the angle in degrees used for converting flat Earth x and
  // y coordinates to the north and east coordinates

  // Convert angles from degrees to radians
  const double lat_rad = lat * M_PI / 180.0;
  const double lon_rad = lon * M_PI / 180.0;
  const double origin_lat_rad =  this->get_parameter("map_origin_lat").as_double() * M_PI / 180.0;
  const double origin_lon_rad = this->get_parameter("map_origin_lon").as_double() * M_PI / 180.0;

  // Calculate delta latitude and delta longitude in radians
  const double dlat = lat_rad - origin_lat_rad;
  const double dlon = lon_rad - origin_lon_rad;

  // Radius of curvature in the vertical prime (RN)
  const double RN =
      R / sqrt(1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));

  // Radius of curvature in the meridian (RM)
  const double RM = RN * (1.0 - (2.0 * f - f * f)) /
                    (1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));

  // Changes in the north (dN) and east (dE) positions
  const double dN = RM * dlat;
  const double dE = RN * cos(origin_lat_rad) * dlon;

  // Transformation from North-East to flat x-y coordinates
  const double px = cos(psi_rad) * dN - sin(psi_rad) * dE;
  const double py = sin(psi_rad) * dN + cos(psi_rad) * dE;

  return {px, py};
}

double DockingTaskNode::get_freya_heading(const geometry_msgs::msg::Quaternion msg) const {
  tf2::Quaternion quat;
  tf2::fromMsg(msg, quat);

  // Convert quaternion to Euler angles
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  return yaw;
}

} // namespace docking_task