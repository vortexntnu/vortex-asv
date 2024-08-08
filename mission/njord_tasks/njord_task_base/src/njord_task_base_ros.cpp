#include <njord_task_base/njord_task_base_ros.hpp>

NjordTaskBaseNode::NjordTaskBaseNode(const std::string &node_name,
                                     const rclcpp::NodeOptions &options)
    : Node(node_name, options) {
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
  declare_parameter<int>("assignment_confidence", 10);

  declare_parameter<std::string>("map_origin_topic", "/map/origin");
  declare_parameter<std::string>("odom_topic", "/seapath/odom/ned");
  declare_parameter<std::string>("landmark_topic", "landmarks_out");

  // Sensor data QoS profile
  rmw_qos_profile_t qos_profile_sensor_data = rmw_qos_profile_sensor_data;
  auto qos_sensor_data =
      rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sensor_data.history, 1),
                  qos_profile_sensor_data);

  // Transient local QoS profile
  rmw_qos_profile_t qos_profile_transient_local = rmw_qos_profile_parameters;
  qos_profile_transient_local.durability =
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  auto qos_transient_local = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile_transient_local.history, 1),
      qos_profile_transient_local);

  gps_map_coord_visualization_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>(
          "/gps_map_coord_visualization", qos_sensor_data);

  buoy_visualization_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/buoy_visualization", qos_sensor_data);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (this->get_parameter("map_origin_set").as_bool()) {
    get_map_odom_tf();
    set_gps_odom_points();
    initialize_subscribers();
    std::unique_lock<std::mutex> setup_lock(navigation_mutex_);
    navigation_ready_ = true;
    setup_lock.unlock();
  } else {
    map_origin_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        get_parameter("map_origin_topic").as_string(), qos_transient_local,
        std::bind(&NjordTaskBaseNode::map_origin_callback, this,
                  std::placeholders::_1));
  }

  waypoint_visualization_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/waypoint_visualization", qos_sensor_data);

  waypoint_client_ =
      this->create_client<vortex_msgs::srv::Waypoint>("waypoint_list");

  heading_client_ =
      this->create_client<vortex_msgs::srv::DesiredVelocity>("yaw_reference");
}

void NjordTaskBaseNode::get_map_odom_tf() {
  // Get the transform between the map and odom frames to avoid the overhead
  // from continuously looking up the static transform between map and odom
  bool tf_set = false;
  while (!tf_set) {
    try {
      auto transform = tf_buffer_->lookupTransform(
          "odom", "map", tf2::TimePointZero, tf2::durationFromSec(1.0));
      map_odom_tf_ = transform;
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }
    tf_set = true;
  }
}

void NjordTaskBaseNode::initialize_subscribers() {
  // Sensor data QoS profile
  rmw_qos_profile_t qos_profile_sensor_data = rmw_qos_profile_sensor_data;
  auto qos_sensor_data =
      rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sensor_data.history, 1),
                  qos_profile_sensor_data);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), qos_sensor_data,
      std::bind(&NjordTaskBaseNode::odom_callback, this,
                std::placeholders::_1));

  landmarks_sub_ = this->create_subscription<vortex_msgs::msg::LandmarkArray>(
      get_parameter("landmark_topic").as_string(), qos_sensor_data,
      std::bind(&NjordTaskBaseNode::landmark_callback, this,
                std::placeholders::_1));
}

void NjordTaskBaseNode::set_gps_odom_points() {
  if (this->get_parameter("gps_frame_coords_set").as_bool()) {
    RCLCPP_INFO(this->get_logger(), "Using predefined GPS frame coordinates");
    geometry_msgs::msg::PoseArray gps_points_odom_frame;
    gps_points_odom_frame.header.frame_id = "odom";
    geometry_msgs::msg::Pose gps_start_odom_frame;
    gps_start_odom_frame.position.x =
        this->get_parameter("gps_start_x").as_double();
    gps_start_odom_frame.position.y =
        this->get_parameter("gps_start_y").as_double();
    geometry_msgs::msg::Pose gps_end_odom_frame;
    gps_end_odom_frame.position.x =
        this->get_parameter("gps_end_x").as_double();
    gps_end_odom_frame.position.y =
        this->get_parameter("gps_end_y").as_double();
    gps_points_odom_frame.poses.push_back(gps_start_odom_frame);
    gps_points_odom_frame.poses.push_back(gps_end_odom_frame);
    gps_map_coord_visualization_pub_->publish(gps_points_odom_frame);
    RCLCPP_INFO(
        this->get_logger(), "GPS odom frame coordinates set to: %f, %f, %f, %f",
        gps_start_odom_frame.position.x, gps_start_odom_frame.position.y,
        gps_end_odom_frame.position.x, gps_end_odom_frame.position.y);
    return;
  }
  auto [gps_start_x, gps_start_y] =
      lla2flat(this->get_parameter("gps_start_lat").as_double(),
               this->get_parameter("gps_start_lon").as_double());
  auto [gps_end_x, gps_end_y] =
      lla2flat(this->get_parameter("gps_end_lat").as_double(),
               this->get_parameter("gps_end_lon").as_double());

  geometry_msgs::msg::PoseStamped gps_start_map_frame;
  gps_start_map_frame.header.frame_id = "map";
  gps_start_map_frame.pose.position.x = gps_start_x;
  gps_start_map_frame.pose.position.y = gps_start_y;
  geometry_msgs::msg::PoseStamped gps_end_map_frame;
  gps_end_map_frame.header.frame_id = "map";
  gps_end_map_frame.pose.position.x = gps_end_x;
  gps_end_map_frame.pose.position.y = gps_end_y;
  geometry_msgs::msg::PoseStamped gps_start_odom_frame;
  geometry_msgs::msg::PoseStamped gps_end_odom_frame;
  try {
    tf2::doTransform(gps_start_map_frame, gps_start_odom_frame, map_odom_tf_);
    tf2::doTransform(gps_end_map_frame, gps_end_odom_frame, map_odom_tf_);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    return;
  }

  this->set_parameter(
      rclcpp::Parameter("gps_start_x", gps_start_odom_frame.pose.position.x));
  this->set_parameter(
      rclcpp::Parameter("gps_start_y", gps_start_odom_frame.pose.position.y));
  this->set_parameter(
      rclcpp::Parameter("gps_end_x", gps_end_odom_frame.pose.position.x));
  this->set_parameter(
      rclcpp::Parameter("gps_end_y", gps_end_odom_frame.pose.position.y));
  this->set_parameter(rclcpp::Parameter("gps_frame_coords_set", true));
  RCLCPP_INFO(
      this->get_logger(), "GPS odom frame coordinates set to: %f, %f, %f, %f",
      gps_start_odom_frame.pose.position.x,
      gps_start_odom_frame.pose.position.y, gps_end_odom_frame.pose.position.x,
      gps_end_odom_frame.pose.position.y);

  geometry_msgs::msg::PoseArray gps_points_odom_frame;
  gps_points_odom_frame.header.frame_id = "odom";

  gps_points_odom_frame.poses.push_back(gps_start_odom_frame.pose);
  gps_points_odom_frame.poses.push_back(gps_end_odom_frame.pose);

  gps_map_coord_visualization_pub_->publish(gps_points_odom_frame);
}

std::pair<double, double> NjordTaskBaseNode::lla2flat(double lat,
                                                      double lon) const {
  const double R = 6378137.0;           // WGS-84 Earth semimajor axis (meters)
  const double f = 1.0 / 298.257223563; // Flattening of the earth
  const double psi_rad = 0.0;

  // Convert angles from degrees to radians
  const double lat_rad = lat * M_PI / 180.0;
  const double lon_rad = lon * M_PI / 180.0;
  const double origin_lat_rad =
      this->get_parameter("map_origin_lat").as_double() * M_PI / 180.0;
  const double origin_lon_rad =
      this->get_parameter("map_origin_lon").as_double() * M_PI / 180.0;

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

void NjordTaskBaseNode::map_origin_callback(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  this->set_parameter(rclcpp::Parameter("map_origin_lat", msg->latitude));
  this->set_parameter(rclcpp::Parameter("map_origin_lon", msg->longitude));
  this->set_parameter(rclcpp::Parameter("map_origin_set", true));
  RCLCPP_INFO(this->get_logger(), "Map origin set to: %f, %f", msg->latitude,
              msg->longitude);

  // Set the map to odom transform
  get_map_odom_tf();
  // Set GPS frame coordinates
  set_gps_odom_points();
  initialize_subscribers();
  map_origin_sub_.reset();
  std::unique_lock<std::mutex> setup_lock(navigation_mutex_);
  navigation_ready_ = true;
  setup_lock.unlock();
}

void NjordTaskBaseNode::navigation_ready() {
  bool ready = false;
  while (!ready) {
    std::unique_lock<std::mutex> setup_lock(navigation_mutex_);
    ready = navigation_ready_;
    setup_lock.unlock();
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Waiting for navigation system to be ready");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

void NjordTaskBaseNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(odom_mutex_);
  odom_msg_ = msg;
  new_odom_msg_ = true;
  lock.unlock();
  odom_cond_var_.notify_one();
}

std::shared_ptr<nav_msgs::msg::Odometry> NjordTaskBaseNode::get_odom() {
  std::unique_lock<std::mutex> lock(odom_mutex_);
  odom_cond_var_.wait(lock, [this] { return new_odom_msg_; });
  new_odom_msg_ = false;
  lock.unlock();
  return odom_msg_;
}

void NjordTaskBaseNode::landmark_callback(
    const vortex_msgs::msg::LandmarkArray::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(landmark_mutex_);
  // transform the landmarks to the odom frame
  for (auto &landmark : msg->landmarks) {
    tf2::doTransform(landmark.odom.pose.pose, landmark.odom.pose.pose,
                     map_odom_tf_);
  }
  landmarks_msg_ = msg;
  new_landmark_msg_ = true;
  lock.unlock();
  landmark_cond_var_.notify_one();
}

std::shared_ptr<vortex_msgs::msg::LandmarkArray>
NjordTaskBaseNode::get_landmarks_odom_frame() {
  std::unique_lock<std::mutex> lock(landmark_mutex_);
  landmark_cond_var_.wait(lock, [this] { return new_landmark_msg_; });
  new_landmark_msg_ = false;
  lock.unlock();
  return landmarks_msg_;
}

Eigen::VectorXi NjordTaskBaseNode::auction_algorithm(
    const Eigen::Array<double, 2, Eigen::Dynamic> &predicted_positions,
    const Eigen::Array<double, 2, Eigen::Dynamic> &measured_positions) {
  int num_predicted = predicted_positions.cols();
  int num_measured = measured_positions.cols();
  Eigen::MatrixXd reward_matrix(num_measured, num_predicted);

  if (num_predicted > num_measured) {
    RCLCPP_ERROR(this->get_logger(),
                 "Number of predicted positions is greater than number of "
                 "measured positions in auction algorithm");
    return Eigen::VectorXi::Constant(num_predicted, -1);
  }

  double epsilon = 1e-6; // Small positive number to prevent division by zero
  for (Eigen::Index i = 0; i < num_measured; ++i) {
    for (Eigen::Index j = 0; j < num_predicted; ++j) {
      double dx = measured_positions(0, i) - predicted_positions(0, j);
      double dy = measured_positions(1, i) - predicted_positions(1, j);
      double distance = std::sqrt(dx * dx + dy * dy);
      reward_matrix(i, j) = 1 / (distance + epsilon);
    }
  }

  Eigen::VectorXi assignment = Eigen::VectorXi::Constant(num_predicted, -1);
  Eigen::VectorXd prices = Eigen::VectorXd::Zero(num_measured);

  std::vector<int> unassigned;
  for (int i = 0; i < num_predicted; ++i) {
    unassigned.push_back(i);
  }

  epsilon = 1.0 / (num_measured + num_predicted + 1);

  while (!unassigned.empty()) {
    int customer = unassigned.back();
    unassigned.pop_back();

    double max_value = std::numeric_limits<double>::lowest();
    double second_max_value = std::numeric_limits<double>::lowest();
    int max_item = -1;

    for (int item = 0; item < num_measured; ++item) {
      double value = reward_matrix.coeff(item, customer) - prices[item];
      if (value > max_value) {
        second_max_value = max_value;
        max_value = value;
        max_item = item;
      } else if (value > second_max_value) {
        second_max_value = value;
      }
    }

    int current_owner = -1;
    for (int i = 0; i < num_predicted; ++i) {
      if (assignment[i] == max_item) {
        current_owner = i;
        break;
      }
    }
    if (current_owner != -1) {
      unassigned.push_back(current_owner);
    }

    assignment[customer] = max_item;
    prices[max_item] += max_value - second_max_value + epsilon;
  }
  return assignment;
}

std::vector<LandmarkPoseID> NjordTaskBaseNode::get_buoy_landmarks(
    const Eigen::Array<double, 2, Eigen::Dynamic> &predicted_positions) {
  std::vector<int32_t> landmark_ids;
  std::vector<int32_t> expected_assignment;
  Eigen::Array<double, 2, Eigen::Dynamic> returned_buoy_positions(
      2, predicted_positions.cols());
  int confidence_threshold =
      this->get_parameter("assignment_confidence").as_int();
  int result = 0;
  while (result < confidence_threshold) {
    landmark_ids.clear();
    auto landmark_msg = get_landmarks_odom_frame();

    // Extract measured positions and corresponding landmark ids
    Eigen::Array<double, 2, Eigen::Dynamic> measured_positions(
        2, landmark_msg->landmarks.size());
    for (size_t i = 0; i < landmark_msg->landmarks.size(); i++) {
      landmark_ids.push_back(landmark_msg->landmarks[i].id);
      measured_positions(0, i) =
          landmark_msg->landmarks[i].odom.pose.pose.position.x;
      measured_positions(1, i) =
          landmark_msg->landmarks[i].odom.pose.pose.position.y;
    }

    // Check if there are enough landmarks detected to perform auction algorithm
    if (predicted_positions.cols() > measured_positions.cols()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Not enough landmarks detected to perform auction algorithm");
      result = 0;
      continue;
    }
    // Perform auction algorithm
    Eigen::VectorXi assignment =
        auction_algorithm(predicted_positions, measured_positions);

    // Extract measured positions of assigned buoys
    for (Eigen::Index i = 0; i < assignment.size(); i++) {
      returned_buoy_positions(0, i) = measured_positions(0, assignment(i));
      returned_buoy_positions(1, i) = measured_positions(1, assignment(i));
    }

    // Check if any buoys are unassigned
    // Should never happen as long as the number of landmarks detected is
    // greater than or equal to the number of buoys
    bool unassigned_buoy = false;
    for (Eigen::Index i = 0; i < assignment.size(); i++) {
      if (assignment(i) == -1) {
        unassigned_buoy = true;
        break;
      }
    }

    // If any buoys are unassigned, restart assignment process
    if (unassigned_buoy) {
      result = 0;
      continue;
    }

    // If this is the first iteration, save the assignment and continue
    if (result == 0) {
      expected_assignment.clear();
      for (Eigen::Index i = 0; i < assignment.size(); i++) {
        expected_assignment.push_back(landmark_ids.at(assignment(i)));
      }
      result++;
      continue;
    }

    // Check if the assignment is consistent with the previous assignment
    bool consistent_assignment = true;
    for (Eigen::Index i = 0; i < assignment.size(); i++) {
      if (landmark_ids.at(assignment(i)) != expected_assignment.at(i)) {
        consistent_assignment = false;
        break;
      }
    }

    // If the assignment is consistent, increment the result counter
    // Otherwise, reset the result counter
    if (consistent_assignment) {
      result++;
      continue;
    } else {
      result = 0;
      continue;
    }
  }
  // Loop has completed, return the id and pose of the landmarks assigned to
  // buoys
  std::vector<LandmarkPoseID> buoys;
  for (size_t i = 0; i < expected_assignment.size(); i++) {
    LandmarkPoseID landmark;
    landmark.id = expected_assignment.at(i);
    landmark.pose_odom_frame.position.x = returned_buoy_positions(0, i);
    landmark.pose_odom_frame.position.y = returned_buoy_positions(1, i);
    buoys.push_back(landmark);
  }
  return buoys;
}

void NjordTaskBaseNode::send_waypoint(
    const geometry_msgs::msg::Point &waypoint) {
  auto request = std::make_shared<vortex_msgs::srv::Waypoint::Request>();
  request->waypoint.push_back(waypoint);
  auto result_future = waypoint_client_->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "Waypoint(odom frame) sent: %f, %f",
              waypoint.x, waypoint.y);
  // Check if the service was successful
  double dx = waypoint.x - previous_waypoint_odom_frame_.x;
  double dy = waypoint.y - previous_waypoint_odom_frame_.y;
  double desired_heading = std::atan2(dy, dx);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, desired_heading);

  geometry_msgs::msg::PoseStamped waypoint_vis;
  waypoint_vis.header.frame_id = "odom";
  waypoint_vis.pose.position.x = waypoint.x;
  waypoint_vis.pose.position.y = waypoint.y;
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

  previous_waypoint_odom_frame_ = waypoint;
}

void NjordTaskBaseNode::reach_waypoint(const double distance_threshold) {
  RCLCPP_INFO(this->get_logger(), "Reach waypoint running");

  auto get_current_position = [&]() {
    auto odom_msg = get_odom();
    return std::make_pair(odom_msg->pose.pose.position.x,
                          odom_msg->pose.pose.position.y);
  };

  auto [x, y] = get_current_position();

  while (std::hypot(x - previous_waypoint_odom_frame_.x,
                    y - previous_waypoint_odom_frame_.y) > distance_threshold) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    std::tie(x, y) = get_current_position();
  }

  RCLCPP_INFO(this->get_logger(), "Reached waypoint");
}

void NjordTaskBaseNode::set_desired_heading(
    const geometry_msgs::msg::Point &prev_waypoint,
    const geometry_msgs::msg::Point &next_waypoint) {
  auto request = std::make_shared<vortex_msgs::srv::DesiredVelocity::Request>();
  double dx = next_waypoint.x - prev_waypoint.x;
  double dy = next_waypoint.y - prev_waypoint.y;
  double desired_heading = std::atan2(dy, dx);
  request->u_desired = desired_heading;
  auto result_future = heading_client_->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "Desired heading sent: %f", desired_heading);
  auto status = result_future.wait_for(std::chrono::seconds(5));
  while (status == std::future_status::timeout) {
    RCLCPP_INFO(this->get_logger(), "Desired heading service timed out");
    status = result_future.wait_for(std::chrono::seconds(5));
  }
  if (!result_future.get()->success) {
    RCLCPP_INFO(this->get_logger(), "Desired heading service failed");
  }
}