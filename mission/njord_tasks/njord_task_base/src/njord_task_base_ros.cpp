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

  declare_parameter<std::string>("map_origin_topic", "/map/origin");
  declare_parameter<std::string>("odom_topic", "/seapath/odom/ned");
  declare_parameter<std::string>("landmark_pose_topic", "/landmark/pose");

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

  map_origin_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      get_parameter("map_origin_topic").as_string(), qos_transient_local,
      std::bind(&NjordTaskBaseNode::map_origin_callback, this,
                std::placeholders::_1));

  waypoint_visualization_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/waypoint_visualization", qos_sensor_data);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  waypoint_client_ =
      this->create_client<vortex_msgs::srv::Waypoint>("/waypoint");
}

void NjordTaskBaseNode::setup_map_odom_tf_and_subs() {
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
      get_parameter("landmark_pose_topic").as_string(), qos_sensor_data,
      std::bind(&NjordTaskBaseNode::landmark_callback, this,
                std::placeholders::_1));
}

void NjordTaskBaseNode::set_gps_frame_coords() {
  auto [gps_start_x, gps_start_y] =
      lla2flat(this->get_parameter("gps_start_lat").as_double(),
               this->get_parameter("gps_start_lon").as_double());
  auto [gps_end_x, gps_end_y] =
      lla2flat(this->get_parameter("gps_end_lat").as_double(),
               this->get_parameter("gps_end_lon").as_double());
  this->set_parameter(rclcpp::Parameter("gps_start_x", gps_start_x));
  this->set_parameter(rclcpp::Parameter("gps_start_y", gps_start_y));
  this->set_parameter(rclcpp::Parameter("gps_end_x", gps_end_x));
  this->set_parameter(rclcpp::Parameter("gps_end_y", gps_end_y));
  this->set_parameter(rclcpp::Parameter("gps_frame_coords_set", true));
  RCLCPP_INFO(this->get_logger(),
              "GPS map frame coordinates set to: %f, %f, %f, %f", gps_start_x,
              gps_start_y, gps_end_x, gps_end_y);

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
  std::unique_lock<std::mutex> setup_lock(setup_mutex_);
  this->set_parameter(rclcpp::Parameter("map_origin_lat", msg->latitude));
  this->set_parameter(rclcpp::Parameter("map_origin_lon", msg->longitude));
  this->set_parameter(rclcpp::Parameter("map_origin_set", true));
  RCLCPP_INFO(this->get_logger(), "Map origin set to: %f, %f", msg->latitude,
              msg->longitude);

  // Set the map to odom transform
  setup_map_odom_tf_and_subs();
  // Set GPS frame coordinates
  set_gps_frame_coords();

  map_origin_sub_.reset();
  setup_lock.unlock();
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

Eigen::VectorXi NjordTaskBaseNode::assign_landmarks(const Eigen::Array<double, 2, Eigen::Dynamic> &predicted_positions,
                         const Eigen::Array<double, 2, Eigen::Dynamic> &measured_positions){
  int num_predicted = predicted_positions.cols();
  int num_measured = measured_positions.cols();
  Eigen::MatrixXd reward_matrix(num_measured, num_predicted);

  if(num_predicted > num_measured){
    RCLCPP_ERROR(this->get_logger(), "Number of predicted positions is greater than number of measured positions in auction algorithm");
  }

  double epsilon = 1e-6; // Small positive number to prevent division by zero
  for (Eigen::Index i = 0; i < num_measured; ++i) {
    for (Eigen::Index j = 0; j < num_predicted; ++j) {
      double dx = measured_positions(0, i) - predicted_positions(0, j);
      double dy = measured_positions(1, i) - predicted_positions(1, j);
      double distance = std::sqrt(dx * dx + dy * dy);
      reward_matrix(i, j) = 1/(distance + epsilon);
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