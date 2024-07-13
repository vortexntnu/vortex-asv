#include <landmark_server/landmark_server.hpp>

using std::placeholders::_1, std::placeholders::_2;
using LandmarkArray = vortex_msgs::msg::LandmarkArray;
using Action = vortex_msgs::action::FilteredLandmarks;

namespace landmark_server {

LandmarkServerNode::LandmarkServerNode(const rclcpp::NodeOptions &options)
    : Node("landmark_server_node", options) {

  // Define the quality of service profile for publisher and subscriber
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1),
                         qos_profile);

  storedLandmarks_ = std::make_shared<LandmarkArray>();

  landmark_sub_ = this->create_subscription<LandmarkArray>(
      "target_tracking/landmarks", qos,
      std::bind(&LandmarkServerNode::landmarksRecievedCallback, this, _1));

  landmarkPublisher_ =
      this->create_publisher<LandmarkArray>("landmarks_out", qos);

  posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "landmark_poses_out", qos);

  gridPublisher_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid", qos);

  rmw_qos_profile_t qos_profile_sensor = rmw_qos_profile_sensor_data;
  qos_profile_sensor.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto qos_sensor =
      rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sensor.history, 1),
                  qos_profile_sensor);

  convex_hull_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("landmark/hull",
                                                            qos_sensor);
  cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "landmark/cluster", qos_sensor);

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  declare_parameter<std::string>("fixed_frame", "world");
  declare_parameter<std::string>("target_frame", "base_link");

  // Create the act.
  action_server_ = rclcpp_action::create_server<Action>(
      this, "landmark_filter",
      std::bind(&LandmarkServerNode::handle_goal, this, _1, _2),
      std::bind(&LandmarkServerNode::handle_cancel, this, _1),
      std::bind(&LandmarkServerNode::handle_accepted, this, _1));

  grid_client_ = create_client<nav_msgs::srv::GetMap>("get_map");

  std::thread(&LandmarkServerNode::get_grid, this).detach();
}

Eigen::Array<float, 2, Eigen::Dynamic> LandmarkServerNode::get_convex_hull(
    const shape_msgs::msg::SolidPrimitive &solid_primitive) {
  pcl::PointCloud<pcl::PointXYZ> cluster;
  cluster.resize(solid_primitive.polygon.points.size());
  for (size_t i = 0; i < solid_primitive.polygon.points.size(); i++) {
    cluster.points[i].x = solid_primitive.polygon.points[i].x;
    cluster.points[i].y = solid_primitive.polygon.points[i].y;
    cluster.points[i].z = 1.0;
  }
  sensor_msgs::msg::PointCloud2 cluster_msg;
  pcl::toROSMsg(cluster, cluster_msg);
  cluster_msg.header.frame_id = "world";
  cluster_publisher_->publish(cluster_msg);
  pcl::PointCloud<pcl::PointXYZ> convex_hull;
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setDimension(2);
  chull.setInputCloud(cluster.makeShared());
  chull.reconstruct(convex_hull);
  sensor_msgs::msg::PointCloud2 hull_msg;
  pcl::toROSMsg(convex_hull, hull_msg);
  hull_msg.header.frame_id = "world";
  convex_hull_publisher_->publish(hull_msg);
  RCLCPP_INFO(this->get_logger(), "Convex hull size: %ld", convex_hull.size());
  Eigen::Array<float, 2, Eigen::Dynamic> hull(2, convex_hull.size());
  for (size_t i = 0; i < convex_hull.size(); i++) {
    hull(0, i) = convex_hull.points[i].x;
    hull(1, i) = convex_hull.points[i].y;
  }
  RCLCPP_INFO(this->get_logger(), "Convex hull: %ld", hull.cols());
  std::cout << "HUll" << hull << std::endl;
  return hull;
}

void LandmarkServerNode::get_grid() {
  while (true) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    if (!grid_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available after waiting");
      continue;
    }

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto result_future = grid_client_->async_send_request(request);

    // Wait for the result within a specified timeout period
    auto status = result_future.wait_for(std::chrono::seconds(5));
    if (status == std::future_status::ready) {
      try {
        auto result = result_future.get();
        if (result->map.data.empty()) {
          RCLCPP_ERROR(this->get_logger(),
                       "Received empty map from grid client");
          continue;
        }
        grid_msg_ = result->map;

        grid_manager_ = std::make_unique<GridManager>(grid_msg_.info.resolution,
                                                      grid_msg_.info.height,
                                                      grid_msg_.info.width);
        RCLCPP_INFO(this->get_logger(),
                    "Successfully received map from grid client");
        return;
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(),
                     "Exception while getting result from future: %s",
                     e.what());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to get map from grid client within timeout period");
      continue;
    }
  }
}

void LandmarkServerNode::landmarksRecievedCallback(
    const LandmarkArray::SharedPtr msg) {
  if (msg->landmarks.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Received empty landmark array");
    return;
  }

  if (grid_manager_ == nullptr) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Grid manager not initialized");
    return;
  }

  for (const auto &landmark : msg->landmarks) {
    auto it = std::find_if(
        storedLandmarks_->landmarks.begin(), storedLandmarks_->landmarks.end(),
        [&](const auto &storedLandmark) {
          return storedLandmark.landmark_type == landmark.landmark_type &&
                 storedLandmark.id == landmark.id;
        });

    if (landmark.action == vortex_msgs::msg::Landmark::ADD_ACTION) {
      if (it != storedLandmarks_->landmarks.end()) {
        RCLCPP_WARN_STREAM_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Requested to add already existing landmark");
      } else {
        // Add the new landmark
        grid_manager_->update_grid(grid_msg_.data.data(),
                                   get_convex_hull(landmark.shape), 200);
        storedLandmarks_->landmarks.push_back(landmark);
      }
      continue;
    }

    if (it == storedLandmarks_->landmarks.end()) {
      RCLCPP_WARN_STREAM_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Requested to remove or update non-existing landmark");
      continue;
    }

    grid_manager_->update_grid(grid_msg_.data.data(),
                               get_convex_hull((*it).shape), -200);

    if (landmark.action == vortex_msgs::msg::Landmark::REMOVE_ACTION) {
      storedLandmarks_->landmarks.erase(it);
    } else if (landmark.action == vortex_msgs::msg::Landmark::UPDATE_ACTION) {
      grid_manager_->update_grid(grid_msg_.data.data(),
                                 get_convex_hull(landmark.shape), 200);
      *it = landmark;
    }
  }
  std_msgs::msg::Header header;

  // header.frame_id = get_parameter("fixed_frame").as_string();
  // header.stamp = rclcpp::Clock().now();

  // // Convert the Eigen array to std::vector<int8_t>
  //   auto grid_eigen = grid_manager_->get_grid();
  //   std::vector<int8_t> grid_data(grid_eigen.data(), grid_eigen.data() +
  //   grid_eigen.size()); grid_msg_.header = header; grid_msg_.data =
  //   grid_data;
  grid_msg_.header.stamp = rclcpp::Clock().now();

  // Publish the landmarks
  gridPublisher_->publish(grid_msg_);
  landmarkPublisher_->publish(*storedLandmarks_);
  posePublisher_->publish(poseArrayCreater(*storedLandmarks_));
}

geometry_msgs::msg::PoseArray LandmarkServerNode::poseArrayCreater(
    vortex_msgs::msg::LandmarkArray landmarks) {

  geometry_msgs::msg::PoseArray poseArray;
  // sets the header for the array to equal the header of the first landmark
  if (!landmarks.landmarks.empty()) {
    poseArray.header.frame_id = landmarks.landmarks.at(0).odom.header.frame_id;
  } else {
    poseArray.header.frame_id = get_parameter("fixed_frame").as_string();
  }
  // Timestamps for stored landmarks may vary so use current time for
  // visualization
  poseArray.header.stamp = rclcpp::Clock().now();
  for (const auto &landmark : landmarks.landmarks) {
    // Convert landmark to pose here...
    geometry_msgs::msg::Pose gridPose;
    poseArray.poses.push_back(landmark.odom.pose.pose);
  }
  return poseArray;
}

rclcpp_action::GoalResponse LandmarkServerNode::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const vortex_msgs::action::FilteredLandmarks::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received request");
  (void)uuid;
  if (goal->distance < 0.0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Distance must be non-negative, aborting goal");

    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LandmarkServerNode::handle_cancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LandmarkServerNode::handle_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>>
        goal_handle) {

  // This needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&LandmarkServerNode::execute, this, _1), goal_handle}
      .detach();
}

void LandmarkServerNode::execute(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>>
        goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Action::Feedback>();
  auto result = std::make_shared<Action::Result>();

  // Log the request
  requestLogger(goal_handle);

  // Create a timer to control the publishing rate at 10 Hz
  rclcpp::Rate rate(10); // 10 Hz

  // Filter the StoredLandmarks by landmark_types in the action request
  while (rclcpp::ok()) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->result = 2;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled by client");
      return;
    }

    if (storedLandmarks_->landmarks.empty()) {
      RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Waiting for landmarks to be detected");
      rate.sleep();
      continue;
    }

    vortex_msgs::msg::OdometryArray filteredLandmarksOdoms =
        filterLandmarks(goal_handle);
    if (filteredLandmarksOdoms.odoms.empty()) {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "No landmarks found matching the request");
    }

    feedback->feedback = filteredLandmarksOdoms;

    // Publish the odometryArray as feedback
    goal_handle->publish_feedback(feedback);
    // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    // "Publishing feedback!");
    rate.sleep();
  }
  RCLCPP_WARN(this->get_logger(), "Connection error, stopping action");
  result->result = 3;
  goal_handle->succeed(result);
}

vortex_msgs::msg::OdometryArray LandmarkServerNode::filterLandmarks(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>>
        goal_handle) {
  const auto goal = goal_handle->get_goal();
  _Float32 distance = goal->distance;
  uint8_t filter_type = goal->landmark_types;
  vortex_msgs::msg::OdometryArray filteredLandmarksOdoms;

  for (const auto &landmark : storedLandmarks_->landmarks) {

    if (filter_type == 6 && distance == 0.0) {
      filteredLandmarksOdoms.odoms.push_back(landmark.odom);
    } else if (filter_type == 6) {

      if (calculateDistance(landmark.odom.pose.pose.position,
                            landmark.odom.header) <= distance) {
        filteredLandmarksOdoms.odoms.push_back(landmark.odom);
      }
    } else if (distance == 0.0) {
      if (landmark.landmark_type == filter_type) {
        filteredLandmarksOdoms.odoms.push_back(landmark.odom);
      }
    } else {
      if (landmark.landmark_type == filter_type &&
          calculateDistance(landmark.odom.pose.pose.position,
                            landmark.odom.header) <= distance) {
        filteredLandmarksOdoms.odoms.push_back(landmark.odom);
      }
    }
  }
  return filteredLandmarksOdoms;
}

void LandmarkServerNode::requestLogger(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>>
        goal_handle) {
  const auto goal = goal_handle->get_goal();
  double distance = goal->distance;

  if (distance == 0.0 &&
      goal->landmark_types == vortex_msgs::msg::Landmark::NONE) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Received request to return all landmarks.");
    return;
  }

  if (goal->landmark_types == vortex_msgs::msg::Landmark::NONE) {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Received request to return all landmarks within distance " << distance
                                                                    << ".");
    return;
  }

  std::stringstream types_log;
  types_log << "Received request to return landmarks by type filter: [";
  if (goal->landmark_types & vortex_msgs::msg::Landmark::BUOY) {
    types_log << "BUOY, ";
  }
  if (goal->landmark_types & vortex_msgs::msg::Landmark::BOAT) {
    types_log << "BOAT, ";
  }
  if (goal->landmark_types & vortex_msgs::msg::Landmark::WALL) {
    types_log << "WALL, ";
  }
  types_log << "].";
  RCLCPP_INFO_STREAM(this->get_logger(), types_log.str());
}

double
LandmarkServerNode::calculateDistance(const geometry_msgs::msg::Point &point,
                                      const std_msgs::msg::Header &header) {
  std::string target_frame = get_parameter("target_frame").as_string();
  // Transform the pose of the landmark in world_frame to pose in frame
  // specified by landmark_pose_frame parameter

  try {
    // Lookup the transformation
    geometry_msgs::msg::TransformStamped transform_stamped =
        tf2_buffer_->lookupTransform(target_frame, header.frame_id,
                                     header.stamp, rclcpp::Duration(1, 0));

    // Transform the point cloud
    geometry_msgs::msg::Point transformed_point;
    tf2::doTransform(point, transformed_point, transform_stamped);
    // Calculate the distance between the drone and the landmark
    double x = transformed_point.x;
    double y = transformed_point.y;
    double z = transformed_point.z;

    return sqrt(x * x + y * y + z * z);

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Could not transform landmark position: "
                                    << ex.what()
                                    << "\nIgnoring distance filter");
  }

  return 0.0;
}

} // namespace landmark_server
