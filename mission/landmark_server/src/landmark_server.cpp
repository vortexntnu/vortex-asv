#include "landmark_server/landmark_server.hpp"

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

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  declare_parameter<std::string>("target_frame", "base_link");

  // Create the act.
  action_server_ = rclcpp_action::create_server<Action>(
      this, "landmark_filter",
      std::bind(&LandmarkServerNode::handle_goal, this, _1, _2),
      std::bind(&LandmarkServerNode::handle_cancel, this, _1),
      std::bind(&LandmarkServerNode::handle_accepted, this, _1));
}

void LandmarkServerNode::landmarksRecievedCallback(
    const LandmarkArray::SharedPtr msg) {
  if (msg->landmarks.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
    "Received empty landmark array");
    return;
  }
  
  for (const auto &landmark : msg->landmarks) {
    // RCLCPP_INFO(this->get_logger(), "Landmarks received");

    if (landmark.action == 0) {
      // Remove landmarks with matching id and landmark_type
      storedLandmarks_->landmarks.erase(
          std::remove_if(storedLandmarks_->landmarks.begin(),
                         storedLandmarks_->landmarks.end(),
                         [&](const auto &storedLandmark) {
                           return storedLandmark.id == landmark.id &&
                                  storedLandmark.landmark_type ==
                                      landmark.landmark_type;
                         }),
          storedLandmarks_->landmarks.end());
    } else if (landmark.action == 1) {
      // Find the landmark if it already exists
      auto it = std::find_if(
          storedLandmarks_->landmarks.begin(),
          storedLandmarks_->landmarks.end(), [&](const auto &storedLandmark) {
            return storedLandmark.landmark_type == landmark.landmark_type &&
                   storedLandmark.id == landmark.id;
          });

      if (it != storedLandmarks_->landmarks.end()) {
        // Update the existing landmark
        *it = landmark;
      } else {
        // Add the new landmark
        storedLandmarks_->landmarks.push_back(landmark);
      }
    }
  }

  // Publish the landmarks
  landmarkPublisher_->publish(*storedLandmarks_);
  posePublisher_->publish(poseArrayCreater(*storedLandmarks_));
}

geometry_msgs::msg::PoseArray LandmarkServerNode::poseArrayCreater(
    vortex_msgs::msg::LandmarkArray landmarks) {

  geometry_msgs::msg::PoseArray poseArray;
  // sets the header for the array to equal the header of the first landmark
  poseArray.header.frame_id = landmarks.landmarks[0].odom.header.frame_id;
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
  vortex_msgs::msg::OdometryArray filteredLandmarksOdoms;

  for (const auto &landmark : storedLandmarks_->landmarks) {

    if (goal->landmark_types.empty() && distance == 0.0) {
      filteredLandmarksOdoms.odoms.push_back(landmark.odom);
    } else if (goal->landmark_types.empty()) {

      if (calculateDistance(landmark.odom.pose.pose.position,
                            landmark.odom.header) <= distance) {
        filteredLandmarksOdoms.odoms.push_back(landmark.odom);
      }
    } else if (distance == 0.0) {
      for (const auto &type : goal->landmark_types) {
        if (landmark.landmark_type == type) {
          filteredLandmarksOdoms.odoms.push_back(landmark.odom);
        }
      }
    } else {
      for (const auto &type : goal->landmark_types) {
        if (landmark.landmark_type == type &&
            calculateDistance(landmark.odom.pose.pose.position,
                              landmark.odom.header) <= distance) {
          filteredLandmarksOdoms.odoms.push_back(landmark.odom);
        }
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

  if (distance == 0.0 && goal->landmark_types.empty()) {  
    RCLCPP_INFO_STREAM(this->get_logger(), "Received request to return all landmarks.");  
    return;  
  }  
  
  if (goal->landmark_types.empty()) {  
    RCLCPP_INFO_STREAM(this->get_logger(),  
                "Received request to return all landmarks within distance " << distance << ".");  
    return;  
  }  

  std::stringstream types_log;  
  types_log << "Received request to return landmarks by type filter: [";  
  for (auto it = goal->landmark_types.begin(); it != goal->landmark_types.end(); it++) {  
    types_log << *it;  
    if (std::next(it) != goal->landmark_types.end()) {  
      types_log << ", ";  
    }  
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
        tf2_buffer_->lookupTransform("base_link", header.frame_id, header.stamp,
                                     rclcpp::Duration(1, 0));

    // Transform the point cloud
    geometry_msgs::msg::Point transformed_point;
    tf2::doTransform(point, transformed_point, transform_stamped);
    // Calculate the distance between the drone and the landmark
    double x = transformed_point.x;
    double y = transformed_point.y;
    double z = transformed_point.z;

    return sqrt(x * x + y * y + z * z);

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Could not transform landmark position: " << ex.what() << 
        "\nIgnoring distance filter");
  }

  return 0.0;
}

} // namespace landmark_server
