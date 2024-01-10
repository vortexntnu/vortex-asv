#include <landmarks/landmarks.hpp>

using std::placeholders::_1, std::placeholders::_2;
using LandmarkArray = vortex_msgs::msg::LandmarkArray;
using Action = vortex_msgs::action::FilteredLandmarks;

namespace landmarks {

LandmarksNode::LandmarksNode(const rclcpp::NodeOptions &options)
    : Node("landmarks_node", options)
{
    // Define the quality of service profile for publisher and subscriber
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    storedLandmarks_ = std::make_shared<LandmarkArray>();
    gridVisualization_ = std::make_shared<grid_visualization::GridVisualization>();

    subscription_ = this->create_subscription<LandmarkArray>(
        "landmarks_in", qos, std::bind(&LandmarksNode::landmarksRecievedCallback, this, _1));

    landmarkPublisher_ = this->create_publisher<LandmarkArray>("landmarks_out", qos);
    gridPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_out", qos);
    posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("pose_out", qos);

    action_server_ = rclcpp_action::create_server<Action>(
        this,
        "landmark_filter",
        std::bind(&LandmarksNode::handle_goal, this, _1, _2),
        std::bind(&LandmarksNode::handle_cancel, this, _1),
        std::bind(&LandmarksNode::handle_accepted, this, _1)
    );
}


void LandmarksNode::landmarksRecievedCallback(const LandmarkArray::SharedPtr msg) {
    if (msg->landmarks.empty()) {
    return;
    }
    for (const auto &landmark : msg->landmarks) {
        RCLCPP_INFO(this->get_logger(), "Landmarks received");

        if (landmark.action == 0) {
            // Remove landmarks with matching id and landmark_type
            storedLandmarks_->landmarks.erase(
                std::remove_if(storedLandmarks_->landmarks.begin(), storedLandmarks_->landmarks.end(),
                               [&](const auto &storedLandmark) {
                                   return storedLandmark.id == landmark.id &&
                                          storedLandmark.landmark_type == landmark.landmark_type;
                               }),
                storedLandmarks_->landmarks.end());
        } else if (landmark.action == 1) {
            // Find if the landmark already exists
            auto it = std::find_if(storedLandmarks_->landmarks.begin(), storedLandmarks_->landmarks.end(),
                                   [&](const auto &storedLandmark) {
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
    posePublisher_->publish(gridVisualization_->poseArrayCreater(*storedLandmarks_));
}


rclcpp_action::GoalResponse LandmarksNode::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const vortex_msgs::action::FilteredLandmarks::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received request");
    (void)uuid;
  if (goal->distance < 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Distance must be non-negative, aborting goal");
    
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LandmarksNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>> goal_handle)
    {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
    }

    void LandmarksNode::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>> goal_handle)
    {
    // This needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&LandmarksNode::execute, this, _1), goal_handle}.detach();
    }



void LandmarksNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>> goal_handle)
    {
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Action::Feedback>();

    // frame_id from request
    std::string request_frame_id = goal->frame_id;
    _Float32 distance = goal->distance;

    if(distance == 0.0 && goal->landmark_types.empty()) {
        RCLCPP_INFO(this->get_logger(), "Received request to return all landmarks");
    }
    else if(goal->landmark_types.empty()) {
        RCLCPP_INFO(this->get_logger(), "Received request to return all landmarks within distance %f", distance);
    }
    else{
    // Log the request to return landmarks by type filter
    std::string types_log = "Received request to return landmarks by type filter: [";
    
    for (const auto& type : goal->landmark_types) {
        types_log += type + ", ";
    }

    // Remove the trailing comma and space
    if (!goal->landmark_types.empty()) {
        types_log = types_log.substr(0, types_log.size() - 2);
    }

    types_log += "]";
    
    RCLCPP_INFO(this->get_logger(), types_log.c_str());
    }
  

    // Filter the StoredLandmarks by landmark_types in the action request
    while (rclcpp::ok()) {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Goal canceled by client");
            return;
        }

        if (storedLandmarks_->landmarks.empty()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for landmarks to be detected");
            loop_rate.sleep();
            continue;
        }

        vortex_msgs::msg::OdometryArray filteredLandmarksOdoms;

        if (goal->landmark_types.empty()) {
            // Filter only by distance when landmark_types is empty
            for (const auto &landmark : storedLandmarks_->landmarks) {
                if (distance == 0.0 || calculateDistance(landmark.odom.pose.pose,landmark.odom.header.frame_id,request_frame_id) <= distance) {
                    filteredLandmarksOdoms.odoms.push_back(landmark.odom);
                }
            }
        } else {
            // Filter by both landmark_types and distance
            for (const auto &string : goal->landmark_types) {
                for (const auto &landmark : storedLandmarks_->landmarks) {
                    if ((distance == 0.0 && landmark.landmark_type == string) || 
                    (landmark.landmark_type == string && calculateDistance(landmark.odom.pose.pose,landmark.odom.header.frame_id,request_frame_id) <= distance)) {
                        filteredLandmarksOdoms.odoms.push_back(landmark.odom);
                    }
                }
            }
        }

        if (filteredLandmarksOdoms.odoms.empty()) {
            RCLCPP_INFO(this->get_logger(), "No landmarks found within after applying filter");
        }

        feedback->feedback = filteredLandmarksOdoms;

        // Publish the odometryArray as feedback
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publishing feedback");
        // adjust sleep timer based on needs or implement chrono timer
        loop_rate.sleep();
    }
}

double LandmarksNode::calculateDistance(const geometry_msgs::msg::Pose &pose, std::string target_frame, std::string source_frame)
{
    // Create a transform buffer
    tf2::BufferCore tf_buffer;

    // Create a transform stamped message
    geometry_msgs::msg::TransformStamped transform_stamped;

    try
    {
        // Lookup the transform from the input frame_id to the robot's frame_id
        transform_stamped = tf_buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform: %s", ex.what());
        return 0.0; // Return 0.0 or handle the error accordingly
    }

    // Transform the pose to the robot's frame of reference
    geometry_msgs::msg::Pose transformed_pose;
    tf2::doTransform(pose, transformed_pose, transform_stamped);

    // Calculate the distance between the transformed pose and the robot's pose
    double dx = transformed_pose.position.x;
    double dy = transformed_pose.position.y;
    double dz = transformed_pose.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}



} // namespace landmarks
