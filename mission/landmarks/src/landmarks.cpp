#include <landmarks/landmarks.hpp>

using std::placeholders::_1, std::placeholders::_2;
using LandmarkArray = vortex_msgs::msg::LandmarkArray;
using Action = vortex_msgs::action::FilteredLandmarks;

namespace landmarks {

LandmarksNode::LandmarksNode(const rclcpp::NodeOptions &options)
    : Node("landmarks_node", options) {
  // Define the quality of service profile for publisher and subscriber
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1),
                         qos_profile);
  storedLandmarks_ = std::make_shared<LandmarkArray>();
  gridVisualization_ =
      std::make_shared<grid_visualization::GridVisualization>();
  subscription_ = this->create_subscription<LandmarkArray>(
      "landmarks_in", qos,
      std::bind(&LandmarksNode::landmarksRecievedCallback, this, _1));
  landmarkPublisher_ =
      this->create_publisher<LandmarkArray>("landmarks_out", qos);
  gridPublisher_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_out", qos);

  posePublisher_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>("pose_out", qos);

  action_server_ = rclcpp_action::create_server<Action>(
      this, "landmark_filter",
      std::bind(&LandmarksNode::handle_goal, this, _1, _2),
      std::bind(&LandmarksNode::handle_cancel, this, _1),
      std::bind(&LandmarksNode::handle_accepted, this, _1));


}

void LandmarksNode::landmarksRecievedCallback(

    const LandmarkArray::SharedPtr msg) {
  for (const auto &landmark : msg->landmarks) {
    RCLCPP_INFO(this->get_logger(), "Landmarks received");
    if (landmark.action == 0) {
      for (int i = 0; i < storedLandmarks_->landmarks.size(); i++) {
        if ((storedLandmarks_)->landmarks[i].id == landmark.id &&
            (storedLandmarks_)->landmarks[i].landmark_type ==
                landmark.landmark_type) {
          // Remove landmark if namespace and id match
          (storedLandmarks_->landmarks)
              .erase(storedLandmarks_->landmarks.begin() + i);

          break;
        }
      }
    } else if (landmark.action == 1) {
      bool landmarkUpdated = false;

      for (auto &storedLandmark : storedLandmarks_->landmarks) {
        if (landmark.landmark_type == storedLandmark.landmark_type &&
            landmark.id == storedLandmark.id) {
          storedLandmark = landmark;
          landmarkUpdated = true;
          break;
        }
      }

      if (!landmarkUpdated && landmark.action == 1) {
        // If no match is found and action is 1, add the new landmark
        (storedLandmarks_->landmarks).push_back(landmark);
      }
    }
  }
  // Publish the landmarks
  landmarkPublisher_->publish(*storedLandmarks_);
  gridPublisher_->publish(gridVisualization_->createGrid(*storedLandmarks_));
  posePublisher_->publish(
      gridVisualization_->poseArrayCreater(*storedLandmarks_));
}

rclcpp_action::GoalResponse LandmarksNode::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const vortex_msgs::action::FilteredLandmarks::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->filter);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LandmarksNode::handle_cancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>>
        goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LandmarksNode::handle_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>>
        goal_handle) {
             // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&LandmarksNode::execute, this, _1), goal_handle}.detach();

        } 



void LandmarksNode::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>> goal_handle)
{
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Action::Feedback>();
    auto &sequence = feedback->feedback;
    // frame_id from request
    std::string frame_id = goal->filter.frame_id;

    // Filter the StoredLandmarks by landmark_types in the action request
    vortex_msgs::msg::OdometryArray filteredLandmarks;
    for(const auto string : goal->filter.landmark_types){
        RCLCPP_INFO(this->get_logger(), "Received request to filter landmarks of type %s", string.c_str());
    
    for (const auto &landmark : storedLandmarks_->landmarks) {
        if (landmark.landmark_type == string){
            filteredLandmarks.odoms.push_back(landmark.odom);
        }
    }
    }
    feedback->feedback = filteredLandmarks;

    // Publish the odometryArray as feedback
    goal_handle->publish_feedback(feedback);
    }

    // Continue executing the goal
    // ...


    



} // namespace landmarks