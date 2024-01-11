#ifndef LANDMARK_SERVER_HPP
#define LANDMARK_SERVER_HPP

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/buffer_core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vortex_msgs/action/filtered_landmarks.hpp>
#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/odometry_array.hpp>

namespace landmark_server {

/**
 * @class LandmarkServerNode
 * @brief A class representing a node for handling landmarks in a ROS 2 system.
 *
 * This class inherits from rclcpp::Node and provides functionality for
 * receiving landmark array messages, publishing the poses of the landmarks,
 * handling filtered landmarks using an action server, and calculating the
 * distance between the poses and the drone.
 */
class LandmarkServerNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the LandmarkServerNode class.
   *
   * @param options The options for configuring the node.
   */
  explicit LandmarkServerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for the LandmarkServerNode class.
   */
  ~LandmarkServerNode(){};

protected:
  /**
   * @brief A shared pointer to a subscription object for receiving
   * vortex_msgs::msg::LandmarkArray messages.
   */
  rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr
      subscription_;

  /**
   * @brief Callback function for receiving landmark array messages.
   *
   * This function is called when a landmark array message of type
   * vortex_msgs::msg::LandmarkArray::SharedPtr is received.
   *
   * @param msg The shared pointer to the received landmark array message.
   */
  void landmarksRecievedCallback(
      const vortex_msgs::msg::LandmarkArray::SharedPtr msg);

  /**
   * @brief A shared pointer to a publisher for the LandmarkArray message type.
   * Publishes all landmarks currently stored in the server.
   */
  rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr
      landmarkPublisher_;

  /**
   * @brief A shared pointer to a publisher for geometry_msgs::msg::PoseArray.
   * Publishes the pose of all landmarks currently stored in the server.
   */
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr posePublisher_;

  /**
   * @brief A shared pointer to a LandmarkArray message.
   * The array contains all landmarks currently stored by the server.
   */
  std::shared_ptr<vortex_msgs::msg::LandmarkArray> storedLandmarks_;

  /**
   * @brief A shared pointer to an rclcpp_action server for handling filtered
   * landmarks.
   */
  rclcpp_action::Server<vortex_msgs::action::FilteredLandmarks>::SharedPtr
      action_server_;

  /**
   * @brief Handles the goal request for the `handle_goal` function.
   *
   * This function is responsible for processing the goal request for the
   * `handle_goal` action.
   *
   * @param uuid The unique identifier of the goal request.
   * @param goal A shared pointer to the goal message containing the requested
   * goal.
   * @return The response to the goal request.
   */
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const vortex_msgs::action::FilteredLandmarks::Goal> goal);

  /**
   * @brief Handles the cancellation of a goal.
   *
   * This function is called when a goal is cancelled by the client.
   *
   * @param goal_handle The goal handle associated with the cancelled goal.
   * @return The response indicating the result of the cancellation.
   */
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                    vortex_msgs::action::FilteredLandmarks>>
                    goal_handle);

  /**
   * @brief Handles the accepted goal for the FilteredLandmarks action server.
   *
   * This function is called when a goal is accepted by the action server.
   *
   * @param goal_handle The goal handle for the accepted goal.
   */
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                           vortex_msgs::action::FilteredLandmarks>>
                           goal_handle);

  /**
   * @brief Executes the action server goal handle for the FilteredLandmarks
   * action.
   *
   * @param goal_handle The goal handle for the FilteredLandmarks action.
   */
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   vortex_msgs::action::FilteredLandmarks>>
                   goal_handle);

  /**
   * Calculates the distance between the landmark and the drone.
   *
   * @param pose The pose of the landmark.
   * @param target_frame The frame of the landmark.
   * @param source_frame The frame of the drone.
   * @return The distance between the landmark and the drone.
   */
  double calculateDistance(const geometry_msgs::msg::Pose &pose,
                           std::string target_frame, std::string source_frame);

  /**
   * @brief Logs messages based on request.
   *
   * This function is responsible for logging messages based on the request
   *
   * @param goal_handle A shared pointer to the goal handle.
   * @param distance The distance parameter.
   */
  void requestLogger(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                         vortex_msgs::action::FilteredLandmarks>>
                         goal_handle,
                     const double distance);

  /**
   * @brief Creates a pose array from a landmark array.
   * @param landmarks The landmark array.
   * @return The pose array.
   */
  geometry_msgs::msg::PoseArray
  poseArrayCreater(vortex_msgs::msg::LandmarkArray landmarks);
};

} // namespace landmark_server

#endif // LANDMARK_SERVER_HPP
