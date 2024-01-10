#ifndef LANDMARKS_HPP
#define LANDMARKS_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <landmarks/grid_visualization.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex_msgs/action/filtered_landmarks.hpp>
#include <vortex_msgs/msg/odometry_array.hpp>
#include <tf2/buffer_core.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>


namespace landmarks {

/**
 * @class LandmarksNode
 * @brief A class representing a node for handling landmarks in a ROS 2 system.
 *
 * This class inherits from rclcpp::Node and provides functionality for receiving landmark array messages,
 * publishing landmark array messages, publishing occupancy grid messages, publishing pose array messages,
 * handling filtered landmarks using an action server, and calculating the distance between poses.
 */
class LandmarksNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the LandmarksNode class.
     *
     * @param options The options for configuring the node.
     */
    explicit LandmarksNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Destructor for the LandmarksNode class.
     */
    ~LandmarksNode() {};

protected:
    /**
     * @brief A shared pointer to a subscription object for receiving vortex_msgs::msg::LandmarkArray messages.
     */
    rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr subscription_;

    /**
     * @brief Callback function for receiving landmark array messages.
     *
     * This function is called when a landmark array message of type vortex_msgs::msg::LandmarkArray::SharedPtr is received.
     *
     * @param msg The shared pointer to the received landmark array message.
     */
    void landmarksRecievedCallback(const vortex_msgs::msg::LandmarkArray::SharedPtr msg);

    /**
     * @brief A shared pointer to a publisher for the LandmarkArray message type.
     */
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmarkPublisher_;

    /**
     * @brief Publisher for sending occupancy grid messages.
     */
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridPublisher_;

    /**
     * @brief A shared pointer to a publisher for geometry_msgs::msg::PoseArray.
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr posePublisher_;

    /**
     * @brief A shared pointer to a LandmarkArray message.
     */
    std::shared_ptr<vortex_msgs::msg::LandmarkArray> storedLandmarks_;

    /**
     * @brief A shared pointer to a GridVisualization object.
     */
    std::shared_ptr<grid_visualization::GridVisualization> gridVisualization_;

    /**
     * @brief A shared pointer to an rclcpp_action server for handling filtered landmarks.
     */
    rclcpp_action::Server<vortex_msgs::action::FilteredLandmarks>::SharedPtr action_server_;

    /**
     * @brief Handles the goal request for the `handle_goal` function.
     *
     * This function is responsible for processing the goal request for the `handle_goal` action.
     *
     * @param uuid The unique identifier of the goal request.
     * @param goal A shared pointer to the goal message containing the requested goal.
     * @return The response to the goal request.
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const vortex_msgs::action::FilteredLandmarks::Goal> goal);

    /**
     * @brief Handles the cancellation of a goal.
     *
     * This function is called when a goal is cancelled by the client.
     *
     * @param goal_handle The goal handle associated with the cancelled goal.
     * @return The response indicating the result of the cancellation.
     */
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>> goal_handle);

    /**
     * @brief Handles the accepted goal for the FilteredLandmarks action server.
     *
     * This function is called when a goal is accepted by the action server.
     *
     * @param goal_handle The goal handle for the accepted goal.
     */
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>> goal_handle);

    /**
     * @brief Executes the action server goal handle for the FilteredLandmarks action.
     *
     * @param goal_handle The goal handle for the FilteredLandmarks action.
     */
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>> goal_handle);

    /**
     * @brief Calculates the distance between the given pose and the current pose.
     *
     * @param pose The pose to calculate the distance from.
     * @return The distance between the given pose and the current pose.
     */
    double calculateDistance(const geometry_msgs::msg::Pose &pose,std::string target_frame,std::string source_frame);
};

}  // namespace landmarks

#endif // LANDMARKS_HPP
