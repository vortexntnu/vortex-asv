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
#include "rclcpp_components/register_node_macro.hpp"
#include <vortex_msgs/msg/odometry_array.hpp>
#include <thread>


namespace landmarks {

class LandmarksNode : public rclcpp::Node {
public:
    explicit LandmarksNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~LandmarksNode() {};

protected:
    rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr subscription_;
    void landmarksRecievedCallback(const vortex_msgs::msg::LandmarkArray::SharedPtr msg);
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmarkPublisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr posePublisher_;
    std::shared_ptr<vortex_msgs::msg::LandmarkArray> storedLandmarks_;
    std::shared_ptr<grid_visualization::GridVisualization> gridVisualization_;

    rclcpp_action::Server<vortex_msgs::action::FilteredLandmarks>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const vortex_msgs::action::FilteredLandmarks::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredLandmarks>> goal_handle);
   
   
};

}  // namespace landmarks

#endif // LANDMARKS_HPP
