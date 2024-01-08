#ifndef LANDMARKS_HPP
#define LANDMARKS_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vector>  
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

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
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twistPublisher_;
    std::shared_ptr<vortex_msgs::msg::LandmarkArray> storedLandmarks_;
    nav_msgs::msg::OccupancyGrid createGrid(vortex_msgs::msg::LandmarkArray landmarks);
    nav_msgs::msg::OccupancyGrid grid_;
    // maybe use timer based publishing?
    // void publishLandmarks();
    void updateGrid(vortex_msgs::msg::Landmark landmark,int number);
    geometry_msgs::msg::PoseArray poseArrayCreater(vortex_msgs::msg::LandmarkArray landmarks);
    void twistCreater(vortex_msgs::msg::LandmarkArray landmarks);
    
};

}  // namespace landmarks

#endif // LANDMARKS_HPP
