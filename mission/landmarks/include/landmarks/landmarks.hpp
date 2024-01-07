#ifndef LANDMARKS_HPP
#define LANDMARKS_HPP

#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/msg/Landmark.msg>
// #include <vortex_msgs/msg/LandmarkArray.msg>
#include <nav_msgs/msg/OccupancyGrid.msg>
#include <vector>  

namespace landmarks {

class LandmarksNode : public rclcpp::Node {
public:
    LandmarksNode();

    ~LandmarksNode() {};

protected:
    rclcpp::Subscription<std::vector<vortex_msgs::msg::Landmark>>::SharedPtr subscription_;
    void landmarksRecievedCallback(const std::vector<vortex_msgs::msg::Landmark::SharedPtr> msg);
    rclcpp::Publisher<std::vector<vortex_msgs::msg::Landmark>::SharedPtr> landmarkPublisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridPublisher_;
    std::vector<vortex_msgs::msg::Landmark> storedLandmarks_;
    nav_msgs::msg::OccupancyGrid createGrid(std::vector<vortex_msgs::msg::Landmark> landmarks);
    nav_msgs::msg::OccupancyGrid grid_;
    // maybe use timer based publishing?
    void publishLandmarks();
};

}  // namespace landmarks

#endif // LANDMARKS_HPP
