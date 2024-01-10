#ifndef GRID_VISUALIZATION_HPP
#define GRID_VISUALIZATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace grid_visualization {

class GridVisualization  {
public:
    explicit GridVisualization();

    ~GridVisualization() {};



    geometry_msgs::msg::PoseArray poseArrayCreater(vortex_msgs::msg::LandmarkArray landmarks);




};

}  // namespace GRID_VISUALIZATION

#endif // GRID_VISUALIZATION_HPP
