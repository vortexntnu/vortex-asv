#ifndef DOCKING_TASK_ROS_HPP
#define DOCKING_TASK_ROS_HPP

#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <njord_task_base/njord_task_base_ros.hpp>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace docking_task {

/**
 * @class DockingTaskNode
 * @brief A class representing a node for handling dock localization in a ROS 2
 * system.
 *
 * This class inherits from rclcpp::Node and provides functionality for
 * localizing the dock in the map.
 */
class DockingTaskNode : public NjordTaskBaseNode {
public:
  /**
   * @brief Constructor for the DockingTaskNode class.
   *
   * @param options The options for configuring the node.
   */
  explicit DockingTaskNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for the DockLocalizationNode class.
   */
  ~DockingTaskNode(){};

  /**
   * @brief Main task for the DockingTaskNode class.
   */
  void main_task();

  Eigen::Array<double, 2, 2> predict_first_buoy_pair();

  Eigen::Array<double, 2, 2>
  predict_second_buoy_pair(const geometry_msgs::msg::Point &buoy_0,
                           const geometry_msgs::msg::Point &buoy_1);

  Eigen::Array<double, 2, 2>
  predict_third_buoy_pair(const geometry_msgs::msg::Point &buoy_0,
                          const geometry_msgs::msg::Point &buoy_1,
                          const geometry_msgs::msg::Point &buoy_2,
                          const geometry_msgs::msg::Point &buoy_3);

  void grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  std::shared_ptr<nav_msgs::msg::OccupancyGrid> get_grid();

  void initialize_grid_sub();

  void find_dock_structure_edges();

  /**
   * @brief Iterate over line in grid map. Returns a vector of booleans
   * representing occupied cells along the line.
   *
   * @param grid The grid map.
   * @param x0 The x-coordinate of the start point.
   * @param y0 The y-coordinate of the start point.
   * @param x1 The x-coordinate of the end point.
   * @param y1 The y-coordinate of the end point.
   * @return A vector of booleans representing occupied cells along the line.
   */
  std::vector<bool> search_line(const nav_msgs::msg::OccupancyGrid &grid, double x0, double y0, double x1, double y1);


private:
  mutable std::mutex grid_mutex_;
  bool new_grid_msg_ = false;
  std::condition_variable grid_cond_var_;
  nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
};

} // namespace docking_task

#endif // DOCKING_TASK_ROS_HPP