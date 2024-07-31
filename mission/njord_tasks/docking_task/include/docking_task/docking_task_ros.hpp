#ifndef DOCKING_TASK_ROS_HPP
#define DOCKING_TASK_ROS_HPP

#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <njord_task_base/njord_task_base_ros.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace docking_task {

struct LandmarkWithID {
  geometry_msgs::msg::Pose pose_odom_frame;
  geometry_msgs::msg::Pose pose_base_link_frame;
  int32_t id;
};

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

  /**
   * @brief Detect the closest buoy pair and set a waypoint between them
   *
   * @return A pair of landmarks representing the closest buoy pair
   */
  std::pair<LandmarkWithID, LandmarkWithID> initial_waypoint();

  /**
   * @brief Predict the 6-tuple formation of buoys
   *
   * @param landmark1 The first landmark/buoy used for initial waypoint
   * @param landmark2 The second landmark/buoy used for initial waypoint
   *
   * @return The predicted posistion of the 6-tuple formation of buoys in odom
   * frame
   */
  Eigen::Array<double, 2, 6>
  predict_buoy_formation(const LandmarkWithID &buoy1,
                         const LandmarkWithID &buoy2);

  /**
   * @brief Navigate the ASV through the formation of buoys. First travels to
   * waypoint between second pair of buoys, then navigates through the formation
   * of buoys and returns control when asv is 0.2m away from crossing the last
   * buoy pair.
   *
   * @param predicted_positions The predicted positions of the buoys
   * @return The ids of the last pair of buoys in the formation
   */
  std::pair<int32_t, int32_t>
  navigate_formation(Eigen::Array<double, 2, 6> &predicted_positions);

  /**
   * @brief Utility function that runs until waypoint is reached.
   * Function returns when within distance_threshold of the waypoint.
   *
   * @param distance_threshold The distance threshold for reaching the waypoint
   */
  void reach_waypoint(const double distance_threshold);

private:
  mutable std::mutex grid_mutex_;
  bool new_grid_msg_ = false;
  std::condition_variable grid_cond_var_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg_;
};

} // namespace docking_task

#endif // DOCKING_TASK_ROS_HPP