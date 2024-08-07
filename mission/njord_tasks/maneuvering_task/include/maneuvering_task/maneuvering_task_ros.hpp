#ifndef MANEUVERING_TASK_ROS_HPP
#define MANEUVERING_TASK_ROS_HPP

#include <njord_task_base/njord_task_base_ros.hpp>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace maneuvering_task {

class ManeuveringTaskNode : public NjordTaskBaseNode {
public:
  explicit ManeuveringTaskNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~ManeuveringTaskNode(){};

  /**
   * @brief Main task for the ManeuveringTaskNode class.
   */
  void main_task();

  Eigen::Array<double, 2, 2> predict_first_buoy_pair();

  Eigen::Array<double, 2, 2>
  predict_second_buoy_pair(const geometry_msgs::msg::Point &buoy_0,
                           const geometry_msgs::msg::Point &buoy_1);

  Eigen::Array<double, 2, 2>
  predict_next_pair_in_formation(const geometry_msgs::msg::Point &buoy_red,
                                 const geometry_msgs::msg::Point &buoy_green,
                                 Eigen::Vector2d direction_vector);

private:
};

} // namespace maneuvering_task

#endif // MANEUVERING_TASK_ROS_HPP