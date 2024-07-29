#ifndef MANEUVERING_TASK_ROS_HPP
#define MANEUVERING_TASK_ROS_HPP

#include <njord_task_base/njord_task_base_ros.hpp>

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

private:
  geometry_msgs::msg::Point previous_waypoint_odom_frame_;
};

} // namespace maneuvering_task

#endif // MANEUVERING_TASK_ROS_HPP