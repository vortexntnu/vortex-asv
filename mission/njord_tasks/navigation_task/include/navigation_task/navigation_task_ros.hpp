#ifndef NAVIGATION_TASK_ROS_HPP
#define NAVIGATION_TASK_ROS_HPP

#include <njord_task_base/njord_task_base_ros.hpp>

namespace navigation_task {

class NavigationTaskNode : public NjordTaskBaseNode {
public:
  explicit NavigationTaskNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~NavigationTaskNode(){};

  /**
   * @brief Main task for the NavigationTaskNode class.
   */
  void main_task();

private:
  geometry_msgs::msg::Point previous_waypoint_odom_frame_;
};

} // namespace navigation_task

#endif // NAVIGATION_TASK_ROS_HPP