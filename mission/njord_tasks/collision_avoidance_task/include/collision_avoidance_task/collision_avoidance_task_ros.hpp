#ifndef COLLISION_AVOIDANCE_TASK_ROS_HPP
#define COLLISION_AVOIDANCE_TASK_ROS_HPP

#include <njord_task_base/njord_task_base_ros.hpp>

namespace collision_avoidance_task {

class CollisionAvoidanceTaskNode : public NjordTaskBaseNode {
public:
  explicit CollisionAvoidanceTaskNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~CollisionAvoidanceTaskNode(){};

  /**
   * @brief Main task for the CollisionAvoidanceTaskNode class.
   */
  void main_task();

private:
  geometry_msgs::msg::Point previous_waypoint_odom_frame_;

};
    
} // namespace collision_avoidance_task

#endif // COLLISION_AVOIDANCE_TASK_ROS_HPP