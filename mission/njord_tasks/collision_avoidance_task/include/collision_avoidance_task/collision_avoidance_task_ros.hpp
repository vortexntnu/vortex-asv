#ifndef COLLISION_AVOIDANCE_TASK_ROS_HPP
#define COLLISION_AVOIDANCE_TASK_ROS_HPP

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <njord_task_base/njord_task_base_ros.hpp>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // Required for tf2::doTransform

namespace collision_avoidance_task {

struct LandmarkOdomID {
  nav_msgs::msg::Odometry odom;
  int32_t id;
};
class CollisionAvoidanceTaskNode : public NjordTaskBaseNode {
public:
  explicit CollisionAvoidanceTaskNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~CollisionAvoidanceTaskNode(){};

  /**
   * @brief Main task for the CollisionAvoidanceTaskNode class.
   */
  void main_task();

  /**
   * @brief Predict the positions of the first two buoys
   *
   * @return Eigen::Array<double, 2, 2> predicted_positions
   */
  Eigen::Array<double, 2, 2> predict_first_buoy_pair();

  Eigen::Array<double, 2, 2>
  predict_second_buoy_pair(const geometry_msgs::msg::Point &buoy_0,
                           const geometry_msgs::msg::Point &buoy_1);

  void track_vessel(const Eigen::Vector2d &direction_vector_downwards,
                    const Eigen::Vector2d &direction_vector_forwards,
                    const geometry_msgs::msg::Point &waypoint_second_buoy_pair);

  nav_msgs::msg::Odometry get_vessel_odom();

  LandmarkOdomID
  filter_landmarks(const vortex_msgs::msg::LandmarkArray &landmarks,
                   const Eigen::Vector2d &direction_vector_downwards,
                   const Eigen::Vector2d &direction_vector_forwards,
                   const geometry_msgs::msg::Point &waypoint_second_buoy_pair);

  void vessel_collision_heading();

  double calculate_angle(const geometry_msgs::msg::Vector3 &twist1,
                         const geometry_msgs::msg::Vector3 &twist2);

  bool
  has_vessel_passed_freya(const Eigen::Vector2d &direction_vector_downwards);

  Eigen::Array<double, 2, 2>
  predict_third_buoy_pair(const geometry_msgs::msg::Point &buoy_0,
                          const geometry_msgs::msg::Point &buoy_1);

  Eigen::Array<double, 2, 2>
  predict_fourth_buoy_pair(const geometry_msgs::msg::Point &buoy_red,
                           const geometry_msgs::msg::Point &buoy_green);

private:
  mutable std::mutex vessel_odom_mutex_;
  bool new_vessel_odom_msg_ = false;
  nav_msgs::msg::Odometry vessel_odom_;
  std::condition_variable vessel_odom_cv_;
};

} // namespace collision_avoidance_task

#endif // COLLISION_AVOIDANCE_TASK_ROS_HPP