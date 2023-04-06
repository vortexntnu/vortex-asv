#ifndef VORTEX_ALLOCATOR_ALLOCATOR_ROS_H
#define VORTEX_ALLOCATOR_ALLOCATOR_ROS_H

#include "geometry_msgs/Wrench.h"
#include "ros/ros.h"

#include "vortex_allocator/pseudoinverse_allocator.h"

#include <map>
#include <string>
#include <vector>

class Allocator {
public:
  explicit Allocator(ros::NodeHandle nh);
  /**
   * @brief Callback function for getting force wrench
   *
   * @param msg Wrench message containing linear forces in BODY
   * frame
   */
  void forceWrenchCallback(const geometry_msgs::Wrench &msg);

  /**
   * @brief Callback function for getting torque wrench
   *
   * @param msg Wrench message containing angular forces in BODY
   * frame
   */
  void torqueWrenchCallback(const geometry_msgs::Wrench &msg);

  /**
   * @brief Function for periodically converting force & torque wrench in body
   * frame to ThrusterForces msg
   *
   */
  void spinOnce();

private:
  ros::NodeHandle m_nh;
  ros::Subscriber m_sub_torque;
  ros::Subscriber m_sub_force;
  ros::Publisher m_pub;

  int m_num_degrees_of_freedom;
  int m_num_thrusters;

  std::vector<int> m_direction;
  double m_min_thrust;
  double m_max_thrust;
  const double c_force_range_limit = 100;

  float body_frame_force_x = 0.0;
  float body_frame_force_y = 0.0;
  float body_frame_torque = 0.0;

  std::unique_ptr<PseudoinverseAllocator> m_pseudoinverse_allocator;

  Eigen::VectorXd wrenchMsgToEigen(const geometry_msgs::Wrench &msg) const;
  Eigen::VectorXd wrenchMsgToEigen(const float force_x, const float force_y,
                                   const float torque) const;
  bool healthyWrench(const Eigen::VectorXd &v) const;
};

#endif // VORTEX_ALLOCATOR_ALLOCATOR_ROS_H
