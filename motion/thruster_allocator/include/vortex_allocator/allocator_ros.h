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
   * @brief Callback function for converting force wrench in body frame to
   * ThrusterForces msg
   *
   * @param msg Wrench message containing linear and angular forces in BODY
   * frame
   */
  void forceWrenchCallback(const geometry_msgs::Wrench &msg) const;

private:
  ros::NodeHandle m_nh;
  ros::Subscriber m_sub;
  ros::Publisher m_pub;

  int m_num_degrees_of_freedom;
  int m_num_thrusters;
  std::vector<int> m_direction;
  double m_min_thrust;
  double m_max_thrust;
  const double c_force_range_limit = 100;

  std::unique_ptr<PseudoinverseAllocator> m_pseudoinverse_allocator;

  Eigen::VectorXd WrenchMsgToEigen(const geometry_msgs::Wrench &msg) const;
  bool healthyWrench(const Eigen::VectorXd &v) const;
};

#endif // VORTEX_ALLOCATOR_ALLOCATOR_ROS_H
