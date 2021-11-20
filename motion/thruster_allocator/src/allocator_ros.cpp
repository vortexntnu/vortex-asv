#include "vortex_allocator/allocator_ros.h"

#include <cmath>
#include <vector>
#include <limits>

#include <eigen_conversions/eigen_msg.h>
#include "vortex_msgs/ThrusterForces.h"
#include "vortex_allocator/eigen_typedefs.h"
#include "vortex_allocator/eigen_helper.h"



Allocator::Allocator(ros::NodeHandle nh)
  :
  m_nh(nh),
  m_min_thrust(-std::numeric_limits<double>::infinity()),
  m_max_thrust(std::numeric_limits<double>::infinity())
{
  std::string sub_topic;
  std::string pub_topic;
  m_nh.getParam("/asv/thruster_manager/input",sub_topic);
  m_nh.getParam("/asv/thruster_manager/thruster_forces",pub_topic);
  m_sub = m_nh.subscribe(sub_topic, 1, &Allocator::forceWrenchCallback, this);   //MUST BE CHANGED when we have the other nodes
  m_pub = m_nh.advertise<vortex_msgs::ThrusterForces>(pub_topic, 1);

  if (!m_nh.getParam("/propulsion/dofs/num", m_num_degrees_of_freedom))
    ROS_FATAL("Failed to read parameter number of dofs.");
  if (!m_nh.getParam("/propulsion/thrusters/num", m_num_thrusters))
    ROS_FATAL("Failed to read parameter number of thrusters.");
  if (!m_nh.getParam("/propulsion/thrusters/direction", m_direction))
  {
    ROS_WARN("Failed to read parameter thruster direction.");
    std::fill(m_direction.begin(), m_direction.begin() + m_num_thrusters, 1);
  }

  // Read thrust limits
  std::vector<double> thrust;
  if (!m_nh.getParam("/propulsion/thrusters/characteristics/thrust", thrust))
  {
    ROS_WARN("Failed to read params min/max thrust. Using (%.2f) to (%.2f).",
      m_min_thrust, m_max_thrust);
  }

  // Read thrust config matrix
  Eigen::MatrixXd thrust_configuration;
  if (!getMatrixParam(m_nh, "/propulsion/thrusters/configuration_matrix", &thrust_configuration))
  {
    ROS_FATAL("Failed to read parameter thrust config matrix. Killing node...");
    ros::shutdown();
  }
  Eigen::MatrixXd thrust_configuration_pseudoinverse;
  if (!pseudoinverse(thrust_configuration, &thrust_configuration_pseudoinverse))
  {
    ROS_FATAL("Failed to compute pseudoinverse of thrust config matrix. Killing node...");
    ros::shutdown();
  }

  m_pseudoinverse_allocator.reset(new PseudoinverseAllocator(thrust_configuration_pseudoinverse));
  ROS_INFO("Initialized.");
}

void Allocator::forceWrenchCallback(const geometry_msgs::Wrench &msg_in) const
{
  const Eigen::VectorXd body_frame_forces = WrenchMsgToEigen(msg_in);

  if (!healthyWrench(body_frame_forces))
  {
    ROS_ERROR("ROV forces vector invalid, ignoring.");
    return;
  }

  Eigen::VectorXd thruster_forces = m_pseudoinverse_allocator->compute(body_frame_forces);

  if (isFucked(thruster_forces))
  {
    ROS_ERROR("Thruster forces vector invalid, ignoring.");
    return;
  }

  if (!saturateVector(&thruster_forces, m_min_thrust, m_max_thrust))
    ROS_WARN_THROTTLE(1, "Thruster forces vector required saturation.");

  vortex_msgs::ThrusterForces msg_out;
  arrayEigenToMsg(thruster_forces, &msg_out);

  for (int i = 0; i < m_num_thrusters; i++)
    msg_out.thrust[i] *= m_direction[i];

  msg_out.header.stamp = ros::Time::now();
  m_pub.publish(msg_out);
}

Eigen::VectorXd Allocator::WrenchMsgToEigen(const geometry_msgs::Wrench &msg) const
{
  Eigen::VectorXd body_frame_forces(m_num_degrees_of_freedom);
  body_frame_forces(0) = msg.force.x;   //surge
  body_frame_forces(1) = msg.force.y;   //sway
  body_frame_forces(2) = msg.torque.z;  //yaw
  return body_frame_forces;
}

bool Allocator::healthyWrench(const Eigen::VectorXd &v) const
{
  // Check for NaN/Inf
  if (isFucked(v))
    return false;

  // Check reasonableness
  for (unsigned i = 0; i < v.size(); ++i)
    if (std::abs(v[i]) > c_force_range_limit)
      return false;

  return true;
}
