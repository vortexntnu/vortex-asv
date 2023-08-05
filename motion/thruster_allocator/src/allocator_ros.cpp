#include "vortex_allocator/allocator_ros.h"

#include <cmath>
#include <limits>
#include <vector>

#include "vortex_allocator/eigen_helper.h"
#include "vortex_msgs/ThrusterForces.h"
#include <eigen_conversions/eigen_msg.h>

Allocator::Allocator(ros::NodeHandle nh)
    : m_nh(nh), m_min_thrust(-std::numeric_limits<double>::infinity()),
      m_max_thrust(std::numeric_limits<double>::infinity()) {
  std::string torque_topic;
  std::string force_topic;
  std::string pub_topic;

  m_nh.getParam("/asv/thruster_manager/torque", torque_topic);
  m_nh.getParam("/asv/thruster_manager/force", force_topic);
  m_nh.getParam("/asv/thruster_manager/output", pub_topic);

  m_sub_torque =
      m_nh.subscribe(torque_topic, 1, &Allocator::torqueWrenchCallback, this);
  m_sub_force =
      m_nh.subscribe(force_topic, 1, &Allocator::forceWrenchCallback, this);

  m_pub = m_nh.advertise<vortex_msgs::ThrusterForces>(pub_topic, 1);

  if (!m_nh.getParam("/propulsion/dofs/num", m_num_degrees_of_freedom))
    ROS_FATAL("Failed to read parameter number of dofs.");
  if (!m_nh.getParam("/propulsion/thrusters/num", m_num_thrusters))
    ROS_FATAL("Failed to read parameter number of thrusters.");
  if (!m_nh.getParam("/propulsion/thrusters/direction", m_direction)) {
    ROS_WARN("Failed to read parameter thruster direction.");
    std::fill(m_direction.begin(), m_direction.begin() + m_num_thrusters, 1);
  }

  // Read thrust limits
  std::vector<double> thrust;
  if (!m_nh.getParam("/propulsion/thrusters/characteristics/thrust", thrust)) {
    ROS_WARN("Failed to read params min/max thrust. Using (%.2f) to (%.2f).",
             m_min_thrust, m_max_thrust);
  }

  // Read thrust config matrix
  Eigen::MatrixXd thrust_configuration;
  if (!getMatrixParam(m_nh, "/propulsion/thrusters/configuration_matrix",
                      &thrust_configuration)) {
    ROS_FATAL("Failed to read parameter thrust config matrix. Killing node...");
    ros::shutdown();
  }
  Eigen::MatrixXd thrust_configuration_pseudoinverse;
  if (!pseudoinverse(thrust_configuration,
                     &thrust_configuration_pseudoinverse)) {
    ROS_FATAL("Failed to compute pseudoinverse of thrust config matrix. "
              "Killing node...");
    ros::shutdown();
  }

  m_pseudoinverse_allocator.reset(
      new PseudoinverseAllocator(thrust_configuration_pseudoinverse));
  ROS_INFO("Initialized.");
}

void Allocator::spinOnce() {
  const Eigen::VectorXd body_frame_forces = wrenchMsgToEigen(
      body_frame_force_x, body_frame_force_y, body_frame_torque);

  Eigen::VectorXd thruster_forces =
      m_pseudoinverse_allocator->compute(body_frame_forces);

  if (isFucked(thruster_forces)) {
    ROS_ERROR("Thruster forces vector invalid, ignoring.");
    return;
  }

  if (!saturateVector(&thruster_forces, m_min_thrust, m_max_thrust))
    ROS_WARN_THROTTLE(1, "Thruster forces vector required saturation.");
  //std::cout << thruster_forces << std::endl;
  vortex_msgs::ThrusterForces msg_out;
  arrayEigenToMsg(thruster_forces, &msg_out);

  for (int i = 0; i < m_num_thrusters; i++)
    msg_out.thrust[i] *= m_direction[i];

  msg_out.header.stamp = ros::Time::now();
  m_pub.publish(msg_out);
}

void Allocator::forceWrenchCallback(const geometry_msgs::Wrench &msg_in) {
  const Eigen::VectorXd body_frame_forces = wrenchMsgToEigen(msg_in);

  if (!healthyWrench(body_frame_forces)) {
    ROS_ERROR("ASV forces vector invalid, ignoring.");
    return;
  }

  body_frame_force_x = msg_in.force.x;
  body_frame_force_y = msg_in.force.y;
}

void Allocator::torqueWrenchCallback(const geometry_msgs::Wrench &msg_in) {
  const Eigen::VectorXd body_frame_forces = wrenchMsgToEigen(msg_in);

  if (!healthyWrench(body_frame_forces)) {
    ROS_ERROR("ASV torque vector invalid, ignoring.");
    return;
  }

  body_frame_torque = msg_in.torque.z;
}

Eigen::VectorXd
Allocator::wrenchMsgToEigen(const geometry_msgs::Wrench &msg) const {
  Eigen::VectorXd body_frame_forces(m_num_degrees_of_freedom);
  body_frame_forces(0) = msg.force.x;  // surge
  body_frame_forces(1) = msg.force.y;  // sway
  body_frame_forces(2) = msg.torque.z; // yaw
  return body_frame_forces;
}

Eigen::VectorXd Allocator::wrenchMsgToEigen(const float force_x,
                                            const float force_y,
                                            const float torque) const {
  Eigen::VectorXd body_frame_forces(m_num_degrees_of_freedom);
  body_frame_forces(0) = force_x; // surge
  body_frame_forces(1) = force_y; // sway
  body_frame_forces(2) = torque;  // yaw
  return body_frame_forces;
}

bool Allocator::healthyWrench(const Eigen::VectorXd &v) const {
  // Check for NaN/Inf
  if (isFucked(v))
    return false;

  // Check reasonableness
  for (unsigned i = 0; i < v.size(); ++i)
    if (std::abs(v[i]) > c_force_range_limit)
      return false;

  return true;
}
