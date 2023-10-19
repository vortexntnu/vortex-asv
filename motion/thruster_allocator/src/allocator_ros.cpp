/**
 * @file allocator_ros.cpp
 * @brief This file contains the implementation of the Allocator class, which
 * allocates thrust to the ASV's thrusters based on the desired body frame
 * forces.
 */

#include <thruster_allocator/allocator_ros.hpp>
#include <thruster_allocator/allocator_utils.hpp>
#include <thruster_allocator/pseudoinverse_allocator.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

ThrusterAllocator::ThrusterAllocator()
    : Node("thruster_allocator_node"),
      pseudoinverse_allocator_(Eigen::MatrixXd::Zero(3, 4)) {
  subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>(
      "thrust/wrench_input", 1,
      std::bind(&ThrusterAllocator::wrench_callback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<vortex_msgs::msg::ThrusterForces>(
      "thrust/thruster_forces", 1);

  timer_ = this->create_wall_timer(100ms,
                                   std::bind(&ThrusterAllocator::timer_callback, this));

  Eigen::MatrixXd thrust_configuration_pseudoinverse;
  calculateRightPseudoinverse(thrust_configuration,
                              thrust_configuration_pseudoinverse);

  // Set T_pinv in pseudoinverse_allocator_
  pseudoinverse_allocator_.T_pinv = thrust_configuration_pseudoinverse;

  body_frame_forces_.setZero();
}

/**
 * @brief This function is called by the timer and calculates the allocated
 * thrust based on the body frame forces. It then saturates the output vector
 * between min and max values and publishes the thruster forces to the topic
 * "thrust/thruster_forces". If the calculated thruster forces vector is
 * invalid, it logs an error message. If the thruster forces vector required
 * saturation, it logs a warning message.
 */

void ThrusterAllocator::timer_callback() {
  Eigen::VectorXd thruster_forces =
      pseudoinverse_allocator_.calculateAllocatedThrust(body_frame_forces_);

  if (isInvalidMatrix(thruster_forces)) {
    RCLCPP_ERROR(get_logger(), "ThrusterForces vector invalid");
    return;
  }

  // Saturates output vector between min and max values
  if (!saturateVectorValues(thruster_forces, -100, 100)) {
    RCLCPP_WARN(get_logger(), "Thruster forces vector required saturation.");
  }

  vortex_msgs::msg::ThrusterForces msg_out;
  arrayEigenToMsg(thruster_forces, msg_out);
  // Number of thrusters = 4
  for (int i = 0; i < 4; i++) {
    msg_out.thrust[i] *= direction_[i];
  }
  publisher_->publish(msg_out);
}

/**
 * @brief Callback function for the wrench input subscription. It extracts the
 * surge, sway and yaw values from the received geometry_msgs::msg::Wrench
 * message and stores them in the body_frame_forces_ Eigen vector. If the wrench
 * vector is invalid (contains NaN or Inf values), it logs an error message and
 * returns.
 * @param msg The received geometry_msgs::msg::Wrench message.
 */
void ThrusterAllocator::wrench_callback(const geometry_msgs::msg::Wrench &msg) {
  body_frame_forces_(0) = msg.force.x;  // surge
  body_frame_forces_(1) = msg.force.y;  // sway
  body_frame_forces_(2) = msg.torque.z; // yaw
  if (!healthyWrench(body_frame_forces_)) {
    RCLCPP_ERROR(get_logger(), "ASV wrench vector invalid, ignoring.");
    return;
  }
}

/**
 * @brief Checks if the given Eigen vector is a healthy wrench vector.
 * A healthy wrench vector is one that does not contain NaN or Inf values, and
 * has reasonable values.
 * @param v The Eigen vector to check.
 * @return True if the vector is healthy, false otherwise.
 */
bool ThrusterAllocator::healthyWrench(const Eigen::VectorXd &v) const {
  // Check for NaN/Inf
  if (isInvalidMatrix(v))
    return false;

  // Check reasonableness
  for (unsigned i = 0; i < v.size(); ++i)
    if (std::abs(v[i]) > 100) // c_force_range_limit
      return false;

  return true;
}
