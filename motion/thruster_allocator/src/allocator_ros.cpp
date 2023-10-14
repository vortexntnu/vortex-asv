#include "thruster_allocator/allocator_ros.hpp"
#include "thruster_allocator/allocator_utils.hpp"
#include "thruster_allocator/pseudoinverse_allocator.hpp"
#include "vortex_msgs/msg/thruster_forces.hpp"

Allocator::Allocator() : Node("thrust_allocator_node") {
  subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>(
      "thrust/wrench_input", 1,
      std::bind(&Allocator::wrench_callback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<vortex_msgs::msg::ThrusterForces>(
      "thrust/thruster_forces", 1);

  Eigen::MatrixXd thrust_configuration_pseudoinverse;
  if (!calculatePseudoinverse(thrust_configuration, &thrust_configuration_pseudoinverse)) {
    RCLCPP_FATAL(get_logger(), "Failed to compute pseudoinverse of thrust config matrix. Killing node...");
    rclcpp::shutdown();
  }

  // Set T_pinv in m_pseudoinverse_allocator
  m_pseudoinverse_allocator.reset(
      new PseudoinverseAllocator(thrust_configuration_pseudoinverse));
}

void Allocator::spinOnce() {
  const Eigen::VectorXd body_frame_forces = wrenchMsgToEigen(
      body_frame_force_x, body_frame_force_y, body_frame_torque);

  Eigen::VectorXd thruster_forces =
      m_pseudoinverse_allocator->calculateAllocatedThrust(body_frame_forces);

  if(isInvalidMatrix(thruster_forces)){
    RCLCPP_ERROR(get_logger(), "ThrusterForces vector invalid");
    return;
  }

  // Saturates output vector between min and max values
  if(!saturateVectorValues(&thruster_forces, -100, 100)){
    RCLCPP_WARN(get_logger(), "Thruster forces vector required saturation.");
  }

  vortex_msgs::msg::ThrusterForces msg_out;
  arrayEigenToMsg(thruster_forces, &msg_out);
  // Number of thrusters = 4
  for (int i = 0; i < 4; i++){
    msg_out.thrust[i] *= m_direction[i];
  }
  publisher_->publish(msg_out);
}

void Allocator::wrench_callback(const geometry_msgs::msg::Wrench &msg) {
  const Eigen::VectorXd body_frame_forces = wrenchMsgToEigen(msg);
  if (!healthyWrench(body_frame_forces)) {
    RCLCPP_ERROR(get_logger(), "ASV wrench vector invalid, ignoring.");
    return;
  }
  body_frame_force_x = msg.force.x;
  body_frame_force_y = msg.force.y;
  body_frame_torque = msg.torque.z;
}

Eigen::VectorXd
Allocator::wrenchMsgToEigen(const geometry_msgs::msg::Wrench &msg) const {
  Eigen::VectorXd body_frame_forces(3); //(m_num_degrees_of_freedom = );
  body_frame_forces(0) = msg.force.x;   // surge
  body_frame_forces(1) = msg.force.y;   // sway
  body_frame_forces(2) = msg.torque.z;  // yaw
  return body_frame_forces;
}

Eigen::VectorXd Allocator::wrenchMsgToEigen(const float force_x,
                                            const float force_y,
                                            const float torque) const {
  Eigen::VectorXd body_frame_forces(3);
  body_frame_forces(0) = force_x; // surge
  body_frame_forces(1) = force_y; // sway
  body_frame_forces(2) = torque;  // yaw
  return body_frame_forces;
}

 bool Allocator::healthyWrench(const Eigen::VectorXd &v) const {
  // Check for NaN/Inf
  if (isInvalidMatrix(v))
    return false;

  // Check reasonableness
  for (unsigned i = 0; i < v.size(); ++i)
    if (std::abs(v[i]) > 100) //c_force_range_limit
      return false;

  return true;
 }