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
      std::bind(&ThrusterAllocator::wrench_callback, this,
                std::placeholders::_1));

  publisher_ = this->create_publisher<vortex_msgs::msg::ThrusterForces>(
      "thrust/thruster_forces", 1);

  timer_ = this->create_wall_timer(
      100ms, std::bind(&ThrusterAllocator::timer_callback, this));

  Eigen::MatrixXd thrust_configuration_pseudoinverse;
  calculateRightPseudoinverse(thrust_configuration,
                              thrust_configuration_pseudoinverse);

  pseudoinverse_allocator_.T_pinv = thrust_configuration_pseudoinverse;

  body_frame_forces_.setZero();
}

void ThrusterAllocator::timer_callback() {
  Eigen::VectorXd thruster_forces =
      pseudoinverse_allocator_.calculateAllocatedThrust(body_frame_forces_);

  if (isInvalidMatrix(thruster_forces)) {
    RCLCPP_ERROR(get_logger(), "ThrusterForces vector invalid");
    return;
  }

  if (!saturateVectorValues(thruster_forces, min_thrust_, max_thrust_)) {
    RCLCPP_WARN(get_logger(), "Thruster forces vector required saturation.");
  }

  vortex_msgs::msg::ThrusterForces msg_out;
  arrayEigenToMsg(thruster_forces, msg_out);
  for (int i = 0; i < num_thrusters_; i++) {
    msg_out.thrust[i] *= direction_[i];
  }
  publisher_->publish(msg_out);
}

void ThrusterAllocator::wrench_callback(const geometry_msgs::msg::Wrench &msg) {
  Eigen::Vector3d msg_vector;
  msg_vector(0) = msg.force.x;  // surge
  msg_vector(1) = msg.force.y;  // sway
  msg_vector(2) = msg.torque.z; // yaw
  if (!healthyWrench(msg_vector)) {
    RCLCPP_ERROR(get_logger(), "ASV wrench vector invalid, ignoring.");
    return;
  }
  std::swap(msg_vector, body_frame_forces_);
}

bool ThrusterAllocator::healthyWrench(const Eigen::VectorXd &v) const {
  if (isInvalidMatrix(v))
    return false;

  for (unsigned i = 0; i < v.size(); ++i)
    if (std::abs(v[i]) > max_thrust_)
      return false;

  return true;
}
