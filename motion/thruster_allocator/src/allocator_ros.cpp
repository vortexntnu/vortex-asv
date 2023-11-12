#include "thruster_allocator/allocator_ros.hpp"
#include "thruster_allocator/allocator_utils.hpp"
#include "thruster_allocator/pseudoinverse_allocator.hpp"
#include <vortex_msgs/msg/thruster_forces.hpp>

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

ThrusterAllocator::ThrusterAllocator()
    : Node("thruster_allocator_node"),
      pseudoinverse_allocator_(Eigen::MatrixXd::Zero(3, 4)) {
  declare_parameter("propulsion.dofs.num", 3);
  declare_parameter("propulsion.thrusters.num", 4);
  declare_parameter("propulsion.thrusters.min", -100);
  declare_parameter("propulsion.thrusters.max", 100);
  declare_parameter("propulsion.thrusters.direction", std::vector<int64_t>{0});
  declare_parameter("propulsion.thrusters.configuration_matrix",
                    std::vector<double>{0});

  num_dof_ = get_parameter("propulsion.dofs.num").as_int();
  num_thrusters_ = get_parameter("propulsion.thrusters.num").as_int();
  min_thrust_ = get_parameter("propulsion.thrusters.min").as_int();
  max_thrust_ = get_parameter("propulsion.thrusters.max").as_int();
  direction_ =
      get_parameter("propulsion.thrusters.direction").as_integer_array();
  thrust_configuration = doubleArrayToEigenMatrix(
      get_parameter("propulsion.thrusters.configuration_matrix")
          .as_double_array(),
      num_dof_, num_thrusters_);

  subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>(
      "thrust/wrench_input", 1,
      std::bind(&ThrusterAllocator::wrench_callback, this,
                std::placeholders::_1));

  publisher_ = this->create_publisher<vortex_msgs::msg::ThrusterForces>(
      "thrust/thruster_forces", 1);

  timer_ = this->create_wall_timer(
      100ms, std::bind(&ThrusterAllocator::timer_callback, this));

  pseudoinverse_allocator_.T_pinv = calculateRightPseudoinverse(thrust_configuration);
  
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
  std::transform(msg_out.thrust.begin(), msg_out.thrust.end(),
                 direction_.begin(), msg_out.thrust.begin(),
                 std::multiplies<>());
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

  bool within_max_thrust = std::none_of(v.begin(), v.end(), [this](double val) {
    return std::abs(val) > max_thrust_;
  });

  return within_max_thrust;
}
