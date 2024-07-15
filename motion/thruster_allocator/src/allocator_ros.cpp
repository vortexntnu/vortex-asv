#include "thruster_allocator/allocator_ros.hpp"
#include "thruster_allocator/allocator_utils.hpp"
#include "thruster_allocator/pseudoinverse_allocator.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>
#include <functional>
#include <lifecycle_msgs/msg/state.hpp>

using namespace std::chrono_literals;

ThrusterAllocator::ThrusterAllocator()
   : rclcpp_lifecycle::LifecycleNode("thruster_allocator_node"),
     pseudoinverse_allocator_(Eigen::MatrixXd::Zero(3, 4)) {
   declare_parameter("propulsion.dofs.num", 3);
   declare_parameter("propulsion.thrusters.num", 4);
   declare_parameter("propulsion.thrusters.min", -100);
   declare_parameter("propulsion.thrusters.max", 100);
   declare_parameter("propulsion.thrusters.configuration_matrix", std::vector<double>{0});
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ThrusterAllocator::on_configure(const rclcpp_lifecycle::State &) {
   num_dof_ = get_parameter("propulsion.dofs.num").as_int();
   num_thrusters_ = get_parameter("propulsion.thrusters.num").as_int();
   min_thrust_ = get_parameter("propulsion.thrusters.min").as_int();
   max_thrust_ = get_parameter("propulsion.thrusters.max").as_int();
   thrust_configuration = double_array_to_eigen_matrix(
       get_parameter("propulsion.thrusters.configuration_matrix").as_double_array(),
       num_dof_, num_thrusters_);

   wrench_subscriber_ = this->create_subscription<geometry_msgs::msg::Wrench>(
       "/thrust/wrench_input", 1,
       std::bind(&ThrusterAllocator::wrench_callback, this, std::placeholders::_1));

   thruster_forces_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
       "/thrust/thruster_forces", 1);

   calculate_thrust_timer_ = this->create_wall_timer(
       100ms, std::bind(&ThrusterAllocator::calculate_thrust_timer_cb, this));

   pseudoinverse_allocator_.T_pinv = calculate_right_pseudoinverse(thrust_configuration);
   body_frame_forces_.setZero();

   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ThrusterAllocator::on_activate(const rclcpp_lifecycle::State &) {
   thruster_forces_publisher_->on_activate();
   RCLCPP_INFO(get_logger(), "Thruster allocator activated");
   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ThrusterAllocator::on_deactivate(const rclcpp_lifecycle::State &) {
   thruster_forces_publisher_->on_deactivate();
   RCLCPP_INFO(get_logger(), "Thruster allocator deactivated");
   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ThrusterAllocator::on_cleanup(const rclcpp_lifecycle::State &) {
   thruster_forces_publisher_.reset();
   wrench_subscriber_.reset();
   calculate_thrust_timer_.reset();
   RCLCPP_INFO(get_logger(), "Thruster allocator cleaned up");
   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ThrusterAllocator::on_shutdown(const rclcpp_lifecycle::State &) {
   RCLCPP_INFO(get_logger(), "Thruster allocator shutting down");
   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ThrusterAllocator::calculate_thrust_timer_cb() {
   if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
       return;
   }

   Eigen::VectorXd thruster_forces = pseudoinverse_allocator_.calculate_allocated_thrust(body_frame_forces_);
   if (is_invalid_matrix(thruster_forces)) {
       RCLCPP_ERROR(get_logger(), "ThrusterForces vector invalid, ignoring");
       return;
   }
   if (!saturate_vector_values(thruster_forces, min_thrust_, max_thrust_)) {
       RCLCPP_WARN(get_logger(), "Thruster forces vector required saturation.");
   }
   std_msgs::msg::Float32MultiArray msg_out;
   array_eigen_to_msg(thruster_forces, msg_out);
   thruster_forces_publisher_->publish(msg_out);
}

void ThrusterAllocator::wrench_callback(const geometry_msgs::msg::Wrench &msg) {
   if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
       return;
   }

   Eigen::Vector3d msg_vector;
   msg_vector(0) = msg.force.x; // surge
   msg_vector(1) = msg.force.y; // sway
   msg_vector(2) = msg.torque.z; // yaw
   if (is_invalid_matrix(msg_vector)) {
       RCLCPP_ERROR(get_logger(), "ASV wrench vector invalid, ignoring.");
       return;
   }
   Eigen::VectorXd saturated_vector = msg_vector;
   if (!saturate_vector_values(saturated_vector, min_thrust_, max_thrust_)) {
       RCLCPP_WARN(get_logger(), "ASV wrench vector required saturation.");
   }
   msg_vector = saturated_vector;
   std::swap(msg_vector, body_frame_forces_);
}