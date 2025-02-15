/**
 * @file allocator_ros.hpp
 * @brief ThrustAllocator class, which
 * allocates thrust to the ASV's thrusters based on the desired body frame
 * forces.
 */

#ifndef VORTEX_ALLOCATOR_ROS_HPP
#define VORTEX_ALLOCATOR_ROS_HPP

#include "thrust_allocator_asv/allocator_utils.hpp"
#include "thrust_allocator_asv/pseudoinverse_allocator.hpp"
#include <eigen3/Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/wrench.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;

class ThrustAllocator : public rclcpp::Node {
public:
  explicit ThrustAllocator();

  /**
   * @brief Calculates the allocated
   * thrust based on the body frame forces. It then saturates the output vector
   * between min and max values and publishes the thruster forces to the topic
   * "thrust/thruster_forces".
   */
  void calculate_thrust_timer_cb();

private:
  Eigen::MatrixXd thrust_configuration;
  /**
   * @brief Callback function for the wrench input subscription. Extracts the
   * surge, sway and yaw values from the received wrench msg
   * and stores them in the body_frame_forces_ Eigen vector.
   * @param msg The received geometry_msgs::msg::Wrench message.
   */
  void wrench_callback(const geometry_msgs::msg::Wrench &msg);
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      thruster_forces_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr
      wrench_subscriber_;
  rclcpp::TimerBase::SharedPtr calculate_thrust_timer_;
  size_t count_;
  int num_dof_;
  int num_thrusters_;
  int min_thrust_;
  int max_thrust_;
  Eigen::Vector3d body_frame_forces_;
  std::vector<int64_t> direction_;
  PseudoinverseAllocator pseudoinverse_allocator_;
};

#endif // VORTEX_ALLOCATOR_ROS_HPP
