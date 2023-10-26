#ifndef VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP
#define VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Eigen>
#include <thruster_allocator/allocator_utils.hpp>
#include <thruster_allocator/pseudoinverse_allocator.hpp>

#include <geometry_msgs/msg/wrench.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>

/**
 * @file allocator_ros.hpp
 * @brief ThrusterAllocator class, which
 * allocates thrust to the ASV's thrusters based on the desired body frame
 * forces.
 */
using namespace std::chrono_literals;

class ThrusterAllocator : public rclcpp::Node {
public:
  explicit ThrusterAllocator();

  /**
 * @brief Calculates the allocated
 * thrust based on the body frame forces. It then saturates the output vector
 * between min and max values and publishes the thruster forces to the topic
 * "thrust/thruster_forces".
 */
  void timer_callback();

private:
  // Hardcoded thruster config matrix for T_pinv
  // clang-format off
  Eigen::MatrixXd thrust_configuration =
      (Eigen::MatrixXd(3, 4) << 
      0.70711, 0.70711, 0.70711, 0.70711, 
      -0.70711, 0.70711, -0.70711, 0.70711, 
      0.27738, 0.27738, -0.27738, -0.27738)
          .finished();
  // clang-format on

  /**
 * @brief Callback function for the wrench input subscription. Extracts the
 * surge, sway and yaw values from the received wrench msg
 * and stores them in the body_frame_forces_ Eigen vector.
 * @param msg The received geometry_msgs::msg::Wrench message.
 */
  void wrench_callback(const geometry_msgs::msg::Wrench &msg);

  /**
 * @brief Checks if the given Eigen vector contains any NaN or Inf values
 * @param v The Eigen vector to check.
 * @return True if the vector is healthy, false otherwise.
 */
  bool healthyWrench(const Eigen::VectorXd &v) const;
  rclcpp::Publisher<vortex_msgs::msg::ThrusterForces>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  int num_dof_;
  int num_thrusters_ = 4;
  int min_thrust_ = -100;
  int max_thrust_ = 100;
  Eigen::Vector3d body_frame_forces_;
  std::vector<int> direction_ = {1, 1, 1, 1};
  PseudoinverseAllocator pseudoinverse_allocator_;
};

#endif // VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP
