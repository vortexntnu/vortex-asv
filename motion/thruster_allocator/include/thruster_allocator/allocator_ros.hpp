#ifndef VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP
#define VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP

#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Eigen>
#include <thruster_allocator/allocator_utils.hpp>
#include <thruster_allocator/pseudoinverse_allocator.hpp>

#include <geometry_msgs/msg/wrench.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>

using namespace std::chrono_literals;

class Allocator : public rclcpp::Node {
public:
  explicit Allocator();
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

  void wrench_callback(const geometry_msgs::msg::Wrench &msg);
  bool healthyWrench(const Eigen::VectorXd &v) const;
  rclcpp::Publisher<vortex_msgs::msg::ThrusterForces>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  int num_degrees_of_freedom_;
  int num_thrusters_;
  Eigen::Vector3d body_frame_forces_;
  std::vector<int> direction_ = {1, 1, 1, 1};
  PseudoinverseAllocator pseudoinverse_allocator_;
};

#endif // VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP
