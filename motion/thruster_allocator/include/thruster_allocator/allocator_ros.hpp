#ifndef VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP
#define VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/wrench.hpp>

#include "allocator_utils.hpp"
#include "pseudoinverse_allocator.hpp"
#include "vortex_msgs/msg/thruster_forces.hpp"

using namespace std::chrono_literals;

class Allocator : public rclcpp::Node {
public:
  explicit Allocator();

private:
  Eigen::MatrixXd thrust_configuration =
      (Eigen::MatrixXd(3, 4) << 0.70711, 0.70711, 0.70711, 0.70711, -0.70711,
       0.70711, -0.70711, 0.70711, 0.27738, 0.27738, -0.27738, -0.27738)
          .finished(); // Hardcoded thruster config matrix for T_pinv

  void spinOnce();
  void wrench_callback(const geometry_msgs::msg::Wrench &msg);
  Eigen::VectorXd wrenchMsgToEigen(const geometry_msgs::msg::Wrench &msg) const;
  Eigen::VectorXd wrenchMsgToEigen(const float force_x, const float force_y,
                                   const float torque) const;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vortex_msgs::msg::ThrusterForces>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr subscription_;
  size_t count_;
  float body_frame_force_x = 0.0;
  float body_frame_force_y = 0.0;
  float body_frame_torque = 0.0;
  std::vector<int> m_direction = {1, 1, 1, 1};
  std::unique_ptr<PseudoinverseAllocator> m_pseudoinverse_allocator;
};

#endif // VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP
