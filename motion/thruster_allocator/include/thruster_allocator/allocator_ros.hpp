#ifndef VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP
#define VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP

#include "thruster_allocator/allocator_utils.hpp"
#include "thruster_allocator/pseudoinverse_allocator.hpp"
#include <eigen3/Eigen/Eigen>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;

class ThrusterAllocator : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit ThrusterAllocator();

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &);

private:
    void calculate_thrust_timer_cb();
    void wrench_callback(const geometry_msgs::msg::Wrench &msg);

    Eigen::MatrixXd thrust_configuration;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>::SharedPtr thruster_forces_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_subscriber_;
    rclcpp::TimerBase::SharedPtr calculate_thrust_timer_;
    int num_dof_;
    int num_thrusters_;
    int min_thrust_;
    int max_thrust_;
    Eigen::Vector3d body_frame_forces_;
    std::vector<int64_t> direction_;
    PseudoinverseAllocator pseudoinverse_allocator_;
};

#endif // VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP