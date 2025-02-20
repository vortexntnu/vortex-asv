/**
 * @file allocator_ros.hpp
 * @brief ThrustAllocator class, which
 * allocates thrust to the ASV's thrusters based on the desired body frame
 * forces.
 */

#ifndef VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP
#define VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/wrench.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "thrust_allocator_asv/allocator_utils.hpp"
#include "thrust_allocator_asv/pseudoinverse_allocator.hpp"

using namespace std::chrono_literals;

class ThrustAllocator : public rclcpp_lifecycle::LifecycleNode {
   public:
    explicit ThrustAllocator();

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State&);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State&);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State&);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State&);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State&);

   private:
    /**
     * @brief Calculates the allocated
     * thrust based on the body frame forces. It then saturates the output
     * vector between min and max values and publishes the thruster forces to
     * the topic "thrust/thruster_forces".
     */
    void calculate_thrust_timer_cb();
    void wrench_callback(const geometry_msgs::msg::Wrench& msg);

    Eigen::MatrixXd thrust_configuration;
    rclcpp_lifecycle::LifecyclePublisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr thruster_forces_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr
        wrench_subscriber_;
    rclcpp::TimerBase::SharedPtr calculate_thrust_timer_;
    int num_dof_;
    int num_thrusters_;
    int min_thrust_;
    int max_thrust_;
    Eigen::Vector3d body_frame_forces_;
    std::vector<int64_t> direction_;
    PseudoinverseAllocator pseudoinverse_allocator_;
};

#endif  // VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP
