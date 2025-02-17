#include "thrust_allocator_asv/allocator_ros.hpp"
#include "thrust_allocator_asv/allocator_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto allocator = std::make_shared<ThrusterAllocator>();

  // Usually this would be done in the launchfile. However, we only want this
  // node to be a lifecycle-node so that it can be shut down externally. Thus,
  // we immediately transition to activate here.
  allocator->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  allocator->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(allocator->get_node_base_interface());
  executor.spin();

  allocator->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  allocator->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  allocator->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);

  rclcpp::shutdown();
  return 0;
}