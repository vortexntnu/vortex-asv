#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/msg/pwm.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>

#include <thruster_interface/thruster_interface.hpp>
using std::placeholders::_1;

class ThrusterInterfaceROS : public rclcpp::Node {
private:
  std::string mapping_file =
      ament_index_cpp::get_package_share_directory("thruster_interface") + 
      "/config/ThrustMe_P1000_force_mapping.csv";
  ThrusterInterface thrusterInterface{mapping_file};

  rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::SharedPtr
      subscription_;
  rclcpp::Publisher<vortex_msgs::msg::Pwm>::SharedPtr publisher_;

public:
  ThrusterInterfaceROS() : Node("thruster_interface") {
    publisher_ = this->create_publisher<vortex_msgs::msg::Pwm>("pwm", 10);
    subscription_ = this->create_subscription<vortex_msgs::msg::ThrusterForces>(
        "thrust/thruster_forces", 10,
        std::bind(&ThrusterInterfaceROS::thrust_callback, this, _1));
  }
  void thrust_callback(const vortex_msgs::msg::ThrusterForces::SharedPtr msg);
};
