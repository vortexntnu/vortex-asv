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
      thruster_forces_sub_;
  rclcpp::Publisher<vortex_msgs::msg::Pwm>::SharedPtr pwm_pub_;
  std::vector<int64_t> thruster_to_pin_map_;
  std::vector<int64_t> thruster_direction_map_;
  std::vector<int64_t> pwm_offsets_;

public:
  ThrusterInterfaceROS() : Node("thruster_interface") {
    pwm_pub_ = this->create_publisher<vortex_msgs::msg::Pwm>("pwm", 10);
    thruster_forces_sub_ =
        this->create_subscription<vortex_msgs::msg::ThrusterForces>(
            "thrust/thruster_forces", 10,
            std::bind(&ThrusterInterfaceROS::thrust_callback, this, _1));

    declare_parameter("thruster_to_pin_map_", std::vector<int64_t>{1, 3, 2, 0});
    declare_parameter("thruster_direction_map_",
                      std::vector<int64_t>{1, 1, 1, -1});
    declare_parameter("pwm_offsets_", std::vector<int64_t>{100, 100, 100, 100});
    thruster_to_pin_map_ =
        get_parameter("propulsion.thrusters.thruster_to_pin_map")
            .as_integer_array();
    thruster_direction_map_ =
        get_parameter("propulsion.thrusters.thruster_direction")
            .as_integer_array();
    pwm_offsets_ =
        get_parameter("propulsion.thrusters.pwm_offsets").as_integer_array();
  }

  void thrust_callback(const vortex_msgs::msg::ThrusterForces::SharedPtr msg);
};
