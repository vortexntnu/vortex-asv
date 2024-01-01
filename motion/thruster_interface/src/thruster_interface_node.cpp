#include <thruster_interface/thruster_interface_node.hpp>

void ThrusterInterfaceROS::thrust_callback(
    const vortex_msgs::msg::ThrusterForces::SharedPtr msg) {
  // Convert from Newton to grams
  
  constexpr double newton_to_gram_conversion_factor = 101.97162;

  std::vector<double> forces_in_grams = {
      msg->thrust[0] * newton_to_gram_conversion_factor,
      msg->thrust[1] * newton_to_gram_conversion_factor,
      msg->thrust[2] * newton_to_gram_conversion_factor,
      msg->thrust[3] * newton_to_gram_conversion_factor};

  std::vector<int> pwm_values;
  for (auto force : forces_in_grams) {
    pwm_values.push_back(thrusterInterface.interpolate(force));
  }

  vortex_msgs::msg::Pwm pwm_msg;
  // TODO: Get mapping and offsets from rosparam
  // Give thrust to thruster 0: publish on pin = thruster_to_pin_map[0]
  std::vector<int> thruster_to_pin_map = {1, 3, 2, 0};
  std::vector<int> thruster_direction_map = {1, 1, 1, -1};
  std::vector<int> pwm_offsets = {100, 100, 100, 100};

  // Iterates through thruster 0 to 3, where 0 is front right, iterated
  // clockwise
  for (int i = 0; i < 4; i++) {

    int center_pwm_value = 1500;
    int offset_from_center_value =
        pwm_values[i] - center_pwm_value + pwm_offsets[i];
    int pwm_value_correct_direction =
        center_pwm_value + thruster_direction_map[i] * offset_from_center_value;

    int pwm_clamped = std::min(std::max(pwm_value_correct_direction, 1400),
                               1600); // min 1100, max 1900
    pwm_msg.positive_width_us.push_back(pwm_clamped);
    pwm_msg.pins.push_back(thruster_to_pin_map[i]);
  }

  pwm_pub_->publish(pwm_msg);

  thrusterInterface.publish_thrust_to_escs(forces_in_grams);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto thruster_interface_node = std::make_shared<ThrusterInterfaceROS>();
  RCLCPP_INFO(thruster_interface_node->get_logger(),
              "Starting thruster_interface_node");
  rclcpp::spin(thruster_interface_node);
  rclcpp::shutdown();
  return 0;
}