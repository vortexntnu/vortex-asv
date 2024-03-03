#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <thruster_interface/thruster_interface.hpp>
//#include <vortex_msgs/msg/pwm.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>

using std::placeholders::_1;

constexpr double newton_to_gram_conversion_factor = 101.97162;

bool started = false;
int8_t hardware_status = 0;
uint8_t software_killswitch = 0;

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("thruster_interface_node") {
    subscription_ = this->create_subscription<vortex_msgs::msg::ThrusterForces>(
        "thruster_topic", 10,
        // std::bind(&MinimalSubscriber::float_topic_callback, this, _1));
        std::bind(&MinimalSubscriber::thrusterForcesCallback, this,
                  std::placeholders::_1));
  }

private:
  void thrusterForcesCallback(
      const vortex_msgs::msg::ThrusterForces::SharedPtr msg) const {

    std::vector<double> forces_in_grams = {
        msg->thrust[0] * newton_to_gram_conversion_factor,
        msg->thrust[1] * newton_to_gram_conversion_factor,
        msg->thrust[2] * newton_to_gram_conversion_factor,
        msg->thrust[3] * newton_to_gram_conversion_factor};

    std::vector<uint16_t> pwm_values;
    pwm_values.resize(4);
    pwm_values = interpolate_all(forces_in_grams);

    for (size_t i = 0; i < forces_in_grams.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "Force[%zu]: %f", i, forces_in_grams[i]);
      RCLCPP_INFO(this->get_logger(), "PWM[%zu]: %u", i, pwm_values[i]);
    }

    //----------------------------------------------------------

    int file;
    if (started == false) {
      init(file);
      started = true;
    }

    // SENDING
    send_status(software_killswitch, file);
    send_pwm(pwm_values, file);

    // RECEIVING

    std::vector<float> temperature;

    temperature = readFloatsFromI2C(file);

    for (size_t i = 0; i < temperature.size(); i++) {
      std::cout << i << "temperature value = " << temperature[i] << std::endl;
    }

    hardware_status = read_hardware_statusFromI2C(file);
    cout << "hardware status = " << (uint16_t)(hardware_status) << endl;

    close(file);
  }
  rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::SharedPtr
      subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
