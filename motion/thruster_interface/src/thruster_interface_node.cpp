#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include <thruster_interface/thruster_interface.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>

using std::placeholders::_1;

constexpr double newton_to_gram_conversion_factor = 101.97162;

bool started = false;
//int8_t hardware_status = 0;
uint8_t software_killswitch = 0;

class ThrusterInterface : public rclcpp::Node {
public:
  ThrusterInterface() : Node("thruster_interface_node") {

    subscription_ = this->create_subscription<vortex_msgs::msg::ThrusterForces>(
        "thruster_topic", 10,
        std::bind(&ThrusterInterface::thrusterForcesCallback, this,
                  std::placeholders::_1));

    publisher_ESC1 = this->create_publisher<std_msgs::msg::Float32>("/asv/temperature/ESC1", 10);
    publisher_ESC2 = this->create_publisher<std_msgs::msg::Float32>("/asv/temperature/ESC2", 10);
    publisher_ESC3 = this->create_publisher<std_msgs::msg::Float32>("/asv/temperature/ESC3", 10);
    publisher_ESC4 = this->create_publisher<std_msgs::msg::Float32>("/asv/temperature/ESC4", 10);
    publisher_status = this->create_publisher<std_msgs::msg::Int8>("/asv/status", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ThrusterInterface::publish_temperature_and_status, this));
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
    init(file);

    // SENDING
    send_status(software_killswitch, file);
    send_pwm(pwm_values, file);

    
  }

  void publish_temperature_and_status()
    {

        // RECEIVING

        //int file;
        //init(file);

        //TEMPERATURE

        std::vector<float> temperature = {22.04, 43.04, 34.99, 49.66};

        //temperature = readFloatsFromI2C(file);

        for (size_t i = 0; i < temperature.size(); i++) {
          std::cout << i << "temperature value = " << temperature[i] << std::endl;
        }

        //hardware_status = read_hardware_statusFromI2C(file);
        //cout << "hardware status = " << (uint16_t)(hardware_status) << endl;

        
        auto message_ESC1 = std_msgs::msg::Float32();
        auto message_ESC2 = std_msgs::msg::Float32();
        auto message_ESC3 = std_msgs::msg::Float32();
        auto message_ESC4 = std_msgs::msg::Float32();

        message_ESC1.data = temperature[0];
        message_ESC2.data = temperature[1];
        message_ESC3.data = temperature[2];
        message_ESC4.data = temperature[3];

        RCLCPP_INFO(this->get_logger(), "Publishing temperature");
        publisher_ESC1->publish(message_ESC1);
        publisher_ESC2->publish(message_ESC2);
        publisher_ESC3->publish(message_ESC3);
        publisher_ESC4->publish(message_ESC4);

        //HARDWARE STATUS

        int8_t hardware_status = 0;

        auto message_status = std_msgs::msg::Int8();

        message_status.data = hardware_status;

        RCLCPP_INFO(this->get_logger(), "Publishing status");

        publisher_status->publish(message_status);

        //close(file);
    }

  rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::SharedPtr subscription_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_ESC1;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_ESC2;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_ESC3;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_ESC4;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_status;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterInterface>());
  rclcpp::shutdown();
  return 0;
}
