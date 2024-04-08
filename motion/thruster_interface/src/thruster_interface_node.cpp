
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include <thruster_interface/thruster_interface.hpp>
//#include <vortex_msgs/msg/thruster_forces.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

constexpr double newton_to_gram_conversion_factor = 101.97162;

bool started = false;
uint8_t software_killswitch;

int8_t hardware_status = 0;

class ThrusterInterface : public rclcpp::Node {
private:

int PWM_min;
int PWM_max;

public:
  ThrusterInterface() : Node("thruster_interface_node") {

    subscription_thruster_forces =
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "thrust/thruster_forces", 10,
            std::bind(&ThrusterInterface::thrusterForcesCallback, this,
                      std::placeholders::_1));

    subscription_hardware_status =
        this->create_subscription<std_msgs::msg::Int8>(
            "asv/software/killswitch", 10,
            std::bind(&ThrusterInterface::SoftwareKillswitchCallback, this,
                      std::placeholders::_1));

    // remove the /test from the topic names later

    publisher_ESC1 = this->create_publisher<std_msgs::msg::Float32>(
        "/asv/temperature/ESC1/test", 10);
    publisher_ESC2 = this->create_publisher<std_msgs::msg::Float32>(
        "/asv/temperature/ESC2/test", 10);
    publisher_ESC3 = this->create_publisher<std_msgs::msg::Float32>(
        "/asv/temperature/ESC3/test", 10);
    publisher_ESC4 = this->create_publisher<std_msgs::msg::Float32>(
        "/asv/temperature/ESC4/test", 10);
    publisher_ambient1 = this->create_publisher<std_msgs::msg::Float32>(
        "/asv/temperature/ambient1/test", 10);
    publisher_ambient2 = this->create_publisher<std_msgs::msg::Float32>(
        "/asv/temperature/ambient2/test", 10);
    publisher_status = this->create_publisher<std_msgs::msg::Int8>(
        "/asv/failsafe/hardware/status", 10);

    // get parameters from the freya.yaml file

    this->declare_parameter("thruster_interface.PWM_min", 1100);
    PWM_min = this->get_parameter("thruster_interface.PWM_min").as_int();
    this->declare_parameter("thruster_interface.PWM_max", 1900);
    PWM_max = this->get_parameter("thruster_interface.PWM_max").as_int();
    this->declare_parameter("thruster_interface.publishing_rate", 500);
    int publishing_rate =
        this->get_parameter("thruster_interface.publishing_rate").as_int();

    cout << "800 : " << interpolate(800, PWM_min, PWM_max) << endl;
    cout << "-10000 : " << interpolate(-10000, PWM_min, PWM_max) << endl;
    cout << "publishing rate : " << publishing_rate << endl;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publishing_rate),
        std::bind(&ThrusterInterface::publish_temperature_and_status, this));
  }

private:
  //----------------------------------------------------------

  void thrusterForcesCallback(
      const std_msgs::msg::Float64MultiArray::SharedPtr msg) const {

    std::vector<double> forces_in_grams = {
        msg->data[0] * newton_to_gram_conversion_factor,
        msg->data[1] * newton_to_gram_conversion_factor,
        msg->data[2] * newton_to_gram_conversion_factor,
        msg->data[3] * newton_to_gram_conversion_factor};

    std::vector<uint16_t> pwm_values;
    pwm_values.resize(4);
    pwm_values = interpolate_all(forces_in_grams,PWM_min,PWM_max);

    for (size_t i = 0; i < forces_in_grams.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "Force[%zu]: %f", i, forces_in_grams[i]);
      RCLCPP_INFO(this->get_logger(), "PWM[%zu]: %u", i, pwm_values[i]);
    }

    try{
    int file;
    init(file);

    // SENDING
    send_status(software_killswitch, file);
    send_pwm(pwm_values, file);
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  }

  //----------------------------------------------------------

  void
  SoftwareKillswitchCallback(const std_msgs::msg::Int8::SharedPtr msg) const {
    software_killswitch = msg->data;
  }

  //----------------------------------------------------------

  void publish_temperature_and_status() {
    try
    {
    // RECEIVING
    int file;
    init(file);

    // TEMPERATURE
    std::vector<float> temperature = {0, 0, 0, 0};
    temperature = readFloatsFromI2C(file);
    /*
    for (size_t i = 0; i < temperature.size(); i++) {
      std::cout << i << "temperature value = " << temperature[i] << std::endl;
    }
    */
    auto message_ESC1 = std_msgs::msg::Float32();
    auto message_ESC2 = std_msgs::msg::Float32();
    auto message_ESC3 = std_msgs::msg::Float32();
    auto message_ESC4 = std_msgs::msg::Float32();
    auto message_ambient1 = std_msgs::msg::Float32();
    auto message_ambient2 = std_msgs::msg::Float32();

    message_ESC1.data = temperature[0];
    message_ESC2.data = temperature[1];
    message_ESC3.data = temperature[2];
    message_ESC4.data = temperature[3];
    message_ambient1.data = temperature[4];
    message_ambient2.data = temperature[5];

    RCLCPP_INFO(this->get_logger(), "Publishing temperature");
    publisher_ESC1->publish(message_ESC1);
    publisher_ESC2->publish(message_ESC2);
    publisher_ESC3->publish(message_ESC3);
    publisher_ESC4->publish(message_ESC4);
    publisher_ambient1->publish(message_ambient1);
    publisher_ambient2->publish(message_ambient2);

    // HARDWARE STATUS
    // int8_t hardware_status = 0;
    hardware_status = read_hardware_statusFromI2C(file);

    // cout << "hardware status = " << (uint16_t)(hardware_status) << endl;

    auto message_status = std_msgs::msg::Int8();

    message_status.data = hardware_status;

    RCLCPP_INFO(this->get_logger(), "Publishing status");

    publisher_status->publish(message_status);

    close(file);

    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  }

  //----------------------------------------------------------

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      subscription_thruster_forces;

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr
      subscription_hardware_status;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_ESC1;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_ESC2;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_ESC3;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_ESC4;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_ambient1;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_ambient2;

  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_status;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterInterface>());
  rclcpp::shutdown();
  return 0;
}

