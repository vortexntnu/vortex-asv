// Custom libraies
#include <thruster_interface_asv/thruster_interface_asv_node.hpp>

class ThrusterInterfaceASVNode : public rclcpp::Node {
private:
  // ROS2 Variables ----------
  // Creates ROS2 subscriber
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      _subscriberThrusterForces;
  // Create ROS2 publisher
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr _publisherPWM;
  // Create ROS2 timer/cycler
  rclcpp::TimerBase::SharedPtr _thruster_driver_timer;

  // Variables that are shared inside the object ----------
  // Mutable allows modification in const methods
  mutable float _thrusterForces[4];
  mutable int16_t *_pwm;

  // Methods ----------
  void _thruster_forces_callback(
      const std_msgs::msg::Float32MultiArray::SharedPtr msg) const {
    // Save subscribed topic values into universal array for further usage
    for (size_t i = 0; i < 4; i++) {
      _thrusterForces[i] = msg->data[i];
    }
  }

  void _thruster_driver_callback() {
    // Drive thrusters according to the thrusterForces provided by subscriber
    _pwm = thruster_interface_asv_driver_lib::drive_thrusters(_thrusterForces);

    // Publish PWM values for debuging purposes
    std::shared_ptr<std_msgs::msg::Int16MultiArray> messagePWM =
        std::make_shared<std_msgs::msg::Int16MultiArray>();
    messagePWM->data.resize(4);
    for (int8_t i = 0; i < 4; i++) {
      messagePWM->data[i] = _pwm[i];
    }
    _publisherPWM->publish(*messagePWM);
  }

public:
  // Builder for the object ----------
  ThrusterInterfaceASVNode() : Node("thruster_interface_asv_node") {
    // Thruster Driver Setup ----------
    // Get MAX and MIN PWM values
    int16_t minPWM[4] = {1100, 1100, 1100, 1100};
    int16_t maxPWM[4] = {1900, 1900, 1900, 1900};

    // Get filepath of .CSV file with ROS2 file path finder
    std::string forcesToPWMDataFilePath =
        ament_index_cpp::get_package_share_directory("thruster_interface_asv");
    forcesToPWMDataFilePath += "/config/ThrustMe_P1000_force_mapping.csv";

    // Initialize Thruster driver
    thruster_interface_asv_driver_lib::init(forcesToPWMDataFilePath, minPWM,
                                            maxPWM);

    // ROS Setup ----------
    // Initialize ROS2 subscribers role
    _subscriberThrusterForces =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/thrust/thruster_forces", 5,
            std::bind(&ThrusterInterfaceASVNode::_thruster_forces_callback,
                      this, std::placeholders::_1));

    // Initialize ROS2 publisher role
    _publisherPWM =
        this->create_publisher<std_msgs::msg::Int16MultiArray>("/pwm", 5);

    // Initialize a never ending cycle that continuisly publishes and drives
    // thrusters depending on what the ThrusterForces value is set to from
    // ThrusterForces subscriber
    using namespace std::chrono_literals;
    _thruster_driver_timer = this->create_wall_timer(
        500ms,
        std::bind(&ThrusterInterfaceASVNode::_thruster_driver_callback, this));

    // Debugging ----------
    RCLCPP_INFO(this->get_logger(),
                "Initialized thruster_interface_asv_node node");
  }
};

int main(int argc, char *argv[]) {
  // Start ROS2
  rclcpp::init(argc, argv);

  // Start infinite loop of ROS2 Node
  rclcpp::spin(std::make_shared<ThrusterInterfaceASVNode>());

  // Shut down ROS2
  rclcpp::shutdown();
  return 0;
}
