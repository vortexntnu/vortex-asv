// Custom libraries
#include <thruster_interface_asv/thruster_interface_asv_node.hpp>

class ThrusterInterfaceASVNode : public rclcpp::Node {
   private:
    // ROS2 Variables ----------
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        _subscriberThrusterForces;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr _publisherPWM;
    rclcpp::TimerBase::SharedPtr _thruster_driver_timer;

    // Variables that are shared inside the object ----------
    // Mutable allows modification in const methods
    mutable float _thrusterForces[4];
    mutable int16_t* _pwm;

    // Methods ----------
    void _thruster_forces_callback(
        const std_msgs::msg::Float64MultiArray::SharedPtr msg) const {
        // Save subscribed topic values into universal array for further usage
        for (size_t i = 0; i < 4; i++) {
            _thrusterForces[i] = msg->data[i];
        }
    }

    void _thruster_driver_callback() {
        // Drive thrusters according to the thrusterForces provided by
        // subscriber
        _pwm =
            thruster_interface_asv_driver_lib::drive_thrusters(_thrusterForces);

        // Publish PWM values for debugging purposes
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
        // Get filepath of .CSV file with force mapping
        std::string forcesToPWMDataFilePath =
            ament_index_cpp::get_package_share_directory(
                "thruster_interface_asv");
        forcesToPWMDataFilePath += "/config/ThrustMe_P1000_force_mapping.csv";

        // Get Thruster mapping values from ROS2 Parameters
        std::vector<int64_t> parameterThrusterMapping =
            this->declare_parameter<std::vector<int64_t>>(
                "propulsion.thrusters.thruster_to_pin_mapping", {0, 1, 2, 3});

        int8_t ThrusterMapping[4] = {
            static_cast<int8_t>(parameterThrusterMapping[0]),
            static_cast<int8_t>(parameterThrusterMapping[1]),
            static_cast<int8_t>(parameterThrusterMapping[2]),
            static_cast<int8_t>(parameterThrusterMapping[3])};

        // Get Thruster direction values from ROS2 Parameters
        std::vector<int64_t> parameterThrusterDirection =
            this->declare_parameter<std::vector<int64_t>>(
                "propulsion.thrusters.thruster_direction", {1, 1, 1, 1});

        int8_t thrusterDirection[4] = {
            static_cast<int8_t>(parameterThrusterDirection[0]),
            static_cast<int8_t>(parameterThrusterDirection[1]),
            static_cast<int8_t>(parameterThrusterDirection[2]),
            static_cast<int8_t>(parameterThrusterDirection[3])};

        // Get PWM Offset values from ROS2 Parameters
        std::vector<int64_t> parameterPWMOffset =
            this->declare_parameter<std::vector<int64_t>>(
                "propulsion.thrusters.thruster_PWM_offset", {0, 0, 0, 0});

        int16_t offsetPWM[4] = {static_cast<int16_t>(parameterPWMOffset[0]),
                                static_cast<int16_t>(parameterPWMOffset[1]),
                                static_cast<int16_t>(parameterPWMOffset[2]),
                                static_cast<int16_t>(parameterPWMOffset[3])};

        // Get MAX and MIN PWM values from ROS2 Parameters
        std::vector<int64_t> parameterPWMMin =
            this->declare_parameter<std::vector<int64_t>>(
                "propulsion.thrusters.thruster_PWM_min",
                {1100, 1100, 1100, 1100});

        std::vector<int64_t> parameterPWMMax =
            this->declare_parameter<std::vector<int64_t>>(
                "propulsion.thrusters.thruster_PWM_max",
                {1900, 1900, 1900, 1900});

        int16_t minPWM[4] = {static_cast<int16_t>(parameterPWMMin[0]),
                             static_cast<int16_t>(parameterPWMMin[1]),
                             static_cast<int16_t>(parameterPWMMin[2]),
                             static_cast<int16_t>(parameterPWMMin[3])};

        int16_t maxPWM[4] = {static_cast<int16_t>(parameterPWMMax[0]),
                             static_cast<int16_t>(parameterPWMMax[1]),
                             static_cast<int16_t>(parameterPWMMax[2]),
                             static_cast<int16_t>(parameterPWMMax[3])};

        // Initialize Thruster driver
        thruster_interface_asv_driver_lib::init(
            forcesToPWMDataFilePath, ThrusterMapping, thrusterDirection,
            offsetPWM, minPWM, maxPWM);

        // ROS Setup ----------
        // Initialize ROS2 thrusterForces subscriber

        this->declare_parameter<std::string>("topics.thruster_forces");
        this->declare_parameter<std::string>("topics.pwm_output");
        std::string thruster_forces_topic =
            this->get_parameter("topics.thruster_forces").as_string();
        std::string pwm_output_topic =
            this->get_parameter("topics.pwm_output").as_string();

        _subscriberThrusterForces =
            this->create_subscription<std_msgs::msg::Float64MultiArray>(
                thruster_forces_topic, 5,
                std::bind(&ThrusterInterfaceASVNode::_thruster_forces_callback,
                          this, std::placeholders::_1));

        // Initialize ROS2 pwm publisher
        _publisherPWM = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            pwm_output_topic, 5);

        // Initialize a never ending cycle that continuously publishes and
        // drives thrusters depending on what the ThrusterForces value is set to
        // from ThrusterForces subscriber
        using namespace std::chrono_literals;
        _thruster_driver_timer = this->create_wall_timer(
            200ms,
            std::bind(&ThrusterInterfaceASVNode::_thruster_driver_callback,
                      this));

        // Debugging ----------
        RCLCPP_INFO(this->get_logger(),
                    "Initialized thruster_interface_asv_node node");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterInterfaceASVNode>());
    rclcpp::shutdown();
    return 0;
}
