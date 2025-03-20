#include "thruster_interface_asv/thruster_interface_asv_ros.hpp"

ThrusterInterfaceASVNode::ThrusterInterfaceASVNode()
    : Node("thruster_interface_asv_node") {
    this->extract_all_parameters();

    // Set up subscriber and publisher
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    thruster_forces_subscriber_ =
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
            subscriber_topic_name_, qos_sensor_data,
            std::bind(&ThrusterInterfaceASVNode::thruster_forces_callback, this,
                      std::placeholders::_1));

    thruster_pwm_publisher_ =
        this->create_publisher<std_msgs::msg::Int16MultiArray>(
            publisher_topic_name_, qos_sensor_data);

    // call constructor for thruster_driver_
    thruster_driver_ = std::make_unique<ThrusterInterfaceASVDriver>(
        i2c_bus_, i2c_address_, thruster_parameters_, poly_coeffs_);

    // Declare thruster_forces_array_ in case no topic comes at the
    // very beginning
    thruster_forces_array_ = std::vector<double>(4, 0.00);

    this->initialize_parameter_handler();

    RCLCPP_INFO(this->get_logger(),
                "\"thruster_interface_asv_node\" correctly initialized");
}

void ThrusterInterfaceASVNode::thruster_forces_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::copy_n(msg->data.begin(), msg->data.size(),
                thruster_forces_array_.begin());
    this->pwm_callback();
}

void ThrusterInterfaceASVNode::pwm_callback() {
    // drive thrusters...
    std::vector<uint16_t> thruster_pwm_array =
        thruster_driver_->drive_thrusters(this->thruster_forces_array_);

    //..and publish PWM values for debugging purposes
    if (debug_flag_) {
        std_msgs::msg::Int16MultiArray pwm_message;
        pwm_message.data = std::vector<int16_t>(thruster_pwm_array.begin(),
                                                thruster_pwm_array.end());
        thruster_pwm_publisher_->publish(pwm_message);
    }
}

void ThrusterInterfaceASVNode::initialize_parameter_handler() {
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Register the parameter event callback directly in the lambda expression
    debug_flag_parameter_cb = param_handler_->add_parameter_callback(
        "debug.flag", std::bind(&ThrusterInterfaceASVNode::update_debug_flag,
                                this, std::placeholders::_1));
}

void ThrusterInterfaceASVNode::update_debug_flag(const rclcpp::Parameter& p) {
    debug_flag_ = p.get_value<bool>();
    RCLCPP_INFO(this->get_logger(),
                "Received parameter event: debug.flag updated to: %s",
                debug_flag_ ? "true" : "false");
}

void ThrusterInterfaceASVNode::extract_all_parameters() {
    // thruster parameters from orca.yaml
    this->declare_parameter<std::vector<int>>(
        "propulsion.thrusters.thruster_to_pin_mapping");
    this->declare_parameter<std::vector<int>>(
        "propulsion.thrusters.thruster_direction");
    this->declare_parameter<std::vector<int>>(
        "propulsion.thrusters.thruster_PWM_min");
    this->declare_parameter<std::vector<int>>(
        "propulsion.thrusters.thruster_PWM_max");

    this->declare_parameter<std::vector<double>>("coeffs.LEFT");
    this->declare_parameter<std::vector<double>>("coeffs.RIGHT");

    this->declare_parameter<int>("i2c.bus");
    this->declare_parameter<int>("i2c.address");

    this->declare_parameter<std::string>("topics.thruster_forces");
    this->declare_parameter<std::string>("topics.pwm_output");

    this->declare_parameter<bool>("debug.flag");

    //-----------------------------------------------------------------------
    // get them
    auto thruster_mapping =
        this->get_parameter("propulsion.thrusters.thruster_to_pin_mapping")
            .as_integer_array();
    auto thruster_direction =
        this->get_parameter("propulsion.thrusters.thruster_direction")
            .as_integer_array();
    auto thruster_PWM_min =
        this->get_parameter("propulsion.thrusters.thruster_PWM_min")
            .as_integer_array();
    auto thruster_PWM_max =
        this->get_parameter("propulsion.thrusters.thruster_PWM_max")
            .as_integer_array();

    std::vector<double> left_coeffs =
        this->get_parameter("coeffs.LEFT").as_double_array();
    std::vector<double> right_coeffs =
        this->get_parameter("coeffs.RIGHT").as_double_array();

    this->i2c_bus_ = this->get_parameter("i2c.bus").as_int();
    this->i2c_address_ = this->get_parameter("i2c.address").as_int();

    this->subscriber_topic_name_ =
        this->get_parameter("topics.thruster_forces").as_string();
    this->publisher_topic_name_ =
        this->get_parameter("topics.pwm_output").as_string();

    this->debug_flag_ = this->get_parameter("debug.flag").as_bool();

    std::transform(
        thruster_mapping.begin(), thruster_mapping.end(),
        thruster_direction.begin(),
        std::back_inserter(this->thruster_parameters_),
        [&](const int64_t& mapping, const int64_t& direction) {
            size_t index =
                std::distance(thruster_mapping.begin(),
                              std::find(thruster_mapping.begin(),
                                        thruster_mapping.end(), mapping));
            return ThrusterParameters{
                static_cast<uint8_t>(mapping), static_cast<int8_t>(direction),
                static_cast<uint16_t>(thruster_PWM_min[index]),
                static_cast<uint16_t>(thruster_PWM_max[index])};
        });

    this->poly_coeffs_.push_back(left_coeffs);
    this->poly_coeffs_.push_back(right_coeffs);
}
