#include "thruster_interface_asv/thruster_interface_asv_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Started thruster_interface_asv_node");
    rclcpp::spin(std::make_shared<ThrusterInterfaceASVNode>());
    rclcpp::shutdown();
    return 0;
}
