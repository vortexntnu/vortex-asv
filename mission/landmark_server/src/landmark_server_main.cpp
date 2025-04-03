#include <rclcpp/rclcpp.hpp>
#include "landmark_server/landmark_server.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<landmark_server::LandmarkServerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
