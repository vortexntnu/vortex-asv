#include <rclcpp/rclcpp.hpp>
#include "landmarks/landmarks.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<landmarks::LandmarksNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
