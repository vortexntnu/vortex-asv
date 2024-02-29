#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <thruster_interface/thruster_interface.hpp>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        "float_topic", 10,
        std::bind(&MinimalSubscriber::float_topic_callback, this, _1));
  }

private:
  void float_topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%u'", interpolate(msg->data));
  }
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
