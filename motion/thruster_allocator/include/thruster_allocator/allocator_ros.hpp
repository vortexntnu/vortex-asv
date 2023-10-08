#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/wrench.hpp>

using namespace std::chrono_literals;

class Allocator : public rclcpp::Node
{
    public:
      explicit Allocator(): Node("thrust_allocator_publisher"), count_(0) 
      {
        publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("wrench_topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&Allocator::timer_callback, this));
      }
    private:
        void timer_callback()
        {
        auto message = geometry_msgs::msg::Wrench();
        message.force.x = 1;
        message.force.y = 1;
        message.force.z = 1;
        
        //message.set__force(geometry_msgs::msg::Vector3(1,1,1));
        
        double surge = message.force.x;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", surge);
        publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
        size_t count_;
};