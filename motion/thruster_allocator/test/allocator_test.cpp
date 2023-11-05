#include <geometry_msgs/msg/wrench.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <thruster_allocator/allocator_ros.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>

#include <chrono>
#include <functional>

class AllocatorTest : public testing::Test, public rclcpp::Node {
protected:
  //  std::shared_ptr<ThrusterAllocator> node_;
public:
  AllocatorTest() : Node("AllocatorTest_node") {
    subscription_ = this->create_subscription<vortex_msgs::msg::ThrusterForces>(
        "thrust/thruster_forces", 1,
        std::bind(&AllocatorTest::thruster_forces_callback, this,
                  std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>(
        "thrust/wrench_input", 1);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&AllocatorTest::timer_callback, this));

    message_received = false;
  }
  std::vector<double> thrust;
  const double MAX_ERROR = 1e-4;

  void SetUp() override {
    // rclcpp::spin(node_);
    while (!isNodeReady()) {
      timer_callback();
    }
  }

  void Publish(double surge, double sway, double yaw) {
    geometry_msgs::msg::Wrench msg;
    msg.force.x = surge;
    msg.force.y = sway;
    msg.torque.z = yaw;
    publisher_->publish(msg);
  }

  void ExpectThrustNear(double *arr) {
    for (int i = 0; i < static_cast<int>(thrust.size()); ++i)
      EXPECT_NEAR(thrust[i], arr[i], MAX_ERROR);
  }

  void WaitForMessage() {
    while (!message_received)
      timer_callback();
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
  rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::SharedPtr
      subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool message_received;
  void thruster_forces_callback(
      const vortex_msgs::msg::ThrusterForces &thruster_msg) {
    thrust = thruster_msg.thrust;
    message_received = true;
  };
  void timer_callback() { std::cout << "hello!" << std::endl; };

  bool isNodeReady() {
    return (count_subscribers("thrust/thruster_forces") > 0) &&
           (count_publishers("thrust/wrench_input") > 0);
  }
};

TEST_F(AllocatorTest, DumbTest) { ASSERT_EQ(1, 1); }

// TEST_F(AllocatorTest, CheckResponsiveness) {
//   Publish(0, 0, 0);
//   WaitForMessage();
// }

// TEST_F(AllocatorTest, ZeroInput) {
//   Publish(0, 0, 0);
//   WaitForMessage();

//   double thrust_expected[] = {0, 0, 0};
//   ExpectThrustNear(thrust_expected);
// }

// TEST_F(AllocatorTest, TurnRight) {
//   Publish(0, 0, 1);
//   WaitForMessage();
//   double thrust_expected[] = {1.1666, -1.1666, 1.1666, -1.1666, 0.0, 0.0};
//   ExpectThrustNear(thrust_expected);
// }

// TEST_F(AllocatorTest, YourTestCaseName) {
//   auto allocator_test = std::make_shared<AllocatorTest>();
//   rclcpp::spin_some(allocator_test);
//   ASSERT_EQ(1, 1); // Example assertion
// }
