#ifndef NAVIGATION_TASK_ROS_HPP
#define NAVIGATION_TASK_ROS_HPP

#include <njord_task_base/njord_task_base_ros.hpp>

namespace navigation_task {

class NavigationTaskNode : public NjordTaskBaseNode {
public:
  explicit NavigationTaskNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~NavigationTaskNode(){};

  /**
   * @brief Main task for the NavigationTaskNode class.
   */
  void main_task();

  /**
   * @brief Predict the positions of the first two buoys
   *
   * @return An Eigen::Array<double, 2, 2> representing the predicted positions
   * of the first two buoys
   */
  Eigen::Array<double, 2, 2> predict_first_buoy_pair();

  /**
   * @brief Predict the positions of the first two buoys pairs
   *
   * @return An Eigen::Array<double, 2, 4> representing the predicted positions
   * of the first two buoy pairs
   */
  Eigen::Array<double, 2, 4>
  predict_first_and_second_buoy_pair(const geometry_msgs::msg::Point &buoy_0,
                                     const geometry_msgs::msg::Point &buoy_1);

private:
};

} // namespace navigation_task

#endif // NAVIGATION_TASK_ROS_HPP