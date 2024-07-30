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

  /**
   * @brief Predict the positions of the west buoy by using the two first buoy
   * pairs
   *
   * @return An Eigen::Array<double, 2, 3> representing the predicted positions
   * of the second buoy pair and the west buoy
   */
  Eigen::Array<double, 2, 3>
  predict_west_buoy(const geometry_msgs::msg::Point &buoy_0,
                    const geometry_msgs::msg::Point &buoy_1,
                    const geometry_msgs::msg::Point &buoy_2,
                    const geometry_msgs::msg::Point &buoy_3);

  /**
   * @brief Predict the positions of the third buoy pair by using the two first
   * buoy pairs
   *
   * @return An Eigen::Array<double, 2, 2> representing the predicted positions
   * of the third buoy pair
   */
  Eigen::Array<double, 2, 2>
  predict_third_buoy_pair(const geometry_msgs::msg::Point &buoy_0,
                          const geometry_msgs::msg::Point &buoy_1,
                          const geometry_msgs::msg::Point &buoy_2,
                          const geometry_msgs::msg::Point &buoy_3);

  /**
   * @brief Predict the positions of the north buoy by using the two first buoy
   * pairs
   *
   * @return An Eigen::Array<double, 2, 3> representing the predicted positions
   * of the third buoy pair and the north buoy
   */
  Eigen::Array<double, 2, 3> predict_north_buoy(
      const geometry_msgs::msg::Point &buoy_5,
      const geometry_msgs::msg::Point &buoy_6,
      const Eigen::Vector2d &direction_vector_second_to_third_pair);

  /**
   * @brief Predict the positions of the south buoy by using the two first buoy
   * pairs
   *
   * @return An Eigen::Array<double, 2, 2> representing the predicted positions
   * of the north and south buoys
   */
  Eigen::Array<double, 2, 2> predict_south_buoy(
      const geometry_msgs::msg::Point &buoy_5,
      const geometry_msgs::msg::Point &buoy_6,
      const geometry_msgs::msg::Point &buoy_7,
      const Eigen::Vector2d &direction_vector_second_to_third_pair);

  /**
   * @brief Predict the positions of the fourth buoy pair by using the third
   * buoy pair
   *
   * @return An Eigen::Array<double, 2, 2> representing the predicted positions
   * of the fourth buoy pair
   */
  Eigen::Array<double, 2, 2>
  predict_fourth_buoy_pair(const geometry_msgs::msg::Point &buoy_5,
                           const geometry_msgs::msg::Point &buoy_6);

  /**
   * @brief Predict the position of the east buoy by using the fourth buoy pair
   * and the direction vector from the second to the third buoy pair
   *
   * @return An Eigen::Array<double, 2, 3> representing the predicted positions
   * of the fourth buoy pair and the east buoy
   */
  Eigen::Array<double, 2, 3> predict_east_buoy(
      const geometry_msgs::msg::Point &buoy_9,
      const geometry_msgs::msg::Point &buoy_10,
      const Eigen::Vector2d &direction_vector_second_to_third_pair);

  /**
   * @brief Predict the position of the fifth buoy pair by using the fourth buoy
   * pair and the direction vector from the second to the third buoy pair
   *
   * @return An Eigen::Array<double, 2, 3> representing the predicted positions
   * of the fifth buoy pair and the east buoy
   */
  Eigen::Array<double, 2, 3> predict_fifth_buoy_pair(
      const geometry_msgs::msg::Point &buoy_9,
      const geometry_msgs::msg::Point &buoy_10,
      const geometry_msgs::msg::Point &buoy_11,
      const Eigen::Vector2d &direction_vector_second_to_third_pair);

  /**
   * @brief Predict the position of the sixth buoy pair by using the fifth buoy
   * pair and fourth buoy pair
   *
   * @return An Eigen::Array<double, 2, 4> representing the predicted positions
   * of the fifth and sixth buoy pairs
   */
  Eigen::Array<double, 2, 4>
  predict_sixth_buoy_pair(const geometry_msgs::msg::Point &buoy_9,
                          const geometry_msgs::msg::Point &buoy_10,
                          const geometry_msgs::msg::Point &buoy_12,
                          const geometry_msgs::msg::Point &buoy_13);
};

} // namespace navigation_task

#endif // NAVIGATION_TASK_ROS_HPP