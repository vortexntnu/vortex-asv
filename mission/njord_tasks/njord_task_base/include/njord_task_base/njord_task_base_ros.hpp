#ifndef NJORD_TASK_BASE_ROS_HPP
#define NJORD_TASK_BASE_ROS_HPP

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vortex_msgs/msg/landmark_array.hpp"
#include "vortex_msgs/srv/waypoint.hpp"
#include <Eigen/Dense>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>

class NjordTaskBaseNode : public rclcpp::Node {
public:
  NjordTaskBaseNode(const std::string &node_name,
                    const rclcpp::NodeOptions &options);

protected:
  void setup_map_odom_tf_and_subs();
  void set_gps_frame_coords();
  std::pair<double, double> lla2flat(double lat, double lon) const;

  void map_origin_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  std::shared_ptr<nav_msgs::msg::Odometry> get_odom();
  void landmark_callback(const vortex_msgs::msg::LandmarkArray::SharedPtr msg);
  std::shared_ptr<vortex_msgs::msg::LandmarkArray> get_landmarks_odom_frame();

  /**
   * @brief Assign buoys to landmarks based on the predicted and measured
   * positions using the auction algorithm.
   *
   * @param predicted_positions The predicted positions of the buoys
   * @param measured_positions The measured positions of landmarks
   *
   * @return A vector of size equal to number of predicted positions(buoys),
   * where each element is the index of the measured position(landmark) assigned
   * to the corresponding predicted position(buoy). If an element is -1, it
   * means that the corresponding predicted position(buoy) is not assigned to
   * any measured position(landmark).
   */
  Eigen::VectorXi assign_landmarks(
      const Eigen::Array<double, 2, Eigen::Dynamic> &predicted_positions,
      const Eigen::Array<double, 2, Eigen::Dynamic> &measured_positions);

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      gps_map_coord_visualization_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      waypoint_visualization_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr map_origin_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr
      landmarks_sub_;
  rclcpp::Client<vortex_msgs::srv::Waypoint>::SharedPtr waypoint_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::TransformStamped map_odom_tf_;

  mutable std::mutex setup_mutex_;
  mutable std::mutex odom_mutex_;
  mutable std::mutex landmark_mutex_;

  std::condition_variable odom_cond_var_;
  std::condition_variable landmark_cond_var_;

  std::shared_ptr<nav_msgs::msg::Odometry> odom_msg_;
  std::shared_ptr<vortex_msgs::msg::LandmarkArray> landmarks_msg_;
  bool new_odom_msg_ = false;
  bool new_landmark_msg_ = false;
};

#endif // NJORD_TASK_BASE_ROS_HPP
