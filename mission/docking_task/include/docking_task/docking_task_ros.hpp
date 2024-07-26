#ifndef DOCKING_TASK_ROS_HPP
#define DOCKING_TASK_ROS_HPP

#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/srv/waypoint.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h> 

namespace docking_task {

struct LandmarkWithID {
      geometry_msgs::msg::Pose pose_map_frame;
      geometry_msgs::msg::Pose pose_base_link_frame;
      uint32_t id;
    };

/**
 * @class DockingTaskNode
 * @brief A class representing a node for handling dock localization in a ROS 2 system.
 *
 * This class inherits from rclcpp::Node and provides functionality for
 * localizing the dock in the map.
 */
class DockingTaskNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the DockingTaskNode class.
   *
   * @param options The options for configuring the node.
   */
  explicit DockingTaskNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for the DockLocalizationNode class.
   */
  ~DockingTaskNode(){};

  /**
   * @brief Main task for the DockingTaskNode class.
   */
  void main_task();

  /**
   * @brief Detect the closest buoy pair and set a waypoint between them
   *
   * @return A pair of landmarks representing the closest buoy pair
   */
  std::pair<LandmarkWithID, LandmarkWithID> initial_waypoint();

  /**
   * @brief Predict the 6-tuple formation of buoys
   *
   * @param landmark1 The first landmark/buoy used for initial waypoint
   * @param landmark2 The second landmark/buoy used for initial waypoint
   * 
   * @return The predicted posistion of the 6-tuple formation of buoys in odom frame
   */
  Eigen::Array<double, 2, 6> predict_buoy_formation(const LandmarkWithID &buoy1, const LandmarkWithID &buoy2) const;

  /**
   * @brief Generate the reward matrix for the auction algorithm
   *
   * @param predicted_positions The predicted positions of the buoys
   * @param measured_positions The measured positions of landmarks
   * 
   * @return The reward matrix for the auction algorithm
   */
  Eigen::MatrixXd generate_reward_matrix(const Eigen::Array<double, 2, 6>& predicted_positions, const Eigen::MatrixXd& measured_positions);

  /**
   * @brief The auction algorithm for the assignment problem
   *
   * @param reward_matrix The reward matrix for the auction algorithm
   * 
   * @return The assignment of landmarks to buoys
   */
  Eigen::VectorXi auction_algorithm(const Eigen::MatrixXd &reward_matrix);

  /**
   * @brief Navigate the ASV through the formation of buoys. First travels to waypoint between second pair of buoys,
   * then navigates through the formation of buoys and returns control when asv is 0.2m away from crossing the last buoy pair.
   *
   * @param predicted_positions The predicted positions of the buoys
   */
  void navigate_formation(const Eigen::Array<double, 2, 6>& predicted_positions);

  /**
   * @brief Calculate the coordinates of a gps point in the map frame
   *
   * @param lat The latitude of the gps point
   * @param lon The longitude of the gps point
   * 
   * @return The coordinates of the gps point in the map frame
   */
  std::pair<double, double> lla2flat(double lat, double lon) const;

  /**
   * @brief Utility function to get the heading of the ASV assuming identical orientation between odom and map frame
   *
   * @param msg The quat message derived from the odom message published by the seapath
   */
  double get_freya_heading(const geometry_msgs::msg::Quaternion msg) const;

  /**
   * @brief Utility function to set the position of the GPS coordinates in the map frame
   */
  void set_gps_frame_coords();

  /**
   * @brief Utility function that runs until waypoint is reached.
   * Function returns when within distance_threshold of the waypoint.
   * 
   * @param distance_threshold The distance threshold for reaching the waypoint
   */
  void reach_waypoint(const double distance_threshold);



private:
  mutable std::mutex odom_mutex_;
  mutable std::mutex grid_mutex_;
  mutable std::mutex landmark_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr map_origin_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr landmarks_sub_;

  rclcpp::Client<vortex_msgs::srv::Waypoint>::SharedPtr waypoint_client_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_visualization_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr gps_map_coord_visualization_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg_;
  nav_msgs::msg::Odometry::SharedPtr odom_msg_;
  vortex_msgs::msg::LandmarkArray::SharedPtr landmarks_msg_;

  geometry_msgs::msg::Point previous_waypoint_odom_frame_;

};

} // namespace docking_task

#endif // DOCKING_TASK_ROS_HPP