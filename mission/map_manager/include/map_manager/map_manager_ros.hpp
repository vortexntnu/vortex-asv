#ifndef MAP_MANAGER_HPP
#define MAP_MANAGER_HPP

#include "nav_msgs/srv/get_map.hpp"
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

namespace map_manager {

/**
 * @class MapManagerNode
 * @brief A class representing a node for handling maps in a ROS 2 system.
 *
 * This class inherits from rclcpp::Node and provides functionality for
 * receiving map messages and storing them in a map.
 */
class MapManagerNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the MapManagerNode class.
   *
   * @param options The options for configuring the node.
   */
  explicit MapManagerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for the MapManagerNode class.
   */
  ~MapManagerNode(){};

private:
  constexpr double deg2rad(double degrees) const;
  std::array<double, 2> lla2flat(double lat, double lon) const;
  std::array<double, 2> flat2lla(double px, double py) const;

  geometry_msgs::msg::PolygonStamped
  processCoordinates(const std::vector<std::array<double, 2>> &coordinates);
  geometry_msgs::msg::PolygonStamped
  readPolygonFromFile(const std::string &filename);
  void mapOriginCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  nav_msgs::msg::OccupancyGrid createOccupancyGrid();
  geometry_msgs::msg::Pose calculate_map_origin();
  void insert_landmask(nav_msgs::msg::OccupancyGrid &grid,
                       const geometry_msgs::msg::PolygonStamped &polygon);
  void handle_get_map_request(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
      const std::shared_ptr<nav_msgs::srv::GetMap::Response> response);
  void fillOutsidePolygon(nav_msgs::msg::OccupancyGrid &grid,
                          const geometry_msgs::msg::PolygonStamped &polygon);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr map_origin_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      landmask_pub_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr grid_service_;
  bool map_origin_set_ = false;
  bool use_predef_landmask_;
  double map_origin_lat_;
  double map_origin_lon_;
  std::string landmask_file_;
  std::string grid_frame_id_;
};

} // namespace map_manager

#endif // MAP_MANAGER_HPP