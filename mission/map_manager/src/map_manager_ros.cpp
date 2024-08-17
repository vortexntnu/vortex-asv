#include <map_manager/map_manager_ros.hpp>

namespace map_manager {

MapManagerNode::MapManagerNode(const rclcpp::NodeOptions &options)
    : Node("map_manager_node", options) {

  rmw_qos_profile_t qos_profile_transient_local = rmw_qos_profile_parameters;
  qos_profile_transient_local.durability =
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  auto qos_transient_local = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile_transient_local.history, 1),
      qos_profile_transient_local);

  declare_parameter("use_predef_landmask", false);
  declare_parameter(
      "landmask_file",
      "src/vortex-asv/mission/map_manager/params/land_polygon.yaml");
  declare_parameter("map_resolution", 0.2);
  declare_parameter("map_width", 1000);
  declare_parameter("map_height", 1000);
  declare_parameter("frame_id", "map");
  declare_parameter("map_origin_lat", 0.0);
  declare_parameter("map_origin_lon", 0.0);
  declare_parameter("use_predef_map_origin", false);

  landmask_file_ = get_parameter("landmask_file").as_string();
  landmask_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "landmask", qos_transient_local);
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", qos_transient_local);

  map_origin_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/map/origin", qos_transient_local);


  if (this->get_parameter("use_predef_map_origin").as_bool()) {
    map_origin_lat_ = this->get_parameter("map_origin_lat").as_double();
    map_origin_lon_ = this->get_parameter("map_origin_lon").as_double();
    map_origin_set_ = true;

    auto grid = createOccupancyGrid();
    if (this->get_parameter("use_predef_landmask").as_bool()) {
      auto polygon = readPolygonFromFile(landmask_file_);
      landmask_pub_->publish(polygon);
      fillOutsidePolygon(grid, polygon);
      insert_landmask(grid, polygon);
    }

    map_pub_->publish(grid);
  } else {
    odom_origin_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/oodm/origin", qos_transient_local,
        std::bind(&MapManagerNode::mapOriginCallback, this,
                  std::placeholders::_1));
  }

  grid_service_ = this->create_service<nav_msgs::srv::GetMap>(
      "get_map",
      std::bind(&MapManagerNode::handle_get_map_request, // Callback function
                this,                  // Pointer to the object
                std::placeholders::_1, // Placeholder for request header
                std::placeholders::_2, // Placeholder for request
                std::placeholders::_3  // Placeholder for response
                ));
}

void MapManagerNode::mapOriginCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  double odom_origin_lat = msg->latitude;
  double odom_origin_lon = msg->longitude;
  map_origin_set_ = true;
  odom_origin_sub_ = nullptr;
  publish_map_to_odom_tf(odom_origin_lat, odom_origin_lon, msg->header.stamp);
  publish_foxglove_vis_frame(msg->header.stamp);
  if (this->get_parameter("use_predef_landmask").as_bool()) {
    landmask_pub_->publish(readPolygonFromFile(landmask_file_));
  }
}

void MapManagerNode::handle_get_map_request(
    [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
    [[maybe_unused]] const std::shared_ptr<nav_msgs::srv::GetMap::Request>
        request,
    const std::shared_ptr<nav_msgs::srv::GetMap::Response> response) {
  if (!map_origin_set_) {
    RCLCPP_WARN(this->get_logger(), "Map origin not set, cannot provide map");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Received request for map");

  nav_msgs::msg::OccupancyGrid grid = createOccupancyGrid();
  if (this->get_parameter("use_predef_landmask").as_bool()) {
    auto polygon = readPolygonFromFile(landmask_file_);
    fillOutsidePolygon(grid, polygon);
    insert_landmask(grid, polygon);
  }
  response->map = grid;
  RCLCPP_INFO(this->get_logger(), "Map sent");
}

constexpr double MapManagerNode::deg2rad(double degrees) const {
  return degrees * M_PI / 180.0;
}

std::array<double, 2> MapManagerNode::lla2flat(double lat, double lon) const {
  const double R = 6378137.0;           // WGS-84 Earth semimajor axis (meters)
  const double f = 1.0 / 298.257223563; // Flattening of the earth
  const double psi_rad =
      0.0; // Angular direction of the flat Earth x-axis, specified as a scalar.
           // The angular direction is the degrees clockwise from north,
           // which is the angle in degrees used for converting flat Earth x and
           // y coordinates to the north and east coordinates

  // Convert angles from degrees to radians
  const double lat_rad = deg2rad(lat);
  const double lon_rad = deg2rad(lon);
  const double origin_lat_rad = deg2rad(map_origin_lat_);
  const double origin_lon_rad = deg2rad(map_origin_lon_);

  // Calculate delta latitude and delta longitude in radians
  const double dlat = lat_rad - origin_lat_rad;
  const double dlon = lon_rad - origin_lon_rad;

  // Radius of curvature in the vertical prime (RN)
  const double RN =
      R / sqrt(1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));

  // Radius of curvature in the meridian (RM)
  const double RM = RN * (1.0 - (2.0 * f - f * f)) /
                    (1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));

  // Changes in the north (dN) and east (dE) positions
  const double dN = RM * dlat;
  const double dE = RN * cos(origin_lat_rad) * dlon;

  // Transformation from North-East to flat x-y coordinates
  const double px = cos(psi_rad) * dN - sin(psi_rad) * dE;
  const double py = sin(psi_rad) * dN + cos(psi_rad) * dE;

  return {px, py};
}

 void MapManagerNode::publish_map_to_odom_tf(double map_lat, double map_lon, const rclcpp::Time& time) const
        {
        // Setup the transform
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = time;
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "odom";

        auto [x, y] = lla2flat(map_lat, map_lon);

        transformStamped.transform.translation.x = -x;
        transformStamped.transform.translation.y = -y;
        transformStamped.transform.translation.z = 0;

        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        // Broadcast the static transform
        static_tf_broadcaster_->sendTransform(transformStamped);

    }

  void MapManagerNode::publish_foxglove_vis_frame(const rclcpp::Time& time) const
        {
        // Setup the transform
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = time;
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "map_visualization";

        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        // NED to SEU
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 1.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 0.0;

        // Broadcast the static transform
        static_tf_broadcaster_->sendTransform(transformStamped);

    }

std::array<double, 2> MapManagerNode::flat2lla(double px, double py) const {
  // Earth constants
  const double R = 6378137.0;           // WGS-84 Earth semimajor axis (meters)
  const double f = 1.0 / 298.257223563; // Flattening of the earth

  // Convert origin from degrees to radians
  const double origin_lat_rad = deg2rad(map_origin_lat_);
  const double origin_lon_rad = deg2rad(map_origin_lon_);

  // Radius of curvature in the vertical prime (RN) and in the meridian (RM)
  const double RN =
      R / sqrt(1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));
  const double RM = RN * (1.0 - (2.0 * f - f * f)) /
                    (1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));

  // Calculate changes in latitude and longitude
  double dN = px; // Change in north direction
  double dE = py; // Change in east direction
  double dlat = dN / RM;
  double dlon = dE / (RN * cos(origin_lat_rad));

  // Convert delta lat and lon from radians back to degrees
  double new_lat = (origin_lat_rad + dlat) * 180.0 / M_PI;
  double new_lon = (origin_lon_rad + dlon) * 180.0 / M_PI;

  return {new_lat, new_lon};
}

geometry_msgs::msg::PolygonStamped
MapManagerNode::readPolygonFromFile(const std::string &filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + filename);
  }

  std::string line;
  std::vector<std::array<double, 2>> coords;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    double lat, lon;
    if (ss >> lat >> lon) {
      coords.push_back({lat, lon});
    } else {
      std::cerr << "Failed to parse line: " << line << std::endl;
    }
  }

  file.close();
  std::cout << "coords size: " << coords.size() << "\n";

  return processCoordinates(coords);
}

geometry_msgs::msg::PolygonStamped MapManagerNode::processCoordinates(
    const std::vector<std::array<double, 2>> &coordinates) {
  geometry_msgs::msg::PolygonStamped polygon;
  polygon.header.frame_id = get_parameter("frame_id").as_string();
  polygon.header.stamp = this->now();
  std::cout << "coords size: " << coordinates.size() << "\n";
  for (const auto &coord : coordinates) {
    std::array<double, 2> flat = lla2flat(coord[0], coord[1]);
    geometry_msgs::msg::Point32 point;
    point.x = flat[0];
    point.y = flat[1];
    point.z = 0;
    polygon.polygon.points.push_back(point);
  }
  std::cout << "Polygon size: " << polygon.polygon.points.size() << std::endl;
  return polygon;
}

nav_msgs::msg::OccupancyGrid MapManagerNode::createOccupancyGrid() {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = get_parameter("frame_id").as_string();
  grid.header.stamp = this->now();
  grid.info.resolution = get_parameter("map_resolution").as_double();
  grid.info.width = get_parameter("map_width").as_int();
  grid.info.height = get_parameter("map_height").as_int();
  grid.info.origin = calculate_map_origin();
  grid.info.map_load_time = this->now();
  // initialize grid with zeros
  grid.data.resize(grid.info.width * grid.info.height, 0);
  return grid;
}

geometry_msgs::msg::Pose MapManagerNode::calculate_map_origin() {

  // Map centre is (0,0) in map frame, origin is bottom left corner
  double half_width_meters = -(get_parameter("map_width").as_int() *
                               get_parameter("map_resolution").as_double()) /
                             2.0;
  double half_height_meters = -(get_parameter("map_height").as_int() *
                                get_parameter("map_resolution").as_double()) /
                              2.0;
  geometry_msgs::msg::Pose map_origin;
  map_origin.position.x = half_width_meters;
  map_origin.position.y = half_height_meters;
  map_origin.position.z = 0.0;
  map_origin.orientation.x = 0.0;
  map_origin.orientation.y = 0.0;
  map_origin.orientation.z = 0.0;
  map_origin.orientation.w = 1.0;
  return map_origin;
}

// Helper function to draw line using Bresenham's algorithm
void drawLine(int x0, int y0, int x1, int y1,
              nav_msgs::msg::OccupancyGrid &grid, int value) {
  bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
  if (steep) {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }
  int dx = x1 - x0;
  int dy = std::abs(y1 - y0);
  int error = 2 * dy - dx;
  int ystep = (y0 < y1) ? 1 : -1;
  int xbounds = steep ? grid.info.height : grid.info.width;
  int ybounds = steep ? grid.info.width : grid.info.height;
  int y = y0;

  for (int x = x0; x <= x1; x++) {
    if (steep) {
      if (x >= 0 && x < xbounds && y >= 0 && y < ybounds) {
        grid.data[y + x * grid.info.width] = value;
      }
    } else {
      if (y >= 0 && y < ybounds && x >= 0 && x < xbounds) {
        grid.data[y * grid.info.width + x] = value;
      }
    }
    if (error > 0) {
      y += ystep;
      error -= 2 * (dx - dy);
    } else {
      error += 2 * dy;
    }
  }
}

bool isPointInsidePolygon(int px, int py,
                          const geometry_msgs::msg::PolygonStamped &polygon,
                          const nav_msgs::msg::OccupancyGrid &grid) {
  int count = 0;
  int n = polygon.polygon.points.size();
  for (int i = 0; i < n; i++) {
    const geometry_msgs::msg::Point32 &start = polygon.polygon.points[i];
    const geometry_msgs::msg::Point32 &end =
        polygon.polygon.points[(i + 1) % n];
    int x0 = start.x / grid.info.resolution + grid.info.width / 2;
    int y0 = start.y / grid.info.resolution + grid.info.height / 2;
    int x1 = end.x / grid.info.resolution + grid.info.width / 2;
    int y1 = end.y / grid.info.resolution + grid.info.height / 2;

    // Check if the line from start to end intersects with the line from (px,
    // py) to (infinity, py)
    if ((y0 > py) !=
        (y1 > py)) { // If the point is between the y bounds of the segment
      // Calculate x coordinate of the intersection point of the line segment
      // with the horizontal line through py
      float intersectX =
          (y1 - y0 != 0) ? (x1 - x0) * (float)(py - y0) / (float)(y1 - y0) + x0
                         : x0;
      if (px < intersectX) // Ray crossing from the left
        count++;
    }
  }
  return (count % 2) == 1; // Odd inside, even outside
}

void MapManagerNode::fillOutsidePolygon(
    nav_msgs::msg::OccupancyGrid &grid,
    const geometry_msgs::msg::PolygonStamped &polygon) {
  int outside_value = 50; // Set occupancy value for outside (0-100)

  // Iterate over each pixel in the grid
  for (size_t x = 0; x < grid.info.width; x++) {
    for (size_t y = 0; y < grid.info.height; y++) {
      if (!isPointInsidePolygon(x, y, polygon, grid)) {
        grid.data[y * grid.info.width + x] = outside_value;
      }
    }
  }
}

void MapManagerNode::insert_landmask(
    nav_msgs::msg::OccupancyGrid &grid,
    const geometry_msgs::msg::PolygonStamped &polygon) {
  int value = 100; // Set this to the desired occupancy value for land (0-100)
  for (uint i = 0; i < polygon.polygon.points.size(); i++) {

    const geometry_msgs::msg::Point32 &current = polygon.polygon.points[i];
    const geometry_msgs::msg::Point32 &next =
        polygon.polygon
            .points[(i + 1) % polygon.polygon.points.size()]; // Loop back to
                                                              // the first point
    // Convert map xy to grid indices
    int x0 = current.x / grid.info.resolution + grid.info.width / 2;
    int y0 = current.y / grid.info.resolution + grid.info.height / 2;
    int x1 = next.x / grid.info.resolution + grid.info.width / 2;
    int y1 = next.y / grid.info.resolution + grid.info.height / 2;

    // Draw line on the grid
    drawLine(x0, y0, x1, y1, grid, value);
  }
}

} // namespace map_manager