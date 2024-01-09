#include <landmarks/grid_visualization.hpp>

namespace grid_visualization {

GridVisualization::GridVisualization() {}

nav_msgs::msg::OccupancyGrid GridVisualization::createGrid(
    const vortex_msgs::msg::LandmarkArray &landmarkArray) {
  // Assuming some parameters for the occupancy grid
  int grid_width = 100;
  int grid_height = 100;

  // Initialize the OccupancyGrid message
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = grid_width;
  grid.info.height = grid_height;
  grid.info.resolution = 0.1; // physical distance between two cells in meters

  // Initialize the data array
  grid.data.resize(grid_width * grid_height,
                   0); // Initialize unoccupied grid
  for (const auto &landmark : landmarkArray.landmarks) {

    int x = landmark.odom.pose.pose.position.x / grid.info.resolution;
    int y = landmark.odom.pose.pose.position.y / grid.info.resolution;

    // Check bounds
    if (x >= 0 && x < grid.info.width && y >= 0 && y < grid.info.height) {
      // Increment the grid index
      grid.data[x + y * grid.info.width]++;
    }
  }

  return grid;
}

geometry_msgs::msg::PoseArray GridVisualization::poseArrayCreater(vortex_msgs::msg::LandmarkArray landmarks) {
  geometry_msgs::msg::PoseArray poseArray;
  poseArray.header.frame_id = "map";
  for (const auto &landmark : landmarks.landmarks) {
    // Convert landmark to pose here...
    poseArray.poses.push_back(landmark.odom.pose.pose);
  }
  return poseArray;
}

} // namespace grid_visualization