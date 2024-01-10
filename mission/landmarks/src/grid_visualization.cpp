#include <landmarks/grid_visualization.hpp>

namespace grid_visualization {

GridVisualization::GridVisualization() {}



geometry_msgs::msg::PoseArray GridVisualization::poseArrayCreater(vortex_msgs::msg::LandmarkArray landmarks) {
  geometry_msgs::msg::PoseArray poseArray;
  poseArray.header.frame_id = "os_lidar";
  for (const auto &landmark : landmarks.landmarks) {
    // Convert landmark to pose here...
    geometry_msgs::msg::Pose gridPose;
    poseArray.poses.push_back(landmark.odom.pose.pose);
  }
  return poseArray;
}

} // namespace grid_visualization