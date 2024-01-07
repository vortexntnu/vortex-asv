#include <landmarks/landmarks.hpp>

using std::placeholders::_1;

namespace landmarks {

LandmarksNode::LandmarksNode() : Node("landmarks_node") {

  LandmarksNode::LandmarksNode() : Node("landmarks_node") {
    // Define the quality of service profile for publisher and subscriber
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1),
                           qos_profile);
    storedLandmarks_ = std::make_shared<vector<vortex_msgs::msg::Landmark>>();
    subscription_ =
        this->create_subscription<vector<vortex_msgs::msg::Landmark>>(
            "landmarks_in", qos,
            std::bind(&LandmarksNode::landmarksRecievedCallback, this, _1));
    landmarkPublisher_ =
        this->create_publisher<vector<vortex_msgs::msg::Landmark>>(
            "landmarks_out", qos);
    gridPublisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_out", qos);
    grid_ = createGrid(storedLandmarks_);

    // Sets up a timer to trigger the publishMessages function every 1 second
    // timer_ = this->create_wall_timer(
    //     std::chrono::second(1),
    //     std::bind(&LandmarksNode::publishLandmarks, this));
  }

  void LandmarksNode::landmarksRecievedCallback(
      const std::vector<vortex_msgs::msg::Landmark::SharedPtr> msg) {
    for (const auto &landmark : msg) {
      if (landmark.action == 0) {
        for (int i = 0; i < storedLandmarks_.size(); i++) {
          if (storedLandmarks_[i].id == landmark.id &&
              storedLandmarks_[i].namespace == landmark.namespace) {
            storedLandmarks_.erase(storedLandmarks_.begin() + i);
          }
        }
      } else if (landmark.action == 1) {
        bool landmarkUpdated = false;

        for (auto &storedLandmark : storedLandmarks_) {
          if (incomingLandmark.namespace == storedLandmark->namespace &&
              incomingLandmark.id == storedLandmark->id) {
            // Update existing landmark if namespace and id match
            storedLandmark = incomingLandmark;
            landmarkUpdated = true;
            break;
          }
        }

        if (!landmarkUpdated && incomingLandmark.action == 1) {
          // If no match is found and action is 1, add the new landmark
          storedLandmarks_.push_back(incomingLandmark);
        }
      }
    //   publish after each landmark is updated or after the callback is done? Or just timer based?
      landmarkPublisher_->publish(storedLandmarks_);
      gridPublisher_->publish(createGrid(storedLandmarks_));
    }
      }

    nav_msgs::msg::OccupancyGrid LandmarksNode::createGrid(
        std::vector<vortex_msgs::msg::Landmark> landmarks) {
      // Assuming some parameters for the occupancy grid
      int grid_width = 100;
      int grid_height = 100;

      // Initialize the OccupancyGrid message
      nav_msgs::msg::OccupancyGrid grid;
      grid.header.stamp = this->get_clock()->now();
      grid.header.frame_id = "map";
      grid.info.width = grid_width;
      grid.info.height = grid_height;
      grid.info.resolution =
          0.1; // physical distance between two cells in meters

      // Initialize the data array
      grid.data.resize(grid_width * grid_height,
                       0); // Initialize unoccupied grid

      // Update the data array based on landmark poses
      for (const auto &landmark : landmarks) {
        int x = landmark.pose.position.x / grid.info.resolution;
        int y = landmark.pose.position.y / grid.info.resolution;

        // Check bounds
        if (x >= 0 && x < grid_width && y >= 0 && y < grid_height) {
          grid.data[x + y * grid.info.width] = 1;
        }
      }
      return grid;
    }
  }


} // namespace landmarks