#include <landmarks/landmarks.hpp>

using std::placeholders::_1;

namespace landmarks {


  LandmarksNode::LandmarksNode(const rclcpp::NodeOptions& options) : Node("landmarks_node",options) {
    // Define the quality of service profile for publisher and subscriber
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1),
                           qos_profile);
    storedLandmarks_ = std::make_shared<vortex_msgs::msg::LandmarkArray>();
    subscription_ =
        this->create_subscription<vortex_msgs::msg::LandmarkArray>(
            "landmarks_in", qos,
            std::bind(&LandmarksNode::landmarksRecievedCallback, this, _1));
    landmarkPublisher_ =
        this->create_publisher<vortex_msgs::msg::LandmarkArray>(
            "landmarks_out", qos);
    gridPublisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_out", qos);
    grid_ = createGrid(*storedLandmarks_);
    posePublisher_ =
        this->create_publisher<geometry_msgs::msg::PoseArray>("pose_out", qos);
    twistPublisher_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>("twist_out", qos);

    // Sets up a timer to trigger the publishMessages function every 1 second
    // timer_ = this->create_wall_timer(
    //     std::chrono::second(1),
    //     std::bind(&LandmarksNode::publishLandmarks, this));
  }

  

  void LandmarksNode::landmarksRecievedCallback(
    
    const vortex_msgs::msg::LandmarkArray::SharedPtr msg) {
    for (const auto &landmark : msg->landmarks) {
      RCLCPP_INFO(this->get_logger(), "Landmarks received");
        if (landmark.action == 0) {
            for (int i = 0; i < storedLandmarks_->landmarks.size(); i++) {
                if ((storedLandmarks_)->landmarks[i].id == landmark.id &&
                    (storedLandmarks_)->landmarks[i].landmark_type == landmark.landmark_type) {

                    updateGrid((storedLandmarks_->landmarks)[i], 0);
                    (storedLandmarks_->landmarks).erase(storedLandmarks_->landmarks.begin() + i);

                    break;
                }
            }
        } else if (landmark.action == 1) {
            bool landmarkUpdated = false;

            for (auto &storedLandmark : storedLandmarks_->landmarks) {
                if (landmark.landmark_type == storedLandmark.landmark_type &&
                    landmark.id == storedLandmark.id) {
                    // Update existing landmark if namespace and id match
                    updateGrid(storedLandmark, 0);
                    updateGrid(landmark, 1);
                    storedLandmark = landmark;
                    landmarkUpdated = true;
                    break;
                }
            }

            if (!landmarkUpdated && landmark.action == 1) {
                // If no match is found and action is 1, add the new landmark
                (storedLandmarks_->landmarks).push_back(landmark);
                updateGrid(landmark, 1);
            }
        }
    }
    //   publish after each landmark is updated or after the callback is done? Or just timer based?
    landmarkPublisher_->publish(*storedLandmarks_);
    gridPublisher_->publish(grid_);
    posePublisher_->publish(poseArrayCreater(*storedLandmarks_));
    twistCreater(*storedLandmarks_);
}


    nav_msgs::msg::OccupancyGrid LandmarksNode::createGrid(
        vortex_msgs::msg::LandmarkArray landmarks) {
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
        for(const auto &landmark : landmarks.landmarks){
            updateGrid(landmark,1);
        }
      return grid;
        }


      void LandmarksNode::updateGrid(vortex_msgs::msg::Landmark landmark, int number ){
      // Update the data array based on landmark poses
    
        int x = landmark.pose.position.x / grid_.info.resolution;
        int y = landmark.pose.position.y / grid_.info.resolution;

        // Check bounds
        if (x >= 0 && x < grid_.info.width && y >= 0 && y < grid_.info.height) {
          grid_.data[x + y * grid_.info.width] = number;
        }
    
  }

  geometry_msgs::msg::PoseArray LandmarksNode::poseArrayCreater(vortex_msgs::msg::LandmarkArray landmarks){
      geometry_msgs::msg::PoseArray poseArray;
      poseArray.header.stamp = this->get_clock()->now();
      poseArray.header.frame_id = "pose_map";
      for (const auto& landmark : landmarks.landmarks) {
      // Convert landmark to pose here...
        poseArray.poses.push_back(landmark.pose);
        }
        return poseArray;
  }

  void LandmarksNode::twistCreater(vortex_msgs::msg::LandmarkArray landmarks){
     
      for (auto landmark : landmarks.landmarks) {
        landmark.twist.header.stamp = this->get_clock()->now();
        landmark.twist.header.frame_id = "pose_map";
      // Convert landmark to pose here...
        twistPublisher_->publish(landmark.twist);
        }
  }

// publiser pose of landmark som en liste med velocity, er bare for visualisering
// fisk grid så flere kan okkupere samme celle
// start med actions for filtrering

} // namespace landmarks