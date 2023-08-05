#include "thruster_interface/thruster_interface_ros.hpp"

ThrusterInterfaceROS::ThrusterInterfaceROS() {
  sub = nh.subscribe("/thrust/thruster_forces", 10,
                     &ThrusterInterfaceROS::thrustCallback, this);
}

void ThrusterInterfaceROS::thrustCallback(
    const vortex_msgs::ThrusterForces::ConstPtr &msg) {
  // Convert from Newton to grams
  double newton_to_gram_conversion_factor = 101.97162;

  std::vector<double> forces_in_grams = {
      msg->thrust[0] * newton_to_gram_conversion_factor,
      msg->thrust[1] * newton_to_gram_conversion_factor,
      msg->thrust[2] * newton_to_gram_conversion_factor,
      msg->thrust[3] * newton_to_gram_conversion_factor};

  thrusterInterface.publish_thrust_to_escs(forces_in_grams);
}