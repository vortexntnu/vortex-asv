#include "thruster_interface/thruster_interface_ros.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "thruster_interface_node");
  ThrusterInterfaceROS thrusterInterfaceROS;

  ros::spin();

  return 0;
}