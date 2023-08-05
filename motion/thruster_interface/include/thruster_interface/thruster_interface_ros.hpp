#ifndef THRUSTER_INTERFACE_ROS_HPP
#define THRUSTER_INTERFACE_ROS_HPP

#include "thruster_interface/thruster_interface.hpp"

#include "vortex_msgs/Pwm.h"
#include "vortex_msgs/ThrusterForces.h"

#include <ros/package.h>
#include <ros/ros.h>

class ThrusterInterfaceROS {
private:
  std::string mapping_file = ros::package::getPath("thruster_interface") +
                             "/config/ThrustMe_P1000_force_mapping.csv";
  ThrusterInterface thrusterInterface{mapping_file};
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pwm_pub;

public:
  ThrusterInterfaceROS();
  void thrustCallback(const vortex_msgs::ThrusterForces::ConstPtr &msg);
};

#endif // THRUSTER_INTERFACE_ROS_HPP
