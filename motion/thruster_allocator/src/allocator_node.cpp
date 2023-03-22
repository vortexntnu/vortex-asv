#include "ros/ros.h"

#include "vortex_allocator/allocator_ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "allocator");
  ros::NodeHandle nh;
  
  ros::Rate r(10); // 10 hz // TODO: Sync with controller
  Allocator allocator(nh);
  while (ros::ok) {
    allocator.spinOnce();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
