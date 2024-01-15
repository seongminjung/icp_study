#include <ros/ros.h>

#include "icp_study/icp.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "icp_node");
  ICP icp;
  ros::spin();
  return 0;
}