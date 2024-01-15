#ifndef ICP_H
#define ICP_H

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>

#include "icp_study/frame.h"

class ICP {
 private:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  ros::Subscriber point_cloud_sub_;
  Frame F1_, F2_;

 public:
  ICP();

  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);
  void RunICP();
};

#endif  // ICP_H
