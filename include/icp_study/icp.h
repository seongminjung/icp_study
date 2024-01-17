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
  // My ICP algorithm
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  ros::Subscriber point_cloud_sub_;
  Frame F1_, F2_;

  // PCL ICP algorithm
  pcl::PointCloud<pcl::PointXYZ>::Ptr src;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt;

 public:
  ICP();

  // My ICP algorithm
  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);
  void RunICP();
  void FindAlignment(Frame& X, Frame& Y, Eigen::Matrix3d& Result);

  // PCL ICP algorithm
  void PointCloudCallbackForPCL(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);
  void RunICPPCL();
};

#endif  // ICP_H
