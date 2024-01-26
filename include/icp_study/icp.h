#ifndef ICP_H
#define ICP_H

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <chrono>
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

  // Convergence criteria
  std::vector<double> errors_;           // recent 10 errors
  double error_stdev_threshold_ = 0.01;  // standard deviation of recent 10 errors

  // Time
  std::chrono::system_clock::time_point t_start_, t_end_, t_start_total_, t_end_total_;

 public:
  ICP();

  // My ICP algorithm
  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);
  void RunICP();
  void FindAlignment(Frame& X, Frame& Y, Eigen::Matrix3d& Result);

  // PCL ICP algorithm
  void PointCloudCallbackForPCL(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);
  void RunICPPCL();

  // PCL GICP algorithm
  void RunGICPPCL();
};

#endif  // ICP_H
