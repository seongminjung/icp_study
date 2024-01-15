#include "icp_study/icp.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>

#include "icp_study/frame.h"
#include "icp_study/visualization.h"

ICP::ICP() {
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker/frame", 1);
  point_cloud_sub_ = nh_.subscribe("/kitti/velo/pointcloud", 1, &ICP::PointCloudCallback, this);
}

void ICP::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
  if (F1_.GetSize() == 0) {
    F1_ = Frame(point_cloud_msg);
    VisualizeFrame(marker_pub_, F1_, 0);
  } else if (F2_.GetSize() == 0) {
    F2_ = Frame(point_cloud_msg);
    VisualizeFrame(marker_pub_, F2_, 1);
  }
}

void ICP::RunICP() {
  /// \brief 2D HeightGrid ICP algorithm. F2 is transformed to F1.

  // Initialization
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();  // rotation
  Eigen::Vector2d t = Eigen::Vector2d::Zero();      // translation
  double err = 0;                                   // error

  int max_iter = 200;
  double thresh = 1e-5;

  unsigned int N_F1 = F1_.GetSize();
  unsigned int N_F2 = F2_.GetSize();

  // Start ICP loop
  for (int iter = 0; iter < max_iter; iter++) {
    ROS_INFO("==========iter: %d==========", iter);
    Frame X(F2_);
    Frame Y;
    Y.ReserveSize(X.GetSize());

    std::vector<std::pair<int, double>> dist_vector;  // <index of new_P, distance>
    dist_vector.clear();
    dist_vector.reserve(X.GetSize());

    // // Find the nearest neighbor for each point in P
    // for (int i = 0; i < N_F2; i++) {
    //   double min_dist = 1e10;
    //   int min_idx = 0;
    //   for (int j = 0; j < N_F1; j++) {
    //     double dist = sqrt(pow(X.GetOnePoint(i)(0) - F1_.GetOnePoint(i)(0), 2) +
    //                        pow(X.GetOnePoint(i)(1) - F1_.GetOnePoint(i)(1), 2));  // Euclidean distance
    //     if (dist < min_dist) {
    //       min_dist = dist;
    //       min_idx = j;
    //     }
    //   }
    //   dist_vector.emplace_back(i, min_dist);
    //   Y.SetOnePoint(i, F1_.GetOnePoint(min_idx));
    // }
  }
}