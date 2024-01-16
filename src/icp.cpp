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
    RunICP();
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

  Frame X(F2_);

  // Start ICP loop
  for (int iter = 0; iter < max_iter; iter++) {
    // if ctrl+c is pressed, stop quickly
    if (!ros::ok()) {
      break;
    }

    ROS_INFO("==========iter: %d==========", iter);
    Frame Y;
    Y.ReserveSize(X.GetSize());

    std::vector<std::pair<int, double>> dist_vector;  // <index of X, distance>
    dist_vector.clear();
    dist_vector.reserve(X.GetSize());

    // Find the nearest neighbor for each point in X
    for (int i = 0; i < N_F2; i++) {
      double min_dist = 1e10;
      int min_idx = 0;
      for (int j = 0; j < N_F1; j++) {
        double dist = sqrt(pow(X.GetOnePoint(i)(0) - F1_.GetOnePoint(j)(0), 2) +
                           pow(X.GetOnePoint(i)(1) - F1_.GetOnePoint(j)(1), 2));  // Euclidean distance
        if (dist < min_dist) {
          min_dist = dist;
          min_idx = j;
        }
      }
      dist_vector.emplace_back(i, min_dist);
      Y.SetOnePoint(i, F1_.GetOnePoint(min_idx));
    }
    VisualizeLineBetweenMatchingPoints(marker_pub_, X, Y);
    VisualizeFrame(marker_pub_, X, 2);

    Eigen::Matrix3d result;
    FindAlignment(X, Y, result);  // left top 2x2: R, right top 2x1: t, left bottom 1x1: err
    Eigen::Matrix2d R_step = result.block<2, 2>(0, 0);
    Eigen::Vector2d t_step = result.block<2, 1>(0, 2);

    // Update R, t, err
    R = R_step * R;
    t = R_step * t + t_step;
    err = result(2, 0);

    // Update X
    X.SetPoints(R * F2_.GetPoints() + t * Eigen::MatrixXd::Ones(1, N_F2));

    // Print R, t, err
    std::cout << "R: " << std::endl << R << std::endl;
    std::cout << "t: " << std::endl << t << std::endl;
    std::cout << "err: " << std::endl << err << std::endl;

    // Check for convergence
    if (err < thresh) {
      VisualizeLineBetweenMatchingPoints(marker_pub_, X, Y);
      break;
    }
  }
}

void ICP::FindAlignment(Frame& X_frame, Frame& Y_frame, Eigen::Matrix3d& result) {
  /// \brief Find the alignment between X and Y
  /// \param X_frame: transformed X from last iteration
  /// \param Y_frame: nearest neighbor of each point in X
  /// \return result: left top 2x2: R, right top 2x1: t, left bottom 1x1 s, center bottom 1x1: err

  // Test the inputs
  if (X_frame.GetSize() != Y_frame.GetSize()) {
    ROS_ERROR("X and Y have different sizes!");
  }
  if (X_frame.GetSize() < 4) {
    ROS_ERROR("Need at least four pairs of points!");
  }

  Eigen::MatrixXd X = X_frame.GetPoints();
  Eigen::MatrixXd Y = Y_frame.GetPoints();
  unsigned int N = X.cols();

  // Seperate coordinates and height
  // Eigen::MatrixXd X = X_matrix.block(0, 0, 2, N);
  // Eigen::MatrixXd Y = Y_matrix.block(0, 0, 2, N);
  // Eigen::VectorXd X_height = X_matrix.block(2, 0, 1, N).transpose();
  // Eigen::VectorXd Y_height = Y_matrix.block(2, 0, 1, N).transpose();

  // Compute the centroid of X and Y
  Eigen::Vector2d X_centroid = X.rowwise().mean();
  Eigen::Vector2d Y_centroid = Y.rowwise().mean();

  // Compute average height of X and Y
  // Eigen::VectorXd Height = (X_height + Y_height) / 2;

  // Compute the demeaned X and Y
  Eigen::MatrixXd X_demeaned = X - X_centroid * Eigen::MatrixXd::Ones(1, N);
  Eigen::MatrixXd Y_demeaned = Y - Y_centroid * Eigen::MatrixXd::Ones(1, N);

  // Compute the covariance matrix including height
  // Eigen::Matrix2d H = X_demeaned * Height.asDiagonal() * Y_demeaned.transpose();
  Eigen::Matrix2d H = X_demeaned * Y_demeaned.transpose();

  // Compute the SVD of H
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d U = svd.matrixU();
  Eigen::Matrix2d V = svd.matrixV();
  Eigen::Matrix2d R = V * U.transpose();

  // get angle in degrees from rotation matrix r
  double angle = atan2(R(1, 0), R(0, 0)) * 180 / M_PI;

  // Compute the translation
  Eigen::Vector2d t = Y_centroid - R * X_centroid;

  // Compute the error
  double err = (Y_demeaned - R * X_demeaned).norm() / N;

  // Construct the result
  result.block<2, 2>(0, 0) = R;
  result.block<2, 1>(0, 2) = t;
  result(2, 0) = err;

  // Visualize centroid of X, Y, and converted X
  VisualizeCentroid(marker_pub_, Y_centroid, X_frame.GetTimestamp(), 0);
  VisualizeCentroid(marker_pub_, X_centroid, X_frame.GetTimestamp(), 1);
  Eigen::Vector2d X_centroid_tf = R * X_centroid + t;
  VisualizeCentroid(marker_pub_, X_centroid_tf, X_frame.GetTimestamp(), 2);
}