#include "icp_study/icp.h"

#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>

#include "icp_study/frame.h"
#include "icp_study/utils.h"
#include "icp_study/visualization.h"

ICP::ICP() {
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker/frame", 1);
  point_cloud_sub_ = nh_.subscribe("/kitti/velo/pointcloud", 1000, &ICP::PointCloudCallback, this);

  R_ = Eigen::Matrix2d::Identity();
  t_ = Eigen::Vector2d::Zero();
}

void ICP::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
  if (Map_.GetNPoints() == 0) {
    // Map_ = Frame(point_cloud_msg, 0);
    // VisualizeFrame(marker_pub_, Map_, 1);
    Map_ = Frame(point_cloud_msg, 1);
    VisualizeFrame(marker_pub_, Map_, 3);
  } else {
    Source_ = Frame(point_cloud_msg, 1);
    std::chrono::system_clock::time_point t_start = std::chrono::system_clock::now();
    RunHeightICP();
    std::chrono::system_clock::time_point t_end = std::chrono::system_clock::now();
    std::chrono::duration<double> t_reg = t_end - t_start;
    std::cout << " " << t_reg.count() << std::endl;
  }
}

// void ICP::PointCloudCallbackForEvaluation(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
//   if (F1_.GetNPoints() == 0) {
//     F1_ = Frame(point_cloud_msg);
//     // VisualizeFrame(marker_pub_, F1_, 0);
//   } else if (F2_.GetNPoints() == 0) {
//     F2_ = Frame(point_cloud_msg);
//     // VisualizeFrame(marker_pub_, F2_, 1);
//     for (int i = 0; i < 1000; i++) {
//       std::chrono::system_clock::time_point t_start = std::chrono::system_clock::now();
//       double err = RunHeightICP();
//       std::chrono::system_clock::time_point t_end = std::chrono::system_clock::now();
//       std::chrono::duration<double> t_reg = t_end - t_start;
//       times_.push_back(t_reg.count());
//       error_.push_back(err);

//       // Simple progress bar
//       if (i % 10 == 0) {
//         std::cout << "*";
//         std::cout.flush();
//       }
//     }
//     std::cout << "==========Evaluation==========\n";
//     double time_mean = std::accumulate(times_.begin(), times_.end(), 0.0) / times_.size();
//     double error_mean = std::accumulate(error_.begin(), error_.end(), 0.0) / error_.size();
//     // stddev
//     double time_sq_sum = std::inner_product(times_.begin(), times_.end(), times_.begin(), 0.0);
//     double time_stdev = std::sqrt(time_sq_sum / times_.size() - time_mean * time_mean);
//     double error_sq_sum = std::inner_product(error_.begin(), error_.end(), error_.begin(), 0.0);
//     double error_stdev = std::sqrt(error_sq_sum / error_.size() - error_mean * error_mean);
//     // median
//     std::sort(times_.begin(), times_.end());
//     double time_median = times_[times_.size() / 2];
//     std::cout << "Time mean: " << time_mean << " sec..." << std::endl;
//     std::cout << "Time stdev: " << time_stdev << " sec..." << std::endl;
//     std::cout << "Time median: " << time_median << " sec..." << std::endl;
//     std::cout << "Error mean: " << error_mean << std::endl;
//     std::cout << "Error stdev: " << error_stdev << std::endl;
//     std::cout << "==============================\n";

//     // print times_ in time.csv file
//     std::ofstream time_file;
//     time_file.open("time.csv");  // ~/.ros/time.csv
//     for (int i = 0; i < times_.size(); i++) {
//       time_file << times_[i] << std::endl;
//     }
//     time_file.close();
//   }
// }

void ICP::PointCloudCallbackForPCL(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
  if (tgt == nullptr) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*point_cloud_msg, *ptr_cloud1);
    tgt = ptr_cloud1;
  } else if (src == nullptr) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*point_cloud_msg, *ptr_cloud2);
    src = ptr_cloud2;
    // RunICPPCL();
    RunGICPPCL();
  }
}

// void ICP::RunICP() {
//   /// \brief 2D HeightGrid ICP algorithm. F2 is transformed to F1.

//   // Initialization
//   Eigen::Matrix2d R = Eigen::Matrix2d::Identity();  // rotation
//   Eigen::Vector2d t = Eigen::Vector2d::Zero();      // translation
//   double err = 0;                                   // error

//   int max_iter = 100;
//   double thresh = 1e-5;

//   unsigned int N_F1 = F1_.GetNPoints();
//   unsigned int N_F2 = F2_.GetNPoints();

//   Frame X(F2_);

//   errors_.clear();

//   // Start ICP loop
//   t_start_ = std::chrono::system_clock::now();
//   for (int iter = 0; iter < max_iter; iter++) {
//     // If ctrl+c is pressed, stop quickly
//     if (!ros::ok()) {
//       break;
//     }

//     // std::printf("==========iter: %d==========\n", iter);

//     Frame X_Downsampled(X);
//     Frame Y;

//     X_Downsampled.RandomDownsample(0.05);  // Randomly subsample 5% from X
//     Y.ReserveSize(X_Downsampled.GetNPoints());

//     // std::printf("X_Downsampled size: %d\n", X_Downsampled.GetNPoints());

//     std::vector<std::pair<int, double>> dist_vector;  // <index of X_Downsampled, distance>
//     dist_vector.reserve(X_Downsampled.GetNPoints());

//     unsigned int N_Downsampled = X_Downsampled.GetNPoints();

//     // Find the nearest neighbor for each point in X_Downsampled
//     for (int i = 0; i < N_Downsampled; i++) {
//       double min_dist = 1e10;
//       int min_idx = 0;
//       for (int j = 0; j < N_F1; j++) {
//         double dist = sqrt(pow(X_Downsampled.GetOnePoint(i)(0) - F1_.GetOnePoint(j)(0), 2) +
//                            pow(X_Downsampled.GetOnePoint(i)(1) - F1_.GetOnePoint(j)(1), 2));  // Euclidean distance
//         if (dist < min_dist) {
//           min_dist = dist;
//           min_idx = j;
//         }
//       }
//       dist_vector.emplace_back(i, min_dist);
//       Y.SetOnePoint(i, F1_.GetOnePoint(min_idx));
//     }

//     // sort dist_vector by distance
//     std::sort(dist_vector.begin(), dist_vector.end(),
//               [](const std::pair<int, double>& a, const std::pair<int, double>& b) { return a.second > b.second; });

//     // Drop points with top 5% distance by making disabled_(i) = 1
//     for (int i = 0; i < N_Downsampled * 0.05; i++) {
//       X_Downsampled.SetOnePointDisabled(dist_vector[i].first, true);
//     }

//     // VisualizeLineBetweenMatchingPoints(marker_pub_, X_Downsampled, Y);
//     // VisualizeFrame(marker_pub_, X, 2);

//     Eigen::Matrix3d result;
//     FindAlignment(X_Downsampled, Y, result);  // left top 2x2: R, right top 2x1: t, left bottom 1x1: err
//     Eigen::Matrix2d R_step = result.block<2, 2>(0, 0);
//     Eigen::Vector2d t_step = result.block<2, 1>(0, 2);

//     // Update R, t, err
//     R = R_step * R;
//     t = R_step * t + t_step;
//     err = result(2, 0);

//     // Update X
//     X.SetPoints(R * F2_.GetPoints() + t * Eigen::MatrixXd::Ones(1, N_F2));
//     VisualizeFrame(marker_pub_, X, 2);

//     // Print R, t, err
//     // std::cout << "R: " << std::endl << R << std::endl;
//     // std::cout << "t: " << std::endl << t << std::endl;
//     // std::cout << "err: " << err << std::endl;

//     // Check convergence
//     errors_.push_back(err);
//     if (errors_.size() > 10) {
//       errors_.erase(errors_.begin());
//     }
//     double error_mean = std::accumulate(errors_.begin(), errors_.end(), 0.0) / errors_.size();
//     double error_sq_sum = std::inner_product(errors_.begin(), errors_.end(), errors_.begin(), 0.0);
//     double error_stdev = std::sqrt(error_sq_sum / errors_.size() - error_mean * error_mean);
//     // std::printf("error_stdev: %f\n", error_stdev);
//     if (errors_.size() == 10 && error_stdev < error_stdev_threshold_) {
//       // std::printf("Converged!\n");
//       break;
//     }
//   }

//   t_end_ = std::chrono::system_clock::now();
//   std::chrono::duration<double> t_reg = t_end_ - t_start_;
//   std::cout << "Takes " << t_reg.count() << " sec..." << std::endl;
// }

void ICP::FindAlignment(Frame& X_frame, Frame& Y_frame, Eigen::Matrix3d& result) {
  /// \brief Find the alignment between X and Y
  /// \param X_frame: transformed X from last iteration
  /// \param Y_frame: nearest neighbor of each point in X
  /// \return result: left top 2x2: R, right top 2x1: t, left bottom 1x1 s, center bottom 1x1: err

  // Test the inputs
  if (X_frame.GetNPoints() != Y_frame.GetNPoints()) {
    ROS_ERROR("X and Y have different sizes!");
  }
  if (X_frame.GetNPoints() < 4) {
    ROS_ERROR("Need at least four pairs of points!");
  }

  // Get matrix without disabled points
  int num_disabled = X_frame.GetDisabled().sum();
  Eigen::MatrixXd X(2, X_frame.GetNPoints() - num_disabled);
  Eigen::MatrixXd Y(2, Y_frame.GetNPoints() - num_disabled);
  int idx = 0;
  for (int i = 0; i < X_frame.GetNPoints(); i++) {
    if (!X_frame.GetOnePointDisabled(i)) {
      X.col(idx) = X_frame.GetOnePoint(i);
      Y.col(idx) = Y_frame.GetOnePoint(i);
      idx++;
    }
  }

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
  // VisualizeCentroid(marker_pub_, Y_centroid, X_frame.GetTimestamp(), 0);
  // VisualizeCentroid(marker_pub_, X_centroid, X_frame.GetTimestamp(), 1);
  // Eigen::Vector2d X_centroid_tf = R * X_centroid + t;
  // VisualizeCentroid(marker_pub_, X_centroid_tf, X_frame.GetTimestamp(), 2);
}

double ICP::RunHeightICP() {
  /// \brief 2D HeightGrid ICP algorithm. F2 is transformed to F1.

  // Initialization
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();  // rotation
  Eigen::Vector2d t = Eigen::Vector2d::Zero();      // translation
  double err = 0;                                   // error
  errors_.clear();

  int max_iter = 100;
  double thresh = 1e-5;

  Frame Map_downsampled = Map_.RadiusDownsample(t_, 50.0);  // Only take points within 10m from the current position

  unsigned int N_Points_Map = Map_downsampled.GetNPoints();
  unsigned int N_Lines_Map = Map_downsampled.GetNLines();

  // First, transform Source_ to the original frame
  Source_.Transform(R_, t_);

  // Start ICP loop
  for (int iter = 0; iter < max_iter; iter++) {
    // If ctrl+c is pressed, stop quickly
    if (!ros::ok()) {
      break;
    }

    // std::printf("==========iter: %d==========\n", iter);
    Frame Source_downsampled(Source_);
    Frame Y;

    Source_downsampled.RandomDownsample(0.05);  // Randomly subsample 5% from Source_

    unsigned int N_Downsampled = Source_downsampled.GetNPoints();

    Y.ReserveSize(N_Downsampled);

    // std::printf("Source_downsampled size: %d\n", N_Downsampled);

    std::vector<std::pair<int, double>> dist_vector;  // <index of Source_downsampled, distance>
    dist_vector.reserve(N_Downsampled);

    Eigen::Vector2d foot_of_perpendicular;  // Used for point-to-line distance calculation below

    // Find the nearest neighbor for each point in Source_downsampled
    for (int i = 0; i < N_Downsampled; i++) {
      double min_dist = 1e10;
      int min_idx = 0;
      bool matched_to_line = false;

      // Point-to-point distance
      for (int j = 0; j < N_Points_Map; j++) {
        double dist_sq = pow(Source_downsampled.GetOnePoint(i)(0) - Map_downsampled.GetOnePoint(j)(0), 2) +
                         pow(Source_downsampled.GetOnePoint(i)(1) - Map_downsampled.GetOnePoint(j)(1), 2);
        if (dist_sq < min_dist) {
          // Update only when height is similar
          // if (abs(Source_downsampled.GetOneHeight(i) - Map_downsampled.GetOneHeight(j)) < 1) {
          min_dist = dist_sq;
          min_idx = j;
          // }
        }
      }

      // Point-to-line distance
      for (int j = 0; j < N_Lines_Map; j++) {
        ////////////////////////////////////////////
        // Calculate the distance and a foot of perpendicular from a point p to a line segment defined by two points a
        // and b.
        // a = Map_downsampled.GetLines().block(0, j, 2, 1), b = Map_downsampled.GetLines().block(2, j, 2, 1) p =
        // Source_downsampled.GetOnePoint(i)
        Eigen::Vector2d ap = Source_downsampled.GetOnePoint(i) - Map_downsampled.GetLines().block(0, j, 2, 1);
        Eigen::Vector2d ab =
            Map_downsampled.GetLines().block(2, j, 2, 1) - Map_downsampled.GetLines().block(0, j, 2, 1);

        // Calculate the dot product
        double ab2 = ab.dot(ab);
        double ap_ab = ap.dot(ab);
        // Calculate the magnitude of the projection of ap onto ab, normalized by the length of ab
        double t = ap_ab / ab2;

        // if the foot of perpendicular is outside the line segment, skip this line
        if (t < 0 || t > 1) {
          continue;
        }

        // Find the projection point (which is a foot of perpendicular from p to the line segment)
        Eigen::Vector2d projection = Map_downsampled.GetLines().block(0, j, 2, 1) + ab * t;

        // Calculate the distance from p to the projection point
        double dist_sq = (Source_downsampled.GetOnePoint(i) - projection).squaredNorm();
        ////////////////////////////////////////////

        // Update only when the distance is smaller than min_dist
        if (dist_sq < min_dist) {
          min_dist = dist_sq;
          min_idx = j;
          foot_of_perpendicular = projection;
          matched_to_line = true;
        }
      }

      // Save the nearest neighbor and its height
      dist_vector.emplace_back(i, min_dist);
      if (matched_to_line) {
        Y.SetOnePoint(i, foot_of_perpendicular);
        Y.SetOneHeight(i, Map_downsampled.GetOneLine(min_idx)(4));
      } else {
        Y.SetOnePoint(i, Map_downsampled.GetOnePoint(min_idx));
        Y.SetOneHeight(i, Map_downsampled.GetOneHeight(min_idx));
      }
    }

    // sort dist_vector by distance
    std::sort(dist_vector.begin(), dist_vector.end(),
              [](const std::pair<int, double>& a, const std::pair<int, double>& b) { return a.second > b.second; });

    // Drop points with top 5% distance by making disabled_(i) = 1
    for (int i = 0; i < N_Downsampled * 0.05; i++) {
      Source_downsampled.SetOnePointDisabled(dist_vector[i].first, true);
    }

    VisualizeLineBetweenMatchingPoints(marker_pub_, Source_downsampled, Y);

    Eigen::Matrix3d result;
    FindHeightAlignment(Source_downsampled, Y, result);  // left top 2x2: R, right top 2x1: t, left bottom 1x1: err
    Eigen::Matrix2d R_step = result.block<2, 2>(0, 0);
    Eigen::Vector2d t_step = result.block<2, 1>(0, 2);

    // Update R, t, err
    R = R_step * R;
    t = R_step * t + t_step;
    err = result(2, 0);

    // Update Source_
    Source_.Transform(R_step, t_step);
    VisualizeFrame(marker_pub_, Source_, 2);

    // Print R, t, err
    // std::cout << "R: " << std::endl << R << std::endl;
    // // std::cout << "t: " << std::endl << t << std::endl;
    // std::cout << "err: " << err << std::endl;

    // Check convergence
    errors_.push_back(err);
    if (errors_.size() > 10) {
      errors_.erase(errors_.begin());
    }
    double error_mean = std::accumulate(errors_.begin(), errors_.end(), 0.0) / errors_.size();
    double error_sq_sum = std::inner_product(errors_.begin(), errors_.end(), errors_.begin(), 0.0);
    double error_stdev = std::sqrt(error_sq_sum / errors_.size() - error_mean * error_mean);
    // std::printf("error_stdev: %f\n", error_stdev);
    if (errors_.size() == 10 && error_stdev < error_stdev_threshold_) {
      // std::printf("Converged!\n");
      break;
    }

    // Visualize pose for each iteration
    VisualizePose(marker_pub_, R * R_, R * t_ + t, Source_.GetTimestamp());
  }
  VisualizeArrow(marker_pub_, t_, R * t_ + t);

  // Accumulate R_ and t_
  R_ = R * R_;
  t_ = R * t_ + t;

  Map_.RegisterPointCloud(Source_);
  VisualizeFrame(marker_pub_, Map_downsampled, 1);
  VisualizeFrame(marker_pub_, Map_, 3);

  std::cout << err;
  return err;
}

void ICP::FindHeightAlignment(Frame& X_frame, Frame& Y_frame, Eigen::Matrix3d& result) {
  /// \brief Find the alignment between X and Y
  /// \param X_frame: transformed X from last iteration
  /// \param Y_frame: nearest neighbor of each point in X
  /// \return result: left top 2x2: R, right top 2x1: t, left bottom 1x1 s, center bottom 1x1: err

  // Test the inputs
  if (X_frame.GetNPoints() != Y_frame.GetNPoints()) {
    ROS_ERROR("X and Y have different sizes!");
  }
  if (X_frame.GetNPoints() < 4) {
    ROS_ERROR("Need at least four pairs of points!");
  }

  // Get matrix without disabled points
  int num_disabled = X_frame.GetDisabled().sum();
  Eigen::MatrixXd X(2, X_frame.GetNPoints() - num_disabled);
  Eigen::MatrixXd Y(2, Y_frame.GetNPoints() - num_disabled);
  Eigen::VectorXd X_height(X_frame.GetNPoints() - num_disabled);
  Eigen::VectorXd Y_height(Y_frame.GetNPoints() - num_disabled);

  int idx = 0;
  for (int i = 0; i < X_frame.GetNPoints(); i++) {
    if (!X_frame.GetOnePointDisabled(i)) {
      X.col(idx) = X_frame.GetOnePoint(i);
      Y.col(idx) = Y_frame.GetOnePoint(i);
      X_height(idx) = X_frame.GetOneHeight(i);
      Y_height(idx) = Y_frame.GetOneHeight(i);
      idx++;
    }
  }

  unsigned int N = X.cols();

  // Compute the centroid of X and Y
  Eigen::Vector2d X_centroid = X.rowwise().mean();
  Eigen::Vector2d Y_centroid = Y.rowwise().mean();

  // Compute average height of X and Y
  Eigen::VectorXd Height = (X_height + Y_height) / 2;

  // Compute the demeaned X and Y
  Eigen::MatrixXd X_demeaned = X - X_centroid * Eigen::MatrixXd::Ones(1, N);
  Eigen::MatrixXd Y_demeaned = Y - Y_centroid * Eigen::MatrixXd::Ones(1, N);

  // Compute the covariance matrix including height
  Eigen::Matrix2d H = X_demeaned * Height.asDiagonal() * Y_demeaned.transpose();

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
}

void ICP::RunICPPCL() {
  /// \brief Run PCL ICP algorithm. src is transformed to tgt.
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaxCorrespondenceDistance(1.0);
  icp.setTransformationEpsilon(0.001);
  icp.setMaximumIterations(1000);

  pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);

  // 걸리는 시간 측정
  t_start_ = std::chrono::system_clock::now();

  // Registration 시행
  icp.setInputSource(src);
  icp.setInputTarget(tgt);
  icp.align(*align);

  t_end_ = std::chrono::system_clock::now();
  /*******************************************/
  std::chrono::duration<double> t_reg = t_end_ - t_start_;
  std::cout << "Takes " << t_reg.count() << " sec..." << std::endl;

  // Set outputs
  Eigen::Matrix4f src2tgt = icp.getFinalTransformation();
  double score = icp.getFitnessScore();
  bool is_converged = icp.hasConverged();

  std::cout << "Transformation: " << src2tgt << std::endl;
  std::cout << "Error: " << score << std::endl;
  std::cout << "Converged: " << is_converged << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  colorize(*src, *src_colored, {255, 0, 0});
  colorize(*tgt, *tgt_colored, {0, 255, 0});
  colorize(*align, *align_colored, {0, 0, 255});

  /**
   * 결과 visualization 하기
   */
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(src_colored, "src_viz");
  viewer.showCloud(tgt_colored, "tgt_viz");
  viewer.showCloud(align_colored, "align_viz");

  int cnt = 0;
  while (!viewer.wasStopped()) {
    // you can also do cool processing here
    // FIXME: Note that this is running in a separate thread from viewerPsycho
    // and you should guard against race conditions yourself...
    cnt++;
  }
}

void ICP::RunGICPPCL() {
  /// \brief Run PCL GICP algorithm. src is transformed to tgt.
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setMaxCorrespondenceDistance(1.0);
  gicp.setTransformationEpsilon(0.001);
  gicp.setMaximumIterations(1000);

  pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);

  // 걸리는 시간 측정
  t_start_ = std::chrono::system_clock::now();

  // Registration 시행
  gicp.setInputSource(src);
  gicp.setInputTarget(tgt);
  gicp.align(*align);

  t_end_ = std::chrono::system_clock::now();
  /*******************************************/
  std::chrono::duration<double> t_reg = t_end_ - t_start_;
  std::cout << "ICP Takes " << t_reg.count() << " sec..." << std::endl;

  // Set outputs
  Eigen::Matrix4f src2tgt = gicp.getFinalTransformation();
  double score = gicp.getFitnessScore();
  bool is_converged = gicp.hasConverged();

  std::cout << "Transformation: " << src2tgt << std::endl;
  std::cout << "Error: " << score << std::endl;
  std::cout << "Converged: " << is_converged << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  colorize(*src, *src_colored, {255, 0, 0});
  colorize(*tgt, *tgt_colored, {0, 255, 0});
  colorize(*align, *align_colored, {0, 0, 255});

  /**
   * 결과 visualization 하기
   */
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(src_colored, "src_viz");
  viewer.showCloud(tgt_colored, "tgt_viz");
  viewer.showCloud(align_colored, "align_viz");

  int cnt = 0;
  while (!viewer.wasStopped()) {
    // you can also do cool processing here
    // FIXME: Note that this is running in a separate thread from viewerPsycho
    // and you should guard against race conditions yourself...
    cnt++;
  }
}
