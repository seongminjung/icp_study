#ifndef FRAME_H
#define FRAME_H

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>

struct cloud_point_index_idx {
  unsigned int idx;
  unsigned int cloud_point_index;

  cloud_point_index_idx(int idx_, unsigned int cloud_point_index_) : idx(idx_), cloud_point_index(cloud_point_index_) {}
  bool operator<(const cloud_point_index_idx& p) const { return (idx < p.idx); }
};

struct voxel_index_idx {
  unsigned int idx;
  unsigned int voxel_index;

  voxel_index_idx(int idx_, unsigned int voxel_index_) : idx(idx_), voxel_index(voxel_index_) {}
  bool operator<(const voxel_index_idx& p) const { return (idx < p.idx); }
};

class Frame {
 private:
  time_t timestamp_;
  float resolution_ = 0.2;          // meter per pixel
  unsigned int map_width_ = 1024;   // total width of the frame
  unsigned int map_height_ = 1024;  // total height of the frame
  unsigned int min_points_per_voxel_ = 2;

  Eigen::MatrixXd points_;    // 2 x N
  Eigen::VectorXd heights_;   // 1 x N
  Eigen::VectorXi disabled_;  // 1 x N

  // Converters
  std::vector<cloud_point_index_idx> index_vector;  // Storage for mapping leaf and pointcloud indexes
  std::vector<voxel_index_idx> v_index_vector;      // Storage for mapping leaf and pointcloud indexes

 public:
  Frame();
  Frame(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);
  Frame(const Frame& other);  // copy constructor

  // Getters
  time_t GetTimestamp();
  double GetResolution();
  unsigned int GetMapWidth();
  unsigned int GetMapHeight();
  unsigned int GetSize();
  Eigen::MatrixXd GetPoints();  // 2 x N
  Eigen::Vector2d GetOnePoint(unsigned int idx);
  Eigen::VectorXd GetHeights();  // 1 x N
  double GetOneHeight(unsigned int idx);
  Eigen::VectorXi GetDisabled();
  bool GetOnePointDisabled(unsigned int idx);

  // Setters
  void SetTimestamp(time_t timestamp);
  void SetResolution(double resolution);
  void SetMapWidth(unsigned int width);
  void SetMapHeight(unsigned int height);
  void SetPoints(Eigen::MatrixXd points);  // 2 x N
  void SetOnePoint(unsigned int idx, Eigen::Vector2d point);
  void SetOneHeight(unsigned int idx, double height);
  void SetAllPointsDisabled(bool disabled);
  void SetOnePointDisabled(unsigned int idx, bool disabled);
  void ReserveSize(unsigned int size);

  // Downsampling
  void RandomDownsample(double ratio);

  // Converters
  void SetIndexVector(pcl::PointCloud<pcl::PointXYZ>& input, double voxel_size);
  void Voxelize(pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& output, double voxel_size,
                unsigned int min_points_per_voxel);
  void ExtractLine(pcl::PointCloud<pcl::PointXYZ>& v_input,
                   std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& output);

  // Registering
  void RegisterPointCloud(Frame& source_tf);
};

#endif  // FRAME_H
