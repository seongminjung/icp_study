#ifndef FRAME_H
#define FRAME_H

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>

class Frame {
 private:
  ros::Time timestamp_;
  float resolution_ = 0.2;          // meter per pixel
  unsigned int map_width_ = 1024;   // total width of the frame
  unsigned int map_height_ = 1024;  // total height of the frame
  unsigned int min_points_per_voxel_ = 2;

  Eigen::MatrixXd points_;    // 2 x N
  Eigen::VectorXd heights_;   // 1 x N
  Eigen::MatrixXd lines_;     // 5 x M  (x1, y1, x2, y2, average height)
  Eigen::VectorXi disabled_;  // 1 x N

  // Hash vectors. Hash must be unsigned int (to use right shift operator)
  std::vector<unsigned int> point_hash_;                          // i-th element is the hash key of the i-th point
  std::vector<unsigned int> voxel_hash_;                          // i-th element is the hash key of the i-th voxel
  std::vector<unsigned int> cell_hash_;                           // i-th element is the hash key of the i-th cell
  std::vector<std::pair<unsigned int, unsigned int>> line_hash_;  // Start and end hash of x-directional lines.

 public:
  Frame();
  Frame(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);
  Frame(const Frame& other);  // copy constructor

  // Getters
  ros::Time GetTimestamp();
  double GetResolution();
  unsigned int GetMapWidth();
  unsigned int GetMapHeight();
  unsigned int GetNPoints();
  Eigen::MatrixXd GetPoints();  // 2 x N
  Eigen::Vector2d GetOnePoint(unsigned int idx);
  Eigen::VectorXd GetHeights();  // 1 x N
  double GetOneHeight(unsigned int idx);
  unsigned int GetNLines();
  Eigen::MatrixXd GetLines();  // 5 x M
  Eigen::VectorXd GetOneLine(unsigned int idx);
  Eigen::VectorXi GetDisabled();
  bool GetOnePointDisabled(unsigned int idx);

  // Setters
  void SetTimestamp(ros::Time timestamp);
  void SetResolution(double resolution);
  void SetMapWidth(unsigned int width);
  void SetMapHeight(unsigned int height);
  void SetPoints(Eigen::MatrixXd points);  // 2 x N
  void SetOnePoint(unsigned int idx, Eigen::Vector2d point);
  void SetOneHeight(unsigned int idx, double height);
  void SetLines(Eigen::MatrixXd lines);  // 5 x M
  void SetOneLine(unsigned int idx, Eigen::VectorXd line);
  void SetAllPointsDisabled(bool disabled);
  void SetOnePointDisabled(unsigned int idx, bool disabled);
  void ReserveSize(unsigned int size);
  void RemoveOneLine(unsigned int idx);

  // Converters
  void Voxelize(pcl::PointCloud<pcl::PointXYZ>& input);
  void ExtractLine();

  // Downsampling
  void RandomDownsample(double ratio);
  Frame RadiusDownsample(Eigen::Vector2d point, double radius);

  // Transformations
  void Transform(Eigen::Matrix2d R, Eigen::Vector2d t);

  // Registering
  void RegisterPointCloud(Frame& source_tf);
};

#endif  // FRAME_H
