#include "icp_study/frame.h"

#include <random>

Frame::Frame() {}

Frame::Frame(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*point_cloud_msg, *ptr_cloud);

  // make 10 x 10 grid with resolution of 0.2 meter
  timestamp_ = point_cloud_msg->header.stamp.toSec();
  resolution_ = 0.2;
  map_width_ = 1024;
  map_height_ = 1024;
  min_points_per_voxel_ = 2;

  ////////////////////////////////
  // Conversion start here

  // voxelization
  Voxelize(*ptr_cloud);

  // line extraction
  ExtractLine();
}

Frame::Frame(const Frame& other) {
  timestamp_ = other.timestamp_;
  resolution_ = other.resolution_;
  map_width_ = other.map_width_;
  map_height_ = other.map_height_;
  points_ = other.points_;
  heights_ = other.heights_;
  disabled_ = other.disabled_;
}

////////////////////////////////
// Getters
time_t Frame::GetTimestamp() { return timestamp_; }
double Frame::GetResolution() { return resolution_; }
unsigned int Frame::GetMapWidth() { return map_width_; }
unsigned int Frame::GetMapHeight() { return map_height_; }
unsigned int Frame::GetSize() { return points_.cols(); }
Eigen::MatrixXd Frame::GetPoints() { return points_; }
Eigen::Vector2d Frame::GetOnePoint(unsigned int idx) { return points_.col(idx); }
Eigen::VectorXd Frame::GetHeights() { return heights_; }
double Frame::GetOneHeight(unsigned int idx) { return heights_(idx); }
Eigen::VectorXi Frame::GetDisabled() { return disabled_; }
bool Frame::GetOnePointDisabled(unsigned int idx) { return disabled_(idx); }

////////////////////////////////
// Setters
void Frame::SetTimestamp(time_t timestamp) { timestamp_ = timestamp; }
void Frame::SetResolution(double resolution) { resolution_ = resolution; }
void Frame::SetMapWidth(unsigned int width) { map_width_ = width; }
void Frame::SetMapHeight(unsigned int height) { map_height_ = height; }
void Frame::SetPoints(Eigen::MatrixXd points) { points_ = points; }
void Frame::SetOnePoint(unsigned int idx, Eigen::Vector2d point) {
  points_(0, idx) = point(0);
  points_(1, idx) = point(1);
}
void Frame::SetOneHeight(unsigned int idx, double height) { heights_(idx) = height; }
void Frame::SetAllPointsDisabled(bool disabled) { disabled_ = Eigen::VectorXi::Ones(points_.cols()) * disabled; }
void Frame::SetOnePointDisabled(unsigned int idx, bool disabled) { disabled_(idx) = disabled; }
void Frame::ReserveSize(unsigned int size) {
  points_.resize(2, size);
  heights_.resize(size);
  disabled_.resize(size);
}

////////////////////////////////
// Converters
void Frame::Voxelize(pcl::PointCloud<pcl::PointXYZ>& input) {
  point_hash_.clear();
  point_hash_.reserve(input.points.size());

  // First pass: go over all points and insert them into the point_hash_ vector with calculated hash. Points with the
  // same hash value will contribute to the same point of resulting CloudPoint
  for (int i = 0; i < input.points.size(); i++) {
    point_hash_.emplace_back(CoordToHash(input.points[i].x, input.points[i].y, input.points[i].z));
  }

  // Second pass: sort the point_hash_ vector so all points belonging to the same output cell will be next to each other
  std::sort(point_hash_.begin(), point_hash_.end());

  // Third pass: count cells
  // we need to skip all the same, adjacent idx values
  unsigned int index = 0;
  voxel_hash_.clear();
  voxel_hash_.reserve(input.points.size());
  while (index < point_hash_.size()) {
    unsigned int i = index + 1;
    while (i < point_hash_.size() && point_hash_[i] == point_hash_[index]) ++i;
    if (i - index >= min_points_per_voxel_) {
      voxel_hash_.emplace_back(point_hash_[index]);
    }
    index = i;
  }
}

void Frame::ExtractLine() {
  int n_voxel = voxel_hash_.size();
  int n_lines = 0;

  Eigen::MatrixXd points_tmp;   // 2 x N  // These are temporary variables, acts the same as points_ and heights_
  Eigen::VectorXd heights_tmp;  // 1 x N  // These are the output of the z-directional line extraction
  points_tmp.resize(2, n_voxel);
  heights_tmp.resize(n_voxel);

  // Extract z-directional lines
  int idx1 = 0, idx2 = 1;
  while (idx2 < n_voxel) {
    if (voxel_hash_[idx2] - voxel_hash_[idx1] == idx2 - idx1) {
      idx2++;
    } else {
      if (idx2 - idx1 > 2) {
        double x1, y1, z1, x2, y2, z2;
        HashToCoord(voxel_hash_[idx1], x1, y1, z1);
        HashToCoord(voxel_hash_[idx2 - 1], x2, y2, z2);
        // Add points_tmp and heights_tmp only when x1 and y1 is not the same as the last entry's. This is for avoiding
        // creating duplicate points.
        if (n_lines == 0 || (n_lines > 0 && x1 != points_tmp(0, n_lines - 1) || y1 != points_tmp(1, n_lines - 1))) {
          points_tmp.col(n_lines) << x1, y1;
          heights_tmp(n_lines) = z2 - z1;
          ++n_lines;
        }
      }
      idx1 = idx2;
      idx2++;
    }
  }

  points_tmp.conservativeResize(2, n_lines);
  heights_tmp.conservativeResize(n_lines);

  // Extract x-directional lines
  idx1 = 0;
  idx2 = 1;
  disabled_.setZero(n_lines);

  while (idx2 < n_lines) {
    if (points_tmp(1, idx2) == points_tmp(1, idx1) &&
        int((points_tmp(0, idx2) - points_tmp(0, idx1)) / resolution_) == idx2 - idx1) {
      idx2++;
    } else {
      if (idx2 - idx1 > 3) {
        // Remove idx1-th to idx2-1-th elements of points_tmp and heights_tmp.
        // Here, we will use disabled_ vector to indicate which points are disabled.
        disabled_.segment(idx1, idx2 - idx1).setOnes();  // segment(start, length) is the syntax for Eigen::VectorXi
      }
      idx1 = idx2;
      idx2++;
    }
  }

  points_.resize(2, n_lines - disabled_.sum());
  heights_.resize(n_lines - disabled_.sum());

  int new_idx = 0;
  for (int i = 0; i < n_lines; i++) {
    if (!disabled_(i)) {
      points_.col(new_idx) = points_tmp.col(i);
      heights_(new_idx) = heights_tmp(i);
      new_idx++;
    }
  }
  disabled_.setZero(n_lines - disabled_.sum());
}

unsigned int Frame::CoordToHash(double x, double y, double z) {
  unsigned int rounded_x = round(x / resolution_) + 512;  // offset 512
  unsigned int rounded_y = round(y / resolution_) + 512;
  unsigned int rounded_z = round(z / resolution_) + 512;

  // hashing
  unsigned int hashed_x = (rounded_x << 10) & 0x000FFC00;
  unsigned int hashed_y = (rounded_y << 20) & 0x3FF00000;
  unsigned int hashed_z = rounded_z & 0x000003FF;
  unsigned int hash = hashed_x + hashed_y + hashed_z;

  return hash;
}

void Frame::HashToCoord(unsigned int hash, double& x, double& y, double& z) {
  // unhashing
  x = (int((hash & 0x000FFC00) >> 10) - 512) * resolution_;
  y = (int((hash & 0x3FF00000) >> 20) - 512) * resolution_;
  z = (int(hash & 0x000003FF) - 512) * resolution_;
}

////////////////////////////////
// Downsampling
void Frame::RandomDownsample(double ratio) {
  // set disabled_ to all false
  SetAllPointsDisabled(false);

  // set random seed
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);

  // set disabled_ to true for ratio of points
  for (int i = 0; i < points_.cols(); i++) {
    if (dis(gen) > ratio) {
      SetOnePointDisabled(i, true);
    }
  }

  // Make new points_ and disabled_ with only enabled points
  Eigen::MatrixXd new_points(2, points_.cols() - disabled_.sum());
  Eigen::VectorXi new_disabled(points_.cols() - disabled_.sum());
  int new_idx = 0;
  for (int i = 0; i < points_.cols(); i++) {
    if (!GetOnePointDisabled(i)) {
      new_points.col(new_idx) = GetOnePoint(i);
      new_disabled(new_idx) = GetOnePointDisabled(i);
      new_idx++;
    }
  }
  points_ = new_points;
  disabled_ = new_disabled;

  // print number of points after downsampling
  // std::cout << "frame size after downsampling: " << points_.cols() << std::endl;
}

////////////////////////////////
// Transformations
void Frame::Transform(Eigen::Matrix2d R, Eigen::Vector2d t) {
  points_ = R * points_ + t * Eigen::MatrixXd::Ones(1, points_.cols());
}

////////////////////////////////
// Registering
void Frame::RegisterPointCloud(Frame& source_tf) {
  // For each point in source_tf, find there is any duplicate in this frame. If not, add the point, height, and disabled
  // to this frame.
  int duplicate_count = 0;
  for (int i = 0; i < source_tf.GetSize(); i++) {
    bool duplicate = false;
    for (int j = 0; j < GetSize(); j++) {
      // If there is a duplicate, break the loop. Duplicate means the x, y value is similar (resolution_)
      if (fabs(GetOnePoint(j)(0) - source_tf.GetOnePoint(i)(0)) < resolution_ &&
          fabs(GetOnePoint(j)(1) - source_tf.GetOnePoint(i)(1)) < resolution_) {
        duplicate = true;
        duplicate_count++;
        break;
      }
    }
    if (!duplicate) {
      points_.conservativeResize(2, points_.cols() + 1);
      heights_.conservativeResize(heights_.size() + 1);
      disabled_.conservativeResize(disabled_.size() + 1);
      SetOnePoint(points_.cols() - 1, source_tf.GetOnePoint(i));
      SetOneHeight(heights_.size() - 1, source_tf.GetOneHeight(i));
      SetOnePointDisabled(disabled_.size() - 1, source_tf.GetOnePointDisabled(i));
    }
  }
  std::cout << "duplicate count: " << duplicate_count << std::endl;
}
