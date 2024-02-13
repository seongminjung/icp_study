#include "icp_study/frame.h"

#include <random>

Frame::Frame() {}

Frame::Frame(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg, int mode) {
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
  ExtractLine(mode);
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

void Frame::ExtractLine(int mode) {
  int n_voxel = voxel_hash_.size();

  Eigen::MatrixXd points_tmp;   // 2 x N  // These are temporary variables, acts the same as points_ and heights_
  Eigen::VectorXd heights_tmp;  // 1 x N  // These are the output of the z-directional line extraction
  line_hash_.reserve(n_voxel);
  points_tmp.resize(2, n_voxel);  // temporary
  heights_tmp.resize(n_voxel);    // not temporary

  // Extract z-directional lines
  int idx1 = 0, idx2 = 1;
  int n_cells = 0;
  while (idx2 < n_voxel) {
    if (voxel_hash_[idx2] - voxel_hash_[idx1] == idx2 - idx1) {
      ++idx2;
    } else {
      if (idx2 - idx1 > 2) {
        // Add point only when x1 or y1 is not the same as the last entry's. This is for avoiding creating duplicate
        // points.
        if (n_cells == 0 || (n_cells > 0 && (voxel_hash_[idx1] >> 10) != (cell_hash_[n_cells - 1] >> 10))) {
          double x, y, z;                           // temporary
          HashToCoord(voxel_hash_[idx1], x, y, z);  // temporary
          points_tmp.col(n_cells) << x, y;          // temporary
          cell_hash_.emplace_back(voxel_hash_[idx1]);
          heights_tmp(n_cells) = double((voxel_hash_[idx2 - 1] - voxel_hash_[idx1]) & 0x000003FF) * resolution_;
          ++n_cells;
        }
      }
      idx1 = idx2;
      idx2++;
    }
  }

  points_tmp.conservativeResize(2, n_cells);  // temporary
  heights_tmp.conservativeResize(n_cells);

  if (mode == 0) {
    points_ = points_tmp;
    heights_ = heights_tmp;
    disabled_ = Eigen::VectorXi::Zero(n_cells);
    return;
  }

  // Extract x-directional lines
  idx1 = 0;
  idx2 = 1;
  disabled_ = Eigen::VectorXi::Zero(n_cells);

  while (idx2 < n_cells) {
    if (points_tmp(1, idx2) == points_tmp(1, idx1) &&
        int((points_tmp(0, idx2) - points_tmp(0, idx1)) / resolution_) == idx2 - idx1) {
      idx2++;
    } else {
      if (idx2 - idx1 >= 5) {
        // Store start and (end + 1) points of lines
        disabled_.segment(idx1, idx2 - idx1).setOnes();  // segment(start, length) is the syntax for Eigen::VectorXi
        line_hash_.emplace_back(voxel_hash_[idx1], voxel_hash_[idx2]);
      }
      idx1 = idx2;
      idx2++;
    }
  }

  int n_cells_not_in_line = n_cells - disabled_.sum();
  points_.resize(2, n_cells_not_in_line);
  heights_.resize(n_cells_not_in_line);

  int new_idx = 0;
  for (int i = 0; i < n_cells; i++) {
    if (!disabled_(i)) {
      double x, y, z;
      HashToCoord(cell_hash_[i], x, y, z);
      points_.col(new_idx) << x, y;
      heights_(new_idx) = heights_tmp(i);
      new_idx++;
    }
  }
  disabled_.setZero(n_cells_not_in_line);
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
  // type casting to double is imperative here to avoid unexpected line extraction fault
  x = (double((hash & 0x000FFC00) >> 10) - 512) * resolution_;
  y = (double((hash & 0x3FF00000) >> 20) - 512) * resolution_;
  z = (double(hash & 0x000003FF) - 512) * resolution_;
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
