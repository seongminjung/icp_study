#include "icp_study/frame.h"

#include <random>

#include "icp_study/utils.h"

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
  lines_ = other.lines_;
  disabled_ = other.disabled_;
}

////////////////////////////////
// Getters
time_t Frame::GetTimestamp() { return timestamp_; }
double Frame::GetResolution() { return resolution_; }
unsigned int Frame::GetMapWidth() { return map_width_; }
unsigned int Frame::GetMapHeight() { return map_height_; }
unsigned int Frame::GetNPoints() { return points_.cols(); }
Eigen::MatrixXd Frame::GetPoints() { return points_; }
Eigen::Vector2d Frame::GetOnePoint(unsigned int idx) { return points_.col(idx); }
Eigen::VectorXd Frame::GetHeights() { return heights_; }
double Frame::GetOneHeight(unsigned int idx) { return heights_(idx); }
unsigned int Frame::GetNLines() { return lines_.cols(); }
Eigen::MatrixXd Frame::GetLines() { return lines_; }
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
void Frame::SetLines(Eigen::MatrixXd lines) { lines_ = lines; }
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
    point_hash_.emplace_back(CoordToHash(input.points[i].x, input.points[i].y, input.points[i].z, resolution_));
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

  Eigen::VectorXd heights_tmp;  // 1 x N  // These are the output of the z-directional line extraction
  cell_hash_.reserve(n_voxel);
  heights_tmp.resize(n_voxel);  // not temporary. heights of every cell

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
          cell_hash_.emplace_back(voxel_hash_[idx1]);
          heights_tmp(n_cells) = double((voxel_hash_[idx2 - 1] - voxel_hash_[idx1]) & 0x000003FF) * resolution_;
          ++n_cells;
        }
      }
      idx1 = idx2;
      idx2++;
    }
  }

  heights_tmp.conservativeResize(n_cells);

  // Extract x-directional lines
  idx1 = 0;
  idx2 = 1;
  unsigned int n_x_lines = 0;            // Number of x-directional lines
  unsigned int n_cells_not_in_line = 0;  // Number of cells not in line

  points_.resize(2, n_cells);  // Worst cases
  heights_.resize(n_cells);
  lines_.resize(5, n_cells);

  while (idx2 < n_cells) {
    if ((cell_hash_[idx2] >> 10) - (cell_hash_[idx1] >> 10) == idx2 - idx1) {
      ++idx2;
    } else {
      if (idx2 - idx1 >= 5) {
        // Store start and (end + 1) points of lines
        double x1, y1, x2, y2;
        HashToXY(cell_hash_[idx1], x1, y1, resolution_);
        HashToXY(cell_hash_[idx2 - 1], x2, y2, resolution_);

        // Get average height of the line
        double h_avg = heights_tmp.segment(idx1, idx2 - idx1).mean();

        lines_.col(n_x_lines) << x1, y1, x2, y2, h_avg;
        ++n_x_lines;
      } else {
        // Store all points in the line to points_ and heights_
        for (int i = idx1; i < idx2; i++) {
          double x, y;
          HashToXY(cell_hash_[i], x, y, resolution_);
          points_.col(n_cells_not_in_line + i - idx1) << x, y;
          heights_(n_cells_not_in_line + i - idx1) = heights_tmp(i);
        }
        n_cells_not_in_line += idx2 - idx1;
      }
      idx1 = idx2;
      ++idx2;
    }
  }

  points_.conservativeResize(2, n_cells_not_in_line);
  heights_.conservativeResize(n_cells_not_in_line);
  lines_.conservativeResize(5, n_x_lines);
  disabled_.setZero(n_cells_not_in_line);
}

////////////////////////////////
// Downsampling
void Frame::RandomDownsample(double ratio) {
  unsigned int new_n_points = std::max(int(points_.cols() * ratio), 4);  // 4 is the minimum number of points for ICP
  unsigned int n = 0;  // Counter for the number of points after downsampling

  // set disabled_ to all true
  SetAllPointsDisabled(true);

  // Randomly enable points, but the number of enabled points after downsampling should be equal to or more than 4.
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dis(0, points_.cols() - 1);
  while (n < new_n_points) {
    int idx = dis(gen);
    if (GetOnePointDisabled(idx)) {
      SetOnePointDisabled(idx, false);
      n++;
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
  lines_.block(0, 0, 2, lines_.cols()) =
      R * lines_.block(0, 0, 2, lines_.cols()) + t * Eigen::MatrixXd::Ones(1, lines_.cols());
  lines_.block(2, 0, 2, lines_.cols()) =
      R * lines_.block(2, 0, 2, lines_.cols()) + t * Eigen::MatrixXd::Ones(1, lines_.cols());
}

////////////////////////////////
// Registering
void Frame::RegisterPointCloud(Frame& source_tf) {
  // For each point in source_tf, find there is any duplicate in this frame. If not, add the point, height, and disabled
  // to this frame.
  int n_duplicate_p2p = 0;
  int n_duplicate_p2l = 0;
  int n_duplicate_l2l = 0;

  for (int i = 0; i < source_tf.GetNPoints(); i++) {
    bool duplicate = false;

    // point to line comparison
    for (int j = 0; j < GetNLines(); j++) {
      // If a point and a line is close enough, break the loop. (resolution_)
      double d = DistancePointToLineSegment(source_tf.GetOnePoint(i), GetLines().block(0, j, 2, 1),
                                            GetLines().block(2, j, 2, 1));
      if (d < resolution_) {
        duplicate = true;
        n_duplicate_p2l++;
        break;
      }
    }

    // point to point comparison
    if (!duplicate) {
      for (int j = 0; j < GetNPoints(); j++) {
        // If there is a duplicate, break the loop. Duplicate means the x, y value is similar. (resolution_)
        if (fabs(GetOnePoint(j)(0) - source_tf.GetOnePoint(i)(0)) < resolution_ &&
            fabs(GetOnePoint(j)(1) - source_tf.GetOnePoint(i)(1)) < resolution_) {
          duplicate = true;
          n_duplicate_p2p++;
          break;
        }
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
  for (int i = 0; i < source_tf.GetNLines(); i++) {
    lines_.conservativeResize(5, lines_.cols() + 1);
    lines_.col(lines_.cols() - 1) = source_tf.GetLines().col(i);
  }
}
