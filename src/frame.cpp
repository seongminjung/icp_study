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
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZ>);
  SetIndexVector(*ptr_cloud, resolution_);
  Voxelize(*ptr_cloud, *ptr_voxelized, resolution_, min_points_per_voxel_);

  // line extraction
  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> lines;
  ExtractLine(*ptr_voxelized, lines);

  // set points
  points_.resize(2, lines.size());
  heights_.resize(lines.size());
  for (int i = 0; i < lines.size(); i++) {
    // We are sure there are no 2 points with the same x, y value, since we have avoided it in ExtractLine().
    points_(0, i) = lines[i].first.x;
    points_(1, i) = lines[i].first.y;
    heights_(i) = lines[i].second.z - lines[i].first.z;
  }

  // set disabled_
  disabled_ = Eigen::VectorXi::Zero(points_.cols());
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
void Frame::SetIndexVector(pcl::PointCloud<pcl::PointXYZ>& input, double voxel_size) {
  index_vector.clear();
  index_vector.reserve(input.points.size());

  // First pass: go over all points and insert them into the index_vector vector
  // with calculated idx. Points with the same idx value will contribute to the
  // same point of resulting CloudPoint
  for (int i = 0; i < input.points.size(); i++) {
    unsigned int x = round(input.points[i].x / voxel_size) + 512 - 1;  // offset 512 - 1
    unsigned int y = round(input.points[i].y / voxel_size) + 512 - 1;
    unsigned int z = round(input.points[i].z / voxel_size) + 512 - 1;

    // hashing
    unsigned int rx = (x << 20) & 0x3FF00000;
    unsigned int ry = (y << 10) & 0x000FFC00;
    unsigned int rz = z & 0x000003FF;
    unsigned int hash = rx + ry + rz;

    index_vector.emplace_back(hash, i);
  }

  // Second pass: sort the index_vector vector using value representing target cell as index
  // in effect all points belonging to the same output cell will be next to each other
  std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());
}

void Frame::Voxelize(pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& output, double voxel_size,
                     unsigned int min_points_per_voxel) {
  // Third pass: count output cells
  // we need to skip all the same, adjacent idx values
  unsigned int total = 0;
  unsigned int index = 0;
  std::vector<int> first_indices_vector;
  v_index_vector.clear();
  v_index_vector.reserve(output.points.size());
  while (index < index_vector.size()) {
    unsigned int i = index + 1;
    while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx) ++i;
    if (i - index >= min_points_per_voxel) {
      ++total;
      first_indices_vector.emplace_back(index);
      // <hash, index of voxel vector>
      v_index_vector.emplace_back(index_vector[index].idx, first_indices_vector.size() - 1);
    }
    index = i;
  }

  // Fourth pass: insert voxels into the output
  output.points.reserve(total);
  for (int first_idx : first_indices_vector) {
    // unhashing
    double x = (int((index_vector[first_idx].idx & 0x3FF00000) >> 20) - (512 - 1)) * voxel_size;
    double y = (int((index_vector[first_idx].idx & 0x000FFC00) >> 10) - (512 - 1)) * voxel_size;
    double z = (int(index_vector[first_idx].idx & 0x000003FF) - (512 - 1)) * voxel_size;

    output.points.emplace_back(x, y, z);
  }
  output.width = static_cast<std::uint32_t>(output.points.size());
  output.height = 1;       // downsampling breaks the organized structure
  output.is_dense = true;  // we filter out invalid points
}

void Frame::ExtractLine(pcl::PointCloud<pcl::PointXYZ>& v_input,
                        std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& output) {
  int idx1 = 0, idx2 = 1;
  while (idx2 < v_index_vector.size()) {
    if (v_index_vector[idx2].idx - v_index_vector[idx1].idx == idx2 - idx1) {
      idx2++;
    } else {
      if (idx2 - idx1 > 2) {
        pcl::PointXYZ p1, p2;
        p1.x = v_input.points[v_index_vector[idx1].voxel_index].x;
        p1.y = v_input.points[v_index_vector[idx1].voxel_index].y;
        p1.z = v_input.points[v_index_vector[idx1].voxel_index].z;
        p2.x = v_input.points[v_index_vector[idx2 - 1].voxel_index].x;
        p2.y = v_input.points[v_index_vector[idx2 - 1].voxel_index].y;
        p2.z = v_input.points[v_index_vector[idx2 - 1].voxel_index].z;
        // emplace back to output only when p1.x and p2.x is not the same as the last entry's. This is for avoiding
        // creating duplicate points.
        if (output.size() == 0 ||
            (output.size() > 0 && p1.x != output.back().first.x || p1.y != output.back().first.y)) {
          output.emplace_back(p1, p2);
        }
      }
      idx1 = idx2;
      idx2++;
    }
  }
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
      // If there is a duplicate, break the loop. Duplicate means the x, y value is similar (half of resolution_)
      if (fabs(GetOnePoint(j)(0) - source_tf.GetOnePoint(i)(0)) < resolution_ / 2 &&
          fabs(GetOnePoint(j)(1) - source_tf.GetOnePoint(i)(1)) < resolution_ / 2) {
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
