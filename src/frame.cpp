#include "icp_study/frame.h"

Frame::Frame() {}

Frame::Frame(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*point_cloud_msg, *ptr_cloud);

  // make 10 x 10 grid with resolution of 0.2 meter
  resolution_ = 0.2;
  width_ = 1024;
  height_ = 1024;

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
  for (int i = 0; i < lines.size(); i++) {
    // line in the same cell
    if (i > 0 && lines[i].first.x == points_(0, i - 1) && lines[i].first.y == points_(1, i - 1)) continue;

    points_(0, i) = lines[i].first.x;
    points_(1, i) = lines[i].first.y;
  }

  // print number of pointcloud points and frame points
  ROS_INFO("pointcloud timestamp: %f", point_cloud_msg->header.stamp.toSec());
  ROS_INFO("pointcloud size: %ld", ptr_cloud->points.size());
  ROS_INFO("frame size: %ld", points_.cols());
}

Frame::Frame(const Frame& other) {
  timestamp_ = other.timestamp_;
  resolution_ = other.resolution_;
  width_ = other.width_;
  height_ = other.height_;
  points_ = other.points_;
}

////////////////////////////////
// Getters
time_t Frame::GetTimestamp() { return timestamp_; }
double Frame::GetResolution() { return resolution_; }
unsigned int Frame::GetWidth() { return width_; }
unsigned int Frame::GetHeight() { return height_; }
unsigned int Frame::GetSize() { return points_.cols(); }
Eigen::MatrixXd Frame::GetPoints() { return points_; }
Eigen::Vector2d Frame::GetOnePoint(unsigned int idx) { return points_.col(idx); }

////////////////////////////////
// Setters
void Frame::SetTimestamp(time_t timestamp) { timestamp_ = timestamp; }
void Frame::SetResolution(double resolution) { resolution_ = resolution; }
void Frame::SetWidth(unsigned int width) { width_ = width; }
void Frame::SetHeight(unsigned int height) { height_ = height; }
void Frame::SetPoints(Eigen::MatrixXd points) { points_ = points; }
void Frame::SetOnePoint(unsigned int idx, Eigen::Vector2d point) {
  points_(0, idx) = point(0);
  points_(1, idx) = point(1);
}
void Frame::ReserveSize(unsigned int size) { points_.resize(2, size); }

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
        output.emplace_back(p1, p2);
      }
      idx1 = idx2;
      idx2++;
    }
  }
}
