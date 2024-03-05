#include "icp_study/frame.h"

#include <random>

#include "icp_study/utils.h"

Frame::Frame() {}

Frame::Frame(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg, int mode) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*point_cloud_msg, *ptr_cloud);

  // make 10 x 10 grid with resolution of 0.2 meter
  timestamp_ = point_cloud_msg->header.stamp;
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
ros::Time Frame::GetTimestamp() { return timestamp_; }
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
Eigen::VectorXd Frame::GetOneLine(unsigned int idx) { return lines_.col(idx); }
Eigen::VectorXi Frame::GetDisabled() { return disabled_; }
bool Frame::GetOnePointDisabled(unsigned int idx) { return disabled_(idx); }

////////////////////////////////
// Setters
void Frame::SetTimestamp(ros::Time timestamp) { timestamp_ = timestamp; }
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
void Frame::SetOneLine(unsigned int idx, Eigen::VectorXd line) { lines_.col(idx) = line; }
void Frame::SetAllPointsDisabled(bool disabled) { disabled_ = Eigen::VectorXi::Ones(points_.cols()) * disabled; }
void Frame::SetOnePointDisabled(unsigned int idx, bool disabled) { disabled_(idx) = disabled; }
void Frame::ReserveSize(unsigned int size) {
  points_.resize(2, size);
  heights_.resize(size);
  disabled_.resize(size);
}
void Frame::RemoveOneLine(unsigned int idx) {
  // Remove one line from lines_
  lines_.block(0, idx, 5, lines_.cols() - idx - 1) = lines_.block(0, idx + 1, 5, lines_.cols() - idx - 1);
  lines_.conservativeResize(5, lines_.cols() - 1);
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
  // For each point in source_tf, find if there is any duplicate in this frame. If not, add the point, height, and
  // disabled to this frame.
  int map_n_points = points_.cols();
  int sum_n_points = map_n_points + source_tf.GetNPoints();
  points_.conservativeResize(2, sum_n_points);
  heights_.conservativeResize(sum_n_points);
  disabled_.conservativeResize(sum_n_points);

  for (int i = 0; i < source_tf.GetNPoints(); i++) {
    bool duplicate = false;

    // point to line comparison
    for (int j = 0; j < GetNLines(); j++) {
      // If a point and a line is close enough, break the loop. (resolution_)
      double d = DistancePointToLineSegment(source_tf.GetOnePoint(i), GetLines().block(0, j, 2, 1),
                                            GetLines().block(2, j, 2, 1));
      if (d < resolution_) {
        duplicate = true;
        break;
      }
    }

    // point to point comparison
    if (!duplicate) {
      for (int j = 0; j < GetNPoints(); j++) {
        // If there is a duplicate, break the loop. Duplicate means the x, y value is similar. (resolution_)
        if (fabs(GetOnePoint(j)(0) - source_tf.GetOnePoint(i)(0)) < resolution_ &&
            fabs(GetOnePoint(j)(1) - source_tf.GetOnePoint(i)(1)) < resolution_) {
          if (source_tf.GetOneHeight(i) > heights_(j)) {
            // If the height of the source_tf is higher, swap the points instead of setting duplicate to true.
            points_.col(j) = source_tf.GetOnePoint(i);
            SetOneHeight(j, source_tf.GetOneHeight(i));
          }
          duplicate = true;
          break;
        }
      }
    }

    if (!duplicate) {
      points_.col(map_n_points) = source_tf.GetOnePoint(i);
      heights_(map_n_points) = source_tf.GetOneHeight(i);
      disabled_(map_n_points) = source_tf.GetOnePointDisabled(i);
      map_n_points++;
    }
  }

  points_.conservativeResize(2, map_n_points);
  heights_.conservativeResize(map_n_points);
  disabled_.conservativeResize(map_n_points);

  int map_n_lines = lines_.cols();
  int sum_n_lines = map_n_lines + source_tf.GetNLines();
  lines_.conservativeResize(5, sum_n_lines);  // Worst case

  // For each line in source_tf, find if there is any duplicate in this frame. If not, add the line to this frame.
  for (int i = 0; i < source_tf.GetNLines(); i++) {
    std::vector<int> state0_lines;  // Lines that will be ignored
    std::vector<int> state1_lines;  // Lines that will substitute map line
    std::vector<int> state2_lines;  // Lines that will be merged with map line

    for (int j = 0; j < map_n_lines; j++) {
      // If there is a duplicate, break the loop.
      double d1_s2m = DistancePointToLineSegment(source_tf.GetLines().block(0, i, 2, 1), lines_.block(0, j, 2, 1),
                                                 lines_.block(2, j, 2, 1));
      double d2_s2m = DistancePointToLineSegment(source_tf.GetLines().block(2, i, 2, 1), lines_.block(0, j, 2, 1),
                                                 lines_.block(2, j, 2, 1));
      double d1_m2s = DistancePointToLineSegment(lines_.block(0, j, 2, 1), source_tf.GetLines().block(0, i, 2, 1),
                                                 source_tf.GetLines().block(2, i, 2, 1));
      double d2_m2s = DistancePointToLineSegment(lines_.block(2, j, 2, 1), source_tf.GetLines().block(0, i, 2, 1),
                                                 source_tf.GetLines().block(2, i, 2, 1));

      if (d1_s2m < resolution_ && d2_s2m < resolution_) {
        state0_lines.emplace_back(j);
      } else if (d1_m2s < resolution_ && d2_m2s < resolution_) {
        state1_lines.emplace_back(j);
      } else if ((d1_s2m - resolution_) * (d2_s2m - resolution_) < 0 &&
                 (d1_m2s - resolution_) * (d2_m2s - resolution_) < 0) {
        state2_lines.emplace_back(j);
      }
    }

    // print for debug
    std::cout << "line " << i << " from source_tf: " << source_tf.GetLines().col(i).transpose() << std::endl;
    std::cout << "state0_lines: ";
    for (int j = 0; j < state0_lines.size(); j++) {
      std::cout << state0_lines[j] << " ";
    }
    std::cout << std::endl;
    std::cout << "state1_lines: ";
    for (int j = 0; j < state1_lines.size(); j++) {
      std::cout << state1_lines[j] << " ";
    }
    std::cout << std::endl;
    std::cout << "state2_lines: ";
    for (int j = 0; j < state2_lines.size(); j++) {
      std::cout << state2_lines[j] << " ";
    }
    std::cout << std::endl;

    // If all of state0_lines, state1_lines, and state2_lines are empty, append the line to lines_
    if (state0_lines.empty() && state1_lines.empty() && state2_lines.empty()) {
      lines_.col(map_n_lines) = source_tf.GetOneLine(i);
      map_n_lines++;
    } else if (state0_lines.empty() && !state1_lines.empty() && state2_lines.empty()) {
      if (state1_lines.size() == 1) {
        lines_.col(state1_lines[0]) = source_tf.GetOneLine(i);
      } else {
        // Remove all lines in state1_lines except the first one
        for (int j = state1_lines.size() - 1; j > 0; j--) {
          RemoveOneLine(state1_lines[j]);
          map_n_lines--;
        }
        lines_.col(state1_lines[0]) = source_tf.GetOneLine(i);
      }
    } else if (state0_lines.empty() && state1_lines.empty() && !state2_lines.empty()) {
      for (int j = 0; j < state2_lines.size(); j++) {
        double d1_s2m =
            DistancePointToLineSegment(source_tf.GetLines().block(0, i, 2, 1), lines_.block(0, state2_lines[j], 2, 1),
                                       lines_.block(2, state2_lines[j], 2, 1));
        double d2_s2m =
            DistancePointToLineSegment(source_tf.GetLines().block(2, i, 2, 1), lines_.block(0, state2_lines[j], 2, 1),
                                       lines_.block(2, state2_lines[j], 2, 1));
        // Update the line based on the distance - Change the source line instead of map line, and remove the map line
        // per each update. This way we can deal with multiple partial overlaps.
        if (d1_s2m < d2_s2m) {
          // x coordinate of the source line is greater than the map line
          Eigen::VectorXd tmp = source_tf.GetOneLine(i);
          tmp.segment(0, 2) = lines_.col(state2_lines[j]).segment(0, 2);
          // Average height
          tmp(4) = (tmp(4) + lines_(4, state2_lines[j])) / 2;
          source_tf.SetOneLine(i, tmp);
        } else {
          // x coordinate of the source line is smaller than the map line
          Eigen::VectorXd tmp = source_tf.GetOneLine(i);
          tmp.segment(2, 2) = lines_.col(state2_lines[j]).segment(2, 2);
          // Average height
          tmp(4) = (tmp(4) + lines_(4, state2_lines[j])) / 2;
          source_tf.SetOneLine(i, tmp);
        }
      }
      // Remove all lines in state2_lines, since we appended the merged source line to lines_
      for (int j = state2_lines.size() - 1; j >= 0; j--) {
        RemoveOneLine(state2_lines[j]);
        map_n_lines--;
      }
      // Append the source line to lines_
      lines_.col(map_n_lines) = source_tf.GetOneLine(i);
      map_n_lines++;
    } else if (!state0_lines.empty() && !state1_lines.empty() && state2_lines.empty()) {
      // Remove all lines in state1_lines
      for (int j = state1_lines.size() - 1; j >= 0; j--) {
        RemoveOneLine(state1_lines[j]);
        map_n_lines--;
      }
    } else if (!state0_lines.empty() && state1_lines.empty() && !state2_lines.empty()) {
      // Here, we only use the first elements of state0_lines
      // We merge the first state0_lines with the state2_lines
      source_tf.SetOneLine(i, lines_.col(state0_lines[0]));
      for (int j = 0; j < state2_lines.size(); j++) {
        double d1_s2m =
            DistancePointToLineSegment(source_tf.GetLines().block(0, i, 2, 1), lines_.block(0, state2_lines[j], 2, 1),
                                       lines_.block(2, state2_lines[j], 2, 1));
        double d2_s2m =
            DistancePointToLineSegment(source_tf.GetLines().block(2, i, 2, 1), lines_.block(0, state2_lines[j], 2, 1),
                                       lines_.block(2, state2_lines[j], 2, 1));
        // Update the line based on the distance - Change the source line instead of map line, and remove the map line
        // per each update. This way we can deal with multiple partial overlaps.
        if (d1_s2m < d2_s2m) {
          // x coordinate of the source line is greater than the map line
          Eigen::VectorXd tmp = source_tf.GetOneLine(i);
          tmp.segment(0, 2) = lines_.col(state2_lines[j]).segment(0, 2);
          // Average height
          tmp(4) = (tmp(4) + lines_(4, state2_lines[j])) / 2;
          source_tf.SetOneLine(i, tmp);
        } else {
          // x coordinate of the source line is smaller than the map line
          Eigen::VectorXd tmp = source_tf.GetOneLine(i);
          tmp.segment(2, 2) = lines_.col(state2_lines[j]).segment(2, 2);
          // Average height
          tmp(4) = (tmp(4) + lines_(4, state2_lines[j])) / 2;
          source_tf.SetOneLine(i, tmp);
        }
      }
      // Remove all lines in state2_lines, since we appended the merged source line to lines_
      for (int j = state2_lines.size() - 1; j >= 0; j--) {
        RemoveOneLine(state2_lines[j]);
        map_n_lines--;
      }
      // Append the source line to lines_
      lines_.col(map_n_lines) = source_tf.GetOneLine(i);
      map_n_lines++;
    } else if (state0_lines.empty() && !state1_lines.empty() && !state2_lines.empty()) {
      for (int j = 0; j < state2_lines.size(); j++) {
        double d1_s2m =
            DistancePointToLineSegment(source_tf.GetLines().block(0, i, 2, 1), lines_.block(0, state2_lines[j], 2, 1),
                                       lines_.block(2, state2_lines[j], 2, 1));
        double d2_s2m =
            DistancePointToLineSegment(source_tf.GetLines().block(2, i, 2, 1), lines_.block(0, state2_lines[j], 2, 1),
                                       lines_.block(2, state2_lines[j], 2, 1));
        // Update the line based on the distance - Change the source line instead of map line, and remove the map line
        // per each update. This way we can deal with multiple partial overlaps.
        if (d1_s2m < d2_s2m) {
          // x coordinate of the source line is greater than the map line
          Eigen::VectorXd tmp = source_tf.GetOneLine(i);
          tmp.segment(0, 2) = lines_.col(state2_lines[j]).segment(0, 2);
          // Average height
          tmp(4) = (tmp(4) + lines_(4, state2_lines[j])) / 2;
          source_tf.SetOneLine(i, tmp);
        } else {
          // x coordinate of the source line is smaller than the map line
          Eigen::VectorXd tmp = source_tf.GetOneLine(i);
          tmp.segment(2, 2) = lines_.col(state2_lines[j]).segment(2, 2);
          // Average height
          tmp(4) = (tmp(4) + lines_(4, state2_lines[j])) / 2;
          source_tf.SetOneLine(i, tmp);
        }
      }
      // Remove all lines in state1_lines and state2_lines, since we appended the merged source line to lines_
      std::vector<int> idx_to_remove;
      idx_to_remove.insert(idx_to_remove.end(), state1_lines.begin(), state1_lines.end());
      idx_to_remove.insert(idx_to_remove.end(), state2_lines.begin(), state2_lines.end());
      std::sort(idx_to_remove.begin(), idx_to_remove.end());
      for (int j = idx_to_remove.size() - 1; j >= 0; j--) {
        RemoveOneLine(idx_to_remove[j]);
        map_n_lines--;
      }
      // Append the source line to lines_
      lines_.col(map_n_lines) = source_tf.GetOneLine(i);
      map_n_lines++;
    }
  }

  lines_.conservativeResize(5, map_n_lines);
}
