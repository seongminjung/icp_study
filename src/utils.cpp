#include <icp_study/utils.h>

#include <Eigen/Core>

unsigned int CoordToHash(double x, double y, double z, double resolution) {
  unsigned int rounded_x = round(x / resolution) + 512;  // offset 512
  unsigned int rounded_y = round(y / resolution) + 512;
  unsigned int rounded_z = round(z / resolution) + 512;

  // hashing
  unsigned int hashed_x = (rounded_x << 10) & 0x000FFC00;
  unsigned int hashed_y = (rounded_y << 20) & 0x3FF00000;
  unsigned int hashed_z = rounded_z & 0x000003FF;
  unsigned int hash = hashed_x + hashed_y + hashed_z;

  return hash;
}

void HashToXY(unsigned int hash, double& x, double& y, double resolution) {
  // unhashing
  // type casting to double is imperative here to avoid unexpected line extraction fault
  x = (double((hash & 0x000FFC00) >> 10) - 512) * resolution;
  y = (double((hash & 0x3FF00000) >> 20) - 512) * resolution;
}

double DistancePointToLineSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  // Function to calculate the distance from a point p to a line segment defined by two points a and b
  Eigen::Vector2d ap = p - a;
  Eigen::Vector2d ab = b - a;

  // Calculate the dot product
  double ab2 = ab.dot(ab);
  double ap_ab = ap.dot(ab);
  // Calculate the magnitude of the projection of ap onto ab, normalized by the length of ab
  double t = std::max(0.0, std::min(1.0, ap_ab / ab2));

  // Find the projection point
  Eigen::Vector2d projection = a + ab * t;

  // Calculate the distance from p to the projection point
  return (p - projection).norm();
}