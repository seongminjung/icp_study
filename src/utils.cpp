#include <icp_study/utils.h>

#include <Eigen/Core>

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