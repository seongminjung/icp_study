#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Core>

double DistancePointToLineSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b);

#endif  // UTILS_H