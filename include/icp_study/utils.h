#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Core>

unsigned int CoordToHash(double x, double y, double z, double resolution);
void HashToXY(unsigned int hash, double& x, double& y, double resolution);

double DistancePointToLineSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b);

#endif  // UTILS_H