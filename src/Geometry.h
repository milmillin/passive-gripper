#pragma once

#include <igl/embree/EmbreeIntersector.h>
#include <Eigen/Core>
#include <random>

#include "ContactPoint.h"

namespace gripper {

const float DEGREE_TO_RADIAN = EIGEN_PI / 180;

Eigen::Vector3f GetDirectionFromAngle(const Eigen::Vector2f& angle);

// Returns
// (1) number of Type A contacts, -1 if impossible, greater is better
// (2) minimum angle between center of mass and triangle on a sphere,
//     greater is better
std::pair<int, double> EvaluateContactPoints(
    const Eigen::Vector3d& centerOfMass,
    const ContactPoint& c1,
    const ContactPoint& c2,
    const ContactPoint& c3);

// clang-format off
// Inline mesh of a cube
const Eigen::MatrixXd cube_V = (Eigen::MatrixXd(8, 3) <<
  0.0, 0.0, 0.0,
  0.0, 0.0, 1.0,
  0.0, 1.0, 0.0,
  0.0, 1.0, 1.0,
  1.0, 0.0, 0.0,
  1.0, 0.0, 1.0,
  1.0, 1.0, 0.0,
  1.0, 1.0, 1.0).finished();
const Eigen::MatrixXi cube_F = (Eigen::MatrixXi(12, 3) <<
  1, 7, 5,
  1, 3, 7,
  1, 4, 3,
  1, 2, 4,
  3, 8, 7,
  3, 4, 8,
  5, 7, 8,
  5, 8, 6,
  1, 5, 6,
  1, 6, 2,
  2, 6, 8,
  2, 8, 4).finished().array() - 1;
// clang-format on

Eigen::MatrixXd GenerateCubeV(Eigen::Vector3d origin, Eigen::Vector3d size);

constexpr size_t cylinderSubdivision = 16;
constexpr size_t cylinderNumV = cylinderSubdivision * 2;
constexpr size_t cylinderNumF = 4 * cylinderSubdivision - 4;
Eigen::MatrixXd GenerateCylinderV(Eigen::Vector3d p0,
                                  Eigen::Vector3d p1,
                                  double radius);
Eigen::MatrixXi GenerateCylinderF();

const Eigen::MatrixXi cylinder_F = GenerateCylinderF();

}  // namespace gripper