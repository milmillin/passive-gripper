#include "Geometry.h"

#include <omp.h>
#include <cmath>
#include <iostream>

namespace gripper {

Eigen::MatrixXd GenerateCubeV(Eigen::Vector3d origin, Eigen::Vector3d size) {
  for (ssize_t i = 0; i < 3; i++) {
    if (size(i) < 0) {
      origin(i) += size(i);
      size(i) = -size(i);
    }
  }

  Eigen::MatrixXd out_V;
  out_V = cube_V.cwiseProduct(size.transpose().replicate<8, 1>()) +
          origin.transpose().replicate<8, 1>();
  return out_V;
}

static const double cylinderAngleStep = 2 * EIGEN_PI / cylinderSubdivision;
Eigen::MatrixXd GenerateCylinderV(Eigen::Vector3d p0,
                                  Eigen::Vector3d p1,
                                  double radius) {
  Eigen::Vector3d L = (p1 - p0).normalized();
  Eigen::Vector3d N = L.cross(Eigen::Vector3d::UnitX());
  if (N.squaredNorm() < 1e-6) N = L.cross(Eigen::Vector3d::UnitY());
  Eigen::Vector3d B = L.cross(N);

  N *= radius;
  B *= radius;

  Eigen::MatrixXd v(cylinderNumV, 3);
  for (size_t i = 0; i < cylinderSubdivision; i++) {
    v.row(i) =
        p0 + N * cos(cylinderAngleStep * i) + B * sin(cylinderAngleStep * i);
    v.row(i + cylinderSubdivision) =
        p1 + N * cos(cylinderAngleStep * i) + B * sin(cylinderAngleStep * i);
  }
  return v;
}

Eigen::MatrixXi GenerateCylinderF() {
  Eigen::MatrixXi f(cylinderNumF, 3);
  for (size_t i = 0; i < cylinderSubdivision; i++) {
    f.row(i) = Eigen::RowVector3i(
        i, (i + 1) % cylinderSubdivision, i + cylinderSubdivision);
    f.row(i + cylinderSubdivision) =
        Eigen::RowVector3i((i + 1) % cylinderSubdivision + cylinderSubdivision,
                           i + cylinderSubdivision,
                           (i + 1) % cylinderSubdivision);
  }
  for (size_t i = 1; i < cylinderSubdivision - 1; i++) {
    f.row(i + 2 * cylinderSubdivision - 1) = Eigen::RowVector3i(i + 1, i, 0);
    f.row(i + 3 * cylinderSubdivision - 3) =
        Eigen::RowVector3i(0, i, i + 1).array() + cylinderSubdivision;
  }
  return f;
}

Eigen::Vector3f GetDirectionFromAngle(const Eigen::Vector2f& angle) {
  float A = angle(0) * DEGREE_TO_RADIAN;
  float B = angle(1) * DEGREE_TO_RADIAN;
  float cosA = std::cos(A);
  float sinA = std::sin(A);
  float cosB = std::cos(B);
  float sinB = std::sin(B);
  return Eigen::Vector3f(cosA * cosB, sinB, sinA * cosB);
}

// Input
// p1, p2: unit vector from center of mass to contact point
// Returns
// (1) Side of the plane p1-p2
// (2) Angle between gravity and plane p1-p2
static std::pair<bool, double> AngleFromSideToGravity(Eigen::Vector3d p1,
                                                      Eigen::Vector3d p2) {
  static const Eigen::Vector3d gravity = -Eigen::Vector3d::UnitY();
  Eigen::Vector3d n = p1.cross(p2);
  double cosAlpha = n.dot(gravity);
  return {cosAlpha > 0, abs(EIGEN_PI / 2. - acos(cosAlpha))};
}

std::pair<int, double> EvaluateContactPoints(
    const Eigen::Vector3d& centerOfMass,
    const ContactPoint& c1,
    const ContactPoint& c2,
    const ContactPoint& c3) {

  Eigen::Vector3d p1 = (c1.position - centerOfMass).normalized();
  Eigen::Vector3d p2 = (c2.position - centerOfMass).normalized();
  Eigen::Vector3d p3 = (c3.position - centerOfMass).normalized();

  auto res1 = AngleFromSideToGravity(p1, p2);
  auto res2 = AngleFromSideToGravity(p2, p3);
  auto res3 = AngleFromSideToGravity(p3, p1);

  // Gravity not in triangle
  if (res1.first != res2.first || res1.first != res3.first) {
    return {-1, 0.};      
  }
  
  int countA = 0;
  if (c1.isTypeA) countA++;
  if (c2.isTypeA) countA++;
  if (c3.isTypeA) countA++;

  return {countA, std::min({res1.second, res2.second, res3.second})};
}

}  // namespace gripper