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
  Eigen::Vector3d p12 = (c2.position - c1.position);
  Eigen::Vector3d p13 = (c3.position - c1.position);
  double orient = p12.z() * p13.x() - p13.z() * p12.x();

  Eigen::Vector3d p1 = (c1.position - centerOfMass).normalized();
  Eigen::Vector3d p2 = (c2.position - centerOfMass).normalized();
  Eigen::Vector3d p3 = (c3.position - centerOfMass).normalized();

  auto res1 = AngleFromSideToGravity(p1, p2);
  auto res2 = AngleFromSideToGravity(p2, p3);
  auto res3 = AngleFromSideToGravity(p3, p1);

  // Gravity not in triangle
  if (res1.first != res2.first || res1.first != res3.first ||
      res1.first != orient < 0) {
    return {-1, 0.};
  }

  int countA = 0;
  if (c1.isTypeA) countA++;
  if (c2.isTypeA) countA++;
  if (c3.isTypeA) countA++;

  return {countA, std::min({res1.second, res2.second, res3.second})};
}

bool IsOptimal(Eigen::Block<Eigen::MatrixXd, 1> lastRow, size_t& pivotCol) {
  size_t nCols = lastRow.cols();
  pivotCol = -1;
  for (size_t i = 0; i < nCols - 1; i++) {
    if (lastRow(i) > 0) {
      if (pivotCol == -1 || lastRow(pivotCol) < lastRow(i)) {
        pivotCol = i;
      }
    }
  }
  return pivotCol == -1;
}

bool CheckForceClosure(const std::vector<ContactPoint>& contactPoints,
                       Eigen::Vector3d centerOfMass,
                       Eigen::Vector3d gravity) {
  // Simplex Method Phase I

  size_t nContacts = contactPoints.size();
  size_t nCols = nContacts + 7;
  Eigen::MatrixXd T(7, nCols);
  T.setZero();

  for (size_t i = 0; i < nContacts; i++) {
    const auto& cp = contactPoints[i];
    // Force
    T.block<3, 1>(0, i) = cp.normal.transpose();
    // Torque
    T.block<3, 1>(3, i) = (cp.position - centerOfMass).cross(cp.normal);
  }
  // -External Force
  T.block<3, 1>(0, nCols - 1) = -gravity.transpose();

  // Flip so that rhs is positive
  for (size_t i = 0; i < 6; i++) {
    if (T(i, nCols - 1) < 0) T.row(i) *= -1;
  }

  // Adjust Variable
  for (size_t i = 0; i < 6; i++) {
    T(i, nContacts + i) = 1;
    T(6, nContacts + i) = -1;
  }
  for (size_t i = 0; i < 6; i++) {
    T.row(6) += T.row(i);
  }

  // std::cout << "Initial:\n" << T << std::endl;
  size_t owner[6];
  for (size_t i = 0; i < 6; i++) {
    owner[i] = nContacts + i;  
  }  

  // Simplex Iteration
  size_t pivotCol = -1;
  size_t pivotRow;
  double tmp;
  double tmp2;
  double minRatio = 0.0;
  while (!IsOptimal(T.row(6), pivotCol)) {
    pivotRow = -1;
    for (size_t i = 0; i < 6; i++) {
      tmp = T(i, nCols - 1);
      tmp2 = T(i, pivotCol);
      if (tmp2 < 1e-12) continue;
      tmp /= tmp2;
      if (tmp < 0) continue;
      if (pivotRow == -1 || tmp < minRatio) {
        pivotRow = i;
        minRatio = tmp;
      }
    }
    if (pivotRow == -1) {
      std::cout << "Unbounded\n";
      // << T.block<7, 7>(0, nContacts) << std::endl;
      break;
    }

    owner[pivotRow] = pivotCol;

    T.row(pivotRow) /= T(pivotRow, pivotCol);
    for (size_t i = 0; i < 7; i++) {
      if (i == pivotRow) continue;
      T.row(i) -= T.row(pivotRow) * T(i, pivotCol);    
    }
  }

  Eigen::MatrixXd result(1, nContacts + 6);
  result.setZero();
  for (size_t i = 0; i < 6; i++) {
    result(owner[i]) = T(i, nCols - 1);
  }
  double obj = T(6, nCols - 1);
  std::cout << "Coeffs:\n" << result.block(0, 0, 1, nContacts) << std::endl;
  std::cout << "Errors:\n" << result.block<1, 6>(0, nContacts) << std::endl;
  std::cout << "Sum error: " << obj << std::endl;
  return abs(obj) < 1e-12;
}

void GetPerp(const Eigen::Vector3d& N, Eigen::Vector3d& B, Eigen::Vector3d& T) {
  B = N.cross(Eigen::Vector3d::UnitX());
  if (B.squaredNorm() < 1e12) B = N.cross(Eigen::Vector3d::UnitY());
  B.normalize();
  T = B.cross(N);
}

}  // namespace gripper