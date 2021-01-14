#include "Geometry.h"

#include <cmath>
#include <iostream>
#include <omp.h>

namespace gripper {

// Horizontal is x-z plane
double HorizontalTwiceSignedArea(const Vector3d &t1, const Vector3d &t2, const Vector3d &t3) {
  return (t1(0) - t3(0)) * (t2(2) - t3(2)) - (t2(0) - t3(0)) * (t1(2) - t3(2));
}

bool IsSupportPointStable(const Vector3d &p,
    const Vector3d &t1, const Vector3d &t2, const Vector3d &t3) {
  double d1 = HorizontalTwiceSignedArea(p, t1, t2);
  double d2 = HorizontalTwiceSignedArea(p, t2, t3);
  double d3 = HorizontalTwiceSignedArea(p, t3, t1);
  // std::cout << "IsSupportPointStable " << d1 << " " << d2 << " " << d3 << std::endl;
  return (d1 > 0 && d2 > 0 && d3 > 0) || (d1 < 0 && d2 < 0 && d3 < 0);
}

double HorizontalDistance(const Vector3d &p, const Vector3d &l1, const Vector3d &l2) {
  Vector2d v(l2(2) - l1(2), -(l2(0) - l1(0)));
  v.normalize();
  Vector2d r(l1(0) - p(0), l1(2) - p(2));
  // std::cout <<"vector "<< v(0) <<" " <<v(1)<<" " <<r(0)<<" "<<r(1)<<std::endl;
  // std::cout <<"dot " << v.dot(r);
  // std::cout <<"abs " << std::abs(v.dot(r));
  return std::abs(v.dot(r));
}

double TriangleStability(const Vector3d &p,
    const Vector3d &t1, const Vector3d &t2, const Vector3d &t3) {
  if (!IsSupportPointStable(p, t1, t2, t3)) {
    return 0;
  }
  double d1 = HorizontalDistance(p, t1, t2);
  double d2 = HorizontalDistance(p, t2, t3);
  double d3 = HorizontalDistance(p, t3, t1);
  // std::cout << "TriangleStability " << d1 << " " << d2 << " " << d3 << std::endl;
  return std::min({d1, d2, d3});
}

std::vector<Voxels::Voxel> FindBestContactDumb(const std::vector<Voxels::Voxel>& voxelCoords, const Voxels::VoxelD& centerOfMass)
{
  typedef Eigen::Matrix<ssize_t, 3, 1> Index3;
  Index3 bestIndex(-1, -1, -1);
  double bestStability = 0;
  ssize_t numVoxels = voxelCoords.size();

  #pragma omp parallel
  {
    Index3 t_bestIndex(-1, -1, -1);
    double t_bestStability = 0;
    double t_stability = 0;

    #pragma omp for
    for (ssize_t i = 0; i < numVoxels; i++) {
      for (ssize_t j = i + 1; j < numVoxels; j++) {
        for (ssize_t k = j + 1; k < numVoxels; k++) {
          t_stability = TriangleStability(
            centerOfMass,
            voxelCoords[i].cast<double>(),
            voxelCoords[j].cast<double>(),
            voxelCoords[k].cast<double>()
          );
          if (t_stability > t_bestStability) {
            t_bestStability = t_stability;
            t_bestIndex = Index3(i, j, k);
          }
        }
      }
    }

    #pragma omp critical
    if (t_bestStability > bestStability) {
      bestStability = t_bestStability;
      bestIndex = t_bestIndex;
    }
  }

  std::vector<Voxels::Voxel> result;
  for (int i = 0; i < 3; i++) {
    if (bestIndex(i) != -1) result.push_back(voxelCoords[bestIndex(i)]);
  }
  return result;
}

Eigen::Vector3f GetDirectionFromAngle(const Eigen::Vector2f& angle) {
  float A = angle(0) / 180.f * EIGEN_PI;
  float B = angle(1) / 180.f * EIGEN_PI;
  float cosA = std::cos(A);
  float sinA = std::sin(A);
  float cosB = std::cos(B);
  float sinB = std::sin(B);
  return Eigen::Vector3f(cosA * cosB, sinB, sinA * cosB);
}

inline double distance(double x, double y) {
  return std::sqrt(x * x + y * y);
}

bool IsSupportPointStable(const Vector3d &center, const Eigen::Matrix3d &rotation,
    double threshold,
    const vector<Vector3d> &p, const vector<Vector3d> &dir) {
  assert(dir.size() == 3 && p.size() == 3);
  assert(threshold >= -0.0);

  for (size_t i = 0; i < dir.size(); i++) {
    Vector3d newDir = rotation * dir[i];
    if (std::atan2(newDir(1), distance(newDir(0), newDir(2))) < threshold)
      return false;
  }

  vector<Vector3d> newP;
  for (const auto &point : p)
    newP.push_back(rotation * point);
  return IsSupportPointStable(rotation * center, newP[0], newP[1], newP[2]);
}

static double getMinStableAngleHelper(const Vector3d &p1, const Vector3d &p2, const Vector3d &other) {
  Vector3d n = p1.cross(p2);
  if (n.dot(other) < 0)
    n = -n;

  auto &gravity = -Vector3d::UnitY();
  return EIGEN_PI / 2 - 2 * std::atan2((n - gravity).norm(), (n + gravity).norm());
}

double getMinStableAngle(const Vector3d &center,
    double threshold,
    const vector<Vector3d> &p, const vector<Vector3d> &dir) {
  assert(dir.size() == 3 && p.size() == 3);
  assert(threshold >= -0.0);

  vector<Vector3d> unitP;
  for (auto &point : p)
    unitP.push_back((point - center).normalized());

  double a1 = getMinStableAngleHelper(unitP[0], unitP[1], unitP[2]);
  double a2 = getMinStableAngleHelper(unitP[1], unitP[2], unitP[0]);
  double a3 = getMinStableAngleHelper(unitP[2], unitP[0], unitP[1]);
  return std::min({a1, a2, a3});
}

}  // namespace gripper