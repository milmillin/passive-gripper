#pragma once

#include <Eigen/Core>
#include <set>
#include <random>

#include "Voxels.h"

namespace gripper {

using Eigen::Vector3d;
using Eigen::Vector2d;
using std::set;


double DoubleSignedArea(const Vector3d &t1, const Vector3d &t2, const Vector3d &t3);

// Test if the object is stable using the support points
// p: center of mass for this object
// t1, t2, t3: support points
bool IsSupportPointStable(const Vector3d &p,
  const Vector3d &t1, const Vector3d &t2, const Vector3d &t3);

double HorizontalDistance(const Vector3d &p, const Vector3d &l1, const Vector3d &l2);

double TriangleStability(const Vector3d &p,
  const Vector3d &t1, const Vector3d &t2, const Vector3d &t3);

template<class T>
set<T> SelectInRange(T start, T end, T n) {
  set<T> selected;
  static std::default_random_engine generator;
  std::uniform_int_distribution<T> distribution(start, end);

  if (end < start || n > end - start + 1)
    return selected;

  for (T i = 0; i < n; i++) {
    int next;
    do
      next = distribution(generator);
    while (selected.find(next) != selected.end());
    selected.insert(next);
  }
  return selected;
}

Eigen::Vector3f GetDirectionFromAngle(const Eigen::Vector2f& angle);

std::vector<Voxels::Voxel> FindBestContactDumb(const std::vector<Voxels::Voxel>& voxelCoords, const Voxels::VoxelD& centerOfMass);

// Inline mesh of a cube
const MatrixXd cube_V = (MatrixXd(8, 3) <<
  0.0, 0.0, 0.0,
  0.0, 0.0, 1.0,
  0.0, 1.0, 0.0,
  0.0, 1.0, 1.0,
  1.0, 0.0, 0.0,
  1.0, 0.0, 1.0,
  1.0, 1.0, 0.0,
  1.0, 1.0, 1.0).finished();
const MatrixXi cube_F = (MatrixXi(12, 3) <<
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

Eigen::MatrixXd GenerateCubeV(Eigen::Vector3d origin, Eigen::Vector3d size);

}  // namespace gripper