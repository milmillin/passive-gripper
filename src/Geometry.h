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

}  // namespace gripper