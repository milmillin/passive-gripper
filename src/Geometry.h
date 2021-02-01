#pragma once

#include <igl/embree/EmbreeIntersector.h>
#include <Eigen/Core>
#include <random>
#include <set>

#include "Voxels.h"

namespace gripper {

using Eigen::RowVector3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::set;

Vector3d ComputeNormal(const RowVector3d& p1,
                       const RowVector3d& p2,
                       const RowVector3d& p3);

// Test if the object is stable using the support points
// p: center of mass for this object
// t1, t2, t3: support points
bool IsSupportPointStable(const Vector3d& p,
                          const Vector3d& t1,
                          const Vector3d& t2,
                          const Vector3d& t3);

double HorizontalDistance(const Vector3d& p,
                          const Vector3d& l1,
                          const Vector3d& l2);

double TriangleStability(const Vector3d& p,
                         const Vector3d& t1,
                         const Vector3d& t2,
                         const Vector3d& t3);

Eigen::Vector3f GetDirectionFromAngle(const Eigen::Vector2f& angle);

std::vector<Voxels::VoxelD> FindBestContactDumb(
    const std::vector<Voxels::VoxelD>& voxelCoords,
    const Voxels::VoxelD& centerOfMass);

std::vector<size_t> FindBestContactDumb2(
    const std::vector<Voxels::VoxelD>& voxelCoords,
    const std::vector<Eigen::Vector3d>& normals,
    const Voxels::VoxelD& centerOfMass);


std::vector<Eigen::Vector3d> RefineContactPoint(
    const Eigen::MatrixXd& mesh_V,
    const Eigen::MatrixXi& mesh_F,
    const Voxels& voxels,
    const std::vector<Voxels::Voxel>& voxelCoords);

bool IsSupportPointStable(const Vector3d& center,
                          const Eigen::Matrix3d& rotation,
                          double threshold,
                          const vector<Vector3d>& p,
                          const vector<Vector3d>& dir);

double GetMinStableAngle(const Vector3d& center,
                         double threshold,
                         const vector<Vector3d>& p,
                         const vector<Vector3d>& dir);

// clang-format off
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
// clang-format on

Eigen::MatrixXd GenerateCubeV(Eigen::Vector3d origin, Eigen::Vector3d size);

constexpr size_t cylinderSubdivision = 8;
constexpr size_t cylinderNumV = cylinderSubdivision * 2;
constexpr size_t cylinderNumF = 4 * cylinderSubdivision - 4;
Eigen::MatrixXd GenerateCylinderV(Eigen::Vector3d p0,
                                  Eigen::Vector3d p1,
                                  double radius);
Eigen::MatrixXi GenerateCylinderF();

const MatrixXi cylinder_F = GenerateCylinderF();

}  // namespace gripper