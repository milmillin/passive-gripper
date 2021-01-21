#pragma once

#include <vector>

#include <embree/common/sys/platform.h>
#include <Eigen/Core>

#include <igl/embree/EmbreeIntersector.h>
#include "Utils.h"

namespace gripper {

using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::RowVector3f;
using Eigen::Vector3d;
using Eigen::Vector3f;
using std::vector;

class Voxels {
 public:
  typedef Eigen::Matrix<ssize_t, 3, 1> Voxel;
  typedef Eigen::Matrix<double, 3, 1> VoxelD;

  static void Voxelize(const MatrixXd& mesh_V,
                       const MatrixXi& mesh_F,
                       double voxelSize,
                       Voxels& out_voxels,
                       std::vector<Voxel>& out_voxelCoords);

  template <typename T>
  void GenerateMesh(const std::vector<Eigen::Matrix<T, 3, 1>>& voxelCoords,
                    float voxelBoxSizeScale,
                    MatrixXd& out_V,
                    MatrixXi& out_F) const;

  template <typename T>
  void GeneratePoints(const std::vector<Eigen::Matrix<T, 3, 1>>& voxelCoords,
                      MatrixXd& out_P) const;

  // UNUSED
  std::vector<Voxel> FilterSupportingVoxels(
      const Eigen::MatrixXd& mesh_V,
      const Eigen::MatrixXi& mesh_F,
      const std::vector<Voxel>& voxelCoords,
      double groundY) const;

  // UNUSED
  std::vector<Voxel> FilterGrabDirection(const std::vector<Voxel>& voxelCoords,
                                         const MatrixXd& mesh_V,
                                         const MatrixXi& mesh_F,
                                         Vector3f grabDirection) const;

  std::vector<VoxelD> GetSupportCandidates(std::vector<Voxel> voxelCoords,
                                           Vector3f grabDirection,
                                           double groundY) const;

  template <typename T, typename U>
  inline Eigen::Matrix<T, 3, 1> GetVoxelCenter(
      const Eigen::Matrix<U, 3, 1>& voxel) const {
    return (origin + (voxel.template cast<double>() + Vector3d(0.5, 0.5, 0.5)) *
                         cubeSize)
        .template cast<T>();
  }

  template <typename T, typename U>
  inline Eigen::Matrix<T, 3, 1> GetVoxelOrigin(
      const Eigen::Matrix<U, 3, 1>& voxel) const {
    return (origin + voxel.template cast<double>() * cubeSize)
        .template cast<T>();
  }

  VoxelD GetCenterOfMass(const std::vector<Voxel>& voxelCoords) const;

  double cubeSize;
  Vector3d origin;

 private:
  igl::embree::EmbreeIntersector m_intersector;
  Eigen::MatrixXd m_mesh_V;
  Eigen::MatrixXi m_mesh_F;
};

template void Voxels::GenerateMesh(
    const std::vector<Eigen::Matrix<ssize_t, 3, 1>>& voxelCoords,
    float voxelBoxSizeScale,
    MatrixXd& out_V,
    MatrixXi& out_F) const;
template void Voxels::GenerateMesh(
    const std::vector<Eigen::Matrix<double, 3, 1>>& voxelCoords,
    float voxelBoxSizeScale,
    MatrixXd& out_V,
    MatrixXi& out_F) const;

template void Voxels::GeneratePoints(
    const std::vector<Eigen::Matrix<ssize_t, 3, 1>>& voxelCoords,
    MatrixXd& out_P) const;
template void Voxels::GeneratePoints(
    const std::vector<Eigen::Matrix<double, 3, 1>>& voxelCoords,
    MatrixXd& out_P) const;

}  // namespace gripper