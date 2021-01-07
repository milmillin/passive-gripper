#pragma once

#include <vector>

#include <Eigen/Core>
#include <embree/common/sys/platform.h>

#include "Utils.h"

namespace gripper {

using std::vector;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::Vector3d;
using Eigen::Vector3f;
using Eigen::RowVector3f;

class Voxels {
public:
  typedef Eigen::Matrix<ssize_t, 3, 1> Voxel;
  typedef Eigen::Matrix<double, 3, 1> VoxelD;

  static void Voxelize(const MatrixXd& mesh_V, const MatrixXi& mesh_F, int numDivision,
    Voxels& out_voxels, std::vector<Voxel>& out_voxelCoords);

  void GenerateMesh(const std::vector<Voxel>& voxelCoords, float voxelBoxSizeScale,
    MatrixXd& out_V, MatrixXi& out_F) const;
  void GeneratePoints(const std::vector<Voxel>& voxelCoords, MatrixXd& out_P) const;

  std::vector<Voxel> FilterSupportingVoxels(const std::vector<Voxel>& voxelCoords) const;
  std::vector<Voxel> FilterGrabDirection(const std::vector<Voxel>& voxelCoords,
    const MatrixXd& mesh_V, const MatrixXi& mesh_F, Vector3f grabDirection) const;


  template<typename T, typename U>
  inline Eigen::Matrix<T, 3, 1> GetVoxelCenter(const Eigen::Matrix<U, 3, 1>& voxel) const {
    return (Origin + (voxel.template cast<double>() + Vector3d(0.5, 0.5, 0.5)) * CubeSize).template cast<T>();
  }

  template<typename T, typename U>
  inline Eigen::Matrix<T, 3, 1> GetVoxelOrigin(const Eigen::Matrix<U, 3, 1>& voxel) const {
    return (Origin + voxel.template cast<double>() * CubeSize).template cast<T>();
  }

  VoxelD GetCenterOfMass(const std::vector<Voxel>& voxelCoords) const;

  double CubeSize;
  Vector3d Origin;
};

}