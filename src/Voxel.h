#pragma once

#include <vector>

#include <Eigen/Core>

#include "Utils.h"

namespace gripper {

using std::vector;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::Vector3d;
using Eigen::RowVector3f;

// Voxel coordinates
struct VoxelCoord {
  ssize_t x, y, z;
};

class Voxel {
public:
  typedef vector<VoxelCoord> VoxelCoordList;

  static Voxel Voxelize(const MatrixXd& V, const MatrixXi& F, int num_division);

  Voxel(size_t nX, size_t nY, size_t nZ);

  bool operator()(size_t x, size_t y, size_t z) const;
  bool& operator()(size_t x, size_t y, size_t z);

  void GenerateMesh(MatrixXd& V, MatrixXi& F, float boxSize,
    VoxelCoordList voxels) const;
  void GeneratePoints(MatrixXd& P) const;

  VoxelCoordList GetAllVoxelIndex() const;
  VoxelCoordList GetSupportPointCandidates(
    const MatrixXd& V,
    const MatrixXi& F,
    Vector3d grabDirection) const;

  size_t nX;
  size_t nY;
  size_t nZ;
  double CubeSize;
  Vector3d Origin;
private:
  Eigen::Matrix<bool, -1, 1> m_data;
  size_t m_nYZ;
  size_t m_nXYZ;


  inline ssize_t GetVoxelIndex(ssize_t x, ssize_t y, ssize_t z) const {
    return x * m_nYZ + y * nZ + z;
  }

  inline void GetVoxelCoord(ssize_t index, ssize_t &x, ssize_t &y, ssize_t &z) const {
    x = index / m_nYZ;
    y = (index / nZ) % nY;
    z = index % nZ;
  }


  inline VoxelCoord GetVoxelCoord(ssize_t index) const {
    if (index >= m_nXYZ)
      throw std::runtime_error(ERROR_MESSAGE("Voxel index out of bound"));
    return VoxelCoord{index / m_nYZ, (index / nZ) % nY, index % nZ};
  }

  template<class VectorClass>
  inline VectorClass GetVoxelCoordVector(ssize_t index) const {
    if (index >= m_nXYZ)
      throw std::runtime_error(ERROR_MESSAGE("Voxel index out of bound"));
    return VoxelCoord{index / m_nYZ, (index / nZ) % nY, index % nZ};
  }

  template<class VectorClass>
  inline VectorClass GetVoxelCoordVector(VoxelCoord coord) const {
    return VectorClass(coord.x, coord.y, coord.z);
  }
};

}