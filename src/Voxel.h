#pragma once

#include <vector>

#include <Eigen/Core>

namespace gripper {

using std::vector;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::Vector3d;
using Eigen::RowVector3f;

class Voxel {
public:
  typedef vector<size_t> VoxelList;

  static Voxel Voxelize(const MatrixXd& V, const MatrixXi& F, int num_division);

  Voxel(size_t nX, size_t nY, size_t nZ);

  bool operator()(size_t x, size_t y, size_t z) const;
  bool& operator()(size_t x, size_t y, size_t z);

  void GenerateMesh(MatrixXd& V, MatrixXi& F, float boxSize,
    VoxelList voxels) const;
  void GeneratePoints(MatrixXd& P) const;

  VoxelList GetAllVoxelIndex() const;
  VoxelList GetSupportPointCandidates(
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


  inline size_t GetVoxelIndex(size_t x, size_t y, size_t z) const {
    return x * m_nYZ + y * nZ + z;
  }

  inline void GetVoxelCoord(size_t index, size_t &x, size_t &y, size_t &z) const {
    x = index / m_nYZ;
    y = (index / nZ) % nY;
    z = index % nZ;
  }

  template<class VectorClass>
  inline VectorClass GetVoxelCoord(size_t index) const {
    return VectorClass(index / m_nYZ, (index / nZ) % nY, index % nZ);
  }
};

}