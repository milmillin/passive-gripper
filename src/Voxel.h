#pragma once

#include <vector>

#include <Eigen/Core>

namespace gripper {

class Voxel {
public:
  static Voxel Voxelize(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, int num_division);

  Voxel(size_t nX, size_t nY, size_t nZ);

  bool operator()(size_t x, size_t y, size_t z) const;
  bool& operator()(size_t x, size_t y, size_t z);

  void GenerateMesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F, float boxSize) const;
  void GeneratePoints(Eigen::MatrixXd& P) const;

  size_t nX;
  size_t nY;
  size_t nZ;
  double CubeSize;
  Eigen::Vector3d Origin;
private:
  Eigen::Matrix<bool, -1, 1> m_data;
  size_t m_nYZ;
  size_t m_nXYZ;

  std::vector<size_t> GetAllVoxelIndex() const;

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