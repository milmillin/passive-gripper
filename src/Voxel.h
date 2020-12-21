#pragma once

#include <Eigen/Core>

namespace gripper {

class Voxel {
public:
  static Voxel Voxelize(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, int num_division);

  Voxel(size_t nX, size_t nY, size_t nZ);

  bool operator()(size_t x, size_t y, size_t z) const;
  bool& operator()(size_t x, size_t y, size_t z);

  void GenerateMesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) const;

  size_t nX;
  size_t nY;
  size_t nZ;
  double CubeSize;
  Eigen::Vector3d Origin;
private:
  Eigen::Matrix<bool, -1, 1> m_data;
  size_t m_nYZ;
  size_t m_nXYZ;
};

}