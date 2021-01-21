#pragma once

#include <Eigen/Core>

namespace gripper {

struct MeshInfo {
  Eigen::Vector3d minimum;
  Eigen::Vector3d maximum;
  Eigen::Vector3d size;

  MeshInfo();
  MeshInfo(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
};

}  // namespace gripper
