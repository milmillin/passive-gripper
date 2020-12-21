#pragma once

#include <Eigen/Core>

namespace gripper {

struct MeshInfo {
  Eigen::Vector3d Minimum;
  Eigen::Vector3d Maximum;
  Eigen::Vector3d Size;

  MeshInfo();
  MeshInfo(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
};

}
