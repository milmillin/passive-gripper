#pragma once

#include <Eigen/Core>
#include <vector>

#include "Voxels.h"

namespace gripper {

class Gripper {
public:
  Gripper(const Eigen::MatrixXd& mesh_V, const Eigen::MatrixXi& mesh_F,
    const Voxels& voxels, const std::vector<Voxels::Voxel>& voxelCoords,
    const Eigen::Vector2f& grabAngle);
  Gripper();

  inline const Eigen::MatrixXd& V() const { return gripper_V; }
  inline const Eigen::MatrixXi& F() const { return gripper_F; }
private:
  Eigen::Vector2f m_grabAngle;

  Eigen::MatrixXd gripper_V;
  Eigen::MatrixXi gripper_F;
};

}