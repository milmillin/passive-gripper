#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

#include "ContactPoint.h"
#include "MeshInfo.h"
#include "VoxelPipelineSettings.h"

namespace gripper {

class Gripper {
 public:
  Gripper(const std::vector<ContactPoint>& contactPoints,
          const Eigen::Vector3d& centerOfMass,
          const VoxelPipelineSettings& settings,
          const MeshInfo& rotatedMeshInfo,
          const Eigen::Affine3d& rotation);

  Gripper();

  void WriteDXF(const std::string& filename) const;

  inline const Eigen::MatrixXd& V() const { return gripper_V; }
  inline const Eigen::MatrixXi& F() const { return gripper_F; }

  Eigen::MatrixX2d rodLocations;
  std::vector<double> rodLengths;
  Eigen::Vector2d plateDimension;
  Eigen::Vector3d plateOrigin;

 private:
  Eigen::MatrixXd gripper_V;
  Eigen::MatrixXi gripper_F;

  double m_cmLocationX;
  double m_mountOriginY;

  double m_rodRadius;
  double m_rodClearance;
  double m_fitterRadius;
  double m_fitterMountRadius;
  double m_fitterScrewRadius;
};

}  // namespace gripper