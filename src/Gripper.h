#pragma once

#include <Eigen/Core>
#include <vector>
#include <string>

#include "VoxelPipelineSettings.h"
#include "ContactPoint.h"

namespace gripper {

class Gripper {
 public:
  Gripper(const Eigen::MatrixXd& mesh_V,
          const Eigen::MatrixXi& mesh_F,
          const std::vector<ContactPoint>& contactPoints,
          const Eigen::Vector3d& centerOfMass,
          const VoxelPipelineSettings& settings);

  Gripper();

  void WriteDXF(const std::string& filename) const;

  inline const Eigen::MatrixXd& V() const { return gripper_V; }
  inline const Eigen::MatrixXi& F() const { return gripper_F; }

 private:
  Eigen::Vector2f m_grabAngle;

  Eigen::MatrixXd gripper_V;
  Eigen::MatrixXi gripper_F;

  Eigen::MatrixX2d m_rodLocations;
  std::vector<double> m_rodLengths;
  Eigen::Vector2d m_plateDimension;
  double m_cmLocationX;

  double m_rodRadius;
  double m_fitterRadius;
  double m_fitterMountRadius;
  double m_fitterScrewRadius;
};

}  // namespace gripper