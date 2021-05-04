#pragma once

#include <Eigen/Core>
#include <iostream>

#include "Geometry.h"

namespace gripper {

struct VoxelPipelineSettings {
  // Contact grid spacing (m)
  double gridSpacing = 0.005;

  double marchingCubeSize = gridSpacing;

  // Center of mass voxel size (m)
  double voxelSize = gridSpacing;

  // Epsilon (m)
  double epsilon = 0.001;

  // Max Extension Length (m)
  double maxExtensionLength = 0.02;

  // Threshold angle (degree)
  float thresholdAngle = 20;

  // Grab angle (degree)
  Eigen::Vector2f grabAngle = Eigen::Vector2f(0, 0);

  // Fitter Dimension (m)
  // https://www.mcmaster.com/9604T14/
  double rodDiameter = 0.008;
  double rodClearance = 0.0002;
  double fitterDiameter = 0.045;
  double fitterMountDiameter = 0.031;
  double fitterScrewDiameter = 0.0034;

  // Solver Settings
  bool findBestContact = true;
};

// TODO: Change this
#define FORMAT(os, x) os << #x ": " << (x) << '\n'
inline std::ostream& operator<<(std::ostream& os,
                                const VoxelPipelineSettings& s) {
  FORMAT(os, s.gridSpacing);
  FORMAT(os, s.voxelSize);
  FORMAT(os, s.epsilon);
  FORMAT(os, s.maxExtensionLength);
  FORMAT(os, s.thresholdAngle);
  FORMAT(os, s.grabAngle(0));
  FORMAT(os, s.grabAngle(1));
  FORMAT(os, s.rodDiameter);
  FORMAT(os, s.rodClearance);
  FORMAT(os, s.fitterDiameter);
  FORMAT(os, s.fitterMountDiameter);
  FORMAT(os, s.fitterScrewDiameter);
  return os;
}
#undef FORMAT

}  // namespace gripper