#pragma once

#include <Eigen/Core>

namespace gripper {

struct VoxelPipelineSettings {
  // Type B contact grid spacing (m)
  double gridSpacing = 0.005;

  // Center of mass voxel size (m)
  double voxelSize = gridSpacing;

  // Threshold angle (degree)
  double thresholdAngle = 30;

  // Grab angle (degree)
  Eigen::Vector2f grabAngle = Eigen::Vector2f(0, 0);

  // Fitter Dimension (m)
  // https://www.mcmaster.com/9604T14/
  double rodDiameter = 0.012;
  double fitterDiameter = 0.05;
  double fitterMountDiameter = 0.039;
  double fitterScrewDiameter = 0.0044;

  // Epsilon (m)
  double epsilon = 0.001;

  // Solver Settings
  bool findBestContact = false;
};

}  // namespace gripper