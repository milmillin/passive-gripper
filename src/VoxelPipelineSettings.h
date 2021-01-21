#pragma once

namespace gripper {

struct VoxelPipelineSettings {
  // Voxelization Settings
  double voxelSize = 0.005;

  // Filter Voxel Settings
  Eigen::Vector2f grabAngle = Eigen::Vector2f(0, 0);  // In degrees

  // Rod Diameter
  double rodDiameter = 0.011;

  // Solver Settings
  bool findBestContact = false;

  // View Settings
  bool showAsPoint = false;
  float voxelScale = 0.5f;
};

}  // namespace gripper