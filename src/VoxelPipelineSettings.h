#pragma once

namespace gripper {

struct VoxelPipelineSettings {
  // Voxelization Settings
  int numDivision = 30;

  // Filter Voxel Settings
  Eigen::Vector2f grabAngle = Eigen::Vector2f(0, 0); // In degrees

  // View Settings
  bool showAsPoint = false;
  float voxelScale = 0.5f;
};

}