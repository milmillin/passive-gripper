#pragma once

namespace gripper {

struct VoxelPipelineSettings {
  // Voxelization Settings
  int numDivision = 30;

  // Filter Voxel Settings
  Eigen::Vector3f grabDirection = Eigen::Vector3f(-1, 0, 0);

  // View Settings
  bool showAsPoint = false;
  float voxelScale = 0.5f;
};

}