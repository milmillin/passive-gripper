#pragma once

#include <Eigen/Core>
#include <atomic>

#include "Gripper.h"
#include "MainUI.h"
#include "VoxelPipelineSettings.h"
#include "Voxels.h"

namespace gripper {

class MainUI;

class VoxelPipeline {
 public:
  VoxelPipeline(MainUI* mainUI,
                const Eigen::MatrixXd& mesh_V,
                const Eigen::MatrixXi& mesh_F);

  void UpdateSettings(const VoxelPipelineSettings& settings,
                      bool isInit = false);
  inline bool IsReady() { return m_isReady; }

 private:
  MainUI* m_mainUI;
  std::atomic<bool> m_isReady;
  VoxelPipelineSettings m_settings;

  Eigen::MatrixXd m_mesh_V;
  Eigen::MatrixXi m_mesh_F;
  MeshInfo m_meshInfo;

  Eigen::MatrixXd m_offset_mesh_V;
  Eigen::MatrixXi m_offset_mesh_F;
  MeshInfo m_offset_meshInfo;

  Voxels m_voxels;
  Voxels::VoxelD m_centerOfMass;

  std::vector<Voxels::Voxel> m_allCoords;
  Eigen::MatrixXd m_all_V;
  Eigen::MatrixXi m_all_F;
  Eigen::MatrixXd m_all_P;

  std::vector<Voxels::Voxel> m_supportingCoords;
  Eigen::MatrixXd m_supporting_V;
  Eigen::MatrixXi m_supporting_F;
  Eigen::MatrixXd m_supporting_P;

  std::vector<Voxels::Voxel> m_filteredCoords;
  Eigen::MatrixXd m_filtered_V;
  Eigen::MatrixXi m_filtered_F;
  Eigen::MatrixXd m_filtered_P;

  std::vector<Voxels::Voxel> m_bestCoords;
  Eigen::MatrixXd m_best_V;
  Eigen::MatrixXi m_best_F;
  Eigen::MatrixXd m_best_P;

  Gripper m_gripper;
};

}  // namespace gripper