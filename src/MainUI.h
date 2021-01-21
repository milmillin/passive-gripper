#pragma once

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <Eigen/Core>

#include "MeshInfo.h"
#include "VoxelPipeline.h"
#include "VoxelPipelineSettings.h"
#include "Voxels.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace gripper {

enum LayerId {
  Mesh = 0,
  VoxelAll,
  VoxelFiltered,
  VoxelBest,
  GripperMesh,
  Offset,
  Max
};

class VoxelPipeline;

class MainUI : public igl::opengl::glfw::imgui::ImGuiMenu {
 public:
  MainUI();
  ~MainUI();

  void init(igl::opengl::glfw::Viewer* _viewer) override;
  void draw_viewer_window() override;
  void draw_viewer_menu() override;
  bool post_load() override;
  inline bool pre_draw() override;
  inline bool post_draw() override;

  std::mutex viewerDataMutex;
  inline igl::opengl::ViewerData& GetViewerData(LayerId layerId) {
    return viewer->data(layerId);
  }

 private:
  void UpdateVoxels();
  void DrawGrabDirection();

  inline Eigen::MatrixXd& GetMeshVertices() {
    return viewer->data(LayerId::Mesh).V;
  }
  inline Eigen::MatrixXi& GetMeshFaces() {
    return viewer->data(LayerId::Mesh).F;
  }

  // Mesh
  bool meshLoaded;
  MeshInfo meshInfo;

  // VoxelPipeline
  std::unique_ptr<VoxelPipeline> voxelPipeline;
  VoxelPipelineSettings voxelSettings;

  // Task
  std::deque<std::pair<std::unique_ptr<std::atomic<bool>>,
                       std::unique_ptr<std::thread>>>
      tasks;
};

}  // namespace gripper