#pragma once

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <Eigen/Core>

#include "MeshInfo.h"
#include "VoxelPipeline.h"
#include "VoxelPipelineSettings.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace gripper {


class VoxelPipeline;

class MainUI : public igl::opengl::glfw::imgui::ImGuiMenu {
 public:
  enum class LayerId : int {
    Mesh = 0,
    Offset,
    GripperDirection,
    CenterOfMass,
    TypeAContacts,
    TypeBContacts,
    FilteredContacts,
    BestContacts,
    GripperMesh,
    ContactRay,
    Max
  };

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
    return viewer->data((int)layerId);
  }
  inline Eigen::MatrixXd& GetMeshVertices() {
    return viewer->data((int)LayerId::Mesh).V;
  }
  inline Eigen::MatrixXi& GetMeshFaces() {
    return viewer->data((int)LayerId::Mesh).F;
  }

  // VoxelPipeline
  std::unique_ptr<VoxelPipeline> voxelPipeline;
  VoxelPipelineSettings voxelSettings;

 private:
  void UpdateVoxels();
  void RunPerformanceTest();
  void DrawGrabDirection();
  void SaveDXF();
  void SaveResult();
  void SaveGripper();
  void SaveRAPID();

  void UpdateCameraAngle(float angle0, float angle1);
  void Render();

  // Mesh
  bool meshLoaded;
  MeshInfo meshInfo;

  // Perf Test
  // number of blocks in the longest dimension
  // lo, hi, step
  int perfSettings[3] = {30, 100, 5};

  // Camera
  float angle[2] = {75, 10};
  int renderSize[2] = {1080, 1080};

  // Task
  std::deque<std::pair<std::unique_ptr<std::atomic<bool>>,
                       std::unique_ptr<std::thread>>>
      tasks;
};

}  // namespace gripper