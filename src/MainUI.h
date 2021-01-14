#pragma once

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <Eigen/Core>

#include "MeshInfo.h"
#include "Voxels.h"
#include "VoxelPipeline.h"
#include "VoxelPipelineSettings.h"

#include <mutex>
#include <vector>
#include <queue>
#include <atomic>
#include <thread>
#include <memory>

namespace gripper {

enum LayerId {
  Mesh = 0,
  VoxelAll,
  VoxelSupporting,
  VoxelFiltered,
  VoxelBest,
  NewMetricTest,
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
  inline igl::opengl::ViewerData& GetViewerData(LayerId layerId) { return viewer->data(layerId); }
private:
  void UpdateVoxels();
  void DrawGrabDirection();

  inline Eigen::MatrixXd& GetMeshVertices() { return viewer->data(LayerId::Mesh).V; }
  inline Eigen::MatrixXi& GetMeshFaces() { return viewer->data(LayerId::Mesh).F; }

  // Mesh
  bool meshLoaded;
  MeshInfo meshInfo;

  // VoxelPipeline
  std::unique_ptr<VoxelPipeline> voxelPipeline;
  VoxelPipelineSettings voxelSettings;

  Vector3f supportPoints[3] = {{2, 0, 0}, {0, 0, 2}, {-2, 0, -2}};
  Vector3f supportDirections[3] = {{0, 1, 0}, {0, 1, 0}, {0, 1, 0}};
  Vector3f center = {0, 5, 0};
  int pitchDivision = 16;
  int yarnDivision = 16;
  float threshold = 0.05;
  double stability = 0;
  double minStablePitch = 0;
  double minStablePitchAccurate = 0;
  std::chrono::nanoseconds calculationTime;

  void evaluateNewMetric();

  // Task
  std::deque<std::pair<std::unique_ptr<std::atomic<bool>>, std::unique_ptr<std::thread>>> tasks;
};

}