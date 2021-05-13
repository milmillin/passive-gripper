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

class SelectUI : public igl::opengl::glfw::imgui::ImGuiMenu {
 public:
  enum class LayerId : int {
    Mesh = 0,
    ContactPoints,
    Max
  };

  SelectUI();
  ~SelectUI();

  void init(igl::opengl::glfw::Viewer* _viewer) override;
  void draw_viewer_window() override;
  void draw_viewer_menu() override;
  bool post_load() override;
  inline bool pre_draw() override;
  inline bool post_draw() override;

  bool mouse_down(int button, int modifier);
  bool mouse_up(int button, int modifier);

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
 private:
  void Invalidate();

  // Mouse
  double m_mouse_x;
  double m_mouse_y;

  // Mesh
  bool meshLoaded;
  MeshInfo meshInfo;

  // Contact Points
  std::vector<ContactPoint> m_contactPoints;

  // Task
  std::deque<std::pair<std::unique_ptr<std::atomic<bool>>,
                       std::unique_ptr<std::thread>>>
      tasks;
};

}  // namespace gripper
