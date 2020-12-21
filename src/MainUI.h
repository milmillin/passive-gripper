#pragma once

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <Eigen/Core>

#include "MeshInfo.h"
#include "Voxel.h"

namespace gripper {

enum LayerId {
  Mesh = 0,
  Voxelized = 1
};

class MainUI : public igl::opengl::glfw::imgui::ImGuiMenu {
public:
  MainUI();

  void init(igl::opengl::glfw::Viewer* _viewer);
  void draw_viewer_menu();
  bool post_load();

private:
  void voxelize();

  bool meshLoaded;
  MeshInfo meshInfo;

  bool voxelized;
  Voxel voxel;
  int num_division;
};

}