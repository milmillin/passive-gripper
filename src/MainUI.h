#pragma once

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <Eigen/Core>

#include "MeshInfo.h"
#include "Voxel.h"

namespace gripper {

enum LayerId {
  Mesh = 0,
  Voxelized,
  CenterPoint,
  Max
};

class MainUI : public igl::opengl::glfw::imgui::ImGuiMenu {
public:
  MainUI();

  void init(igl::opengl::glfw::Viewer* _viewer);
  void draw_viewer_menu();
  bool post_load();

private:
  void voxelize();
  void refreshVoxel();

  inline Eigen::MatrixXd &getMeshVertices() { return viewer->data(LayerId::Mesh).V; }
  inline Eigen::MatrixXi &getMeshFaces() { return viewer->data(LayerId::Mesh).F; }

  bool meshLoaded;
  MeshInfo meshInfo;

  bool voxelized;
  Voxel voxel;
  int num_division;

  float voxelBoxSizeScale = 0.5;
  bool showPoints = false;
  bool showSupportPointCandidates = false;
  bool filterByGripDirection = false;
  Eigen::Vector3f gripDirection = Eigen::Vector3f(-1, 0, 0);

  bool showSupportPoints;
  Voxel::VoxelCoordList selectedSupportPoints;

  Voxel::VoxelCoordList getCandidateSupportPoints();
  void selectRandomSupportPoints();
  double evaluateSupportPoints();
};

}