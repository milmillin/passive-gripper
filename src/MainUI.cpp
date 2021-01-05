#include "MainUI.h"

#include "Voxel.h"
#include "Geometry.h"

namespace gripper {

MainUI::MainUI():
  igl::opengl::glfw::imgui::ImGuiMenu(),
  meshLoaded(false),
  voxelized(false),
  voxel(0, 0, 0),
  num_division(30)
{ }

void MainUI::init(igl::opengl::glfw::Viewer* _viewer)
{
  igl::opengl::glfw::imgui::ImGuiMenu::init(_viewer);

  // Add other layers
  for (int layerID = 1; layerID < LayerId::Max; layerID++) {
    viewer->data_list.emplace_back();
    viewer->data_list.back().id = (LayerId) layerID;
  }
  viewer->next_data_id = LayerId::Max;

  // Set default point size
  viewer->data_list[LayerId::Voxelized].point_size = 3;
}

void MainUI::draw_viewer_menu()
{
  float w = ImGui::GetContentRegionAvailWidth();
  float p = ImGui::GetStyle().FramePadding.x;
  if (ImGui::CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Load##Mesh", ImVec2((w - p) / 2.f, 0))) {
      viewer->selected_data_index = LayerId::Mesh;
      viewer->data().V.resize(0, 0);
      viewer->data().F.resize(0, 0);
      viewer->open_dialog_load_mesh();
    }
    ImGui::SameLine(0, p);
    if (ImGui::Button("Save##Mesh", ImVec2((w - p) / 2.f, 0))) {
      viewer->open_dialog_save_mesh();
    }
  }
  if (meshLoaded) {
    if (ImGui::CollapsingHeader("Voxelization", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Text("Minimum: (%.2lf, %.2lf, %.2lf)",
        meshInfo.Minimum.x(), meshInfo.Minimum.y(), meshInfo.Minimum.z());
      ImGui::Text("Maximum: (%.2lf, %.2lf, %.2lf)",
        meshInfo.Maximum.x(), meshInfo.Maximum.y(), meshInfo.Maximum.z());
      ImGui::Text("Size: (%.2lf, %.2lf, %.2lf)",
        meshInfo.Size.x(), meshInfo.Size.y(), meshInfo.Size.z());
      ImGui::InputInt("# of Division", &num_division, 1, 10);
      ImGui::DragFloat3("Grip Direction", gripDirection.data());
      if (ImGui::Button("Voxelize")) {
        voxelize();
      }
    }
    if (voxelized) {
      if (ImGui::CollapsingHeader("Result", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("# of Cubes %llu", (long long unsigned) voxel.nX * voxel.nY * voxel.nZ);
        ImGui::Text("%llu x %llu x %llu",
          (long long unsigned) voxel.nX,
          (long long unsigned) voxel.nY,
          (long long unsigned) voxel.nZ);
      }
    }
  }
  if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushID("View");

    bool needRefresh = false;

    ImGui::Checkbox("Mesh", (bool*)&(viewer->data(LayerId::Mesh).is_visible));
    ImGui::Checkbox("Voxelized", (bool*)&(viewer->data(LayerId::Voxelized).is_visible));
    needRefresh |= ImGui::Checkbox("Show voxel as points", &showPoints);
    needRefresh |= ImGui::Checkbox("Show support point candidates", &showSupportPointCandidates);
    if (showPoints) {
      ImGui::DragFloat("Point size", &(viewer->data_list[LayerId::Voxelized].point_size));
    } else {
      needRefresh |= ImGui::DragFloat("Voxel box size scale", &(voxelBoxSizeScale));
    }

    needRefresh |= ImGui::DragFloat3("Grip direction", gripDirection.data());
    needRefresh |= ImGui::Checkbox("Filter by grip direction", &filterByGripDirection);
    needRefresh |= ImGui::Checkbox("Show support points", &showSupportPoints);

    if (ImGui::Button("Generate random support point")) {
      do
        selectRandomSupportPoints();
      while (evaluateSupportPoints() < 0.001);
      needRefresh = true;
    }
    if (selectedSupportPoints.size() == 3)
      ImGui::Text("Stability: %f", evaluateSupportPoints());

    if (needRefresh)
      refreshVoxel();

    ImGui::PopID();
  }
}

bool MainUI::post_load()
{
  meshLoaded = true;
  meshInfo = MeshInfo(getMeshVertices(), getMeshFaces());
  return true;
}

void MainUI::voxelize()
{
  voxel = Voxel::Voxelize(getMeshVertices(), getMeshFaces(), num_division);
  voxelized = true;

  refreshVoxel();
}

void MainUI::refreshVoxel() {
  igl::opengl::ViewerData& viewerData = viewer->data(LayerId::Voxelized);
  viewerData.clear();

  if (showPoints) {
    Eigen::MatrixXd P;
    voxel.GeneratePoints(P);

    viewerData.set_points(P, Eigen::RowVector3d(1, 1, 1));
  } else {
    Voxel::VoxelCoordList voxels;
    if (showSupportPointCandidates) {
      voxels = getCandidateSupportPoints();
    } else if (showSupportPoints) {
      voxels = selectedSupportPoints;
    } else {
      voxels = voxel.GetAllVoxelIndex();
    }

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    voxel.GenerateMesh(V, F, voxelBoxSizeScale, voxels);

    viewerData.set_face_based(true);
    viewerData.set_mesh(V, F);
  }
}


Voxel::VoxelCoordList MainUI::getCandidateSupportPoints() {
    auto voxels = voxel.GetSupportPointCandidates();

    if (filterByGripDirection) {
      voxels = voxel.FilterByGrabDirection(voxels,
        getMeshVertices(), getMeshFaces(),
        gripDirection.cast<double>());
    }

    return voxels;
}

void MainUI::selectRandomSupportPoints() {
  auto voxels = getCandidateSupportPoints();
  auto selectedIndices = SelectInRange<size_t>(0, voxels.size() - 1, 3);

  selectedSupportPoints.clear();
  for (auto i : selectedIndices)
    selectedSupportPoints.push_back(voxels[i]);
}

double MainUI::evaluateSupportPoints() {
  auto allVoxels = voxel.GetAllVoxelIndex();
  auto center = voxel.GetCenterPoint(allVoxels);

  // Convert index to coordinate
  vector<Vector3d> points;
  for (auto p : selectedSupportPoints)
    points.push_back(voxel.GetVoxelCoordVector<Vector3d>(p));

  // Draw center point
  auto &layer = viewer->data(LayerId::CenterPoint);
  layer.clear();
  layer.add_points((voxel.Origin + voxel.CubeSize * center).transpose(), Vector3d(1, 1, 1).transpose());
  layer.point_size = 10;
  layer.show_overlay = true;
  std::cout << "Center " << center;

  if (points.size() != 3)
    throw std::runtime_error(ERROR_MESSAGE("Number of support points should be 3"));

  return TriangleStability(center, points[0], points[1], points[2]);
}

}  // namespace gripper