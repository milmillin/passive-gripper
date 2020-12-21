#include "MainUI.h"

#include "Voxel.h"

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

  viewer->data_list.emplace_back();
  viewer->data_list.back().id = LayerId::Voxelized;
  viewer->next_data_id = 2;
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
    ImGui::Checkbox("Mesh##View", (bool*)&(viewer->data(LayerId::Mesh).is_visible));
    ImGui::Checkbox("Voxelized##View", (bool*)&(viewer->data(LayerId::Voxelized).is_visible));
  }
}

bool MainUI::post_load()
{
  meshLoaded = true;
  meshInfo = MeshInfo(viewer->data(LayerId::Mesh).V, viewer->data(LayerId::Mesh).F);
  return true;
}

void MainUI::voxelize()
{
  voxel = Voxel::Voxelize(viewer->data(LayerId::Mesh).V, viewer->data(LayerId::Mesh).F,
    num_division);

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  voxel.GenerateMesh(V, F);

  igl::opengl::ViewerData& viewerData = viewer->data(LayerId::Voxelized);
  viewerData.clear();
  viewerData.set_face_based(true);
  viewerData.set_mesh(V, F);
  voxelized = true;
}

}