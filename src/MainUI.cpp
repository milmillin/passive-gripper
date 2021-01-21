#include "MainUI.h"

#include <imgui.h>
#include "Geometry.h"

namespace gripper {

MainUI::MainUI() : igl::opengl::glfw::imgui::ImGuiMenu(), meshLoaded(false) {}

MainUI::~MainUI() {
  while (!tasks.empty()) {
    tasks.front().second->join();
    tasks.pop_front();
  }
}

void MainUI::init(igl::opengl::glfw::Viewer* _viewer) {
  igl::opengl::glfw::imgui::ImGuiMenu::init(_viewer);

  // Add other layers
  for (int layerID = 1; layerID < LayerId::Max; layerID++) {
    viewer->data_list.emplace_back();
    viewer->data_list.back().id = (LayerId)layerID;
  }
  viewer->next_data_id = LayerId::Max;

  // Set default point size
  viewer->data_list[LayerId::VoxelAll].point_size = 3;
  viewer->data_list[LayerId::VoxelSupporting].point_size = 3;
  viewer->data_list[LayerId::VoxelFiltered].point_size = 3;
}

void MainUI::draw_viewer_window() {
  float menu_width = 180.f * menu_scaling();
  ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f),
                                      ImVec2(300.0f, -1.0f));
  bool _viewer_menu_visible = true;
  ImGui::Begin(
      "Viewer",
      &_viewer_menu_visible,
      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);
  if (callback_draw_viewer_menu) {
    callback_draw_viewer_menu();
  } else {
    draw_viewer_menu();
  }
  ImGui::PopItemWidth();
  ImGui::End();
}

void MainUI::draw_viewer_menu() {
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
    if (ImGui::CollapsingHeader("Voxelization",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Text("Minimum: (%.2lf, %.2lf, %.2lf)",
                  meshInfo.minimum.x(),
                  meshInfo.minimum.y(),
                  meshInfo.minimum.z());
      ImGui::Text("Maximum: (%.2lf, %.2lf, %.2lf)",
                  meshInfo.maximum.x(),
                  meshInfo.maximum.y(),
                  meshInfo.maximum.z());
      ImGui::Text("Size: (%.2lf, %.2lf, %.2lf)",
                  meshInfo.size.x(),
                  meshInfo.size.y(),
                  meshInfo.size.z());
      ImGui::Separator();

      ImGui::PushItemWidth(120.f);
      ImGui::InputDouble(
          "Voxel Size (m)", &voxelSettings.voxelSize, 0.001, 0.01);
      ImGui::InputDouble(
          "Rod Diameter (m)", &voxelSettings.rodDiameter, 0.001, 0.01);
      if (ImGui::DragFloat2("Grab angle",
                            voxelSettings.grabAngle.data(),
                            1.f,
                            -360.f,
                            360.f)) {
        DrawGrabDirection();
      }
      ImGui::InputFloat("Voxel scale", &voxelSettings.voxelScale, 0.1, 0.2);
      ImGui::Checkbox("Show as point", &voxelSettings.showAsPoint);
      ImGui::Checkbox("Find Best Contact", &voxelSettings.findBestContact);
      ImGui::PopItemWidth();

      if (voxelPipeline == nullptr || voxelPipeline->IsReady()) {
        if (ImGui::Button("Update Voxels", ImVec2(w - p, 0))) {
          UpdateVoxels();
        }
      } else {
        ImGui::Text("Busy...");
      }
    }
  }
  if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushID("View");
    if (ImGui::InputFloat("Point Size",
                          &(viewer->data(LayerId::VoxelAll).point_size),
                          1,
                          2,
                          "%.0f")) {
      viewer->data(LayerId::VoxelFiltered).point_size =
          viewer->data(LayerId::VoxelSupporting).point_size =
              viewer->data(LayerId::VoxelAll).point_size;
    }

    ImGui::Checkbox("Mesh", (bool*)&(viewer->data(LayerId::Mesh).is_visible));
    ImGui::Checkbox("Offset Mesh",
                    (bool*)&(viewer->data(LayerId::Offset).is_visible));
    ImGui::Checkbox("Voxel",
                    (bool*)&(viewer->data(LayerId::VoxelAll).is_visible));
    ImGui::Checkbox(
        "Voxel Support",
        (bool*)&(viewer->data(LayerId::VoxelSupporting).is_visible));
    ImGui::Checkbox("Voxel Filtered",
                    (bool*)&(viewer->data(LayerId::VoxelFiltered).is_visible));
    ImGui::Checkbox("Voxel Best",
                    (bool*)&(viewer->data(LayerId::VoxelBest).is_visible));
    ImGui::Checkbox("Gripper",
                    (bool*)&(viewer->data(LayerId::GripperMesh).is_visible));

    ImGui::PopID();
  }
}

void MainUI::UpdateVoxels() {
  bool isInit = false;
  if (voxelPipeline == nullptr) {
    voxelPipeline.reset(
        new VoxelPipeline(this, GetMeshVertices(), GetMeshFaces()));
    isInit = true;
  }

  // Update using new thread
  auto done = new std::atomic<bool>(false);
  auto task = new std::thread([=] {
    voxelPipeline->UpdateSettings(voxelSettings, isInit);
    done->store(true);
  });
  tasks.push_back(
      std::move(std::make_pair(std::unique_ptr<std::atomic<bool>>(done),
                               std::unique_ptr<std::thread>(task))));
}

void MainUI::DrawGrabDirection() {
  static Eigen::RowVector3d directionColor = Eigen::RowVector3d(1, 0.5, 0);
  auto& layer = viewer->data(LayerId::Mesh);
  layer.clear_edges();
  layer.add_edges(Eigen::RowVector3d::Zero(),
                  GetDirectionFromAngle(voxelSettings.grabAngle)
                          .cast<double>()
                          .transpose() *
                      3,
                  directionColor);
}

bool MainUI::pre_draw() {
  while (!tasks.empty() && tasks.front().first->load()) {
    tasks.front().second->join();
    tasks.pop_front();
  }
  viewerDataMutex.lock();
  ImGuiMenu::pre_draw();
  return false;
}

bool MainUI::post_draw() {
  ImGuiMenu::post_draw();
  viewerDataMutex.unlock();
  return false;
}

bool MainUI::post_load() {
  meshLoaded = true;
  meshInfo = MeshInfo(GetMeshVertices(), GetMeshFaces());
  voxelPipeline.reset();
  DrawGrabDirection();
  return true;
}

}  // namespace gripper