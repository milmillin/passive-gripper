#include "MainUI.h"

#include <igl/png/writePNG.h>
#include <imgui.h>
#include "Geometry.h"
#include "Utils.h"

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
  viewer->data_list[LayerId::CenterOfMass].point_size = 8;
  viewer->data_list[LayerId::TypeAContacts].point_size = 8;
  viewer->data_list[LayerId::TypeBContacts].point_size = 8;
  viewer->data_list[LayerId::FilteredContacts].point_size = 8;
  viewer->data_list[LayerId::BestContacts].point_size = 8;

  // Set default
  viewer->data_list[LayerId::Mesh].is_visible = true;
  viewer->data_list[LayerId::Offset].is_visible = false;
  viewer->data_list[LayerId::GripperDirection].is_visible = false;
  viewer->data_list[LayerId::CenterOfMass].is_visible = false;
  viewer->data_list[LayerId::TypeAContacts].is_visible = false;
  viewer->data_list[LayerId::TypeBContacts].is_visible = false;
  viewer->data_list[LayerId::FilteredContacts].is_visible = false;
  viewer->data_list[LayerId::BestContacts].is_visible = false;
  viewer->data_list[LayerId::GripperMesh].is_visible = true;
  viewer->data_list[LayerId::ContactRay].is_visible = false;

  viewer->data_list[LayerId::GripperMesh].shininess = 10;
  viewer->data_list[LayerId::Mesh].shininess = 10;

  viewer->core().orthographic = true;
}

void MainUI::draw_viewer_window() {
  float menu_width = 180.f * menu_scaling();
  ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);

// Work around for https://github.com/libigl/libigl/issues/1669 at the moment
#ifdef _MSC_VER
  ImGui::SetNextWindowSize(ImVec2(300.f, 600.f), ImGuiCond_FirstUseEver);
#else
  ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f),
                                      ImVec2(300.0f, -1.0f));
#endif
  bool _viewer_menu_visible = true;

#ifdef _MSC_VER
  ImGui::Begin(
      "Viewer", &_viewer_menu_visible, ImGuiWindowFlags_NoSavedSettings);
#else
  ImGui::Begin(
      "Viewer",
      &_viewer_menu_visible,
      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
#endif
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
          "Rod Diameter (m)", &voxelSettings.rodDiameter, 0.001, 0.01);
      ImGui::InputDouble(
          "Rod Clearance (m)", &voxelSettings.rodClearance, 0.0001, 0.001);
      ImGui::InputDouble(
          "Fitter Diameter (m)", &voxelSettings.fitterDiameter, 0.001, 0.01);
      ImGui::InputDouble("Mount Diameter (m)",
                         &voxelSettings.fitterMountDiameter,
                         0.001,
                         0.01);
      ImGui::InputDouble("Screw Diameter (m)",
                         &voxelSettings.fitterScrewDiameter,
                         0.001,
                         0.01);
      ImGui::InputDouble("Grid Spacing for Type B (m)",
                         &voxelSettings.gridSpacing,
                         0.001,
                         0.01);
      ImGui::InputDouble("Marching Cube Size (m)",
                         &voxelSettings.marchingCubeSize,
                         0.001,
                         0.01);
      ImGui::InputDouble(
          "Voxel Size for CM (m)", &voxelSettings.voxelSize, 0.001, 0.01);
      ImGui::InputDouble("Epsilon (m)", &voxelSettings.epsilon, 0.0001, 0.01);
      if (ImGui::InputFloat2(
              "Grab angle y, xz (degree)", voxelSettings.grabAngle.data(), 0)) {
        DrawGrabDirection();
      }
      ImGui::InputFloat(
          "Threshold Angle", &voxelSettings.thresholdAngle, 1, 1, 0);
      ImGui::Checkbox("Find Best Contact", &voxelSettings.findBestContact);
      ImGui::PopItemWidth();

      if (voxelPipeline == nullptr || voxelPipeline->IsReady()) {
        if (ImGui::Button("Update", ImVec2(w - p, 0))) {
          UpdateVoxels();
        }
      } else {
        ImGui::Text("Busy...");
      }
      if (voxelPipeline != nullptr && voxelPipeline->IsReady()) {
        if (ImGui::Button("Save DXF", ImVec2(w - p, 0))) {
          SaveDXF();
        }
        if (ImGui::Button("Save Result", ImVec2(w - p, 0))) {
          SaveResult();
        }
        if (ImGui::Button("Save Gripper OBJ", ImVec2(w - p, 0))) {
          SaveGripper();
        }
        if (ImGui::Button("Save RAPID", ImVec2(w - p, 0))) {
          SaveRAPID();
        }
        if (ImGui::Button("Save Offset Mesh", ImVec2(w - p, 0))) {
          std::string filename = igl::file_dialog_save();
          igl::writeOBJ(filename,
                        viewer->data(LayerId::Offset).V,
                        viewer->data(LayerId::Offset).F);
        }
      }
    }
  }
  if (ImGui::CollapsingHeader("Performance Test",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::InputInt2("Number of Blocks", perfSettings);
    ImGui::InputInt("Step", &perfSettings[2], 1, 5);
    if (voxelPipeline != nullptr && voxelPipeline->IsReady()) {
      if (ImGui::Button("Run Perf Test", ImVec2(w - p, 0))) {
        RunPerformanceTest();
      }
    } else {
      ImGui::Text("Busy...");
    }
  }
  if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::InputFloat2("Camera angle y, xz (degree)", angle, 0);
    if (ImGui::Button("Update View", ImVec2(w - p, 0))) {
      UpdateCameraAngle(angle[0], angle[1]);
    }
    ImGui::InputInt2("Render Size", renderSize);
    if (ImGui::Button("Render", ImVec2(w - p, 0))) {
      Render();
    }
  }
  if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushID("View");
    if (ImGui::InputFloat("Point Size",
                          &(viewer->data(LayerId::CenterOfMass).point_size),
                          1,
                          2,
                          "%.1f")) {
      float pointSize = viewer->data(LayerId::CenterOfMass).point_size;
      viewer->data(LayerId::TypeAContacts).point_size = pointSize;
      viewer->data(LayerId::TypeBContacts).point_size = pointSize;
      viewer->data(LayerId::FilteredContacts).point_size = pointSize;
      viewer->data(LayerId::BestContacts).point_size = pointSize;
    }

    if (ImGui::Checkbox("Show lines",
                        (bool*)&(viewer->data(LayerId::Mesh).show_lines))) {
      viewer->data(LayerId::GripperMesh).show_lines =
          viewer->data(LayerId::Offset).show_lines =
              viewer->data(LayerId::Mesh).show_lines;
    }

    ImGui::Checkbox("Mesh", (bool*)&(viewer->data(LayerId::Mesh).is_visible));
    ImGui::Checkbox("Offset Mesh",
                    (bool*)&(viewer->data(LayerId::Offset).is_visible));
    ImGui::Checkbox(
        "Gripper Direction",
        (bool*)&(viewer->data(LayerId::GripperDirection).is_visible));
    ImGui::Checkbox("Center of Mass",
                    (bool*)&(viewer->data(LayerId::CenterOfMass).is_visible));
    ImGui::Checkbox("All Type A Contacts",
                    (bool*)&(viewer->data(LayerId::TypeAContacts).is_visible));
    ImGui::Checkbox("All Type B Contacts",
                    (bool*)&(viewer->data(LayerId::TypeBContacts).is_visible));
    ImGui::Checkbox(
        "Filtered Contacts",
        (bool*)&(viewer->data(LayerId::FilteredContacts).is_visible));
    ImGui::Checkbox("Best Contacts",
                    (bool*)&(viewer->data(LayerId::BestContacts).is_visible));
    ImGui::Checkbox("Gripper",
                    (bool*)&(viewer->data(LayerId::GripperMesh).is_visible));
    ImGui::Checkbox("Contact Ray",
                    (bool*)&(viewer->data(LayerId::ContactRay).is_visible));
    ImGui::InputFloat("Contact Ray Size",
                      &(viewer->data(LayerId::ContactRay).line_width),
                      0.5,
                      2,
                      "%.1f");

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

void MainUI::RunPerformanceTest() {
  // Update using new thread
  auto done = new std::atomic<bool>(false);
  auto task = new std::thread([=] {
    voxelPipeline->RunPerformanceTest(
        voxelSettings, perfSettings[0], perfSettings[1], perfSettings[2]);
    done->store(true);
  });
  tasks.push_back(
      std::move(std::make_pair(std::unique_ptr<std::atomic<bool>>(done),
                               std::unique_ptr<std::thread>(task))));
}

void MainUI::SaveDXF() {
  std::string filename = igl::file_dialog_save();
  voxelPipeline->WriteDXF(filename);
}

void MainUI::SaveResult() {
  std::string filename = igl::file_dialog_save();
  voxelPipeline->WriteResult(filename);
}

void MainUI::SaveGripper() {
  std::string filename = igl::file_dialog_save();
  voxelPipeline->WriteGripper(filename);
}

void MainUI::SaveRAPID() {
  std::string filename = igl::file_dialog_save();
  voxelPipeline->WriteRAPID(filename);
}

void MainUI::UpdateCameraAngle(float angle0, float angle1) {
  viewer->core().trackball_angle =
      Eigen::AngleAxisf(angle1 * DEGREE_TO_RADIAN, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(angle0 * DEGREE_TO_RADIAN, Eigen::Vector3f::UnitY());
}

void MainUI::Render() {
  std::string filename = igl::file_dialog_save();
  utils::CaptureScreen(viewer->core(),
                       viewer->data_list,
                       renderSize[0],
                       renderSize[1],
                       filename);
}

void MainUI::DrawGrabDirection() {
  static Eigen::RowVector3d directionColor = Eigen::RowVector3d(1, 0.5, 0);
  auto& layer = viewer->data(LayerId::GripperDirection);
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
  viewer->data(LayerId::Mesh)
      .uniform_colors((gold * 0.3).transpose(),
                      (Eigen::Vector3d)gold.transpose(),
                      Eigen::Vector3d::Zero());
  return true;
}

}  // namespace gripper