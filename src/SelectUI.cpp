#include "SelectUI.h"

#include <igl/png/writePNG.h>
#include <igl/unproject_onto_mesh.h>
#include <imgui.h>
#include <iostream>
#include "Geometry.h"
#include "Utils.h"

namespace gripper {

SelectUI::SelectUI()
    : igl::opengl::glfw::imgui::ImGuiMenu(), meshLoaded(false) {}

SelectUI::~SelectUI() {
  while (!tasks.empty()) {
    tasks.front().second->join();
    tasks.pop_front();
  }
}

void SelectUI::init(igl::opengl::glfw::Viewer* _viewer) {
  igl::opengl::glfw::imgui::ImGuiMenu::init(_viewer);

  viewer->data_list[0].point_size = 10;
  // Add other layers
  for (int layerID = 1; layerID < (int)LayerId::Max; layerID++) {
    viewer->data_list.emplace_back();
    viewer->data_list.back().id = layerID;
    viewer->data_list.back().point_size = 10;
  }
  viewer->next_data_id = (int)LayerId::Max;

  viewer->core().orthographic = true;
}

void SelectUI::draw_viewer_window() {
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

static char buf[128];
void SelectUI::draw_viewer_menu() {
  float w = ImGui::GetContentRegionAvailWidth();
  float p = ImGui::GetStyle().FramePadding.x;
  if (ImGui::CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Load##Mesh", ImVec2((w - p) / 2.f, 0))) {
      meshLoaded = false;
      viewer->selected_data_index = (int)LayerId::Mesh;
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
    if (ImGui::CollapsingHeader("Contact Points",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      size_t toDelete = -1;
      for (size_t i = 0; i < m_contactPoints.size(); i++) {
        const auto& cp = m_contactPoints[i];
        snprintf(buf,
                 sizeof(buf),
                 "Delete (%.4lf,%.4lf,%.4lf)",
                 cp.position(0),
                 cp.position(1),
                 cp.position(2));
        if (ImGui::Button(buf, ImVec2(w - p, 0))) {
          toDelete = i;
        }
      }
      if (toDelete != -1) {
        m_contactPoints.erase(m_contactPoints.begin() + toDelete);
        InvalidateContactPoints();
      }
    }
    if (ImGui::CollapsingHeader("Options", ImGuiTreeNodeFlags_DefaultOpen)) {
      bool update = false;
      update |= ImGui::InputDouble("Friction Coeff", &m_friction, 0.1, 0.5);
      update |= ImGui::InputInt("Cone Resolution", (int*)&m_coneRes);
      if (update) InvalidateContactPoints();
      ImGui::InputInt("Angle subdivision", &m_angleRes);
    }
    if (ImGui::CollapsingHeader("Actions", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Check Feasibility", ImVec2(w - p, 0))) {
        CheckFeasibility();
      }
      if (ImGui::Button("Check Min Tipping Action", ImVec2(w - p, 0))) {
        CheckMinForce();
      }
      if (ImGui::Button("Clear All Contact Points", ImVec2(w - p, 0))) {
        m_contactPoints.clear();
        InvalidateContactPoints();
      }
    }
    if (ImGui::CollapsingHeader("Feasible Result",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Text("Status: %s", m_isFeasibleValid ? "Valid" : "Not Valid");
      if (m_isFeasibleValid) {
        size_t startCol = m_contactPoints.size() * m_coneRes;
        ImGui::Text("Feasible: %s", m_isFeasible ? "Yes" : "No");
        ImGui::Text("%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
                    m_coeffs(startCol),
                    m_coeffs(startCol + 1),
                    m_coeffs(startCol + 2),
                    m_coeffs(startCol + 3),
                    m_coeffs(startCol + 4),
                    m_coeffs(startCol + 5));
      }
    }
  }
  if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushID("View");
    float pointSize = viewer->data(0).point_size;
    if (ImGui::InputFloat("Point Size", &pointSize, 1, 2, "%.1f")) {
      for (int i = 0; i < (int)LayerId::Max; i++) {
        viewer->data(i).point_size = pointSize;
      }
    }

    bool showLines = viewer->data(0).show_lines;
    if (ImGui::Checkbox("Show lines", &showLines)) {
      for (int i = 0; i < (int)LayerId::Max; i++) {
        viewer->data(i).show_lines = showLines;
      }
    }

    ImGui::Checkbox("Mesh",
                    (bool*)&(viewer->data((int)LayerId::Mesh).is_visible));
    ImGui::Checkbox(
        "Contact Points",
        (bool*)&(viewer->data((int)LayerId::ContactPoints).is_visible));
    ImGui::Checkbox(
        "Center of Mass",
        (bool*)&(viewer->data((int)LayerId::CenterOfMass).is_visible));
    ImGui::Checkbox("Feasible Force",
                    (bool*)&(viewer->data((int)LayerId::Feasible).is_visible));
    ImGui::Checkbox("Min Forces",
                    (bool*)&(viewer->data((int)LayerId::MinForces).is_visible));
    ImGui::PopID();
  }
}

bool SelectUI::pre_draw() {
  while (!tasks.empty() && tasks.front().first->load()) {
    tasks.front().second->join();
    tasks.pop_front();
  }
  viewerDataMutex.lock();
  ImGuiMenu::pre_draw();
  return false;
}

bool SelectUI::post_draw() {
  ImGuiMenu::post_draw();
  viewerDataMutex.unlock();
  return false;
}

bool SelectUI::mouse_down(int button, int modifier) {
  if (ImGuiMenu::mouse_up(button, modifier)) return true;

  m_mouse_x = viewer->current_mouse_x;
  m_mouse_y = viewer->core().viewport(3) - viewer->current_mouse_y;

  return false;
}

bool SelectUI::mouse_up(int button, int modifier) {
  if (ImGuiMenu::mouse_up(button, modifier)) return true;

  double x = viewer->current_mouse_x;
  double y = viewer->core().viewport(3) - viewer->current_mouse_y;

  if (abs(x - m_mouse_x) > 10 || abs(y - m_mouse_y)) return false;

  const auto& V = GetMeshVertices();
  const auto& F = GetMeshFaces();
  const auto& N = viewer->data((int)LayerId::Mesh).F_normals;

  if (meshLoaded) {
    int fid;
    Eigen::Vector3f bc;
    if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y),
                                 viewer->core().view,
                                 viewer->core().proj,
                                 viewer->core().viewport,
                                 V,
                                 F,
                                 fid,
                                 bc)) {
      Eigen::Vector3d a = V.row(F(fid, 0));
      Eigen::Vector3d b = V.row(F(fid, 1));
      Eigen::Vector3d c = V.row(F(fid, 2));
      m_contactPoints.push_back(
          ContactPoint{bc(0) * a + bc(1) * b + bc(2) * c, -N.row(fid), false});
      InvalidateContactPoints();
      return true;
    }
  }
  return false;
}

void SelectUI::InvalidateContactPoints() {
  // Generate cones
  size_t nContacts = m_contactPoints.size();
  m_contactCones.resize(nContacts * m_coneRes);
  Eigen::Vector3d B;
  Eigen::Vector3d T;
  double stepSize = EIGEN_PI * 2 / m_coneRes;
  double curStep;
  for (size_t i = 0; i < nContacts; i++) {
    const auto& cp = m_contactPoints[i];
    GetPerp(cp.normal, B, T);
    B *= m_friction;
    T *= m_friction;
    for (size_t j = 0; j < m_coneRes; j++) {
      curStep = j * stepSize;
      m_contactCones[i * m_coneRes + j] =
          ContactPoint{cp.position,
                       cp.normal + B * cos(curStep) + T * sin(curStep),
                       cp.isTypeA};
    }
  }

  auto& cpLayer = viewer->data((int)LayerId::ContactPoints);
  cpLayer.clear();

  Eigen::MatrixXd P(nContacts, 3);
  Eigen::MatrixXd V(nContacts * m_coneRes * 2, 3);
  Eigen::MatrixXi VE(nContacts * m_coneRes * 3, 2);
  size_t tmp;
  size_t tmp2;
  for (size_t i = 0; i < m_contactPoints.size(); i++) {
    P.row(i) = m_contactPoints[i].position;
    for (size_t j = 0; j < m_coneRes; j++) {
      const auto& cp = m_contactCones[i * m_coneRes + j];
      tmp = i * m_coneRes * 2 + j * 2;
      tmp2 = i * m_coneRes * 2 + ((j + 1) % m_coneRes) * 2;
      V.row(tmp) = cp.position - 0.02 * cp.normal;
      V.row(tmp + 1) = cp.position + 0.02 * cp.normal;
      VE.row(i * m_coneRes * 3 + j * 3) = Eigen::RowVector2i(tmp, tmp + 1);
      VE.row(i * m_coneRes * 3 + j * 3 + 1) = Eigen::RowVector2i(tmp, tmp2);
      VE.row(i * m_coneRes * 3 + j * 3 + 2) =
          Eigen::RowVector2i(tmp + 1, tmp2 + 1);
    }
  }
  cpLayer.set_points(P, Eigen::RowVector3d(0.7, 0, 0.2));
  cpLayer.set_edges(V, VE, Eigen::RowVector3d(0.2, 0.5, 0.1));
  cpLayer.line_width = 2;

  m_isFeasibleValid = false;
  InvalidateFeasible();
}

void SelectUI::InvalidateMesh() {
  auto& cmLayer = viewer->data((int)LayerId::CenterOfMass);
  cmLayer.clear();
  cmLayer.set_points(m_centerOfMass.transpose(),
                     Eigen::RowVector3d(0.7, 0.2, 0));
  m_isFeasibleValid = false;
  InvalidateFeasible();
}

void SelectUI::InvalidateFeasible() {
  auto& feLayer = viewer->data((int)LayerId::Feasible);
  feLayer.clear();

  if (!m_isFeasibleValid) return;

  size_t nContacts = m_contactPoints.size();

  Eigen::MatrixXd V(nContacts * 2, 3);
  Eigen::MatrixXi VE(nContacts, 2);

  for (size_t i = 0; i < nContacts; i++) {
    const auto& cp = m_contactPoints[i];
    Eigen::Vector3d f;
    f.setZero();
    for (size_t j = 0; j < m_coneRes; j++) {
      const auto& cp_cone = m_contactCones[i * m_coneRes + j];
      f += m_coeffs(i * m_coneRes + j) * cp_cone.normal;
    }
    V.row(i * 2) = cp.position + 0.02 * f;
    V.row(i * 2 + 1) = cp.position - 0.02 * f;
    VE.row(i) = Eigen::RowVector2i(i * 2, i * 2 + 1);
  }

  feLayer.set_edges(V, VE, Eigen::RowVector3d(0.4, 0.8, 0.1));
  feLayer.line_width = 2;
}

void SelectUI::InvalidateMinForces() {
  const static float scale = 0.2;
  auto& mfLayer = viewer->data((int)LayerId::MinForces);
  mfLayer.clear();

  size_t n = m_directions.size();
  Eigen::MatrixXd V(n * (n + 1), 3);
  Eigen::MatrixXi VE(n * n, 2);
  for (size_t i = 0; i < n; i++) {
    Eigen::Vector3d origin = m_centerOfMass + 0.5 * m_directions[i];
    V.row(i * (n + 1) + n) = origin;
    for (size_t j = 0; j < n; j++) {
      V.row(i * (n + 1) + j) =
          origin + (0.1 * m_minForces[i * n + j]) * m_directions[j];
      VE(i * n + j, 0) = i * (n + 1) + j;
      VE(i * n + j, 1) = i * (n + 1) + n;
    }
  }
  /*
  V.row(m_angleRes) = origin;
  double step = 2. * EIGEN_PI / m_angleRes;
  for (int i = 0; i < m_angleRes; i++) {
    V.row(i) = origin + (0.5 * m_minForces[i]) * Eigen::Vector3d(cos(step * i), 0, sin(step * i));
    VE(i, 0) = i;
    VE(i, 1) = m_angleRes;
  }
  */

  mfLayer.set_edges(V, VE, Eigen::RowVector3d(0.6, 0.3, 0.1));
  mfLayer.line_width = 2;
}

static const Eigen::Vector3d c_gravity(0, -1, 0);

void SelectUI::CheckFeasibility() {
  m_isFeasible = CheckForceClosure(m_contactCones,
                                   m_centerOfMass,
                                   c_gravity,
                                   Eigen::Vector3d::Zero(),
                                   m_coeffs);
  m_isFeasibleValid = true;
  InvalidateFeasible();
}

void SelectUI::CheckMinForce() {
  m_directions = ComputeDirections(m_angleRes);
  size_t n = m_directions.size();
  m_minForces.resize(n * n);
  Eigen::MatrixXd tmp;
  for (size_t i = 0; i < n; i++) {
    const auto& offset = m_directions[i];
    for (size_t j = 0; j < n; j++) {
      const auto& force = m_directions[j];
      // binary search to find minimum force
      double lo = 0;
      double hi = 4;
      double mid;
      bool works;
      // assume the force is applied horizontally 1 unit above the CM
      Eigen::Vector3d torque = offset.cross(force);
      while (hi - lo > 1e-4) {
        mid = (lo + hi) / 2.;
        works = CheckForceClosure(m_contactCones,
                                  m_centerOfMass,
                                  c_gravity + force * mid,
                                  torque * mid,
                                  tmp);
        if (works)
          lo = mid;
        else
          hi = mid;
      }
      m_minForces[i * n + j] = lo;
    }
  }
  InvalidateMinForces();
}

bool SelectUI::post_load() {
  meshLoaded = true;
  meshInfo = MeshInfo(GetMeshVertices(), GetMeshFaces());

  // FIXME: Average COM for now
  m_centerOfMass = (meshInfo.minimum + meshInfo.maximum) / 2;
  InvalidateMesh();

  viewer->data((int)LayerId::Mesh)
      .uniform_colors((gold * 0.3).transpose(),
                      (Eigen::Vector3d)gold.transpose(),
                      Eigen::Vector3d::Zero());
  return true;
}

}  // namespace gripper
