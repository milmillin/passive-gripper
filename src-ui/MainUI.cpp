#include "MainUI.h"

#include <iostream>

#include <passive-gripper/GeometryUtils.h>

#include <igl/unproject_onto_mesh.h>
#include "Components.h"
#include <passive-gripper/Initialization.h>
#include <passive-gripper/TopoOpt.h>
#include <igl/voxel_grid.h>
#include <igl/marching_cubes.h>

namespace psg {
namespace ui {

MainUI::MainUI() {}

void MainUI::init(igl::opengl::glfw::Viewer* viewer_) {
  using namespace std::placeholders;
  igl::opengl::glfw::imgui::ImGuiMenu::init(viewer_);

  for (int i = 1; i < (int)Layer::kMax; i++) {
    viewer->data_list.emplace_back();
    viewer->data_list.back().id = i;
  }
  viewer->next_data_id = (int)Layer::kMax;

  viewer->core().orthographic = true;
  viewer->core().is_animating = true;

  vm_.RegisterInvalidatedDelegate(
      std::bind(&MainUI::OnLayerInvalidated, this, _1));

  auto& axisLayer = GetLayer(Layer::kAxis);
  axisLayer.set_edges(axis_V * 0.1, axis_E, Eigen::Matrix3d::Identity());
  axisLayer.line_width = 2;

  GetLayer(Layer::kSweptSurface).show_lines = false;

  auto& floorLayer = GetLayer(Layer::kFloor);
  floorLayer.set_mesh(plane_V * 10, plane_F);
  floorLayer.uniform_colors((Eigen::Vector3d)Eigen::Vector3d::Constant(0.5),
                            Eigen::Vector3d::Constant(0.75),
                            Eigen::Vector3d::Ones());
  floorLayer.show_lines = false;
  floorLayer.set_visible(false, -1);
  GetLayer(Layer::kContactFloor).set_visible(false, -1);
}

inline bool MainUI::pre_draw() {
  if (vm_.GetIsAnimating()) vm_.NextFrame();
  bool res = ImGuiMenu::pre_draw();
  imguizmo_pre_draw();
  return res;
}

inline bool MainUI::post_draw() {
  imguizmo_post_draw();
  return ImGuiMenu::post_draw();
}

bool MainUI::mouse_down(int button, int modifier) {
  if (ImGuiMenu::mouse_down(button, modifier)) return true;

  double x = viewer->current_mouse_x;
  double y = viewer->core().viewport(3) - viewer->current_mouse_y;

  if (modifier & IGL_MOD_CONTROL) {
    if (vm_.PSG().IsMeshLoaded() && selected_tools_ == Tools::kContactPoint) {
      const auto& meshLayer = GetLayer(Layer::kMesh);
      const auto& V = meshLayer.V;
      const auto& F = meshLayer.F;
      const auto& N = meshLayer.F_normals;

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
        ContactPoint cp;
        cp.position = bc(0) * a + bc(1) * b + bc(2) * c;
        cp.normal = N.row(fid);
        cp.fid = fid;
        vm_.PSG().AddContactPoint(cp);
        return true;
      }
    }
  }
  return false;
}

void MainUI::draw_viewer_window() {
  float menu_width = 320.f * menu_scaling();
  ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f),
                                      ImVec2(menu_width, -1.0f));
  bool _viewer_menu_visible = true;
  ImGui::Begin(
      "Toolbar",
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
  if (ImGui::Button("Load Mesh", ImVec2((w - p) / 2, 0))) {
    OnLoadMeshClicked();
  }
  ImGui::SameLine();
  if (ImGui::Button("Save Mesh", ImVec2((w - p) / 2, 0))) {
    OnSaveMeshClicked();
  }
  if (ImGui::Button("Merge mesh", ImVec2(w, 0))) {
    OnMergeMeshClicked();
  }
  ImGui::Checkbox("Millimeter", &is_millimeter_);
  ImGui::Checkbox("Swap YZ", &is_swap_yz_);
  // ImGui::Separator();
  // ImGui::Checkbox("Reinit Trajectory", &vm_.PSG().reinit_trajectory);
  ImGui::Separator();
  if (ImGui::Button("Load PSG", ImVec2((w - p) / 2, 0))) {
    OnLoadPSGClicked();
  }
  ImGui::SameLine();
  if (MyButton("Save PSG", ImVec2((w - p) / 2, 0), !vm_.PSG().IsMeshLoaded())) {
    OnSavePSGClicked();
  }
  if (vm_.PSG().IsMeshLoaded()) {
    if (ImGui::Button("Load Gripper STL", ImVec2((w - p) / 2, 0))) {
      std::string filename = igl::file_dialog_open();
      if (!filename.empty()) {
        vm_.LoadGripper(filename);
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Save Gripper STL", ImVec2((w - p) / 2, 0))) {
      std::string filename = igl::file_dialog_save();
      if (!filename.empty()) {
        vm_.SaveGripper(filename);
      }
    }
    if (ImGui::Button("Load Params", ImVec2(w, 0))) {
      std::string filename = igl::file_dialog_open();
      if (!filename.empty()) {
        GripperParams params;
        params.DeserializeFn(filename);
        vm_.PSG().SetParams(params);
      }
    }
    if (optimizer_.IsRunning() || optimizer_.IsResultAvailable()) {
      DrawOptimizationStatusPanel();
    }
    // DrawCostDebugPanel();
    DrawMetricPanel();
    DrawToolPanel();
    switch (selected_tools_) {
      case Tools::kContactPoint:
        DrawContactPointPanel();
        break;
      case Tools::kMeshPosition:
        DrawGuizmoOptionPanel();
        DrawTransformPanel();
        break;
      case Tools::kRobot:
        DrawGuizmoOptionPanel();
        DrawRobotPanel();
        break;
      case Tools::kOptimization:
        DrawOptimizationPanel();
        break;
      case Tools::kTopoOpt:
        DrawTopoOptPanel();
        break;
      case Tools::kDebug:
        DrawDebugPanel();
        break;
      case Tools::kNone:
        break;
      default:
        break;
    }
  }
  DrawViewPanel();
}

void MainUI::DrawCostDebugPanel() {
  float w = ImGui::GetContentRegionAvailWidth();
  if (ImGui::CollapsingHeader("Cost Debug", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Load dbginfos from file", ImVec2(w, 0))) {
      std::string filename = igl::file_dialog_open();
      if (!filename.empty()) {
        psg::serialization::Deserialize(cost_dbg_infos_, filename);
      }
    }
    if (optimizer_.debug) {
      if (ImGui::Button("Load Debug Info", ImVec2(w, 0))) {
        cost_dbg_infos_ = optimizer_.GetCostDebugInfos();
      }
    }

    ImGui::LabelText("Num Iters: ", "%d", (int)cost_dbg_infos_.size());
    if (ImGui::InputInt("Iter: ", &selected_iter_)) {
      if (selected_iter_ < 0) selected_iter_ = 0;
      if (selected_iter_ > cost_dbg_infos_.size())
        selected_iter_ = cost_dbg_infos_.size() - 1;

      if (selected_iter_ >= 0) {
        const GripperParams& param = cost_dbg_infos_[selected_iter_].param;

        vm_.PSG().SetParams(param);

        Debugger debugger;
        double cost = ComputeCost_SP(param,
                                     param,
                                     vm_.PSG().GetSettings(),
                                     vm_.PSG().GetRemeshedMDR(),
                                     CostContext{&debugger, -1});
        VisualizeDebugger(debugger);
      }
    }
  }
}

void MainUI::DrawMetricPanel() {
  static const char* kFalseTrue[2] = {"False", "True"};
  if (ImGui::CollapsingHeader("Metric", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushID("Metric");
    ImGui::Text("Force Closure: %s", kFalseTrue[vm_.PSG().GetIsForceClosure()]);
    ImGui::Text("Partial Force Closure: %s",
                kFalseTrue[vm_.PSG().GetIsPartialClosure()]);
    ImGui::Text("Min Wrench: %.4e", vm_.PSG().GetMinWrench());
    ImGui::Text("Partial Min Wrench: %.4e", vm_.PSG().GetPartialMinWrench());
    ImGui::Text("Cost: %.4e", vm_.PSG().GetCost());
    ImGui::Text("Min Dist: %.4e", vm_.PSG().GetMinDist());
    ImGui::PopID();
  }
}

void MainUI::DrawToolPanel() {
  float w = ImGui::GetContentRegionAvailWidth();
  if (ImGui::CollapsingHeader("Tools", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushID("Tools");
    DrawToolButton("None", Tools::kNone, w);
    DrawToolButton("Contact Point", Tools::kContactPoint, w);
    DrawToolButton("Mesh Position", Tools::kMeshPosition, w);
    DrawToolButton("Trajectory", Tools::kRobot, w);
    DrawToolButton("Optimization", Tools::kOptimization, w);
    DrawToolButton("Topo Opt", Tools::kTopoOpt, w);
    DrawToolButton("Debug", Tools::kDebug, w);
    ImGui::PopID();
  }
}

void MainUI::DrawContactPointPanel() {
  static char buf_[32];
  float w = ImGui::GetContentRegionAvailWidth();
  if (ImGui::CollapsingHeader("Options", ImGuiTreeNodeFlags_DefaultOpen)) {
    FingerSettings finger_settings = vm_.PSG().GetFingerSettings();
    ContactSettings contact_settings = vm_.PSG().GetContactSettings();
    bool contact_update = false;
    bool finger_update = false;
    contact_update |= ImGui::InputDouble(
        "Friction Coeff", &contact_settings.friction, 0.1, 0.5);
    contact_update |=
        ImGui::InputInt("Cone Resolution", (int*)&contact_settings.cone_res);
    contact_update |= ImGui::InputDouble(
        "Contact Floor", &contact_settings.floor, 0.001, 0.001);
    contact_update |= MyInputDoubleConvert(
        "Max Angle", &contact_settings.max_angle, kRadToDeg, 0.001);
    finger_update |= ImGui::InputInt("Finger Joints",
                                     (int*)&finger_settings.n_finger_joints);
    if (finger_update) vm_.PSG().SetFingerSettings(finger_settings);
    if (contact_update) vm_.PSG().SetContactSettings(contact_settings);
  }
  if (ImGui::CollapsingHeader("Contact Points",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    size_t to_delete = -1;
    const auto& contact_points = vm_.PSG().GetContactPoints();
    for (size_t i = 0; i < contact_points.size(); i++) {
      snprintf(buf_, sizeof(buf_), "Delete C%llu", i);
      if (ImGui::Button(buf_, ImVec2(w, 0))) {
        to_delete = i;
      }
    }
    if (to_delete != -1) {
      vm_.PSG().RemoveContactPoint(to_delete);
    }
    ImGui::Separator();
    if (ImGui::Button("Delete All Contact Points", ImVec2(w, 0))) {
      vm_.PSG().ClearContactPoint();
    }
  }
  if (ImGui::CollapsingHeader("Candidates", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::InputInt("# Candidates", (int*)&cp_num_candidates, 1000);
    ImGui::InputInt("# Seeds", (int*)&cp_num_seeds, 1000);
    if (ImGui::Button("Generate Candidates", ImVec2(w, 0))) {
      auto start_time = std::chrono::high_resolution_clock::now();
      contact_point_candidates_ = InitializeGCs(
          vm_.PSG(), cp_num_candidates, cp_num_seeds);
      auto stop_time = std::chrono::high_resolution_clock::now();
      long long duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(stop_time -
                                                                start_time)
              .count();
      std::cout << "Candidate Generation took " << duration << " ms."
                << std::endl;
      std::cout << "Got " << contact_point_candidates_.size() << " candidates.";
    }
    ImGui::Separator();
    size_t k = std::min((size_t)30, contact_point_candidates_.size());
    ImGui::PushID("CPC");
    for (size_t i = 0; i < k; i++) {
      ImGui::PushID(std::to_string(i).c_str());
      if (ImGui::Button("Select")) {
        const ContactPointMetric& cp = contact_point_candidates_[i];
        // vm_.PSG().reinit_trajectory = false;
        vm_.PSG().SetContactPoints(cp.contact_points);
        std::cout << i << "-th candidate selected" << std::endl;
        // vm_.PSG().ClearKeyframe();

        // std::vector<Pose> candidates;
        // size_t best_i;
        // if (robots::BestInverse(cp.trans * robots::Forward(kInitPose),
        // kInitPose,
        // candidates,
        // best_i)) {
        // vm_.PSG().AddKeyframe(FixAngles(kInitPose, candidates[best_i]));
        // }
      }
      ImGui::SameLine();
      ImGui::Text("(%d) dist: %d mw: %.4e, pmw: %.4e",
                  i,
                  (int)contact_point_candidates_[i].finger_distance,
                  contact_point_candidates_[i].min_wrench,
                  contact_point_candidates_[i].partial_min_wrench);
      ImGui::PopID();
    }
    ImGui::PopID();
    ImGui::Separator();
    if (ImGui::Button("Save Candidates", ImVec2(w, 0))) {
      OnExportContactPointCandidates();
    }
    if (ImGui::Button("Load Candidates", ImVec2(w, 0))) {
      OnLoadContactPointCandidates();
    }
  }
}

void MainUI::DrawTransformPanel() {
  float w = ImGui::GetContentRegionAvailWidth();
  if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Translation");
    MyInputDouble3("Translate", mesh_translate_.data());
    ImGui::Text("Rotation");
    MyInputDouble3Convert(
        "Rotation", mesh_rotation_.data(), kRadToDeg, 1, "%.1f");
    if (ImGui::Button("Apply", ImVec2(w, 0))) {
      Eigen::Affine3d mesh_trans = vm_.PSG().GetMeshTrans();
      Eigen::Affine3d trans =
          mesh_trans * Eigen::Translation3d(mesh_translate_) *
          Eigen::AngleAxisd(mesh_rotation_(1), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(mesh_rotation_(0), Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(mesh_rotation_(2), Eigen::Vector3d::UnitZ()) *
          mesh_trans.inverse();
      vm_.PSG().TransformMesh(trans);
    }
  }
  if (ImGui::CollapsingHeader("Remesh", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Remesh##a", ImVec2(w, 0))) {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      Remesh(vm_.PSG().GetMeshV(), vm_.PSG().GetMeshF(), 1, V, F);
      vm_.SetMesh(V, F);
    }
  }
}

void MainUI::DrawRobotPanel() {
  float w = ImGui::GetContentRegionAvailWidth();
  ImGui::PushID("RobotPanel");
  if (ImGui::CollapsingHeader("Forward Kinematics")) {
    bool updateFK = false;
    Pose p = vm_.GetCurrentPose();
    updateFK |= MyInputDouble3Convert("FK13", &p[0], kRadToDeg, 1, "%.1f");
    updateFK |= MyInputDouble3Convert("FK46", &p[3], kRadToDeg, 1, "%.1f");
    if (updateFK) {
      vm_.SetCurrentPose(p);
    }
  }
  if (ImGui::CollapsingHeader("Inverse Kinematics")) {
    ImGui::PushID("IK");
    bool updateIK = false;
    Eigen::Vector3d eff_pos = vm_.GetEffPosition();
    Eigen::Vector3d eff_ang = vm_.GetEffAngles();
    ImGui::Text("Translation");
    updateIK |= MyInputDouble3("IKPos", eff_pos.data());
    ImGui::Text("Rotation");
    updateIK |=
        MyInputDouble3Convert("IKRot", eff_ang.data(), kRadToDeg, 1, "%.1f");
    if (MyButton("Toggle Pose", ImVec2(w, 0), !vm_.CanTogglePose())) {
      vm_.TogglePose();
    }
    if (updateIK) {
      vm_.SetCurrentPose(eff_pos, eff_ang);
    }
    ImGui::PopID();  // IK
  }
  if (ImGui::CollapsingHeader("Keyframes", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushID("Keyframes");
    size_t to_delete = -1;
    const auto& trajectory = vm_.PSG().GetTrajectory();
    for (size_t i = 0; i < trajectory.size(); i++) {
      ImGui::PushID(std::to_string(i).c_str());

      if (ImGui::Button("X")) {
        to_delete = i;
      }
      ImGui::SameLine();
      if (ImGui::Button("Goto")) {
        vm_.AnimateTo(trajectory[i]);
      }
      ImGui::SameLine();
      if (ImGui::Button(">>")) {
        selected_keyframe_index_ = i;
        vm_.SetCurrentPose(trajectory[i]);
      }
      if (selected_keyframe_index_ == i) {
        ImGui::SameLine();
        ImGui::Text(">> ");
      }
      ImGui::SameLine();
      ImGui::Text("P%llu", i);

      ImGui::PopID();  // i
    }
    if (to_delete != -1) {
      vm_.PSG().RemoveKeyframe(to_delete);
    }
    if (ImGui::Button(">>")) {
      selected_keyframe_index_ = -1;
    }
    if (selected_keyframe_index_ == -1) {
      ImGui::SameLine();
      if (ImGui::Button(">> New Keyframe")) {
        vm_.PSG().reinit_trajectory = false;
        vm_.PSG().AddKeyframe(vm_.GetCurrentPose());
      }
    } else {
      ImGui::SameLine();
      ImGui::Text("New Keyframe");
    }
    ImGui::PopID();  // Keyframes
  }
  ImGui::PopID();  // RobotPanel
}

void MainUI::DrawOptimizationPanel() {
  ImGui::PushID("Optimization");
  float w = ImGui::GetContentRegionAvailWidth();
  if (ImGui::CollapsingHeader("Optimization", ImGuiTreeNodeFlags_DefaultOpen)) {
    OptSettings opt_settings = vm_.PSG().GetOptSettings();
    CostSettings cost_settings = vm_.PSG().GetCostSettings();
    bool opt_update = false;
    bool cost_update = false;
    ImGui::Text("Trajectory Wiggle (deg)");
    opt_update |= MyInputDouble3Convert(
        "traj13", &opt_settings.trajectory_wiggle[0], kRadToDeg, 1, "%.1f");
    opt_update |= MyInputDouble3Convert(
        "traj46", &opt_settings.trajectory_wiggle[3], kRadToDeg, 1, "%.1f");
    opt_update |= ImGui::InputDouble(
        "Finger Wiggle (m)", &opt_settings.finger_wiggle, 0.01);
    opt_update |=
        ImGui::InputDouble("Max Runtime (s)", &opt_settings.max_runtime, 1);
    opt_update |=
        ImGui::InputInt("Max Iters", (int*)&opt_settings.max_iters, 1);
    opt_update |=
        ImGui::InputDouble("Tolerance", &opt_settings.tolerance, 0.0001);

    if (ImGui::BeginCombo("Algorithm",
                          labels::kAlgorithms[opt_settings.algorithm])) {
      for (int i = 0; i < NLOPT_NUM_ALGORITHMS; i++) {
        bool is_selected = (opt_settings.algorithm == i);
        if (ImGui::Selectable(labels::kAlgorithms[i], is_selected)) {
          opt_settings.algorithm = (nlopt_algorithm)i;
          opt_update = true;
        }
        if (is_selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    opt_update |=
        ImGui::InputInt("Population", (int*)&opt_settings.population, 1000);

    cost_update |= ImGui::InputDouble("Floor", &cost_settings.floor, 0.001);
    cost_update |= ImGui::InputInt(
        "Finger Subdivision", (int*)&cost_settings.n_finger_steps, 1);
    cost_update |= ImGui::InputInt(
        "Trajectory Subdivision", (int*)&cost_settings.n_trajectory_steps, 1);
    cost_update |=
        ImGui::InputDouble("Subdivision", &cost_settings.d_subdivision, 1);
    cost_update |=
        ImGui::InputDouble("Linearity", &cost_settings.d_linearity, 1);
    cost_update |= ImGui::InputDouble(
        "Inner Contrib", &cost_settings.inner_dis_contrib, 1);
    cost_update |= ImGui::InputDouble(
        "Geodesic Contrib", &cost_settings.geodesic_contrib, 1);
    cost_update |=
        ImGui::InputDouble("Gripper Energy", &cost_settings.gripper_energy, 1);
    cost_update |=
        ImGui::InputDouble("Traj Energy", &cost_settings.traj_energy, 1);
    ImGui::Checkbox("Debug", &optimizer_.debug);
    if (opt_update) vm_.PSG().SetOptSettings(opt_settings);
    if (cost_update) vm_.PSG().SetCostSettings(cost_settings);
    if (ImGui::Button("Optimize", ImVec2(w, 0))) {
      optimizer_.Optimize(vm_.PSG());
    }
  }
  ImGui::PopID();
}

void MainUI::DrawTopoOptPanel() {
  ImGui::PushID("TopoOpt");
  float w = ImGui::GetContentRegionAvailWidth();
  if (ImGui::CollapsingHeader("Topo Opt", ImGuiTreeNodeFlags_DefaultOpen)) {
    TopoOptSettings settings = vm_.PSG().GetTopoOptSettings();
    bool update = false;
    ImGui::Text("Lower Bound");
    update |= MyInputDouble3("lb", settings.lower_bound.data(), 0.001, "%.3f");
    ImGui::Text("Lower Bound");
    update |= MyInputDouble3("ub", settings.upper_bound.data(), 0.001, "%.3f");
    update |= ImGui::InputDouble(
        "Topo Res (mm)", &settings.topo_res, 0.001, 0.001, "%.3f");
    update |= ImGui::InputDouble(
        "Neg Vol Res (mm)", &settings.neg_vol_res, 0.001, 0.001, "%.3f");
    update |= ImGui::InputDouble("Attachment Size (mm)",
                                 &settings.attachment_size,
                                 0.001,
                                 0.001,
                                 "%.3f");
    update |= ImGui::InputDouble("Contact Pt Size (mm)",
                                 &settings.contact_point_size,
                                 0.001,
                                 0.001,
                                 "%.3f");
    update |= ImGui::InputDouble(
        "Base Thickness (mm)", &settings.base_thickness, 0.001, 0.001, "%.3f");

    if (update) vm_.PSG().SetTopoOptSettings(settings);
    if (ImGui::Button("Init Gripper Bound", ImVec2(w, 0))) {
      vm_.PSG().InitGripperBound();
    }
    if (ImGui::Button("Compute Neg Vol", ImVec2(w, 0))) {
      vm_.ComputeNegativeVolume();
    }
    if (ImGui::Button("Generate Topy Config", ImVec2(w, 0))) {
      std::string filename = igl::file_dialog_save();
      if (!filename.empty()) {
        Debugger debugger;
        vm_.ComputeNegativeVolume();
        GenerateTopyConfig(
            vm_.PSG(), vm_.GetNegVolV(), vm_.GetNegVolF(), filename, &debugger);
        VisualizeDebugger(debugger);
      }
    }
    if (ImGui::Button("Load Result Bin", ImVec2(w, 0))) {
      std::string filename = igl::file_dialog_open();
      if (!filename.empty()) {
        vm_.LoadResultBin(filename);
      }
    }
    if (ImGui::Button("Refine Result", ImVec2(w, 0))) {
      vm_.RefineGripper();
    }
  }
  ImGui::PopID();
}

void MainUI::DrawViewPanel() {
  float w = ImGui::GetContentRegionAvailWidth();
  if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_None)) {
    ImGui::PushID("View");
    if (ImGui::Button("View All", ImVec2(w, 0))) {
      for (int i = 0; i < (int)Layer::kMax; i++) {
        GetLayer((Layer)i).set_visible(true);
      }
    }
    if (ImGui::Button("Visualization", ImVec2(w, 0))) {
      for (int i = 0; i < (int)Layer::kMax; i++) {
        GetLayer((Layer)i).set_visible(false);
      }
      GetLayer(Layer::kRobot).set_visible(true);
      GetLayer(Layer::kMesh).set_visible(true);
      GetLayer(Layer::kGripper).set_visible(true);
    }
    if (ImGui::Checkbox("Robot Viz", &robot_viz_)) {
      OnRobotInvalidated();
    }

    DrawLayerOptions(Layer::kMesh, "Mesh");
    DrawLayerOptions(Layer::kContactPoints, "Contact Points");
    DrawLayerOptions(Layer::kRobot, "Robot");
    DrawLayerOptions(Layer::kCenterOfMass, "Center Of Mass");
    DrawLayerOptions(Layer::kAxis, "Axis");
    DrawLayerOptions(Layer::kFingers, "Fingers");
    DrawLayerOptions(Layer::kTrajectory, "Trajectory");
    DrawLayerOptions(Layer::kSweptSurface, "Swept Finger");
    DrawLayerOptions(Layer::kGripperBound, "Gripper Bound");
    DrawLayerOptions(Layer::kNegVol, "Negative Volume");
    DrawLayerOptions(Layer::kGripper, "Gripper");
    DrawLayerOptions(Layer::kInitFingers, "Init Finger");
    DrawLayerOptions(Layer::kInitTrajectory, "Init Trajectory");
    DrawLayerOptions(Layer::kFloor, "Floor");
    DrawLayerOptions(Layer::kContactFloor, "Contact Floor");
    DrawLayerOptions(Layer::kGradient, "Gradient");
    DrawLayerOptions(Layer::kDebug, "Debug");
    DrawLayerOptions(Layer::kRemesh, "Remesh");
    ImGui::PopID();
  }
}

void MainUI::DrawGuizmoOptionPanel() {
  float w = ImGui::GetContentRegionAvailWidth();
  float p = ImGui::GetStyle().FramePadding.x;
  if (ImGui::CollapsingHeader("Manipulation Mode",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushID("Manipulation");
    if (MyButton("Trans",
                 ImVec2((w - p) / 2, 0),
                 imguizmo_operation == ImGuizmo::TRANSLATE)) {
      imguizmo_operation = ImGuizmo::TRANSLATE;
    }
    ImGui::SameLine(0, p);
    if (MyButton("Rot",
                 ImVec2((w - p) / 2, 0),
                 imguizmo_operation == ImGuizmo::ROTATE)) {
      imguizmo_operation = ImGuizmo::ROTATE;
    }
    ImGui::PopID();
  }
}

void MainUI::DrawOptimizationStatusPanel() {
  float w = ImGui::GetContentRegionAvailWidth();
  float p = ImGui::GetStyle().FramePadding.x;

  ImGui::PushID("OptProg");
  if (ImGui::CollapsingHeader("Optimization Progress",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    bool isRunning = optimizer_.IsRunning();
    auto start = optimizer_.GetStartTime();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    if (isRunning) {
      ImGui::Text("Optimizer Running...");
      ImGui::Text("Elapsed: %llds", duration.count());
    } else {
      ImGui::Text("Optimizer Stopped...");
    }
    ImGui::Text("Current Cost: %.4e", optimizer_.GetCurrentCost());
    if (isRunning) {
      if (ImGui::Button("Pause", ImVec2(w, 0))) {
        optimizer_.Cancel();
      }
    } else {
      if (ImGui::Button("Resume", ImVec2(w, 0))) {
        optimizer_.Resume();
      }
    }
    if (optimizer_.IsResultAvailable()) {
      if (ImGui::Button("Load Result", ImVec2(w, 0))) {
        vm_.PSG().SetParams(optimizer_.GetCurrentParams());
      }
    }
  }
  ImGui::PopID();
}

void MainUI::DrawDebugPanel() {
  float w = ImGui::GetContentRegionAvailWidth();
  float p = ImGui::GetStyle().FramePadding.x;

  ImGui::PushID("Debug");
  if (ImGui::CollapsingHeader("Debug", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Compute Init Params", ImVec2(w, 0))) {
      vm_.ComputeInitParams();
    }
    if (ImGui::Button("Adaptive Subdivide Trajectory", ImVec2(w, 0))) {
      Trajectory trajectory;
      std::vector<std::pair<int, double>> contrib;
      AdaptiveSubdivideTrajectory(vm_.PSG().GetTrajectory(),
                                  vm_.PSG().GetFingers(),
                                  0.001,
                                  trajectory,
                                  contrib);
      std::cerr << "New Trajectory: " << trajectory.size() << std::endl;
      vm_.PSG().SetTrajectory(trajectory);
    }
    if (ImGui::Button("Debug Expand Mesh", ImVec2(w, 0))) {
      Eigen::RowVector3d p_min = vm_.PSG().GetMDR().minimum.array() - 0.004;
      Eigen::RowVector3d p_max = vm_.PSG().GetMDR().maximum.array() + 0.004;
      Eigen::MatrixXd GV;
      Eigen::RowVector3i res;
      int s = ceil((p_max - p_min).maxCoeff() / 0.001) + 8;
      igl::voxel_grid(Eigen::AlignedBox3d(p_min, p_max), s, 4, GV, res);
      Eigen::VectorXd S(GV.rows());
#pragma omp parallel for
      for (long long i = 0; i < GV.rows(); i++) {
        double s_;
        Eigen::RowVector3d c_;
        S(i) = vm_.PSG().GetMDR().ComputeSignedDistance(GV.row(i), c_, s_);
      }
      Eigen::MatrixXd mc_V;
      Eigen::MatrixXi mc_F;
      igl::marching_cubes(S, GV, res(0), res(1), res(2), 0.002, mc_V, mc_F);
      Eigen::MatrixXd RV;
      Eigen::MatrixXi RF;
      if (!Remesh(mc_V, mc_F, 5, RV, RF)) {
        std::cerr << "Remesh failed! Defaulted to using original mesh"
                  << std::endl;
      }
      std::cerr << "Remesh: V: " << RV.rows() << std::endl;
      auto& layer = GetLayer(Layer::kDebug);
      layer.clear();
      layer.set_mesh(RV, RF);
    }
    if (ImGui::Button("Debug SP Cost", ImVec2(w, 0))) {
      Debugger debugger;
      MeshDependentResource mdr;
      ComputeCost_SP(vm_.PSG().GetParams(),
                     vm_.PSG().GetParams(),
                     vm_.PSG().GetSettings(),
                     vm_.PSG().GetRemeshedMDR(),
                     CostContext{&debugger, -1});
      VisualizeDebugger(debugger);
    }
    if (ImGui::Button("Dump Cost Info", ImVec2(w, 0))) {
      std::string filename = igl::file_dialog_save();
      if (!filename.empty()) {
        std::ofstream f(filename);

        for (size_t i = 0; i < cost_dbg_infos_.size(); i++) {
          f << i << '\t' << cost_dbg_infos_[i].cost << '\n';
        }
      }
    }
    if (ImGui::Button("Debug CP Seeds", ImVec2(w, 0))) {
      std::vector<int> FI;
      std::vector<Eigen::Vector3d> X;
      InitializeContactPointSeeds(vm_.PSG(), cp_num_seeds, cp_filter, FI, X);
      Eigen::MatrixXd P(X.size(), 3);
      for (size_t i = 0; i < X.size(); i++) {
        P.row(i) = X[i];
      }
      auto& layer = GetLayer(Layer::kDebug);
      layer.clear();
      layer.set_points(P, colors::kBlue);
      layer.point_size = 5;
    }
    ImGui::InputInt("# Seeds", (int*)&cp_num_seeds, 1000);
    ImGui::InputDouble("Filter Hole", &cp_filter.hole, 0.001);
    ImGui::InputDouble(
        "Filter Curvature Radius", &cp_filter.curvature_radius, 0.001);
    MyInputDoubleConvert("Filter Angle", &cp_filter.angle, kRadToDeg, 1);
    if (ImGui::Button("Dump CSV", ImVec2(w, 0))) {
      std::string filename = igl::file_dialog_save();
      if (!filename.empty()) {
        std::ofstream traj_csv_file(filename);
        const psg::Trajectory& traj = vm_.PSG().GetTrajectory();
        for (int i = traj.size() - 1; i >= 0; i--) {
          for (int j = 0; j < psg::kNumDOFs; j++) {
            traj_csv_file << std::setprecision(15) << traj[i](j) << ",";
          }
          traj_csv_file << std::endl;
        }
      }
    }
  }
  ImGui::PopID();
}

void MainUI::DrawLayerOptions(Layer layer, const char* id) {
  auto& data = GetLayer(layer);
  ImGui::PushID(id);
  ImGui::Checkbox("##V", (bool*)&data.is_visible);
  ImGui::SameLine();
  ImGui::Checkbox("##L", (bool*)&data.show_lines);
  ImGui::SameLine();
  ImGui::Checkbox("##F", (bool*)&data.show_faces);
  ImGui::SameLine();
  bool face_based = data.face_based;
  if (ImGui::Checkbox("##S", &face_based)) {
    data.set_face_based(face_based);
  }
  ImGui::SameLine();
  ImGui::Text(id);
  ImGui::PopID();
}

void MainUI::DrawToolButton(const char* label, Tools thisTool, float width) {
  bool disabled = thisTool == selected_tools_;
  if (MyButton(label, ImVec2(width, 0), disabled)) {
    selected_tools_ = thisTool;
  }
}

void MainUI::VisualizeDebugger(const Debugger& debugger) {
  auto& layer = GetLayer(Layer::kDebug);
  Eigen::MatrixXd P;
  Eigen::MatrixXi E;
  Eigen::MatrixXd C;
  Eigen::MatrixXd V;
  Eigen::MatrixXi VE;
  Eigen::MatrixXd PP;
  Eigen::MatrixXd PC;
  debugger.GetEdges(P, E, C);
  debugger.GetMesh(V, VE);
  debugger.GetPoints(PP, PC);
  layer.clear();
  layer.set_edges(P, E, C);
  layer.show_lines = true;
  layer.line_width = 5;
  layer.set_mesh(V, VE);
  layer.face_based = true;
  layer.set_points(PP, PC);
  layer.point_size = 9;
}

inline bool MainUI::IsGuizmoVisible() {
  return vm_.PSG().IsMeshLoaded() && (selected_tools_ == Tools::kMeshPosition ||
                                      selected_tools_ == Tools::kRobot);
}

Eigen::Affine3d MainUI::GetGuizmoTransform() const {
  if (selected_tools_ == Tools::kMeshPosition) {
    return vm_.PSG().GetMeshTrans();
  } else if (selected_tools_ == Tools::kRobot) {
    if (selected_keyframe_index_ == -1) {
      return robots::Forward(vm_.GetCurrentPose());
    } else {
      return robots::Forward(
          vm_.PSG().GetTrajectory()[selected_keyframe_index_]);
    }
  }
  return Eigen::Affine3d::Identity();
}

void MainUI::SetGuizmoTransform(const Eigen::Affine3d& trans) {
  if (selected_tools_ == Tools::kMeshPosition) {
    vm_.PSG().TransformMesh(trans * vm_.PSG().GetMeshTrans().inverse());
  } else {
    vm_.SetCurrentPose(trans);
    if (selected_keyframe_index_ != -1) {
      vm_.PSG().reinit_trajectory = false;
      vm_.PSG().EditKeyframe(selected_keyframe_index_, vm_.GetCurrentPose());
    }
  }
}

bool MainUI::imguizmo_pre_draw() {
  if (IsGuizmoVisible()) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
    ImGuizmo::BeginFrame();
    ImGui::PopStyleVar();
  }
  return false;
}

bool MainUI::imguizmo_post_draw() {
  if (IsGuizmoVisible()) {
    // Don't draw the Viewer's default menu: draw just the ImGuizmo
    Eigen::Matrix4f view = (viewer->core().view / viewer->core().camera_zoom);
    Eigen::Matrix4f proj = viewer->core().proj;
    if (viewer->core().orthographic) {
      view(2, 3) -= 1000; /* Hack depth */
    }
    // ImGuizmo doesn't like a lot of scaling in view, shift it to T
    const float z = viewer->core().camera_base_zoom;
    const Eigen::Matrix4f S =
        (Eigen::Matrix4f() << z, 0, 0, 0, 0, z, 0, 0, 0, 0, z, 0, 0, 0, 0, 1)
            .finished();
    view = (view * S.inverse()).eval();
    Eigen::Matrix4f imguizmo_T = GetGuizmoTransform().matrix().cast<float>();
    const Eigen::Matrix4f T0 = imguizmo_T;
    imguizmo_T = (S * imguizmo_T).eval();
    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
    ImGuizmo::Manipulate(view.data(),
                         proj.data(),
                         imguizmo_operation,
                         ImGuizmo::LOCAL,
                         imguizmo_T.data(),
                         NULL,
                         NULL);
    // invert scaling that was shifted onto T
    imguizmo_T = (S.inverse() * imguizmo_T).eval();
    const float diff = (imguizmo_T - T0).array().abs().maxCoeff();
    // Only call if actually changed; otherwise, triggers on all mouse events
    if (diff > 2e-6) {
      SetGuizmoTransform(Eigen::Affine3d(imguizmo_T.cast<double>()));
    }
  }
  return false;
}

}  // namespace ui
}  // namespace psg
