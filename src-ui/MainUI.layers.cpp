#include "MainUI.h"

#include <passive-gripper/GeometryUtils.h>

#include "Assets.h"

namespace psg {
namespace ui {

void MainUI::OnLayerInvalidated(Layer layer) {
  switch (layer) {
    case Layer::kMesh:
      OnMeshInvalidated();
      break;
    case Layer::kCenterOfMass:
      OnCenterOfMassInvalidated();
      break;
    case Layer::kContactPoints:
      OnContactPointsInvalidated();
      break;
    case Layer::kFingers:
    case Layer::kInitFingers:
      OnFingersInvalidated(layer);
      break;
    case Layer::kRobot:
      OnRobotInvalidated();
      break;
    case Layer::kInitTrajectory:
    case Layer::kTrajectory:
      OnTrajectoryInvalidated(layer);
      break;
    case Layer::kSweptSurface:
      OnSweptSurfaceInvalidated();
      break;
    case Layer::kGripperBound:
      OnGripperBoundInvalidated();
      break;
    case Layer::kNegVol:
      OnNegVolInvalidated();
      break;
    case Layer::kGripper:
      OnGripperInvalidated();
      break;
    case Layer::kGradient:
      break;
    case Layer::kContactFloor:
      OnContactFloorInvalidated();
      break;
  }
}

void MainUI::OnMeshInvalidated() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  vm_.PSG().GetMesh(V, F);
  auto& meshLayer = GetLayer(Layer::kMesh);
  meshLayer.clear();
  meshLayer.set_mesh(V, F);
  meshLayer.uniform_colors((colors::kGold * 0.3).transpose(),
                           (Eigen::Vector3d)colors::kGold.transpose(),
                           Eigen::Vector3d::Zero());
  auto& remeshLayer = GetLayer(Layer::kRemesh);
  remeshLayer.clear();
  remeshLayer.set_mesh(vm_.PSG().GetRemeshedMDR().V,
                       vm_.PSG().GetRemeshedMDR().F);
  remeshLayer.uniform_colors((colors::kGold * 0.3).transpose(),
                             (Eigen::Vector3d)colors::kGold.transpose(),
                             Eigen::Vector3d::Zero());
}

void MainUI::OnCenterOfMassInvalidated() {
  auto& comLayer = GetLayer(Layer::kCenterOfMass);
  comLayer.clear();
  comLayer.set_points(vm_.PSG().GetCenterOfMass().transpose(),
                      Eigen::RowVector3d(0.7, 0.2, 0));
}

void MainUI::OnContactPointsInvalidated() {
  auto& cpLayer = GetLayer(Layer::kContactPoints);
  cpLayer.clear();

  const auto& contact_points = vm_.PSG().GetContactPoints();
  const auto& contact_cones = vm_.PSG().GetContactCones();
  const auto& contact_settings = vm_.PSG().GetContactSettings();

  size_t nContacts = contact_points.size();
  Eigen::MatrixXd V(nContacts * contact_settings.cone_res * 2, 3);
  Eigen::MatrixXi VE(nContacts * contact_settings.cone_res * 3, 2);
  Eigen::MatrixXd PL(nContacts, 3);
  std::vector<std::string> labels(nContacts);
  size_t tmp;
  size_t tmp2;
  for (size_t i = 0; i < contact_points.size(); i++) {
    PL.row(i) = contact_points[i].position + contact_points[i].normal * 0.01;
    labels[i] = "C" + std::to_string(i);
    for (size_t j = 0; j < contact_settings.cone_res; j++) {
      const auto& cp = contact_cones[i * contact_settings.cone_res + j];
      tmp = i * contact_settings.cone_res * 2 + j * 2;
      tmp2 = i * contact_settings.cone_res * 2 +
             ((j + 1) % contact_settings.cone_res) * 2;
      V.row(tmp) = cp.position + 0.02 * cp.normal;
      V.row(tmp + 1) = cp.position;  // + 0.02 * cp.normal;
      VE.row(i * contact_settings.cone_res * 3 + j * 3) =
          Eigen::RowVector2i(tmp, tmp + 1);
      VE.row(i * contact_settings.cone_res * 3 + j * 3 + 1) =
          Eigen::RowVector2i(tmp, tmp2);
      VE.row(i * contact_settings.cone_res * 3 + j * 3 + 2) =
          Eigen::RowVector2i(tmp + 1, tmp2 + 1);
    }
  }
  cpLayer.set_edges(V, VE, Eigen::RowVector3d(0.2, 0.5, 0.1));
  cpLayer.set_labels(PL, labels);
  cpLayer.line_width = 2;
  cpLayer.show_custom_labels = true;
  cpLayer.show_overlay = true;
}

void MainUI::OnFingersInvalidated(Layer layer) {
  auto& fingerLayer = GetLayer(layer);

  Eigen::RowVector3d color;
  if (layer == Layer::kFingers)
    color = colors::kRed;
  else {
    if (!vm_.GetInitParamValid()) return;
    color = colors::kPurple;
  }

  const auto& params =
      (layer == Layer::kFingers) ? vm_.PSG().GetParams() : vm_.GetInitParam();

  const auto& fingers = params.fingers;
  const auto& finger_settings = vm_.PSG().GetFingerSettings();
  const Pose& first = params.trajectory.front();
  const Pose& current_pose = vm_.GetCurrentPose();

  Eigen::Affine3d finger_trans_inv = robots::Forward(first).inverse();

  size_t nFingers = fingers.size();
  Eigen::MatrixXd V(nFingers * finger_settings.n_finger_joints, 3);
  Eigen::MatrixXi E(nFingers * (finger_settings.n_finger_joints - 1), 2);

  Eigen::Affine3d curTrans = robots::Forward(current_pose) * finger_trans_inv;

  for (size_t i = 0; i < nFingers; i++) {
    V.block(i * finger_settings.n_finger_joints,
            0,
            finger_settings.n_finger_joints,
            3)
        .transpose() =
        curTrans * fingers[i].transpose().colwise().homogeneous();
    for (size_t j = 0; j < finger_settings.n_finger_joints - 1; j++) {
      E(i * (finger_settings.n_finger_joints - 1) + j, 0) =
          i * finger_settings.n_finger_joints + j;
      E(i * (finger_settings.n_finger_joints - 1) + j, 1) =
          i * finger_settings.n_finger_joints + j + 1;
    }
  }

  fingerLayer.set_points(V, Eigen::RowVector3d(0.8, 0.4, 0));
  fingerLayer.set_edges(V, E, color);
  fingerLayer.line_width = 5;
  fingerLayer.point_size = 9;
}

void MainUI::OnRobotInvalidated() {
  static const Eigen::Affine3d globalTrans =
      (Eigen::Affine3d)(Eigen::Matrix3d() << -1, 0, 0, 0, 0, 1, 0, 1, 0)
          .finished();

  auto& robotLayer = GetLayer(Layer::kRobot);
  robotLayer.clear();

  if (robot_viz_) {
    Pose current_pose = vm_.GetCurrentPose();
    Eigen::Affine3d cur_trans;
    cur_trans.setIdentity();

    std::vector<Eigen::Affine3d> trans(6);
    // robots::ForwardIntermediate(vm_.GetCurrentPose(), trans);

    size_t v_count = 0;
    size_t f_count = 0;
    std::vector<Eigen::MatrixXd> VS(7);

    VS[0] = (globalTrans * kAssets[0].first.transpose().colwise().homogeneous())
                .transpose();
    v_count += kAssets[0].first.rows();
    f_count += kAssets[0].second.rows();

    for (size_t i = 0; i < 6; i++) {
      cur_trans = cur_trans * kUrdfTrans[i] *
                  Eigen::AngleAxisd(current_pose(i), kUrdfAxis[i]);
      trans[i] = globalTrans * cur_trans;
      VS[i + 1] =
          (trans[i] * kAssets[i + 1].first.transpose().colwise().homogeneous())
              .transpose();
      v_count += kAssets[i + 1].first.rows();
      f_count += kAssets[i + 1].second.rows();
    }
    Eigen::MatrixXd V(v_count, 3);
    Eigen::MatrixXi F(f_count, 3);
    v_count = 0;
    f_count = 0;
    for (size_t i = 0; i < 7; i++) {
      size_t cur_v = VS[i].rows();
      size_t cur_f = kAssets[i].second.rows();
      V.block(v_count, 0, cur_v, 3) = VS[i];
      F.block(f_count, 0, cur_f, 3) = kAssets[i].second.array() + v_count;
      v_count += cur_v;
      f_count += cur_f;
    }
    robotLayer.show_lines = false;
    robotLayer.set_mesh(V, F);
    robotLayer.set_face_based(false);
  } else {
    std::vector<Eigen::Affine3d> trans;
    robots::ForwardIntermediate(vm_.GetCurrentPose(), trans);

    Eigen::MatrixXd AV(6 * 4, 3);
    Eigen::MatrixXi AE(6 * 3, 2);

    Eigen::MatrixXd V(6 * 8, 3);
    Eigen::MatrixXi F(6 * 12, 3);

    for (size_t i = 0; i < 6; i++) {
      Eigen::Affine3d curTrans = trans[i];
      F.block<12, 3>(i * 12, 0) = cube_F.array() + (8 * i);
      V.block<8, 3>(i * 8, 0).transpose() =
          (curTrans * kLocalTrans[i] * cube_V.transpose());

      AV.block<4, 3>(i * 4, 0).transpose() =
          curTrans * (0.1 * axis_V).transpose();
      AE.block<3, 2>(i * 3, 0) = axis_E.array() + (4 * i);
    }

    robotLayer.show_lines = true;
    robotLayer.line_width = 2;
    robotLayer.set_mesh(V, F);
    robotLayer.set_edges(AV, AE, Eigen::Matrix3d::Identity().replicate<6, 1>());
    robotLayer.set_face_based(true);
  }
  robotLayer.uniform_colors((Eigen::Vector3d)Eigen::Vector3d::Constant(0.1),
                            Eigen::Vector3d::Constant(0.41984),
                            Eigen::Vector3d::Constant(0.5));
}

void MainUI::OnTrajectoryInvalidated(Layer layer) {
  auto& trajectoryLayer = GetLayer(layer);

  Eigen::RowVector3d color;
  if (layer == Layer::kTrajectory)
    color = Eigen::RowVector3d(1, 0.8, 0);
  else {
    if (!vm_.GetInitParamValid()) return;
    color = Eigen::RowVector3d(0, 0.8, 0.8);
  }

  const Trajectory& trajectory = (layer == Layer::kTrajectory)
                                     ? vm_.PSG().GetTrajectory()
                                     : vm_.GetInitParam().trajectory;

  size_t nKeyframe = trajectory.size();
  if (nKeyframe == 0) return;

  Eigen::MatrixXd V(4 * nKeyframe, 3);
  Eigen::MatrixXi E(4 * nKeyframe - 1, 2);
  Eigen::MatrixXd C(4 * nKeyframe - 1, 3);

  for (size_t i = 0; i < nKeyframe; i++) {
    Eigen::Affine3d curTrans = robots::Forward(trajectory[i]);
    V.block<4, 3>(i * 4, 0).transpose() = curTrans * (0.01 * axis_V).transpose();
    E.block<3, 2>(i * 3, 0) = axis_E.array() + (4 * i);
    C.block<3, 3>(i * 3, 0) = Eigen::Matrix3d::Identity();
    if (i + 1 < nKeyframe) {
      E(3 * nKeyframe + i, 0) = i * 4;
      E(3 * nKeyframe + i, 1) = (i + 1) * 4;
      C.row(3 * nKeyframe + i) = color;
    }
  }

  trajectoryLayer.set_edges(V, E, C);
  trajectoryLayer.line_width = 5;
}

void MainUI::OnSweptSurfaceInvalidated() {
  const auto& fingers = vm_.PSG().GetFingers();
  const auto& trajectory = vm_.PSG().GetTrajectory();
  auto& layer = GetLayer(Layer::kSweptSurface);
  layer.clear();
  if (fingers.empty()) return;
  const size_t nFingers = fingers.size();
  static constexpr size_t nTrajectorySteps = 32;
  static constexpr size_t nFingerSteps = 32;
  static constexpr double trajectoryStep = 1. / nTrajectorySteps;
  static constexpr double fingerStep = 1. / nFingerSteps;

  const size_t nKeyframes = trajectory.size();
  const size_t nDOF = trajectory.front().size();
  const size_t nFingerJoints = fingers.front().rows();

  const size_t nFrames = (nKeyframes - 1) * nTrajectorySteps + 1;
  const size_t nEvalsPerFingerPerFrame = (nFingerJoints - 1) * nFingerSteps + 1;
  const size_t nEvals = nFrames * nFingers * nEvalsPerFingerPerFrame;
  Eigen::MatrixXd P(nEvals, 3);
  Eigen::MatrixXi PF(
      (nFrames - 1) * nFingers * (nEvalsPerFingerPerFrame - 1) * 2, 3);
  Eigen::VectorXd Cost(nEvals);

  Eigen::Affine3d finger_trans_inv =
      robots::Forward(trajectory.front()).inverse();

  {
    Eigen::Affine3d curTrans =
        robots::Forward(trajectory.front()) * finger_trans_inv;
    for (size_t i = 0; i < nFingers; i++) {
      const Eigen::MatrixXd& finger = fingers[i];
      Eigen::MatrixXd transformedFinger =
          (curTrans * finger.transpose().colwise().homogeneous()).transpose();
      P.row(i * nEvalsPerFingerPerFrame) = transformedFinger.row(0);
      for (size_t joint = 1; joint < nFingerJoints; joint++) {
        for (size_t kk = 1; kk <= nFingerSteps; kk++) {
          double fingerT = kk * fingerStep;
          Eigen::RowVector3d lerpedFinger =
              transformedFinger.row(joint - 1) * (1. - fingerT) +
              transformedFinger.row(joint) * fingerT;
          P.row(i * nEvalsPerFingerPerFrame + (joint - 1) * nFingerSteps + kk) =
              lerpedFinger;
        }
      }
    }
  }
  for (size_t iKf = 1; iKf < nKeyframes; iKf++) {
    Pose t_lerpedKeyframe;
    for (long long j = 1; j <= nTrajectorySteps; j++) {
      double trajectoryT = j * trajectoryStep;
      for (size_t k = 0; k < kNumDOFs; k++) {
        t_lerpedKeyframe[k] = trajectory[iKf - 1][k] * (1 - trajectoryT) +
                              trajectory[iKf][k] * trajectoryT;
      }
      Eigen::Affine3d curTrans =
          robots::Forward(t_lerpedKeyframe) * finger_trans_inv;
      for (size_t i = 0; i < nFingers; i++) {
        const Eigen::MatrixXd& finger = fingers[i];
        Eigen::MatrixXd transformedFinger =
            (curTrans * finger.transpose().colwise().homogeneous()).transpose();
        P.row(((iKf - 1) * nTrajectorySteps + j) * nFingers *
                  nEvalsPerFingerPerFrame +
              i * nEvalsPerFingerPerFrame) = transformedFinger.row(0);
        for (size_t joint = 1; joint < nFingerJoints; joint++) {
          for (size_t kk = 1; kk <= nFingerSteps; kk++) {
            double fingerT = kk * fingerStep;
            Eigen::RowVector3d lerpedFinger =
                transformedFinger.row(joint - 1) * (1. - fingerT) +
                transformedFinger.row(joint) * fingerT;
            P.row(((iKf - 1) * nTrajectorySteps + j) * nFingers *
                      nEvalsPerFingerPerFrame +
                  i * nEvalsPerFingerPerFrame + (joint - 1) * nFingerSteps +
                  kk) = lerpedFinger;
          }
        }
        for (size_t jj = 1; jj < nEvalsPerFingerPerFrame; jj++) {
          // (nFrames - 1) * nFingers * (nEvalsPerFingerPerFrame - 1) * 2, 2);

          int cur = ((iKf - 1) * nTrajectorySteps + j) * nFingers *
                        nEvalsPerFingerPerFrame +
                    i * nEvalsPerFingerPerFrame + jj;
          int last = ((iKf - 1) * nTrajectorySteps + j - 1) * nFingers *
                         nEvalsPerFingerPerFrame +
                     i * nEvalsPerFingerPerFrame + jj;

          PF.block<2, 3>(((iKf - 1) * nTrajectorySteps + j - 1) * nFingers *
                                 (nEvalsPerFingerPerFrame - 1) * 2 +
                             i * (nEvalsPerFingerPerFrame - 1) * 2 +
                             (jj - 1) * 2,
                         0) = (Eigen::MatrixXi(2, 3) << cur,
                               cur - 1,
                               last - 1,
                               cur,
                               last - 1,
                               last)
                                  .finished();
        }
      }
    }
  }
  for (size_t i = 0; i < nEvals; i++) {
    Cost(i) = EvalAt(
        P.row(i).transpose(), vm_.PSG().GetSettings().cost, vm_.PSG().GetMDR());
  }
  layer.set_mesh(P, PF);
  layer.set_data(Cost);
  layer.set_face_based(true);
  layer.double_sided = true;
}

void MainUI::OnGripperBoundInvalidated() {
  auto& layer = GetLayer(Layer::kGripperBound);
  auto V = cube_V;
  Eigen::Vector3d lb = vm_.PSG().GetTopoOptSettings().lower_bound;
  Eigen::Vector3d ub = vm_.PSG().GetTopoOptSettings().upper_bound;
  V = V.array().rowwise() * (ub - lb).transpose().array();
  V = V.array().rowwise() + lb.transpose().array();
  V = (robots::Forward(vm_.GetCurrentPose()) *
       V.transpose().colwise().homogeneous())
          .transpose();
  layer.set_edges(V, cube_E, colors::kBlue);
  layer.line_width = 2;
}

void MainUI::OnNegVolInvalidated() {
  auto& layer = GetLayer(Layer::kNegVol);
  layer.clear();
  if (vm_.GetNegVolValid()) {
    layer.set_mesh((robots::Forward(vm_.GetCurrentPose()) *
                    vm_.GetNegVolV().transpose().colwise().homogeneous())
                       .transpose(),
                   vm_.GetNegVolF());
  }
}

void MainUI::OnGripperInvalidated() {
  auto& layer = GetLayer(Layer::kGripper);
  layer.clear();
  if (vm_.GetGripperV().size() != 0)
    layer.set_mesh((robots::Forward(vm_.GetCurrentPose()) *
                    vm_.GetGripperV().transpose().colwise().homogeneous())
                       .transpose(),
                   vm_.GetGripperF());
  layer.set_colors(colors::kBrown);
}

void MainUI::OnContactFloorInvalidated() {
  auto& layer = GetLayer(Layer::kContactFloor);
  Eigen::MatrixXd V = plane_V * 3;
  V.col(1).array() += vm_.PSG().GetContactSettings().floor;
  layer.set_mesh(V, plane_F);
  layer.show_lines = false;
  layer.uniform_colors((Eigen::Vector3d)Eigen::Vector3d::Constant(0.5),
                       colors::kBlue.transpose(),
                       colors::kWhite.transpose());
}

}  // namespace ui
}  // namespace psg