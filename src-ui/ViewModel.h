#pragma once

#include <passive-gripper/Constants.h>
#include <passive-gripper/PassiveGripper.h>

#include "Layer.h"

namespace psg {
namespace ui {

class ViewModel {
 public:
  typedef std::function<void(Layer)> LayerInvalidatedDelegate;

  ViewModel();

  inline void RegisterInvalidatedDelegate(const LayerInvalidatedDelegate& d) {
    LayerInvalidated_ = d;
    PoseChanged();
  }

  inline PassiveGripper& PSG() { return psg_; }

  void SetMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
  void SetCurrentPose(const Pose& pose);
  void SetCurrentPose(const Eigen::Affine3d& trans);
  void SetCurrentPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& ang);
  void TogglePose();
  inline bool CanTogglePose() const { return ik_sols_index_ != -1; }

  void AnimateTo(const Pose& pose);
  void NextFrame();

  void ComputeNegativeVolume();
  void LoadResultBin(const std::string& filename);
  void RefineGripper();
  bool LoadGripper(const std::string& filename);
  bool SaveGripper(const std::string& filename);

  bool ComputeInitParams();

 private:
  LayerInvalidatedDelegate LayerInvalidated_;
  void InvokeLayerInvalidated(Layer layer);

  PassiveGripper psg_;
  void OnPsgInvalidated(PassiveGripper::InvalidatedReason reason);

  Pose current_pose_;

  std::vector<Pose> ik_sols_;
  size_t ik_sols_index_;
  Eigen::Vector3d eff_position_;
  Eigen::Vector3d eff_angles_;

  void ComputeIK();
  void PoseChanged();

  static const size_t kAnimationSteps = 30;
  bool is_animating_ = false;
  Pose src_pose_;
  Pose dst_pose_;
  size_t cur_step_ = 0;

  bool is_neg_valid_ = false;
  Eigen::MatrixXd neg_V_;
  Eigen::MatrixXi neg_F_;
  Eigen::MatrixXd gripper_V_;
  Eigen::MatrixXi gripper_F_;

  bool is_init_params_valid_ = false;
  GripperParams init_params_;

 public:
  DECLARE_GETTER(PSG, psg_)
  DECLARE_GETTER(GetEffPosition, eff_position_)
  DECLARE_GETTER(GetEffAngles, eff_angles_)
  DECLARE_GETTER(GetCurrentPose, current_pose_)
  DECLARE_GETTER(GetIsAnimating, is_animating_)
  DECLARE_GETTER(GetNegVolV, neg_V_)
  DECLARE_GETTER(GetNegVolF, neg_F_)
  DECLARE_GETTER(GetGripperV, gripper_V_)
  DECLARE_GETTER(GetGripperF, gripper_F_)
  DECLARE_GETTER(GetNegVolValid, is_neg_valid_)
  DECLARE_GETTER(GetInitParamValid, is_init_params_valid_)
  DECLARE_GETTER(GetInitParam, init_params_)
};

}  // namespace ui
}  // namespace psg