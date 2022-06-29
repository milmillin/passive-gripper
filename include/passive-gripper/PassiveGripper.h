#pragma once

#include <Eigen/Core>
#include <functional>
#include <vector>

#include "models/ContactPoint.h"
#include "models/GripperParams.h"
#include "models/GripperSettings.h"
#include "models/MeshDependentResource.h"
#include "robots/Robots.h"
#include "serialization/Serialization.h"

namespace psg {

class PassiveGripper : public psg::serialization::Serializable {
 public:
  enum class InvalidatedReason {
    kMesh,
    kContactPoints,
    kFingers,
    kTrajectory,
    kTopoOptSettings,
    kCost
  };
  typedef std::function<void(InvalidatedReason)> InvalidatedDelegate;

  PassiveGripper();

  inline void RegisterInvalidatedDelegate(const InvalidatedDelegate& d) {
    Invalidated_ = d;
  }
  void ForceInvalidateAll(bool disable_reinit = false);

  // Mesh
  static constexpr int kRemeshVersion = 1;
  void GenerateRemesh();
  void SetMesh(const Eigen::MatrixXd& V,
               const Eigen::MatrixXi& F,
               const Eigen::MatrixXd& remesh_V,
               const Eigen::MatrixXi& remesh_F,
               int remesh_version,
               bool invalidate = true);
  void SetMesh(const Eigen::MatrixXd& V,
               const Eigen::MatrixXi& F,
               bool invalidate = true);
  inline void GetMesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) const {
    V = mdr_.V;
    F = mdr_.F;
  }
  void SetMeshTrans(const Eigen::Affine3d& trans);
  void TransformMesh(const Eigen::Affine3d& trans);

  // Contact Point
  void AddContactPoint(const ContactPoint& contact_point);
  void SetContactPoints(const std::vector<ContactPoint>& contact_points);
  void RemoveContactPoint(size_t index);
  void ClearContactPoint();

  // Trajectory
  void AddKeyframe(const Pose& pose);
  void EditKeyframe(size_t index, const Pose& pose);
  void RemoveKeyframe(size_t index);
  void ClearKeyframe();
  void SetTrajectory(const Trajectory& trajectory);
  inline const Trajectory& GetTrajectory() const { return params_.trajectory; }
  inline Eigen::Affine3d GetFingerTransInv() const {
    return robots::Forward(params_.trajectory.front()).inverse();
  }

  // Settings
  void SetContactSettings(const ContactSettings& settings);
  void SetFingerSettings(const FingerSettings& settings);
  void SetTrajectorySettings(const TrajectorySettings& settings);
  void SetOptSettings(const OptSettings& settings);
  void SetTopoOptSettings(const TopoOptSettings& settings);
  void SetCostSettings(const CostSettings& settings);
  void SetSettings(const GripperSettings& settings, bool invalidate = true);

  // Params
  void SetParams(const GripperParams& params, bool invalidate = true);

  // Initialization
  void InitGripperBound();

  bool reinit_trajectory = true;
  bool reinit_fingers = true;

 private:
  GripperParams params_;
  GripperSettings settings_;
  MeshDependentResource mdr_;
  std::vector<ContactPoint> contact_cones_;
  Eigen::Affine3d mesh_trans_;

  // remesh'd
  MeshDependentResource mdr_remeshed_;

  // mdr with contact floor
  MeshDependentResource mdr_contact_;
  double mdr_contact_floor_ = -1;

  bool mesh_loaded_ = false;

  bool is_force_closure_;
  bool is_partial_closure_;
  double min_wrench_;
  double partial_min_wrench_;
  double cost_;
  double min_dist_;

  GripperParams dCost_dParam_;

  // state dependency
  bool mesh_changed_ = false;
  bool contact_settings_changed_ = false;
  bool finger_settings_changed_ = false;
  bool trajectory_settings_changed_ = false;
  bool opt_settings_changed_ = false;
  bool topo_opt_settings_changed_ = false;
  bool cost_settings_changed_ = false;
  bool contact_changed_ = false;
  bool finger_changed_ = false;
  bool trajectory_changed_ = false;
  bool quality_changed_ = false;
  bool cost_changed_ = false;

  InvalidatedDelegate Invalidated_;

  void Invalidate();
  void InvokeInvalidated(InvalidatedReason reason);

  // To be called by Invalidate()
  void InvalidateMesh();
  void InvalidateContactSettings();
  void InvalidateFingerSettings();
  void InvalidateTrajectorySettings();
  void InvalidateCostSettings();
  void InvalidateContact();
  void InvalidateFinger();
  void InvalidateTrajectory();
  void InvalidateTopoOptSettings();
  void InvalidateQuality();
  void InvalidateCost();

 public:
  DECLARE_GETTER(GetCenterOfMass, mdr_.center_of_mass)
  DECLARE_GETTER(GetMeshV, mdr_.V)
  DECLARE_GETTER(GetMeshF, mdr_.F)
  DECLARE_GETTER(GetMeshTrans, mesh_trans_)
  DECLARE_GETTER(IsMeshLoaded, mesh_loaded_)
  DECLARE_GETTER(GetContactPoints, params_.contact_points)
  DECLARE_GETTER(GetContactCones, contact_cones_)
  DECLARE_GETTER(GetFingers, params_.fingers)
  DECLARE_GETTER(GetContactSettings, settings_.contact)
  DECLARE_GETTER(GetFingerSettings, settings_.finger)
  DECLARE_GETTER(GetTrajectorySettings, settings_.trajectory)
  DECLARE_GETTER(GetOptSettings, settings_.opt)
  DECLARE_GETTER(GetTopoOptSettings, settings_.topo_opt)
  DECLARE_GETTER(GetCostSettings, settings_.cost)
  DECLARE_GETTER(GetIsForceClosure, is_force_closure_)
  DECLARE_GETTER(GetIsPartialClosure, is_partial_closure_)
  DECLARE_GETTER(GetMinWrench, min_wrench_)
  DECLARE_GETTER(GetPartialMinWrench, partial_min_wrench_)
  DECLARE_GETTER(GetCost, cost_)
  DECLARE_GETTER(GetGradient, dCost_dParam_)
  DECLARE_GETTER(GetMinDist, min_dist_)
  DECLARE_GETTER(GetParams, params_)
  DECLARE_GETTER(GetSettings, settings_)
  DECLARE_GETTER(GetMDR, mdr_)
  DECLARE_GETTER(GetFloorMDR, mdr_contact_)
  DECLARE_GETTER(GetRemeshedMDR, mdr_remeshed_)

  DECL_SERIALIZE() {
    constexpr int version = 2;
    SERIALIZE(version);
    SERIALIZE(GetMDR().V);
    SERIALIZE(GetMDR().F);
    SERIALIZE(GetParams());
    SERIALIZE(GetSettings());
    SERIALIZE(kRemeshVersion);
    SERIALIZE(GetRemeshedMDR().V);
    SERIALIZE(GetRemeshedMDR().F);
  }

  DECL_DESERIALIZE() {
    int version;
    int remesh_version;
    DESERIALIZE(version);
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd RV;
    Eigen::MatrixXi RF;
    GripperParams params;
    GripperSettings settings;
    if (version == 1) {
      DESERIALIZE(V);
      DESERIALIZE(F);
      DESERIALIZE(params);
      DESERIALIZE(settings);
      SetMesh(V, F);
      SetSettings(settings);
      SetParams(params);
    } else if (version == 2) {
      DESERIALIZE(V);
      DESERIALIZE(F);
      DESERIALIZE(params);
      DESERIALIZE(settings);
      DESERIALIZE(remesh_version);
      DESERIALIZE(RV);
      DESERIALIZE(RF);
      SetMesh(V, F, RV, RF, remesh_version);
      SetSettings(settings);
      SetParams(params);
    }
  }
};

}  // namespace psg
