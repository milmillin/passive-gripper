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

/// <summary>
/// PassiveGripper contains the mesh, current parameters, and settings.
/// </summary>
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

  // === Mesh ===
  static constexpr int kRemeshVersion = 1;
  void GenerateRemesh();
  void SetMesh(const Eigen::MatrixXd& V,
               const Eigen::MatrixXi& F,
               const Eigen::MatrixXd& remesh_V,
               const Eigen::MatrixXi& remesh_F,
               int remesh_version,
               bool invalidate = true);

  /// <summary>
  /// Set the input mesh.
  ///
  /// This will invalidate the skeleton and trajectory unless disabled
  /// (see <see cref="reinit_trajectory" /> and <see cref="reinit_fingers" />).
  /// </summary>
  /// <param name="V">Input mesh vertices</param>
  /// <param name="F">Input mesh faces</param>
  /// <param name="invalidate">If false, defer the invalidation.</param>
  void SetMesh(const Eigen::MatrixXd& V,
               const Eigen::MatrixXi& F,
               bool invalidate = true);

  inline void GetMesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) const {
    V = mdr_.V;
    F = mdr_.F;
  }
  void SetMeshTrans(const Eigen::Affine3d& trans);
  void TransformMesh(const Eigen::Affine3d& trans);

  // === Contact Point ===

  /// <summary>
  /// Add a contact point.
  /// This will invalidate the skeleton and trajectory unless disabled.
  /// </summary>
  /// <param name="contact_point">Contact point to be added</param>
  void AddContactPoint(const ContactPoint& contact_point);

  /// <summary>
  /// Set the contact points (aka GC).
  /// This will invalidate the skeleton and trajectory unless disabled.
  /// </summary>
  /// <param name="contact_points">A list of contact points to be set</param>
  void SetContactPoints(const std::vector<ContactPoint>& contact_points);

  /// <summary>
  /// Remove a contact point given the index.
  /// This will invalidate the skeleton and trajectory unless disabled.
  /// </summary>
  /// <param name="index">The index of the contact point to be removed</param>
  void RemoveContactPoint(size_t index);

  /// <summary>
  /// Remove all the contact points
  /// This will invalidate the skeleton and trajectory unless disabled.
  /// </summary>
  void ClearContactPoint();

  // === Trajectory ===

  /// <summary>
  /// Add a keyframe to the end of the trajectory.
  /// Note that the trajectory is reversed (i.e., the first keyframe is when the
  /// gripper touches the object).
  /// </summary>
  /// <param name="pose">The keyframe to be added</param>
  void AddKeyframe(const Pose& pose);

  /// <summary>
  /// Edit the indexed keyframe
  /// </summary>
  /// <param name="index">Index of the keyframe to be edited</param>
  /// <param name="pose">New keyframe value</param>
  void EditKeyframe(size_t index, const Pose& pose);

  /// <summary>
  /// Remove the indexed keyframe
  /// </summary>
  /// <param name="index">The index of the keyframe to be removed</param>
  void RemoveKeyframe(size_t index);

  /// <summary>
  /// Remove all the keyframes except the first keyframe (when the gripper
  /// touches the object).
  /// </summary>
  void ClearKeyframe();

  /// <summary>
  /// Set the trajectory.
  /// </summary>
  /// <param name="trajectory">A list of new keyframes</param>
  void SetTrajectory(const Trajectory& trajectory);

  inline const Trajectory& GetTrajectory() const { return params_.trajectory; }
  inline Eigen::Affine3d GetFingerTransInv() const {
    return robots::Forward(params_.trajectory.front()).inverse();
  }

  // === Settings ===

  void SetContactSettings(const ContactSettings& settings);
  void SetFingerSettings(const FingerSettings& settings);
  void SetTrajectorySettings(const TrajectorySettings& settings);
  void SetOptSettings(const OptSettings& settings);
  void SetTopoOptSettings(const TopoOptSettings& settings);
  void SetCostSettings(const CostSettings& settings);
  void SetSettings(const GripperSettings& settings, bool invalidate = true);

  // === Params ===

  /// <summary>
  /// Set the parameters (skeleton and trajectory).
  /// </summary>
  /// <param name="params">New parameters</param>
  /// <param name="invalidate">If false, defer the invalidation.</param>
  void SetParams(const GripperParams& params, bool invalidate = true);

  // === Initialization ===

  /// <summary>
  /// Initialize bounding box for topology optimization.
  /// </summary>
  void InitGripperBound();

  // whether to reinitialize the trajectory when settings changed
  bool reinit_trajectory = true;
  // whether to reinitialize the fingers when settings changed.
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
