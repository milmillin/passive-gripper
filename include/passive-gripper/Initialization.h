// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#pragma once

#include <Eigen/Core>
#include "PassiveGripper.h"
#include "models/ContactPoint.h"
#include "models/ContactPointMetric.h"
#include "models/GripperSettings.h"
#include "models/MeshDependentResource.h"

namespace psg {

/// <summary>
/// Move the mesh to be in front of the FFO in the initial robot pose (when the
/// gripper touches the object). The mesh center is aligned with the FFO and
/// touches the ground.
/// </summary>
/// <param name="V">Input mesh vertices</param>
/// <param name="out_V">Output mesh vertices</param>
/// <param name="out_trans">Transformation on the input to get the
/// output</param>
void InitializeMeshPosition(const Eigen::MatrixXd& V,
                            Eigen::MatrixXd& out_V,
                            Eigen::Affine3d& out_trans);

/// <summary>
/// Generate fingers given the contact points.
/// </summary>
/// <param name="contact_points">Contact points</param>
/// <param name="mdr">Mesh dependent resources</param>
/// <param name="effector_pos">FFO position</param>
/// <param name="num_finger_joints">Desired number of joints per fingers</param>
/// <returns></returns>
std::vector<Eigen::MatrixXd> InitializeFingers(
    const std::vector<ContactPoint>& contact_points,
    const MeshDependentResource& mdr,
    const Eigen::Vector3d& effector_pos,
    size_t num_finger_joints);

/// <summary>
/// Initialize the insert trajectory. The insert trajectory has 4 keyframes
/// initialized to be a straight line calculated from the average direction of
/// the fingers. The first and the second keyframes are the same, so do the
/// third and the forth. Note that the trajectory is backward.
/// </summary>
/// <param name="fingers">Joint positions of the fingers</param>
/// <param name="init_pose">Initial robot pose (when gripper touches the
/// object)</param> <returns></returns>
Trajectory InitializeTrajectory(const std::vector<Eigen::MatrixXd>& fingers,
                                const Pose& init_pose);

/// <summary>
/// Generate potential contact points.
/// </summary>
/// <param name="psg">Passive gripper object</param>
/// <param name="num_seeds">Number of points to be generated</param>
/// <param name="out_FI">Face indices of the generated points</param>
/// <param name="out_X">Positions of the generated points</param>
void InitializeContactPointSeeds(const PassiveGripper& psg,
                                 size_t num_seeds,
                                 std::vector<int>& out_FI,
                                 std::vector<Eigen::Vector3d>& out_X);

/// <summary>
/// Generate a ranked list of GC candidates.
/// </summary>
/// <param name="psg">Passive gripper object</param>
/// <param name="num_candidates">Number of GCs to be generated</param>
/// <param name="num_seeds">Number of potential contact points to be
/// generated</param>
/// <returns>A ranked list of GCs and metrics</returns>
std::vector<ContactPointMetric> InitializeGCs(const PassiveGripper& psg,
                                              size_t num_candidates,
                                              size_t num_seeds);

/// <summary>
/// Compute the bounding box for topology optimization. The bound contains the
/// skeleton and the FFO with some paddings.
/// </summary>
/// <param name="psg">Passive gripper object</param>
/// <param name="out_lb">Lower bound of the box</param>
/// <param name="out_ub">Upper bound of the box</param>
void InitializeGripperBound(const PassiveGripper& psg,
                            Eigen::Vector3d& out_lb,
                            Eigen::Vector3d& out_ub);

/// <summary>
/// Compute the bounding box for topology optimization. The bound contains the
/// mesh and the FFO with some paddings.
/// </summary>
/// <param name="psg">Passive gripper object</param>
/// <param name="out_lb">Lower bound of the box</param>
/// <param name="out_ub">Upper bound of the box</param>
void InitializeConservativeBound(const PassiveGripper& psg,
                                 Eigen::Vector3d& out_lb,
                                 Eigen::Vector3d& out_ub);

}  // namespace psg