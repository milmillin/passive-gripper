// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <vector>

#include "../Constants.h"

namespace psg {
namespace robots {

/// <summary>
/// Compute a transformation matrix given a robot pose.
/// </summary>
/// <param name="pose">Robot pose</param>
/// <returns></returns>
Eigen::Affine3d Forward(const Pose& pose);

/// <summary>
/// Perform inverse kinematics to find robot poses that correspond to a given
/// transformation.
/// </summary>
/// <param name="trans">Transformation</param>
/// <param name="out_poses">List of poses</param>
/// <returns>If an inverse is successful</returns>
bool Inverse(Eigen::Affine3d trans, std::vector<Pose>& out_poses);

/// <summary>
/// Perform inverse kinematics to find robot poses that correspond to a given
/// transformation and select the pose that is closest to the given pose.
/// </summary>
/// <param name="trans">Transformation</param>
/// <param name="base">Pose for comparison</param>
/// <param name="out_poses">List of poses</param>
/// <param name="best_i">Index for the pose closest to base</param>
/// <returns>If an inverse is successful</returns>
bool BestInverse(Eigen::Affine3d trans,
                 Pose base,
                 std::vector<Pose>& out_poses,
                 size_t& best_i);

/// <summary>
/// Perform forward kinematics at a given pose. Return a transformation after
/// each robot joint.
/// </summary>
/// <param name="pose">Robot pose</param>
/// <param name="out_trans">Lise of transformations after each robot
/// joint</param>
void ForwardIntermediate(const Pose& pose,
                         std::vector<Eigen::Affine3d>& out_trans);

/// <summary>
/// Returns a function that given a position returns a Jacobian:
/// [dpos/dtheta_0 | ... | dpos/dtheta_5]
/// </summary>
/// <param name="pose">Robot's pose</param>
/// <returns>The function</returns>
JacobianFunc ComputeJacobian(const Pose& pose);

}  // namespace robots
}  // namespace psg
