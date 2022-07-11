// The `CheckForceClosureQP`, `CheckPartialClosureQP` and `ComputeMinWrenchQP`
// functions are adapted from the Dex-Net package, which is licensed under:
//   Copyright (c) 2017. The Regents of the University of California (Regents). All Rights Reserved.
//   SPDX-License-Identifier: LicenseRef-UC-Berkeley-Copyright-and-Disclaimer-Notice
//
// Other parts are licensed under:
//   Copyright (c) 2022 The University of Washington and Contributors
//   SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#pragma once

#include <Eigen/Core>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Constants.h"
#include "DiscreteDistanceField.h"
#include "models/ContactPoint.h"

namespace psg {

// Source:
// https://github.com/BerkeleyAutomation/dex-net/blob/master/src/dexnet/grasping/quality.py

/// <summary>
/// Checks force closure by solving a quadratic program
/// (whether or not zero is in the convex hull)
/// </summary>
/// <param name="contact_cones">List of contact point information</param>
/// <param name="center_of_mass">Target object's center of mass</param>
bool CheckForceClosureQP(const std::vector<ContactPoint>& contact_cones,
                         const Eigen::Vector3d& center_of_mass);

/// <summary>
/// Evalutes partial closure: whether or not the contact points can resist a
/// specific wrench. Estimates resistance by solving a quadratic program
/// (whether or not the target wrench is in the convex hull).
/// </summary>
/// <param name="contact_cones">List of contact point information</param>
/// <param name="center_of_mass">Target object's center of mass</param>
/// <param name="ext_force">Force applied on the center of mass</param>
/// <param name="ext_torque">Torque applied on the center of mass</param>
bool CheckPartialClosureQP(const std::vector<ContactPoint>& contact_cones,
                           const Eigen::Vector3d& center_of_mass,
                           const Eigen::Vector3d& ext_force,
                           const Eigen::Vector3d& ext_torque);

/// <summary>
/// Ferrari & Canny's L1 metric. Also known as the epsilon metric.
/// </summary>
/// <param name="contact_cones">List of contact point information</param>
/// <param name="center_of_mass">Target object's center of mass</param>
/// <returns>Ferrari & Canny's L1 metric</returns>
double ComputeMinWrenchQP(const std::vector<ContactPoint>& contact_cones,
                          const Eigen::Vector3d& center_of_mass);

/// <summary>
/// Partial minimum wrench
/// </summary>
/// <param name="contact_cones">List of contact point information</param>
/// <param name="center_of_mass">Target object's center of mass</param>
/// <param name="ext_force">Force applied on the center of mass</param>
/// <param name="ext_torque">Torque applied on the center of mass</param>
/// <returns>Minimum magnitude of additional force and torque that would
/// violate the partial force closure.</returns>
double ComputePartialMinWrenchQP(const std::vector<ContactPoint>& contact_cones,
                                 const Eigen::Vector3d& center_of_mass,
                                 const Eigen::Vector3d& ext_force,
                                 const Eigen::Vector3d& ext_torque);

/// <summary>
/// Check if it is possible to approach or leave object.
///
/// Solve for a translational and rotational velocity of the gripper, so that
/// the velocity of all finger tips align with the direction of the contact
/// point normal. It's possible to leave the object (and also approach the
/// object in reverse direction) if such a solution exists.
/// </summary>
/// <param name="contact_point">List of contact point information</param>
/// <param name="out_trans">If leaving is possible, store the transformation to
/// leave the object (transition and rotation) in this variable</param>
/// <param name="max_angle">Maximum angle against the surface normal that is
/// considered as a valid trajectory</param>
/// <param name="step_size">Step size for gradient descent</param>
/// <param name="threshold">Solution with the loss function less than this
/// threshold is considered as valid solution</param>
/// <param name="max_iterations">Maximum iteration for gradient descent</param>
/// <returns>True iff a valid solution is found</returns>
bool CheckApproachDirection(const std::vector<ContactPoint>& contact_point,
                            Eigen::Affine3d& out_trans,
                            double max_angle = kHeuristicsThetaMax,
                            double step_size = kHeuristicsLR,
                            double threshold = kHeuristicsThreshold,
                            int max_iterations = kHeuristicsMaxIter);

/// <summary>
/// Compute the maximum finger distance from the FFO among all contact points.
/// </summary>
/// <param name="distance_field">Distance field from the FFO</param>
/// <param name="contact_point">List of contact point information</param>
/// <returns>Maximum finger distance (discritized)</returns>
int GetFingerDistance(const DiscreteDistanceField& distance_field,
                      const std::vector<ContactPoint>& contact_points);

/// <summary>
/// Evaluate the trajectory complexity.
/// </summary>
/// <param name="trajectory">Trajectory information</param>
/// <returns>The sum of L1 difference of joint angles at every keyframe
/// interval.</returns>
double GetTrajectoryComplexity(const Trajectory& trajectory);

}  // namespace psg
