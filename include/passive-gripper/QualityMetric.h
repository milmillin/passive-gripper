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

// Checks force closure by solving a quadratic program
// (whether or not zero is in the convex hull)
bool CheckForceClosureQP(const std::vector<ContactPoint>& contact_cones,
                         const Eigen::Vector3d& center_of_mass);

// Evalutes partial closure: whether or not the forces and torques
// can resist a specific wrench. Estimates resistance by solving a quadratic
// program (whether or not the target wrench is in the convex hull).
bool CheckPartialClosureQP(const std::vector<ContactPoint>& contact_cones,
                           const Eigen::Vector3d& center_of_mass,
                           const Eigen::Vector3d& ext_force,
                           const Eigen::Vector3d& ext_torque);

// Ferrari & Canny's L1 metric. Also known as the epsilon metric.
double ComputeMinWrenchQP(const std::vector<ContactPoint>& contact_cones,
                          const Eigen::Vector3d& center_of_mass);

// Minimum magnitude of additional force and torque that would violate the
// partial force closure.
double ComputePartialMinWrenchQP(const std::vector<ContactPoint>& contact_cones,
                                 const Eigen::Vector3d& center_of_mass,
                                 const Eigen::Vector3d& ext_force,
                                 const Eigen::Vector3d& ext_torque);

// Solve for a translational and rotational velocity of the gripper, so that
// the velocity of all finger tips align with the direction of the contact
// point normal. Return true iff a solution is possible.
bool CheckApproachDirection(const std::vector<ContactPoint>& contact_point,
                            Eigen::Affine3d& out_trans,
                            double max_angle = kHeuristicsThetaMax,
                            double lr = kHeuristicsLR,
                            double threshold = kHeuristicsThreshold,
                            int max_iterations = kHeuristicsMaxIter);

// Compute the maximum finger distance from the FFO.
int GetFingerDistance(const DiscreteDistanceField& distance_field,
                      const std::vector<ContactPoint>& contact_points);

// Compute the sum of L1 difference of joint angles at every keyframe interval.
double GetTrajectoryComplexity(const Trajectory& trajectory);

}  // namespace psg
