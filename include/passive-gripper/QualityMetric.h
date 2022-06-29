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
bool CheckForceClosureQP(const std::vector<ContactPoint>& contactCones,
                         const Eigen::Vector3d& centerOfMass);

// Evalutes partial closure: whether or not the forces and torques
// can resist a specific wrench. Estimates resistance by solving a quadratic
// program (whether or not the target wrench is in the convex hull).
bool CheckPartialClosureQP(const std::vector<ContactPoint>& contactCones,
                           const Eigen::Vector3d& centerOfMass,
                           const Eigen::Vector3d& extForce,
                           const Eigen::Vector3d& extTorque);

// Ferrari & Canny's L1 metric. Also known as the epsilon metric.
double ComputeMinWrenchQP(const std::vector<ContactPoint>& contactCones,
                          const Eigen::Vector3d& centerOfMass);

//
double ComputePartialMinWrenchQP(const std::vector<ContactPoint>& contactCones,
                                 const Eigen::Vector3d& centerOfMass,
                                 const Eigen::Vector3d& extForce,
                                 const Eigen::Vector3d& extTorque);

// Solve for a translational and rotational velocity of the gripper, so that
// the velocity of all finger tips align with the direction of the contact
// point normal. Return true iff a solution is possible.
bool CheckApproachDirection(const std::vector<ContactPoint>& contactPoints,
                            double maxAngle,
                            double maxV,
                            double learningRate,
                            double threshold,
                            int max_iterations,
                            Eigen::Affine3d& out_trans);

bool CheckApproachDirection2(const std::vector<ContactPoint>& contact_points,
                             double away_dist,
                             double max_angle,
                             const Eigen::Vector3d& center_of_mass,
                             Eigen::Affine3d& out_trans);

struct NeighborInfo {
  std::unordered_map<int, std::unordered_set<int>> neighbor;

  NeighborInfo(const Eigen::MatrixXi& F);
  std::vector<int> GetNeighbors(const ContactPoint& contact_point,
                                const Eigen::MatrixXd& V,
                                const Eigen::MatrixXi& F,
                                double tolerance) const;
};

int GetFingerDistance(const DiscreteDistanceField& distanceField,
                      const std::vector<ContactPoint>& contact_points);

double GetTrajectoryComplexity(const Trajectory& trajectory);

}  // namespace psg
