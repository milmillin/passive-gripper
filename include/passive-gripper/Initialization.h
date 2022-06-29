#pragma once

#include <Eigen/Core>
#include "PassiveGripper.h"
#include "models/ContactPoint.h"
#include "models/ContactPointFilter.h"
#include "models/ContactPointMetric.h"
#include "models/GripperSettings.h"
#include "models/MeshDependentResource.h"

namespace psg {

void InitializeMeshPosition(const Eigen::MatrixXd& V,
                            Eigen::MatrixXd& out_V,
                            Eigen::Affine3d& out_trans);

std::vector<Eigen::MatrixXd> InitializeFingers(
    const std::vector<ContactPoint>& contact_points,
    const MeshDependentResource& mdr,
    const Eigen::Vector3d& effector_pos,
    size_t num_finger_joints);

Trajectory InitializeTrajectory(const std::vector<Eigen::MatrixXd>& fingers,
                                const Pose& init_pose,
                                size_t n_keyframes);

void InitializeContactPointSeeds(const PassiveGripper& psg,
                                 size_t num_seeds,
                                 const ContactPointFilter& filter,
                                 std::vector<int>& out_FI,
                                 std::vector<Eigen::Vector3d>& out_X);

std::vector<ContactPointMetric> InitializeContactPoints(
    const PassiveGripper& psg,
    const ContactPointFilter& filter,
    size_t num_candidates,
    size_t num_seeds);

void InitializeGripperBound(const PassiveGripper& psg,
                            Eigen::Vector3d& out_lb,
                            Eigen::Vector3d& out_ub);

void InitializeConservativeBound(const PassiveGripper& psg,
                                 Eigen::Vector3d& out_lb,
                                 Eigen::Vector3d& out_ub);

}  // namespace psg