#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <vector>

#include "../Constants.h"

namespace psg {
namespace robots {

Eigen::Affine3d Forward(const Pose& jointConfig);

bool Inverse(Eigen::Affine3d trans, std::vector<Pose>& out_jointConfigs);

bool BestInverse(Eigen::Affine3d trans,
                 Pose base,
                 std::vector<Pose>& out_joint_configs,
                 size_t& best_i);

void ForwardIntermediate(const Pose& jointConfig,
                         std::vector<Eigen::Affine3d>& out_trans);


// Returns a function that returns Jacobian
//  pos: position in effector space
//  out_J: Jacobian [dpos/dtheta_0 | ... | dpos/dtheta_5]
JacobianFunc ComputeJacobian(const Pose& jointConfig);

}  // namespace robots
}  // namespace psg
