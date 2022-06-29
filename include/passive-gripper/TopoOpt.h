#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include "PassiveGripper.h"

namespace psg {

std::vector<Eigen::Vector3i> GetForbiddenVoxels(const Eigen::MatrixXd& V,
                                                const Eigen::MatrixXi& F,
                                                const Eigen::Vector3d& lb,
                                                const Eigen::Vector3d& ub,
                                                double res,
                                                Eigen::Vector3i& out_range);

void GenerateTopyConfig(const PassiveGripper& psg,
                        const Eigen::MatrixXd& neg_V,
                        const Eigen::MatrixXi& neg_F,
                        const std::string& filename,
                        Debugger* debugger);

bool LoadResultBin(const PassiveGripper& psg,
                   const std::string& filename,
                   Eigen::MatrixXd& out_V,
                   Eigen::MatrixXi& out_F);

// Refine gripper mesh
// V, F    : gripper mesh
// sv_     : swept volume
// out_    : refined gripper
void RefineGripper(const PassiveGripper& psg,
                   const Eigen::MatrixXd& V,
                   const Eigen::MatrixXi& F,
                   const Eigen::MatrixXd& neg_V,
                   const Eigen::MatrixXi& neg_F,
                   Eigen::MatrixXd& out_V,
                   Eigen::MatrixXi& out_F);

}  // namespace psg