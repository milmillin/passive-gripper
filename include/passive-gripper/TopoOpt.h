#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include "PassiveGripper.h"

namespace psg {

/// <summary>
/// Get voxels that the gripper should not occupy.
/// </summary>
/// <param name="V">A vertex list of the target object's mesh</param>
/// <param name="F">A face list of the target object's mesh</param>
/// <param name="lb">Lower bound of the boundary box for discretization</param>
/// <param name="ub">Upper bound of the boundary box for discretization</param>
/// <param name="res">Side length of the grid</param>
/// <param name="out_range">Stores the number of grids on each dimension</param>
std::vector<Eigen::Vector3i> GetForbiddenVoxels(const Eigen::MatrixXd& V,
                                                const Eigen::MatrixXi& F,
                                                const Eigen::Vector3d& lb,
                                                const Eigen::Vector3d& ub,
                                                double res,
                                                Eigen::Vector3i& out_range);
/// <summary>
/// Generate and export a Topy configuration file for topology optimization.
///
/// You may download Topy here: https://github.com/williamhunter/topy
///
/// Need to provide the free space, which is the space that the gripper is
/// allowed to occupy. In the paper, this is the negative space of the
/// target object's swept volume.
/// </summary>
/// <param name="psg">Passive gripper parameters</param>
/// <param name="neg_V">A vertex list of the free space's mesh</param>
/// <param name="neg_F">A face list of the free space's mesh</param>
/// <param name="filename">File path to export the configuration</param>
/// <param name="debugger">If not NULL, send debug information to here</param>
void GenerateTopyConfig(const PassiveGripper& psg,
                        const Eigen::MatrixXd& neg_V,
                        const Eigen::MatrixXi& neg_F,
                        const std::string& filename,
                        Debugger* debugger);

/// <summary>
/// Import Topy optimization result.
/// </summary>
/// <param name="psg">Passive gripper parameters</param>
/// <param name="filename">File path to import the result</param>
/// <param name="out_V">Store the the imported mesh's vertex list</param>
/// <param name="out_F">Store the the imported mesh's face list</param>
/// <returns>True iff successful</returns>
bool LoadResultBin(const PassiveGripper& psg,
                   const std::string& filename,
                   Eigen::MatrixXd& out_V,
                   Eigen::MatrixXi& out_F);

/// <summary>
/// Post-process the gripper.
///
/// This will add features such as the base and the finger tips.
/// </summary>
/// <param name="psg">Passive gripper parameters</param>
/// <param name="V">Original gripper's vertex list</param>
/// <param name="F">Original gripper's face list</param>
/// <param name="neg_V">A vertex list of the free space's mesh</param>
/// <param name="neg_F">A face list of the free space's mesh</param>
/// <param name="out_V">Store the the processed gripper's vertex list</param>
/// <param name="out_F">Store the the processed gripper's face list</param>
void RefineGripper(const PassiveGripper& psg,
                   const Eigen::MatrixXd& V,
                   const Eigen::MatrixXi& F,
                   const Eigen::MatrixXd& neg_V,
                   const Eigen::MatrixXi& neg_F,
                   Eigen::MatrixXd& out_V,
                   Eigen::MatrixXi& out_F);

}  // namespace psg