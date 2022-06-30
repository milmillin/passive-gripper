#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "PassiveGripper.h"

namespace psg {

/// <summary>
/// Compute the negative swept volume. I.e., the bounding box minus the swept
/// volume
/// </summary>
/// <param name="V">Input mesh vertices</param>
/// <param name="F">Input mesh faces</param>
/// <param name="transformations">List of transformations</param>
/// <param name="box_lb">Lower bound of the bounding box</param>
/// <param name="box_ub">Upper bound of the bounding box</param>
/// <param name="floor">Any point on the floor</param>
/// <param name="floor_N">The normal of the floor</param>
/// <param name="res">Resolution of the marching cubes</param>
/// <param name="out_V">Output mesh vertices</param>
/// <param name="out_F">Output mesh faces</param>
/// <param name="num_seeds">Number of seeds. Refer to documentation of
/// external/swept-volumes.</param>
void NegativeSweptVolume(const Eigen::MatrixXd& V,
                         const Eigen::MatrixXi& F,
                         const std::vector<Eigen::Matrix4d>& transformations,
                         const Eigen::Vector3d& box_lb,
                         const Eigen::Vector3d& box_ub,
                         const Eigen::Vector3d& floor,
                         const Eigen::Vector3d& floor_N,
                         double res,
                         Eigen::MatrixXd& out_V,
                         Eigen::MatrixXi& out_F,
                         int num_seeds = 100);

/// <summary>
/// Compute the swept volume.
/// </summary>
/// <param name="V">Input mesh vertices</param>
/// <param name="F">Input mesh faces</param>
/// <param name="transformations">List of transformations</param>
/// <param name="res">Resolution of the marching cubes</param>
/// <param name="out_V">Output mesh vertices</param>
/// <param name="out_F">Output mesh faces</param>
/// <param name="num_seeds">Number of seeds. Refer to documentation of
/// external/swept-volumes.</param>
void SweptVolume(const Eigen::MatrixXd& V,
                 const Eigen::MatrixXi& F,
                 const std::vector<Eigen::Matrix4d>& transformations,
                 double res,
                 Eigen::MatrixXd& out_V,
                 Eigen::MatrixXi& out_F,
                 int num_seeds = 100);

/// <summary>
/// Wrapper for <see cref="NegativeSweptVolume" /> with settings from the
/// PassiveGripper object.
/// </summary>
/// <param name="psg">Passive gripper object</param>
/// <param name="out_V">Output mesh vertices</param>
/// <param name="out_F">Output mesh faces</param>
/// <param name="num_seeds">Number of seeds. Refer to documentation of
/// external/swept-volumes.</param>
void NegativeSweptVolumePSG(const PassiveGripper& psg,
                            Eigen::MatrixXd& out_V,
                            Eigen::MatrixXi& out_F,
                            const int num_seeds = 100);

}  // namespace psg