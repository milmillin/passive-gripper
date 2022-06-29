#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "PassiveGripper.h"

namespace psg {

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

void SweptVolume(const Eigen::MatrixXd& V,
                 const Eigen::MatrixXi& F,
                 const std::vector<Eigen::Matrix4d>& transformations,
                 double res,
                 Eigen::MatrixXd& out_V,
                 Eigen::MatrixXi& out_F,
                 int num_seeds = 100);

// Bound - Swept Volume
void NegativeSweptVolumePSG(const PassiveGripper& psg,
                            Eigen::MatrixXd& out_V,
                            Eigen::MatrixXi& out_F,
                            const int num_seeds = 100);

// Params-Independent-Bound - Swept Volume
void PiNegativeSweptVolumePSG(const PassiveGripper& psg,
                              Eigen::MatrixXd& out_V,
                              Eigen::MatrixXi& out_F,
                              const int num_seeds = 100);

}  // namespace psg