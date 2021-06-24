#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ikfast>
#include <vector>

namespace robots {

void Forward(const std::vector<double>& jointConfig,
             Eigen::Matrix3d& out_rot,
             Eigen::Vector3d& out_trans);

Eigen::Affine3d Forward(const std::vector<double>& jointConfig);

bool Inverse(const Eigen::Matrix3d& rot,
             const Eigen::Vector3d& trans,
             std::vector<std::vector<double>>& out_jointConfigs);

bool Inverse(const Eigen::Affine3d trans,
             std::vector<std::vector<double>>& out_jointConfigs);

void ForwardIntermediate(const std::vector<double>& jointConfig,
                         std::vector<Eigen::Affine3d>& out_trans);

}  // namespace robots
