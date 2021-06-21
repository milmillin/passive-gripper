#include <Eigen/Core>
#include <ikfast>
#include <vector>

namespace robots {

void Forward(const std::vector<double>& jointConfig,
             Eigen::Matrix3d& out_rot,
             Eigen::Vector3d& out_trans);

bool Inverse(const Eigen::Matrix3d& rot,
             const Eigen::Vector3d& trans,
             std::vector<std::vector<double>>& out_jointConfigs);

}  // namespace robots
