#include "MeshInfo.h"

namespace gripper {

MeshInfo::MeshInfo() :
  Minimum(Eigen::Vector3d::Zero()),
  Maximum(Eigen::Vector3d::Zero()),
  Size(Eigen::Vector3d::Zero())
{ }

MeshInfo::MeshInfo(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
  Minimum = V.colwise().minCoeff();
  Maximum = V.colwise().maxCoeff();
  Size = Maximum - Minimum;
}

}

