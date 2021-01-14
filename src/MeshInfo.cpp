#include "MeshInfo.h"

namespace gripper {

MeshInfo::MeshInfo() :
  minimum(Eigen::Vector3d::Zero()),
  maximum(Eigen::Vector3d::Zero()),
  size(Eigen::Vector3d::Zero())
{ }

MeshInfo::MeshInfo(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
  minimum = V.colwise().minCoeff();
  maximum = V.colwise().maxCoeff();
  size = maximum - minimum;
}

}

