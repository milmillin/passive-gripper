#pragma once

#include <Eigen/Core>

namespace gripper {

struct ContactPoint {
  Eigen::Vector3d position;
  Eigen::Vector3d normal;
  bool isTypeA;
};

}  // namespace gripper