#include "Gripper.h"

#include <Eigen/Geometry>

#include "Geometry.h"
#include "MeshInfo.h"

namespace gripper {

const float ANGLE_TO_RADIAN = EIGEN_PI / 180;

Gripper::Gripper(const Eigen::MatrixXd& mesh_V,
                 const Eigen::MatrixXi& mesh_F,
                 const std::vector<Eigen::Vector3d>& contactPoints,
                 double rodDiameter,
                 const Eigen::Vector2f& grabAngle)
    : m_grabAngle(grabAngle * ANGLE_TO_RADIAN) {
  Eigen::Affine3d t = Eigen::Affine3d::Identity();
  t.rotate(Eigen::AngleAxisd(-m_grabAngle(0), Eigen::Vector3d::UnitY()));
  t.rotate(Eigen::AngleAxisd(m_grabAngle(1), Eigen::Vector3d::UnitZ()));

  Eigen::Affine3d tInverse = t.inverse();

  // Rotated mesh so that the gripper comes in -x direction
  Eigen::MatrixXd rotated_V(mesh_V.rows(), 3);
  for (size_t i = 0; i < mesh_V.rows(); i++) {
    Vector3d v = mesh_V.row(i);
    rotated_V.row(i) = tInverse * v;
  }

  MeshInfo info(rotated_V, mesh_F);

  gripper_V.resize(8 + cylinderNumV * contactPoints.size(), 3);
  gripper_F.resize(12 + cylinderNumF * contactPoints.size(), 3);

  // Generate backplate
  gripper_V.block<8, 3>(0, 0) = GenerateCubeV(
      Eigen::Vector3d(info.maximum.x(), info.minimum.y(), info.minimum.z()),
      // TODO: Check backplate thickness
      Eigen::Vector3d(0.02, info.size.y(), info.size.z()));
  gripper_F.block<12, 3>(0, 0) = cube_F;

  // Generate rods
  for (size_t i = 0; i < contactPoints.size(); i++) {
    Vector3d contactPosition = tInverse * contactPoints[i];
    Vector3d origin(info.maximum.x(), contactPosition.y(), contactPosition.z());

    gripper_V.block<cylinderNumV, 3>(8 + i * cylinderNumV, 0) =
        GenerateCylinderV(origin, contactPosition, rodDiameter / 2);
    gripper_F.block<cylinderNumF, 3>(12 + i * cylinderNumF, 0) =
        cylinder_F.array() + (8 + i * cylinderNumV);
  }

  // Rotate back
  for (size_t i = 0; i < gripper_V.rows(); i++) {
    Vector3d v = gripper_V.row(i);
    gripper_V.row(i) = t * v;
  }
}

Gripper::Gripper() {}

}  // namespace gripper