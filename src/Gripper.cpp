#include "Gripper.h"

#include <Eigen/Geometry>
#include "MeshInfo.h"
#include "Geometry.h"

namespace gripper {

const float ANGLE_TO_RADIAN = EIGEN_PI / 180;

Gripper::Gripper(const Eigen::MatrixXd& mesh_V, const Eigen::MatrixXi& mesh_F,
  const Voxels& voxels, const std::vector<Voxels::Voxel>& voxelCoords,
  const Eigen::Vector2f& grabAngle):
  m_grabAngle(grabAngle * ANGLE_TO_RADIAN)
{
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

  gripper_V.resize(8 * (1 + voxelCoords.size()), 3);
  gripper_F.resize(12 * (1 + voxelCoords.size()), 3);

  // Generate backplate
  gripper_V.block<8, 3>(0, 0) = GenerateCubeV(
    Eigen::Vector3d(info.maximum.x(), info.minimum.y(), info.minimum.z()),
    Eigen::Vector3d(voxels.cubeSize, info.size.y(), info.size.z())
  );
  gripper_F.block<12, 3>(0, 0) = cube_F;

  // Generate poles
  double poleSize = voxels.cubeSize / 2;
  for (size_t i = 0; i < voxelCoords.size(); i++) {
    Vector3d contactPosition = tInverse * voxels.GetVoxelCenter<double>(voxelCoords[i]);
    gripper_V.block<8, 3>((i + 1) * 8, 0) = GenerateCubeV(
      Eigen::Vector3d(
        info.maximum.x(),
        contactPosition.y() - poleSize / 2,
        contactPosition.z() - poleSize / 2),
      Eigen::Vector3d(
        contactPosition.x() - info.maximum.x(),
        poleSize,
        poleSize)
    );
    gripper_F.block<12, 3>((i + 1) * 12, 0) = cube_F.array() + (i + 1) * 8;
  }

  // Rotate back
  for (size_t i = 0; i < gripper_V.rows(); i++) {
    Vector3d v = gripper_V.row(i);
    gripper_V.row(i) = t * v;
  }
}

Gripper::Gripper()
{ }

}