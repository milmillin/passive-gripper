#include "Voxel.h"

#include <Eigen/Core>
#include <igl/embree/EmbreeIntersector.h>
#include <vector>

#include "MeshInfo.h"

namespace gripper {

Voxel Voxel::Voxelize(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, int num_division)
{
  igl::embree::EmbreeIntersector intersector;
  intersector.init(V.cast<float>(), F, true);

  MeshInfo meshInfo(V, F);
  double cubeSize = (meshInfo.Size / num_division).minCoeff();
  Eigen::RowVector3f offset = Eigen::RowVector3f::Constant(cubeSize / 2);
  Eigen::RowVector3f direction(0, 0, 1);
  double tFar = meshInfo.Size.maxCoeff();

  size_t nX = std::ceil(meshInfo.Size.x() / cubeSize);
  size_t nY = std::ceil(meshInfo.Size.y() / cubeSize);
  size_t nZ = std::ceil(meshInfo.Size.z() / cubeSize);

  
  Voxel voxel(nX, nY, nZ);
  voxel.CubeSize = cubeSize;
  voxel.Origin = meshInfo.Minimum;
  Eigen::RowVector3f origin = meshInfo.Minimum.cast<float>().transpose() + offset;

#pragma omp parallel for
  for (int i = 0; i < (int)nX; i++) {
    for (size_t j = 0; j < nY; j++) {
      for (size_t k = 0; k < nZ; k++) {
        std::vector<igl::Hit> hits;
        Eigen::RowVector3f position = origin + Eigen::RowVector3f(i, j, k) * cubeSize;
        int num_rays = 0;
        intersector.intersectRay(position, direction, hits, num_rays, 0.f, std::numeric_limits<float>::infinity(), -1);
        voxel(i, j, k) = hits.size() % 2 == 1;
      }
    }
  }
  return voxel;
}

Voxel::Voxel(size_t nX, size_t nY, size_t nZ) :
  nX(nX),
  nY(nY),
  nZ(nZ),
  CubeSize(1.0),
  Origin(0, 0, 0),
  m_nYZ(nY * nZ),
  m_nXYZ(nX * nY * nZ)
{ 
  m_data.resize(m_nXYZ);
}

bool Voxel::operator()(size_t x, size_t y, size_t z) const
{
  return m_data(getVoxelIndex(x, y, z));
}

bool& Voxel::operator()(size_t x, size_t y, size_t z)
{
  return m_data(getVoxelIndex(x, y, z));
}

void Voxel::GenerateMesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F, float boxSize) const
{
  // Inline mesh of a cube
  static const Eigen::MatrixXd cube_V = (Eigen::MatrixXd(8, 3) <<
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, 1.0, 0.0,
    0.0, 1.0, 1.0,
    1.0, 0.0, 0.0,
    1.0, 0.0, 1.0,
    1.0, 1.0, 0.0,
    1.0, 1.0, 1.0).finished();
  static const Eigen::MatrixXi cube_F = (Eigen::MatrixXi(12, 3) <<
    1, 7, 5,
    1, 3, 7,
    1, 4, 3,
    1, 2, 4,
    3, 8, 7,
    3, 4, 8,
    5, 7, 8,
    5, 8, 6,
    1, 5, 6,
    1, 6, 2,
    2, 6, 8,
    2, 8, 4).finished().array() - 1;

  auto solid = getAllVoxelIndex();
  size_t numSolid = solid.size();

  V.resize(8 * numSolid, 3);
  F.resize(12 * numSolid, 3);
  for (size_t i = 0; i < numSolid; i++) {
    V.block<8, 3>(8 * i, 0) = cube_V * (CubeSize * boxSize) +
      (Origin + getVoxelCoord<Eigen::Vector3d>(solid[i]) * CubeSize).transpose().replicate<8, 1>();
    F.block<12, 3>(12 * i, 0) = cube_F.array() + 8 * i;
  }
}

void Voxel::GeneratePoints(Eigen::MatrixXd& P) const {
  auto voxels = getAllVoxelIndex();

  P.resize(voxels.size(), 3);

  for (size_t i = 0; i < voxels.size(); i++) {
    P.row(i) = Origin + CubeSize *
      (Eigen::Vector3d(getVoxelCoord<Eigen::Vector3d>(voxels[i]))
       + Eigen::Vector3d(0.5, 0.5, 0.5));
  }
}

std::vector<size_t> Voxel::getAllVoxelIndex() const {
  std::vector<size_t> voxelIndices;
  for (size_t i = 0; i < m_nXYZ; i++)
    if (m_data(i))
      voxelIndices.push_back(i);

  return voxelIndices;
}

}  // namespace gripper