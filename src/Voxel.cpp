#include "Voxel.h"

#include <Eigen/Core>
#include <igl/embree/EmbreeIntersector.h>
#include <vector>

#include "MeshInfo.h"

namespace gripper {

Voxel Voxel::Voxelize(const MatrixXd& V, const MatrixXi& F, int num_division)
{
  igl::embree::EmbreeIntersector intersector;
  intersector.init(V.cast<float>(), F, true);

  MeshInfo meshInfo(V, F);
  double cubeSize = (meshInfo.Size / num_division).minCoeff();
  RowVector3f offset = RowVector3f::Constant(cubeSize / 2);
  RowVector3f direction(0, 0, 1);
  double tFar = meshInfo.Size.maxCoeff();

  size_t nX = std::ceil(meshInfo.Size.x() / cubeSize);
  size_t nY = std::ceil(meshInfo.Size.y() / cubeSize);
  size_t nZ = std::ceil(meshInfo.Size.z() / cubeSize);


  Voxel voxel(nX, nY, nZ);
  voxel.CubeSize = cubeSize;
  voxel.Origin = meshInfo.Minimum;
  RowVector3f origin = meshInfo.Minimum.cast<float>().transpose() + offset;

#pragma omp parallel for
  for (int i = 0; i < (int)nX; i++) {
    for (size_t j = 0; j < nY; j++) {
      for (size_t k = 0; k < nZ; k++) {
        vector<igl::Hit> hits;
        RowVector3f position = origin + RowVector3f(i, j, k) * cubeSize;
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
  return m_data(GetVoxelIndex(x, y, z));
}

bool& Voxel::operator()(size_t x, size_t y, size_t z)
{
  return m_data(GetVoxelIndex(x, y, z));
}

Voxel::VoxelList Voxel::GetSupportPointCandidates(
    const MatrixXd& V,
    const MatrixXi& F,
    Vector3d grabDirection) const {
  igl::embree::EmbreeIntersector intersector;
  intersector.init(V.cast<float>(), F, true);
  VoxelList result;

  #pragma omp parallel for
  for (int i = 0; i < (int)nX; i++) {
    for (size_t j = 0; j < nY; j++) {
      for (size_t k = 0; k < nZ; k++) {
        if (j + 1 < nY && !(*this)(i, j, k) && (*this)(i, j + 1, k)) {
          result.push_back(GetVoxelIndex(i, j, k));
        }
      }
    }
  }
  return result;
}

void Voxel::GenerateMesh(MatrixXd& V, MatrixXi& F, float boxSize,
    VoxelList voxels) const {
  // Inline mesh of a cube
  static const MatrixXd cube_V = (MatrixXd(8, 3) <<
    0.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, 1.0, 0.0,
    0.0, 1.0, 1.0,
    1.0, 0.0, 0.0,
    1.0, 0.0, 1.0,
    1.0, 1.0, 0.0,
    1.0, 1.0, 1.0).finished();
  static const MatrixXi cube_F = (MatrixXi(12, 3) <<
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

  size_t numSolid = voxels.size();

  V.resize(8 * numSolid, 3);
  F.resize(12 * numSolid, 3);
  for (size_t i = 0; i < numSolid; i++) {
    V.block<8, 3>(8 * i, 0) = cube_V * (CubeSize * boxSize) +
      (Origin + GetVoxelCoord<Vector3d>(voxels[i]) * CubeSize).transpose().replicate<8, 1>();
    F.block<12, 3>(12 * i, 0) = cube_F.array() + 8 * i;
  }
}

void Voxel::GeneratePoints(MatrixXd& P) const {
  auto voxels = GetAllVoxelIndex();

  P.resize(voxels.size(), 3);

  for (size_t i = 0; i < voxels.size(); i++) {
    P.row(i) = Origin + CubeSize *
      (Vector3d(GetVoxelCoord<Vector3d>(voxels[i]))
       + Vector3d(0.5, 0.5, 0.5));
  }
}

Voxel::VoxelList Voxel::GetAllVoxelIndex() const {
  VoxelList voxelIndices;
  for (size_t i = 0; i < m_nXYZ; i++)
    if (m_data(i))
      voxelIndices.push_back(i);

  return voxelIndices;
}

}  // namespace gripper