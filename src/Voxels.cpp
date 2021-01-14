#include "Voxels.h"

#include <Eigen/Core>
#include <igl/embree/EmbreeIntersector.h>
#include <vector>
#include <omp.h>

#include "MeshInfo.h"
#include "Utils.h"
#include "Geometry.h"

namespace gripper {

void Voxels::Voxelize(const MatrixXd& mesh_V, const MatrixXi& mesh_F, int numDivision,
    Voxels& out_voxels, std::vector<Voxel>& out_voxelCoords)
{
  igl::embree::EmbreeIntersector intersector;
  intersector.init(mesh_V.cast<float>(), mesh_F, true);

  MeshInfo meshInfo(mesh_V, mesh_F);
  double cubeSize = (meshInfo.size / numDivision).minCoeff();
  RowVector3f offset = RowVector3f::Constant(cubeSize / 2);
  RowVector3f direction(0, 0, 1);
  double tFar = meshInfo.size.maxCoeff();

  ssize_t nX = std::ceil(meshInfo.size.x() / cubeSize);
  ssize_t nY = std::ceil(meshInfo.size.y() / cubeSize);
  ssize_t nZ = std::ceil(meshInfo.size.z() / cubeSize);

  out_voxels.cubeSize = cubeSize;
  out_voxels.origin = meshInfo.minimum;
  RowVector3f origin = meshInfo.minimum.cast<float>().transpose() + offset;

  out_voxelCoords.clear();
  #pragma omp parallel
  {
    vector<Voxel> t_voxelCoords;
    vector<igl::Hit> hits;
    int num_rays = 0;

    #pragma omp for
    for (ssize_t i = 0; i < nX; i++) {
      for (ssize_t j = 0; j < nY; j++) {
        for (ssize_t k = 0; k < nZ; k++) {
          RowVector3f position = out_voxels.GetVoxelCenter<float>(Voxel(i, j, k)).transpose();
          intersector.intersectRay(position, direction, hits, num_rays, 0.f, std::numeric_limits<float>::infinity(), -1);
          if (hits.size() % 2 == 1) {
            t_voxelCoords.push_back(Voxel(i, j, k));
          }
        }
      }
    }

    #pragma omp critical
    out_voxelCoords.insert(out_voxelCoords.end(), t_voxelCoords.begin(), t_voxelCoords.end());
  }
}

std::vector<Voxels::Voxel> Voxels::FilterSupportingVoxels(const std::vector<Voxel>& voxelCoords, double groundY) const
{
  std::vector<Voxel> m_voxelCoords = voxelCoords;

  // Sort by x, z, y
  std::sort(m_voxelCoords.begin(), m_voxelCoords.end(), [](const Voxel& a, const Voxel& b)->bool {
    if (a(0) != b(0)) return a(0) < b(0);
    if (a(2) != b(2)) return a(2) < b(2);
    return a(1) < b(1);
    });

  std::vector<Voxel> result;
  Voxel oneY(0, 1, 0);
  for (size_t i = 0; i < m_voxelCoords.size(); i++) {
    Voxel support = m_voxelCoords[i] - oneY;
    if (i == 0 || m_voxelCoords[i - 1] != support) {
      if (GetVoxelCenter<double>(support).y() > groundY) {
        result.push_back(support);
      }
    }
  }
  return result;
}

std::vector<Voxels::Voxel> Voxels::FilterGrabDirection(
  const std::vector<Voxel>& voxelCoords,
  const MatrixXd& mesh_V,
  const MatrixXi& mesh_F,
  Vector3f grabDirection) const
{
  igl::embree::EmbreeIntersector intersector;
  intersector.init(mesh_V.cast<float>(), mesh_F, true);

  std::vector<Voxel> result;
  ssize_t numVoxels = voxelCoords.size();

  #pragma omp parallel
  {
    std::vector<Voxel> t_result;
    Eigen::RowVector3f t_grabDirection = -grabDirection.transpose();
    igl::Hit hit;

    #pragma omp for
    for (ssize_t i = 0; i < numVoxels; i++) {
      RowVector3f position = GetVoxelCenter<float>(voxelCoords[i]).transpose();
      // TODO: check multiple rays
      if (!intersector.intersectRay(position, t_grabDirection, hit)) {
        t_result.push_back(voxelCoords[i]);
      }
    }

    #pragma omp critical
    result.insert(result.end(), t_result.begin(), t_result.end());
  }
  return result;
}

void Voxels::GenerateMesh(const std::vector<Voxel>& voxelCoords, float voxelBoxSizeScale,
    MatrixXd& out_V, MatrixXi& out_F) const {
  ssize_t numVoxels = voxelCoords.size();

  out_V.resize(8 * numVoxels, 3);
  out_F.resize(12 * numVoxels, 3);

  Eigen::Vector3d offset = Eigen::Vector3d::Constant(cubeSize * voxelBoxSizeScale * 0.5);

  #pragma omp parallel for
  for (ssize_t i = 0; i < numVoxels; i++) {
    out_V.block<8, 3>(8 * i, 0) = cube_V * (cubeSize * voxelBoxSizeScale) +
      (GetVoxelCenter<double>(voxelCoords[i]) - offset).transpose().replicate<8, 1>();
    out_F.block<12, 3>(12 * i, 0) = cube_F.array() + 8 * i;
  }
}

void Voxels::GeneratePoints(const std::vector<Voxel>& voxelCoords, MatrixXd& out_P) const {
  ssize_t numVoxels = voxelCoords.size();

  out_P.resize(numVoxels, 3);

  #pragma omp parallel for
  for (ssize_t i = 0; i < numVoxels; i++) {
    out_P.row(i) = GetVoxelCenter<double>(voxelCoords[i]);
  }
}

Voxels::VoxelD Voxels::GetCenterOfMass(const std::vector<Voxel>& voxelCoords) const {
  size_t numVoxels = voxelCoords.size();

  double x = 0, y = 0, z = 0;

  #pragma omp parallel for reduction(+:x,y,z)
  for (ssize_t i = 0; i < numVoxels; i++) {
    x += voxelCoords[i](0);
    y += voxelCoords[i](1);
    z += voxelCoords[i](2);
  }

  return VoxelD(x / numVoxels, y / numVoxels, z / numVoxels);
}

}  // namespace gripper