#include "Voxels.h"

#include <omp.h>
#include <Eigen/Core>
#include <iostream>
#include <vector>

#include "Geometry.h"
#include "MeshInfo.h"
#include "Utils.h"

namespace gripper {

void Voxels::Voxelize(const MatrixXd& mesh_V,
                      const MatrixXi& mesh_F,
                      double voxelSize,
                      Voxels& out_voxels,
                      std::vector<Voxel>& out_voxelCoords) {
  out_voxels.m_intersector.init(mesh_V.cast<float>(), mesh_F, true);
  out_voxels.m_mesh_V = mesh_V;
  out_voxels.m_mesh_F = mesh_F;

  MeshInfo meshInfo(mesh_V, mesh_F);
  double cubeSize = voxelSize;
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
          RowVector3f position =
              out_voxels.GetVoxelCenter<float>(Voxel(i, j, k)).transpose();
          out_voxels.m_intersector.intersectRay(
              position,
              direction,
              hits,
              num_rays,
              0.f,
              std::numeric_limits<float>::infinity(),
              -1);
          if (hits.size() % 2 == 1) {
            t_voxelCoords.push_back(Voxel(i, j, k));
          }
        }
      }
    }

#pragma omp critical
    out_voxelCoords.insert(
        out_voxelCoords.end(), t_voxelCoords.begin(), t_voxelCoords.end());
  }
}

std::vector<Voxels::Voxel> Voxels::FilterSupportingVoxels(
    const Eigen::MatrixXd& mesh_V,
    const Eigen::MatrixXi& mesh_F,
    const std::vector<Voxel>& voxelCoords,
    double groundY) const {
  std::vector<Voxel> m_voxelCoords = voxelCoords;

  // Sort by x, z, y
  std::sort(m_voxelCoords.begin(),
            m_voxelCoords.end(),
            [](const Voxel& a, const Voxel& b) -> bool {
              if (a(0) != b(0)) return a(0) < b(0);
              if (a(2) != b(2)) return a(2) < b(2);
              return a(1) < b(1);
            });

  igl::embree::EmbreeIntersector intersector;
  intersector.init(mesh_V.cast<float>(), mesh_F, true);
  Eigen::RowVector3f rayDirection = Eigen::RowVector3f::UnitY();

  std::vector<Voxel> result;
  Voxel oneY(0, 1, 0);
  for (size_t i = 0; i < m_voxelCoords.size(); i++) {
    Voxel support = m_voxelCoords[i] - oneY;
    Eigen::RowVector3f supportPos = GetVoxelCenter<float>(support).transpose();
    igl::Hit hit;
    if (i == 0 || m_voxelCoords[i - 1] != support) {
      if (supportPos.y() > groundY) {
        if (intersector.intersectRay(supportPos, rayDirection, hit)) {
          Vector3d normal = ComputeNormal(mesh_V.row(mesh_F(hit.id, 0)),
                                          mesh_V.row(mesh_F(hit.id, 1)),
                                          mesh_V.row(mesh_F(hit.id, 2)));
          // Only consider downward facing faces (45 degrees)
          if (abs(normal.y()) > cos(EIGEN_PI / 4)) {
            result.push_back(support);
          }
        }
      }
    }
  }
  return result;
}

std::vector<Voxels::Voxel> Voxels::FilterGrabDirection(
    const std::vector<Voxel>& voxelCoords,
    const MatrixXd& mesh_V,
    const MatrixXi& mesh_F,
    Vector3f grabDirection) const {
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

void Voxels::GetSupportCandidates(
    std::vector<Voxel> voxelCoords,
    Vector3f grabDirection,
    double groundY,
    std::vector<VoxelD>& out_voxelCoords,
    std::vector<Eigen::Vector3d>& out_normals) const {
  // Sort by x, z, y
  std::sort(voxelCoords.begin(),
            voxelCoords.end(),
            [](const Voxel& a, const Voxel& b) -> bool {
              if (a(0) != b(0)) return a(0) < b(0);
              if (a(2) != b(2)) return a(2) < b(2);
              return a(1) < b(1);
            });

  Eigen::RowVector3f upRay = Eigen::RowVector3f::UnitY();
  Eigen::RowVector3f grabRay = -grabDirection.transpose();
  Voxel oneY(0, 1, 0);

  out_voxelCoords.clear();
  out_normals.clear();

  for (size_t i = 0; i < voxelCoords.size(); i++) {
    Voxel support = voxelCoords[i] - oneY;
    Eigen::RowVector3f supportPos = GetVoxelCenter<float>(support).transpose();
    igl::Hit hit;
    if (i != 0 && voxelCoords[i - 1] == support) continue;
    if (supportPos.y() <= groundY) continue;
    if (!m_intersector.intersectRay(supportPos, upRay, hit)) continue;

    Vector3d normal = ComputeNormal(m_mesh_V.row(m_mesh_F(hit.id, 0)),
                                    m_mesh_V.row(m_mesh_F(hit.id, 1)),
                                    m_mesh_V.row(m_mesh_F(hit.id, 2)));

    // Only consider downward facing faces (45 degrees)
    if (abs(normal.y()) <= cos(EIGEN_PI / 4)) continue;

    Eigen::Vector3f contactPos = supportPos + upRay * (hit.t - 0.0001f);

    // Filter Grab Direction
    if (m_intersector.intersectRay(contactPos, grabRay, hit)) continue;
    out_voxelCoords.push_back(((contactPos.cast<double>() - origin) / cubeSize).array() -
                     0.5);
    out_normals.push_back(-normal.normalized());
  }
}

template <typename T>
void Voxels::GenerateMesh(
    const std::vector<Eigen::Matrix<T, 3, 1>>& voxelCoords,
    float voxelBoxSizeScale,
    MatrixXd& out_V,
    MatrixXi& out_F) const {
  ssize_t numVoxels = voxelCoords.size();

  out_V.resize(8 * numVoxels, 3);
  out_F.resize(12 * numVoxels, 3);

  Eigen::Vector3d offset =
      Eigen::Vector3d::Constant(cubeSize * voxelBoxSizeScale * 0.5);

#pragma omp parallel for
  for (ssize_t i = 0; i < numVoxels; i++) {
    out_V.block<8, 3>(8 * i, 0) =
        cube_V * (cubeSize * voxelBoxSizeScale) +
        (GetVoxelCenter<double>(voxelCoords[i]) - offset)
            .transpose()
            .replicate<8, 1>();
    out_F.block<12, 3>(12 * i, 0) = cube_F.array() + 8 * i;
  }
}

template <typename T>
void Voxels::GeneratePoints(
    const std::vector<Eigen::Matrix<T, 3, 1>>& voxelCoords,
    MatrixXd& out_P) const {
  ssize_t numVoxels = voxelCoords.size();

  out_P.resize(numVoxels, 3);

#pragma omp parallel for
  for (ssize_t i = 0; i < numVoxels; i++) {
    out_P.row(i) = GetVoxelCenter<double>(voxelCoords[i]);
  }
}

Voxels::VoxelD Voxels::GetCenterOfMass(
    const std::vector<Voxel>& voxelCoords) const {
  size_t numVoxels = voxelCoords.size();

  double x = 0, y = 0, z = 0;

#pragma omp parallel for reduction(+ : x, y, z)
  for (ssize_t i = 0; i < numVoxels; i++) {
    x += voxelCoords[i](0);
    y += voxelCoords[i](1);
    z += voxelCoords[i](2);
  }

  return VoxelD(x / numVoxels, y / numVoxels, z / numVoxels);
}

}  // namespace gripper