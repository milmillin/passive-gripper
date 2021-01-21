#include "Geometry.h"

#include <omp.h>
#include <cmath>
#include <iostream>

namespace gripper {

// Horizontal is x-z plane
double HorizontalTwiceSignedArea(const Vector3d& t1,
                                 const Vector3d& t2,
                                 const Vector3d& t3) {
  return (t1(0) - t3(0)) * (t2(2) - t3(2)) - (t2(0) - t3(0)) * (t1(2) - t3(2));
}

bool IsSupportPointStable(const Vector3d& p,
                          const Vector3d& t1,
                          const Vector3d& t2,
                          const Vector3d& t3) {
  double d1 = HorizontalTwiceSignedArea(p, t1, t2);
  double d2 = HorizontalTwiceSignedArea(p, t2, t3);
  double d3 = HorizontalTwiceSignedArea(p, t3, t1);
  // std::cout << "IsSupportPointStable " << d1 << " " << d2 << " " << d3 <<
  // std::endl;
  return (d1 > 0 && d2 > 0 && d3 > 0) || (d1 < 0 && d2 < 0 && d3 < 0);
}

double HorizontalDistance(const Vector3d& p,
                          const Vector3d& l1,
                          const Vector3d& l2) {
  Vector2d v(l2(2) - l1(2), -(l2(0) - l1(0)));
  v.normalize();
  Vector2d r(l1(0) - p(0), l1(2) - p(2));
  // std::cout <<"vector "<< v(0) <<" " <<v(1)<<" " <<r(0)<<"
  // "<<r(1)<<std::endl; std::cout <<"dot " << v.dot(r); std::cout <<"abs " <<
  // std::abs(v.dot(r));
  return std::abs(v.dot(r));
}

double TriangleStability(const Vector3d& p,
                         const Vector3d& t1,
                         const Vector3d& t2,
                         const Vector3d& t3) {
  if (!IsSupportPointStable(p, t1, t2, t3)) {
    return 0;
  }
  double d1 = HorizontalDistance(p, t1, t2);
  double d2 = HorizontalDistance(p, t2, t3);
  double d3 = HorizontalDistance(p, t3, t1);
  // std::cout << "TriangleStability " << d1 << " " << d2 << " " << d3 <<
  // std::endl;
  return std::min({d1, d2, d3});
}

std::vector<Voxels::Voxel> FindBestContactDumb(
    const std::vector<Voxels::Voxel>& voxelCoords,
    const Voxels::VoxelD& centerOfMass) {
  typedef Eigen::Matrix<ssize_t, 3, 1> Index3;
  Index3 bestIndex(-1, -1, -1);
  double bestStability = 0;
  ssize_t numVoxels = voxelCoords.size();

#pragma omp parallel
  {
    Index3 t_bestIndex(-1, -1, -1);
    double t_bestStability = 0;
    double t_stability = 0;

#pragma omp for
    for (ssize_t i = 0; i < numVoxels; i++) {
      for (ssize_t j = i + 1; j < numVoxels; j++) {
        for (ssize_t k = j + 1; k < numVoxels; k++) {
          t_stability = TriangleStability(centerOfMass,
                                          voxelCoords[i].cast<double>(),
                                          voxelCoords[j].cast<double>(),
                                          voxelCoords[k].cast<double>());
          if (t_stability > t_bestStability) {
            t_bestStability = t_stability;
            t_bestIndex = Index3(i, j, k);
          }
        }
      }
    }

#pragma omp critical
    if (t_bestStability > bestStability) {
      bestStability = t_bestStability;
      bestIndex = t_bestIndex;
    }
  }

  std::vector<Voxels::Voxel> result;
  for (int i = 0; i < 3; i++) {
    if (bestIndex(i) != -1)
      result.push_back(voxelCoords[bestIndex(i)]);
  }
  return result;
}

// Nudge the contact point so that the rod can support
std::vector<Eigen::Vector3d> RefineContactPoint(
    const Eigen::MatrixXd& mesh_V,
    const Eigen::MatrixXi& mesh_F,
    const Voxels& voxels,
    const std::vector<Voxels::Voxel>& voxelCoords,
    double rodDiameter) {
  static const float tolerance = 0.005f;

  igl::embree::EmbreeIntersector intersector;
  intersector.init(mesh_V.cast<float>(), mesh_F, true);
  Eigen::RowVector3f rayDirection = Eigen::RowVector3f::UnitY();

  std::vector<Eigen::Vector3d> result;
  for (const auto& coord : voxelCoords) {
    RowVector3f position = voxels.GetVoxelCenter<float>(coord).transpose();
    igl::Hit hit;
    if (intersector.intersectRay(position, rayDirection, hit)) {
      position =
          position + (hit.t - rodDiameter / 2 - tolerance) * rayDirection;
    }
    result.push_back(position.cast<double>().transpose());
  }
  return result;
}

Eigen::MatrixXd GenerateCubeV(Eigen::Vector3d origin, Eigen::Vector3d size) {
  for (ssize_t i = 0; i < 3; i++) {
    if (size(i) < 0) {
      origin(i) += size(i);
      size(i) = -size(i);
    }
  }

  Eigen::MatrixXd out_V;
  out_V = cube_V.cwiseProduct(size.transpose().replicate<8, 1>()) +
          origin.transpose().replicate<8, 1>();
  return out_V;
}

Eigen::Vector3f GetDirectionFromAngle(const Eigen::Vector2f& angle) {
  float A = angle(0) / 180.f * EIGEN_PI;
  float B = angle(1) / 180.f * EIGEN_PI;
  float cosA = std::cos(A);
  float sinA = std::sin(A);
  float cosB = std::cos(B);
  float sinB = std::sin(B);
  return Eigen::Vector3f(cosA * cosB, sinB, sinA * cosB);
}

}  // namespace gripper