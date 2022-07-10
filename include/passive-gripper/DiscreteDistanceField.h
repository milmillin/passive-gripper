#pragma once

#include <Eigen/Core>
#include <vector>

namespace psg {

class DiscreteDistanceField {
 public:
  std::vector<int> distance;
  Eigen::Vector3i size;
  Eigen::Vector3d lower_bound;
  Eigen::Vector3d upper_bound;
  double resolution;

  /// <summary>
  /// Discretize the mesh and calculate the distance from base to all points.
  /// </summary>
  /// <param name="V">The mesh's vertex list</param>
  /// <param name="F">The mesh's face list</param>
  /// <param name="units">Side length of each grid cell</param>
  /// <param name="base">Coordinate of the base</param>
  DiscreteDistanceField(const Eigen::MatrixXd& V,
                        const Eigen::MatrixXi& F,
                        int units,
                        Eigen::Vector3d base);

 private:
  int& GetVoxelI(const Eigen::Vector3i& coord) {
    return distance[(size_t)coord(0) * size(1) * size(2) +
                    (size_t)coord(1) * size(2) + coord(2)];
  }

  int GetVoxelI(const Eigen::Vector3i& coord) const {
    return distance[(size_t)coord(0) * size(1) * size(2) +
                    (size_t)coord(1) * size(2) + coord(2)];
  }

 public:
  /// <summary>
  /// Find the distance to the base.
  /// </summary>
  /// <param name="coord">Target location</param>
  /// <returns>The distance to the base, from the closest valid voxel in the
  /// target location</returns>
  int GetVoxel(Eigen::Vector3d coord) const {
    coord = (coord - lower_bound) / resolution;
    Eigen::Vector3i coordi = coord.cast<int>();
    for (int i = 0; i < 10; i++)
      for (int dx = -i; dx <= i; dx++)
        for (int dy = -i; dy <= i; dy++)
          for (int dz = -i; dz <= i; dz++) {
            Eigen::Vector3i coord_next = coordi + Eigen::Vector3i(dx, dy, dz);
            if (GetVoxelI(coord_next) != -1) return GetVoxelI(coord_next);
          }
  }
};

}  // namespace psg
