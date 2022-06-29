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

  DiscreteDistanceField(const Eigen::MatrixXd& V,
                        const Eigen::MatrixXi& F,
                        int units,
                        Eigen::Vector3d base);

  const int& getVoxel(double x, double y, double z) const;

  int& getVoxel(int x, int y, int z) {
    return distance[x * size(1) * size(2) + y * size(2) + z];
  }

  const int& getVoxel(int x, int y, int z) const {
    return distance[x * size(1) * size(2) + y * size(2) + z];
  }

  int& getVoxel(Eigen::Vector3i coord) {
    return getVoxel(coord(0), coord(1), coord(2));
  }

  const int& getVoxel(Eigen::Vector3i coord) const {
    return getVoxel(coord(0), coord(1), coord(2));
  }

  int& getVoxel(Eigen::Vector3d coord) {
    coord = (coord - lower_bound) / resolution;
    Eigen::Vector3i coordi = coord.cast<int>();
    for (int i = 0; i < 10; i++)
      for (int dx = -i; dx <= i; dx++)
        for (int dy = -i; dy <= i; dy++)
          for (int dz = -i; dz <= i; dz++) {
            Eigen::Vector3i coord_next = coordi + Eigen::Vector3i(dx, dy, dz);
            if (getVoxel(coord_next) != -1) return getVoxel(coord_next);
          }
  }

  const int& getVoxel(Eigen::Vector3d coord) const {
    coord = (coord - lower_bound) / resolution;
    Eigen::Vector3i coordi = coord.cast<int>();
    for (int i = 0; i < 10; i++)
      for (int dx = -i; dx <= i; dx++)
        for (int dy = -i; dy <= i; dy++)
          for (int dz = -i; dz <= i; dz++) {
            Eigen::Vector3i coord_next = coordi + Eigen::Vector3i(dx, dy, dz);
            if (getVoxel(coord_next) != -1) return getVoxel(coord_next);
          }
  }
};

}  // namespace psg
