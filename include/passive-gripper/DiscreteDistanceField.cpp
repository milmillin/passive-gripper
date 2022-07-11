// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#include "DiscreteDistanceField.h"

#include "TopoOpt.h"

#include <Eigen/Core>
#include <climits>
#include <iostream>
#include <queue>
#include <vector>

namespace psg {

DiscreteDistanceField::DiscreteDistanceField(const Eigen::MatrixXd& V,
                                             const Eigen::MatrixXi& F,
                                             int units,
                                             Eigen::Vector3d base) {
  lower_bound = V.colwise().minCoeff().transpose();
  upper_bound = V.colwise().maxCoeff().transpose();
  lower_bound = lower_bound.cwiseMin(base);
  upper_bound = upper_bound.cwiseMax(base);

  double shortest_side = (upper_bound - lower_bound).minCoeff();
  resolution = shortest_side / units;

  auto margin = 2 * Eigen::Vector3d(resolution, resolution, resolution);
  lower_bound -= margin;
  upper_bound += margin;

  auto voxels =
      GetForbiddenVoxels(V, F, lower_bound, upper_bound, resolution, size);
  std::cout << "Map size: " << size(0) << " " << size(1) << " " << size(2)
            << std::endl;

  distance.resize(size(0) * size(1) * size(2));
  for (int& item : distance)
    item = -1;

  for (const auto& voxel : voxels)
    GetVoxelI(voxel) = INT_MAX;

  std::queue<Eigen::Vector3i> queue;
  Eigen::Vector3i start = ((base - lower_bound) / resolution).cast<int>();
  queue.push(start);
  GetVoxelI(start) = 0;
  while (queue.size() > 0) {
    Eigen::Vector3i next = queue.front();
    queue.pop();

    int current_cost = GetVoxelI(next) + 1;
    // std::cout << current_cost << std::endl;

    for (int dx = -1; dx <= 1; dx++)
      for (int dy = -1; dy <= 1; dy++)
        for (int dz = -1; dz <= 1; dz++) {
          Eigen::Vector3i next_coord = next + Eigen::Vector3i(dx, dy, dz);
          next_coord = next_coord.cwiseMax(Eigen::Vector3i(0, 0, 0));
          next_coord = next_coord.cwiseMin(size - Eigen::Vector3i(1, 1, 1));
          if (GetVoxelI(next_coord) != -1 &&
              GetVoxelI(next_coord) > current_cost) {
            GetVoxelI(next_coord) = current_cost;
            queue.push(next_coord);
          }
        }
  }
}

}  // namespace psg
