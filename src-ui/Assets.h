// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#pragma once

#include <igl/readSTL.h>
#include <Eigen/Core>
#include <boost/dll/runtime_symbol_info.hpp>
#include <vector>

namespace psg {
namespace ui {

// https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_description/meshes/ur5/visual
static const char* kAssetNames[] = {"base.stl",
                                    "shoulder.stl",
                                    "upperarm.stl",
                                    "forearm.stl",
                                    "wrist1.stl",
                                    "wrist2.stl",
                                    "wrist3.stl"};

static const size_t kAssetSize = 7;

const std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>> kAssets = ([]() {
  std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>> res(kAssetSize);
  auto path = boost::dll::program_location().parent_path() / "assets";

  Eigen::MatrixXd N;
  for (size_t i = 0; i < kAssetSize; i++) {
    auto fn = path / kAssetNames[i];
    std::ifstream f(fn.string(), std::ios::in | std::ios::binary);
    if (!f.is_open()) {
      std::cout << "Cannot load asset: " << fn.string() << std::endl;
      continue;
    }
    igl::readSTL(f, res[i].first, res[i].second, N);
  }
  return res;
})();

}  // namespace ui
}  // namespace psg