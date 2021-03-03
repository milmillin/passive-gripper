#pragma once

#include <igl/opengl/ViewerCore.h>
#include <igl/opengl/ViewerData.h>
#include <Eigen/Core>
#include <string>

namespace utils {

using std::string;

inline string error(string message, string file, unsigned line) {
  return file + ":" + std::to_string(line) + " " + message;
}

void CaptureScreen(igl::opengl::ViewerCore& core,
                   std::vector<igl::opengl::ViewerData>& data_list,
                   unsigned width,
                   unsigned height,
                   const std::string& filename);

#define ERROR_MESSAGE(message) utils::error(message, __FILE__, __LINE__)

}  // namespace utils

namespace gripper {

const Eigen::RowVector3d purple = Eigen::RowVector3d(219, 76, 178) / 255;
const Eigen::RowVector3d orange = Eigen::RowVector3d(239, 126, 50) / 255;
const Eigen::RowVector3d red = Eigen::RowVector3d(192, 35, 35) / 255;
const Eigen::RowVector3d brown = Eigen::RowVector3d(130, 4, 1) / 255;
const Eigen::RowVector3d darkBlue = Eigen::RowVector3d(20, 36, 89) / 255;
const Eigen::RowVector3d gold = Eigen::RowVector3d(252, 181, 9) / 255;

}  // namespace gripper