#pragma once

#include <igl/opengl/ViewerCore.h>
#include <igl/opengl/ViewerData.h>
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