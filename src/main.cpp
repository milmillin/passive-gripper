#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <iostream>

#include "MainUI.h"

using namespace gripper;

int main(int argc, char* argv[]) {
  std::cout << "Num Threads: " << Eigen::nbThreads() << std::endl;

  igl::opengl::glfw::Viewer viewer;

  MainUI voxelizeUI;
  viewer.plugins.push_back(&voxelizeUI);
  viewer.launch();
}
