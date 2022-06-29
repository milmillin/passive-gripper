
#include <igl/opengl/glfw/Viewer.h>

#include "MainUI.h"

int main(int argv, char** argc) {
  psg::ui::MainUI ui;
  igl::opengl::glfw::Viewer viewer;
  viewer.plugins.push_back(&ui);
  viewer.launch();
  return 0;
}