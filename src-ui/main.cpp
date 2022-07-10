
#include <igl/opengl/glfw/Viewer.h>

#include "MainUI.h"

int main() {
  psg::ui::MainUI ui;
  igl::opengl::glfw::Viewer viewer;
  viewer.plugins.push_back(&ui);
  viewer.launch();
  return 0;
}