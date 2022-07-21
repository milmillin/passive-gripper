// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#include <igl/opengl/glfw/Viewer.h>

#include "MainUI.h"

int main() {
  psg::ui::MainUI ui;
  igl::opengl::glfw::Viewer viewer;
  viewer.plugins.push_back(&ui);
  viewer.launch();
  return 0;
}