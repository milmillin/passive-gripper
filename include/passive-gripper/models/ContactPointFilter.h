#pragma once

#include "../Constants.h"

namespace psg {

struct ContactPointFilter {
  double hole = 0.006;
  double curvature_radius = 0;
  double angle = kPi;  // 180 deg
};

}  // namespace psg