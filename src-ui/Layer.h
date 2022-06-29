#pragma once

namespace psg {
namespace ui {

enum class Layer : int {
  kMesh = 0,
  kCenterOfMass,
  kContactPoints,
  kFingers,
  kTrajectory,
  kAxis,
  kSweptSurface,
  kGripperBound,
  kNegVol,
  kRobot,
  kGripper,
  kInitFingers,
  kInitTrajectory,
  kGradient,
  kFloor,
  kContactFloor,
  kDebug,
  kRemesh,
  kMax
};

}
}  // namespace psg