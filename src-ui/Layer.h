// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

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