// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#pragma once

#include "ContactPoint.h"

#include "../Constants.h"
#include "../serialization/Serialization.h"

namespace psg {

struct GripperParams : psg::serialization::Serializable {
  std::vector<Eigen::MatrixXd> fingers;
  Trajectory trajectory;
  std::vector<ContactPoint> contact_points;

  DECL_SERIALIZE() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(fingers);
    SERIALIZE(trajectory);
    SERIALIZE(contact_points);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(fingers);
      DESERIALIZE(trajectory);
      DESERIALIZE(contact_points);
    }
  }
};

}  // namespace psg
