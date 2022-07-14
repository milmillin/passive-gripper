// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#pragma once

#include "../serialization/Serialization.h"

namespace psg {

struct TrajectorySettings : psg::serialization::Serializable {
  DECL_SERIALIZE() {
    constexpr int version = 2;
    SERIALIZE(version);
  }

  DECL_DESERIALIZE() {
    int version;
    size_t unused_size_t;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(unused_size_t);
    } else if (version == 2) {
      // does nothing
    }
  }
};

}  // namespace psg
