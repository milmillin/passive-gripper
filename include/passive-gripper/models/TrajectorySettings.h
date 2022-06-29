#pragma once

#include "../serialization/Serialization.h"

namespace psg {

struct TrajectorySettings : psg::serialization::Serializable {
  size_t n_keyframes = 4;

  DECL_SERIALIZE() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(n_keyframes);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(n_keyframes);
    }
  }
};

}  // namespace psg
