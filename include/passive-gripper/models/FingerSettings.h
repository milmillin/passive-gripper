#pragma once

#include "../serialization/Serialization.h"

namespace psg {

struct FingerSettings : psg::serialization::Serializable {
  size_t n_finger_joints = kNumFingerJoints;

  DECL_SERIALIZE() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(n_finger_joints);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(n_finger_joints);
    }
  }
};

}  // namespace psg
