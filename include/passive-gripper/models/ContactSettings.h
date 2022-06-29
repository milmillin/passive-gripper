#pragma once

#include "../serialization/Serialization.h"

namespace psg {

struct ContactSettings : psg::serialization::Serializable {
  double friction = 0.5;
  size_t cone_res = 4;
  double floor = 0.01;
  double max_angle = kDegToRad * 80;

  DECL_SERIALIZE() {
    constexpr int version = 3;
    SERIALIZE(version);
    SERIALIZE(friction);
    SERIALIZE(cone_res);
    SERIALIZE(floor);
    SERIALIZE(max_angle);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(friction);
      DESERIALIZE(cone_res);
    } else if (version == 2) {
      DESERIALIZE(friction);
      DESERIALIZE(cone_res);
      DESERIALIZE(floor);
    } else if (version == 3) {
      DESERIALIZE(friction);
      DESERIALIZE(cone_res);
      DESERIALIZE(floor);
      DESERIALIZE(max_angle);
    }
  }
};

}  // namespace models
