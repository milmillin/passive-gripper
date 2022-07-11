// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#pragma once

#include <Eigen/Core>

#include "../serialization/Serialization.h"

namespace psg {

struct ContactPoint : psg::serialization::Serializable {
  Eigen::Vector3d position;  // Position of contact
  Eigen::Vector3d normal;    // Normal direction pointing out of mesh
  int fid;                   // Face ID on the object's mesh that makes contact

  DECL_SERIALIZE() {
    constexpr int version = 1;
    SERIALIZE(version);
    SERIALIZE(position);
    SERIALIZE(normal);
    SERIALIZE(fid);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(position);
      DESERIALIZE(normal);
      DESERIALIZE(fid);
    }
  }
};

inline std::ostream& operator<<(std::ostream& f, const ContactPoint& c) {
  f << "ContactPoint:\n"
    << "  position: " << c.position.transpose() << "\n"
    << "  normal: " << c.normal.transpose() << "\n"
    << "  fid: " << c.fid << std::endl;
  return f;
}

}  // namespace psg
