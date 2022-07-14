// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#pragma once

#include <Eigen/Geometry>
#include <vector>

#include "../serialization/Serialization.h"
#include "ContactPoint.h"

namespace psg {

struct ContactPointMetric : psg::serialization::Serializable {
  std::vector<ContactPoint> contact_points;
  double min_wrench;
  double partial_min_wrench;
  double finger_distance;
  Eigen::Affine3d trans;

  DECL_SERIALIZE() {
    constexpr int version = 3;
    SERIALIZE(version);
    SERIALIZE(contact_points);
    SERIALIZE(min_wrench);
    SERIALIZE(partial_min_wrench);
    SERIALIZE(finger_distance);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    if (version >= 1) {
      DESERIALIZE(contact_points);
      DESERIALIZE(min_wrench);
      DESERIALIZE(partial_min_wrench);
    }
    if (version == 2) {
      int dis;
      DESERIALIZE(dis);
      finger_distance = dis;
    } else if (version > 2) {
      DESERIALIZE(finger_distance);
    }
  }

  bool operator<(const ContactPointMetric& r) const {
    if (min_wrench == r.min_wrench)
      return partial_min_wrench > r.partial_min_wrench;
    return min_wrench > r.min_wrench;
  }
};

}  // namespace psg