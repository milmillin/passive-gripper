// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#pragma once

#include "../serialization/Serialization.h"

namespace psg {

struct CostSettings : psg::serialization::Serializable {
  double floor = kCostFloor;
  size_t n_trajectory_steps = kNumTrajSteps;
  size_t n_finger_steps = kNumFingerSteps;
  double regularization = kTrajRegularization;
  double robot_collision = kRobotCollisionContrib;
  double traj_energy = kTrajEnergyContrib;
  double gripper_energy = kGripperEnergyContrib;
  double geodesic_contrib = kGeodesicContrib;
  double inner_dis_contrib = kInnerDistContrib;
  double d_subdivision = kDistSubdivision;
  double d_linearity = kDistLinearity;
  bool use_adaptive_subdivision = kUseAdaptiveSubdivision;

  DECL_SERIALIZE() {
    constexpr int version = 6;
    SERIALIZE(version);
    SERIALIZE(floor);
    SERIALIZE(n_trajectory_steps);
    SERIALIZE(n_finger_steps);
    SERIALIZE(regularization);
    SERIALIZE(robot_collision);
    SERIALIZE(traj_energy);
    SERIALIZE(gripper_energy);
    SERIALIZE(geodesic_contrib);
    SERIALIZE(inner_dis_contrib);
    SERIALIZE(d_subdivision);
    SERIALIZE(d_linearity);
    SERIALIZE(use_adaptive_subdivision);
  }

  DECL_DESERIALIZE() {
    int version;
    double unused_double;
    int unused_int;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
    } else if (version == 2) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
      DESERIALIZE(unused_double);
    } else if (version == 3) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
      DESERIALIZE(unused_double);
      DESERIALIZE(unused_int);
    } else if (version == 4) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
      DESERIALIZE(unused_double);
      DESERIALIZE(unused_int);
      DESERIALIZE(regularization);
    } else if (version == 5) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
      DESERIALIZE(unused_double);
      DESERIALIZE(unused_int);
      DESERIALIZE(regularization);
      DESERIALIZE(robot_collision);
      DESERIALIZE(traj_energy);
      DESERIALIZE(gripper_energy);
      DESERIALIZE(geodesic_contrib);
      DESERIALIZE(inner_dis_contrib);
      DESERIALIZE(d_subdivision);
      DESERIALIZE(d_linearity);
    } else if (version == 6) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
      DESERIALIZE(regularization);
      DESERIALIZE(robot_collision);
      DESERIALIZE(traj_energy);
      DESERIALIZE(gripper_energy);
      DESERIALIZE(geodesic_contrib);
      DESERIALIZE(inner_dis_contrib);
      DESERIALIZE(d_subdivision);
      DESERIALIZE(d_linearity);
      DESERIALIZE(use_adaptive_subdivision);
    }
  }
};

inline std::ostream& operator<<(std::ostream& f, const CostSettings& c) {
  f << "CostSettings:\n"
    << "  n_traj_subs: " << c.n_trajectory_steps << "\n"
    << "  n_gripper_subs: " << c.n_finger_steps << "\n"
    << "  floor: " << c.floor << "\n"
    << "  traj_reg: " << c.regularization << "\n"
    << "  robot_collision: " << c.robot_collision << "\n"
    << "  traj_energy: " << c.traj_energy << "\n"
    << "  gripper_energy: " << c.gripper_energy << "\n"
    << "  geodesic_contrib: " << c.geodesic_contrib << "\n"
    << "  inner_dis_contrib: " << c.inner_dis_contrib << "\n"
    << "  d_subdivision: " << c.d_subdivision << "\n"
    << "  d_linearity: " << c.d_linearity << std::endl;
  return f;
}

}  // namespace psg
