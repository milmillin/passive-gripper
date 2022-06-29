#pragma once

#include "../serialization/Serialization.h"

namespace psg {

struct CostSettings : psg::serialization::Serializable {
  double floor = 0.0075;
  size_t n_trajectory_steps = 768;
  size_t n_finger_steps = 128;
  double ang_velocity = kDegToRad * 60.;
  CostFunctionEnum cost_function = CostFunctionEnum::kSP;
  double regularization = 1e-6;
  double robot_collision = 1000;
  double traj_energy = 0.;
  double gripper_energy = 1.;
  double geodesic_contrib = 0.;
  double inner_dis_contrib = 1.;
  double d_subdivision = 0.001;
  double d_linearity = 0.001;

  DECL_SERIALIZE() {
    constexpr int version = 5;
    SERIALIZE(version);
    SERIALIZE(floor);
    SERIALIZE(n_trajectory_steps);
    SERIALIZE(n_finger_steps);
    SERIALIZE(ang_velocity);
    SERIALIZE(cost_function);
    SERIALIZE(regularization);
    SERIALIZE(robot_collision);
    SERIALIZE(traj_energy);
    SERIALIZE(gripper_energy);
    SERIALIZE(geodesic_contrib);
    SERIALIZE(inner_dis_contrib);
    SERIALIZE(d_subdivision);
    SERIALIZE(d_linearity);
  }

  DECL_DESERIALIZE() {
    int version;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
    } else if (version == 2) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
      DESERIALIZE(ang_velocity);
    } else if (version == 3) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
      DESERIALIZE(ang_velocity);
      DESERIALIZE(cost_function);
    } else if (version == 4) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
      DESERIALIZE(ang_velocity);
      DESERIALIZE(cost_function);
      DESERIALIZE(regularization);
    } else if (version == 5) {
      DESERIALIZE(floor);
      DESERIALIZE(n_trajectory_steps);
      DESERIALIZE(n_finger_steps);
      DESERIALIZE(ang_velocity);
      DESERIALIZE(cost_function);
      DESERIALIZE(regularization);
      DESERIALIZE(robot_collision);
      DESERIALIZE(traj_energy);
      DESERIALIZE(gripper_energy);
      DESERIALIZE(geodesic_contrib);
      DESERIALIZE(inner_dis_contrib);
      DESERIALIZE(d_subdivision);
      DESERIALIZE(d_linearity);
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
