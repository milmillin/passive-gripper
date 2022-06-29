#pragma once

#include <nlopt.h>

#include "../Constants.h"
#include "../serialization/Serialization.h"

namespace psg {

struct OptSettings : psg::serialization::Serializable {
  double max_runtime = 0;  // seconds
  size_t max_iters = 300000;
  double finger_wiggle = 0.01;
  Pose trajectory_wiggle = (Pose() << 5. * kDegToRad,
                            5. * kDegToRad,
                            5. * kDegToRad,
                            45. * kDegToRad,
                            25. * kDegToRad,
                            90. * kDegToRad)
                               .finished();
  double tolerance = 0;
  nlopt_algorithm algorithm = NLOPT_GN_CRS2_LM;
  size_t population = 30000;

  DECL_SERIALIZE() {
    constexpr int version = 3;
    SERIALIZE(version);
    SERIALIZE(max_runtime);
    SERIALIZE(max_iters);
    SERIALIZE(finger_wiggle);
    SERIALIZE(trajectory_wiggle);
    SERIALIZE(tolerance);
    SERIALIZE(algorithm);
    SERIALIZE(population);
  }

  DECL_DESERIALIZE() {
    double __unused;
    int version = 1;
    DESERIALIZE(version);
    if (version == 1) {
      DESERIALIZE(max_runtime);
      DESERIALIZE(finger_wiggle);
      DESERIALIZE(trajectory_wiggle);
      DESERIALIZE(tolerance);
      DESERIALIZE(algorithm);
      DESERIALIZE(population);
    } else if (version == 2) {
      DESERIALIZE(max_runtime);
      DESERIALIZE(__unused);  // double
      max_iters = __unused;
      DESERIALIZE(finger_wiggle);
      DESERIALIZE(trajectory_wiggle);
      DESERIALIZE(tolerance);
      DESERIALIZE(algorithm);
      DESERIALIZE(population);
    } else if (version == 3) {
      DESERIALIZE(max_runtime);
      DESERIALIZE(max_iters);
      DESERIALIZE(finger_wiggle);
      DESERIALIZE(trajectory_wiggle);
      DESERIALIZE(tolerance);
      DESERIALIZE(algorithm);
      DESERIALIZE(population);
    } else if (version == 4) {
      DESERIALIZE(max_runtime);
      DESERIALIZE(max_iters);
      DESERIALIZE(finger_wiggle);
      DESERIALIZE(trajectory_wiggle);
      DESERIALIZE(tolerance);
      DESERIALIZE(algorithm);
      DESERIALIZE(population);
      int unused;
      DESERIALIZE(unused);
    }
  }
};

inline std::ostream& operator<<(std::ostream& f, const OptSettings& c) {
  f << "OptSettings:\n"
    << "  max_runtime: " << c.max_runtime << "\n"
    << "  max_iters: " << c.max_iters << "\n"
    << "  finger_wiggle: " << c.finger_wiggle << "\n"
    << "  trajectory_wiggle: " << c.trajectory_wiggle.transpose() << "\n"
    << "  tolerance: " << c.tolerance << "\n"
    << "  algorithm: " << psg::labels::kAlgorithms[c.algorithm] << "\n"
    << "  population: " << c.population << std::endl;
  return f;
}

}  // namespace psg
