#pragma once

#include <iomanip>
#include <iostream>
#include <string>

#include <passive-gripper/Constants.h>

struct Result {
  std::string name;
  size_t cp_idx;
  std::string out_fn;
  bool failed;
  bool force_closure;
  bool partial_force_closure;
  double min_wrench;
  double partial_min_wrench;
  double cost;
  double min_dist;
  bool intersecting;
  double volume;
  double pi_volume;
  long long duration;
};

inline std::ostream& operator<<(std::ostream& f, const Result& r) {
  f << std::setprecision(5) << std::scientific;
  f << r.name << '\t' << r.cp_idx << '\t' << psg::kBoolStr[!r.failed] << '\t'
    << psg::kBoolStr[r.force_closure] << '\t'
    << psg::kBoolStr[r.partial_force_closure] << '\t' << r.min_wrench << '\t'
    << r.partial_min_wrench << '\t' << r.cost << '\t' << r.min_dist << '\t'
    << psg::kBoolStr[r.intersecting] << '\t' << r.volume << '\t' << r.pi_volume
    << '\t' << r.duration;
  return f;
}

struct ResultHeader {};

inline std::ostream& operator<<(std::ostream& f, const ResultHeader&) {
  f << "name\tidx\tsuccess\tfc\tpfc\tmw\tpmw\tcost\tdist\tintersecting\tvol\tpi"
       "_vol\ttime";
  return f;
}
