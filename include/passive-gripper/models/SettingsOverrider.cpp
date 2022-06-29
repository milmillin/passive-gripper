#include "SettingsOverrider.h"

#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../../utils.h"
#include "../PassiveGripper.h"

namespace psg {

void SettingsOverrider::Load(std::string fn) {
  std::ifstream f(fn);
  if (!f.is_open()) {
    throw std::invalid_argument("Cannot open file " + fn);
  }

  mp.clear();
  std::string key;
  std::string value;
  while (f >> key) {
    f >> value;
    mp.insert({key, value});
    Log() << key << ": " << value << std::endl;
    if (key == "algorithm") {
      Log() << "  > " << psg::labels::kAlgorithms[std::stoi(value)]
            << std::endl;
    }
  }
}

std::vector<std::string> Split(const std::string& s, char del = ';') {
  std::vector<std::string> result;
  std::string tmp = "";

  for (char c : s) {
    if (c == del) {
      result.push_back(tmp);
      tmp.clear();
    } else {
      tmp.push_back(c);
    }
  }
  result.push_back(tmp);

  return result;
}

void SettingsOverrider::Apply(PassiveGripper& psg) const {
  bool tmp_reinit_fingers = psg.reinit_fingers;
  bool tmp_reinit_trajectory = psg.reinit_trajectory;
  psg.reinit_fingers = false;
  psg.reinit_trajectory = false;

  OptSettings opt_settings = psg.GetOptSettings();
  TopoOptSettings topo_opt_settings = psg.GetTopoOptSettings();
  ContactSettings contact_settings = psg.GetContactSettings();
  CostSettings cost_settings = psg.GetCostSettings();
  bool opt_changed = false;
  bool topo_opt_changed = false;
  bool cost_settings_changed = false;
  bool contact_settings_changed = false;

  auto Contains = [this](const std::string& key, std::string& out_val) -> bool {
    auto it = mp.find(key);
    if (it == mp.end()) return false;
    out_val = it->second;
    return true;
  };

  std::string value;
  if (Contains("algorithm", value)) {
    opt_settings.algorithm = (nlopt_algorithm)std::stoi(value);
    opt_changed = true;
  }
  if (Contains("tolerance", value)) {
    opt_settings.tolerance = (nlopt_algorithm)std::stoi(value);
    opt_changed = true;
  }
  if (Contains("population", value)) {
    opt_settings.population = std::stoull(value);
    opt_changed = true;
  }
  if (Contains("max_runtime", value)) {
    opt_settings.max_runtime = std::stod(value);
    opt_changed = true;
  }
  if (Contains("max_iters", value)) {
    opt_settings.max_iters = std::stoull(value);
    opt_changed = true;
  }
  if (Contains("trajectory_wiggle", value)) {
    auto vec = Split(value);
    if (vec.size() != kNumDOFs) {
      Error() << "trajectory_wiggle does not contain " << kNumDOFs
              << " elements" << std::endl;
    } else {
      for (size_t i = 0; i < kNumDOFs; i++) {
        opt_settings.trajectory_wiggle(i) = kDegToRad * std::stod(vec[i]);
      }
      opt_changed = true;
    }
  }
  if (Contains("vol_frac", value)) {
    topo_opt_settings.vol_frac = std::stod(value);
    topo_opt_changed = true;
  }
  if (Contains("topo_res", value)) {
    topo_opt_settings.topo_res = std::stod(value);
    topo_opt_changed = true;
  }
  if (Contains("neg_vol_res", value)) {
    topo_opt_settings.neg_vol_res = std::stod(value);
    topo_opt_changed = true;
  }
  if (Contains("cost.floor", value)) {
    cost_settings.floor = std::stod(value);
    cost_settings_changed = true;
  }
  if (Contains("contact.floor", value)) {
    contact_settings.floor = std::stod(value);
    contact_settings_changed = true;
  }
  if (Contains("contact.max_angle", value)) {
    contact_settings.max_angle = kDegToRad * std::stod(value);
    contact_settings_changed = true;
  }

  if (opt_changed) psg.SetOptSettings(opt_settings);
  if (topo_opt_changed) psg.SetTopoOptSettings(topo_opt_settings);
  if (contact_settings_changed) psg.SetContactSettings(contact_settings);
  if (cost_settings_changed) psg.SetCostSettings(cost_settings);

  psg.reinit_fingers = tmp_reinit_fingers;
  psg.reinit_trajectory = tmp_reinit_trajectory;
}

}  // namespace psg
