#pragma once

#include <map>
#include <string>
#include <vector>

#include "../PassiveGripper.h"

namespace psg {

class SettingsOverrider {
 private:
  std::map<std::string, std::string> mp;

 public:
  void Load(std::string fn);
  void Apply(PassiveGripper& psg) const;
};

}  // namespace psg