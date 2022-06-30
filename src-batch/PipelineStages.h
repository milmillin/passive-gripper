#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <passive-gripper/PassiveGripper.h>

namespace psg_batch {

// Generates a PSG file from an STL
void GeneratePSG(const std::string& stl_fn, const std::string& psg_fn);

// Generates a CPX file containing a ranked list of grasp configurations.
void GenerateCPX(const psg::PassiveGripper& psg,
                 const std::string& cpx_fn);

}  // namespace psg_batch
