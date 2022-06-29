#pragma once

#include <functional>
#include <string>

#include <passive-gripper/models/SettingsOverrider.h>
#include "Result.h"

typedef std::function<void(size_t, size_t, const Result&)> TestcaseCallback;

void ProcessFrom(std::string raw_fn,
                 std::string output_dir,
                 size_t i_cp,
                 size_t need,
                 size_t maxiters,
                 const psg::SettingsOverrider& stgo,
                 const TestcaseCallback& cb);
