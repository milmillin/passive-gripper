#pragma once

#include <imgui.h>

namespace psg {
namespace ui {

bool MyInputDoubleConvert(const char* name,
                          double* v,
                          double factor = 1,
                          double step = 0.001,
                          const char* fmt = "%.4f");

bool MyInputDouble3(const char* name,
                    double* v,
                    double step = 0.001,
                    const char* fmt = "%.4f");

bool MyInputDouble3Convert(const char* name,
                           double* v,
                           double factor = 1,
                           double step = 0.001,
                           const char* fmt = "%.4f");

bool MyButton(const char* label,
              const ImVec2& size = ImVec2(0, 0),
              bool disabled = false);

}  // namespace ui
}  // namespace psg