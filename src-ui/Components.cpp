// Copyright (c) 2022 The University of Washington and Contributors
//
// SPDX-License-Identifier: LicenseRef-UW-Non-Commercial

#include "Components.h"

#include <imgui_internal.h>

namespace psg {
namespace ui {

bool MyInputDoubleConvert(const char* name,
                          double* v,
                          double factor,
                          double step,
                          const char* fmt) {
  double conv = *v * factor;
  if (ImGui::InputDouble(name, &conv, step, step, fmt)) {
    *v = conv / factor;
    return true;
  }
  return false;
}

bool MyInputDouble3(const char* name, double* v, double step, const char* fmt) {
  float w = ImGui::GetContentRegionAvailWidth();
  float p = ImGui::GetStyle().FramePadding.x;
  bool updated = false;
  ImGui::PushID(name);
  ImGui::PushItemWidth((w - 2 * p) / 3.);
  updated |= ImGui::InputDouble("##x", &v[0], step, step, fmt);
  ImGui::SameLine(0, p);
  updated |= ImGui::InputDouble("##y", &v[1], step, step, fmt);
  ImGui::SameLine(0, p);
  updated |= ImGui::InputDouble("##z", &v[2], step, step, fmt);
  ImGui::PopItemWidth();
  ImGui::PopID();
  return updated;
}

bool MyInputDouble3Convert(const char* name,
                           double* v,
                           double factor,
                           double step,
                           const char* fmt) {
  double conv[3];
  conv[0] = v[0] * factor;
  conv[1] = v[1] * factor;
  conv[2] = v[2] * factor;
  float w = ImGui::GetContentRegionAvailWidth();
  float p = ImGui::GetStyle().FramePadding.x;
  bool updated = false;
  ImGui::PushID(name);
  ImGui::PushItemWidth((w - 2 * p) / 3.);
  updated |= ImGui::InputDouble("##x", &conv[0], step, step, fmt);
  ImGui::SameLine(0, p);
  updated |= ImGui::InputDouble("##y", &conv[1], step, step, fmt);
  ImGui::SameLine(0, p);
  updated |= ImGui::InputDouble("##z", &conv[2], step, step, fmt);
  ImGui::PopItemWidth();
  ImGui::PopID();
  if (updated) {
    v[0] = conv[0] / factor;
    v[1] = conv[1] / factor;
    v[2] = conv[2] / factor;
  }
  return updated;
}

bool MyButton(const char* label, const ImVec2& size, bool disabled) {
  if (disabled) {
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
  }
  bool res = ImGui::Button(label, size);
  if (disabled) {
    ImGui::PopStyleVar();
    ImGui::PopItemFlag();
  }
  return res;
}

}  // namespace ui
}  // namespace psg