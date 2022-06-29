#pragma once

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>

inline std::ostream& LogImpl(const char* type) {
  static thread_local char buf[64];
  std::time_t t = std::time(nullptr);
  std::strftime(buf, 64, "%F %T", std::localtime(&t));
  return std::cerr << "[" << buf << "][" << type << "] ";
}

static std::ostream* out = &std::cout;

inline std::ostream& Log() {
  return LogImpl("info");
}
inline std::ostream& Error() {
  return LogImpl("error");
}
inline void SetOutStream(std::ostream& o) {
  out = &o;
}
inline std::ostream& Out() {
  static thread_local char buf[64];
  std::time_t t = std::time(nullptr);
  std::strftime(buf, 64, "%F %T", std::localtime(&t));
  *out << "[" << buf << "] ";
  return *out;
}

inline std::string ToString(double v) {
  std::ostringstream ss;
  ss << std::setprecision(8) << std::scientific << v;
  return ss.str();
}
