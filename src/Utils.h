#pragma once

#include <string>

namespace utils {

using std::string;

inline string error(string message, string file, unsigned line) {
  return file + ":" + std::to_string(line) + " " + message;
}

#define ERROR_MESSAGE(message) utils::error(message, __FILE__, __LINE__)

}  // namespace utils