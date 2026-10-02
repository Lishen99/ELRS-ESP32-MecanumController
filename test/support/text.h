// Checking text output in tests.
#pragma once

#include <string>

inline bool contains(const std::string& haystack, const char* needle) {
  return haystack.find(needle) != std::string::npos;
}
