#pragma once

#include <string>
#include <chrono>

namespace kas_utils {

template <typename T>
double toSeconds(const T& duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

}