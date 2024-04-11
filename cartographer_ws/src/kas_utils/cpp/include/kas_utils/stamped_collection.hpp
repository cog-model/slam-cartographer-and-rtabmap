#pragma once

#include "kas_utils/collection.hpp"

#include <vector>
#include <string>
#include <mutex>
#include <functional>
#include <cstdlib>
#include <fstream>
#include <ctime>
#include <iostream>

namespace kas_utils {

template <typename T>
class StampedCollection: Collection<std::pair<double, T>> {
public:
  template <typename U, typename V, typename C>
  StampedCollection(const std::string& name,
      U&& printSummary = nullptr,
      V&& headerToOut = nullptr, C&& observationToOut = nullptr) :
    Collection<std::pair<double, T>>(name, std::forward<U>(printSummary),
        std::forward<V>(headerToOut), std::forward<C>(observationToOut)) {};
  ~StampedCollection() = default;

  template <typename U>
  void add(U&& observation) {
    auto stamp = std::chrono::system_clock::now();
    double stampSec = toSeconds(stamp.time_since_epoch());
    Collection<std::pair<double, T>>::add(
        std::make_pair(stampSec, std::forward<U>(observation)));
  }
};
  
}
