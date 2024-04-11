#include "kas_utils/stamped_collection.hpp"

#include <unistd.h>
#include <iostream>
#include <iomanip>

int main() {
  kas_utils::StampedCollection<double> coll("test_stamped_collection", nullptr, nullptr,
      [](std::ostream& out, std::pair<double, double> stamp_d) {
        out << std::fixed << std::setprecision(6) << stamp_d.first << ' ' << stamp_d.second;
      });
  coll.add(1);
  usleep(1000000);  // 1 sec
  coll.add(12.34);
}
