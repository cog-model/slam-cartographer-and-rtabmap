#pragma once

#include "kas_utils/collection.hpp"

#include <map>
#include <utility>

namespace kas_utils {

class TimeMeasurer : Collection<std::pair<double, double>> /* stamp, time */ {
public:
  TimeMeasurer(const std::string& name,
      bool print_results_on_destruction = false);
  ~TimeMeasurer();

  void start();
  void stop();

private:
  std::mutex mutex_;
  bool print_results_on_destruction_;

  std::map<pthread_t, std::pair<
      std::chrono::time_point<std::chrono::system_clock>,
      std::chrono::time_point<std::chrono::steady_clock>>> start_stamps_times_;
};

}

#define MEASURE_TIME_FROM_HERE(name) \
  static kas_utils::TimeMeasurer (time_measurer_ ## name)(#name, true); \
  (time_measurer_ ## name).start()

#define STOP_TIME_MEASUREMENT(name) \
  (time_measurer_ ## name).stop()

#define MEASURE_BLOCK_TIME(name) \
  static kas_utils::TimeMeasurer (time_measurer_ ## name)(#name, true); \
  class time_measurer_stop_trigger_class_ ## name { \
  public: \
    (time_measurer_stop_trigger_class_ ## name)() {}; \
    (~time_measurer_stop_trigger_class_ ## name)() {(time_measurer_ ## name).stop();}; \
  }; \
  time_measurer_stop_trigger_class_ ## name    time_measurer_stop_trigger_ ## name; \
  (time_measurer_ ## name).start()
