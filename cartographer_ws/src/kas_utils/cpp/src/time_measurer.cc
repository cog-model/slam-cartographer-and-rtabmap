#include "kas_utils/utils.h"
#include "kas_utils/time_measurer.h"

#include <sstream>
#include <iomanip>

namespace kas_utils {

static void printSummary(const std::string& name,
    const std::vector<std::pair<double, double>>& observations,
    int number_of_threads) {
  if (observations.size() > 0) {
    double total_measured_time = 0.;
    double avarage_time = 0.;
    double max_time = 0;
    for (const auto& observation : observations) {
      double time = observation.second;
      total_measured_time += time;
      max_time = std::max(max_time, time);
    }
    avarage_time = total_measured_time / observations.size();

    std::string log_string;
    log_string += name + ":\n";
    log_string += "    Number of measurements: " + std::to_string(observations.size()) + "\n";
    log_string += "    Total measured time: " + std::to_string(total_measured_time) + "\n";
    log_string += "    Average time: " + std::to_string(avarage_time) + "\n";
    log_string += "    Max time: " + std::to_string(max_time) + "\n";
    log_string += "    Number of threads: " + std::to_string(number_of_threads) + "\n";

    std::cout << log_string;
  }
}

static void observationToOut(std::ostream& out,
    const std::pair<double, double>& stamp_time) {
  double stamp = stamp_time.first;
  double time = stamp_time.second;
  out << std::fixed << std::setprecision(6) << stamp << ' ' << time;
}

TimeMeasurer::TimeMeasurer(const std::string& name,
    bool print_results_on_destruction /* false */) :
  Collection<std::pair<double, double>>(name, "TM",
      nullptr /* will be set in destructor */, nullptr, observationToOut),
  print_results_on_destruction_(print_results_on_destruction) {}

void TimeMeasurer::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& [start_stamp, start_time] = start_stamps_times_[pthread_self()];
  start_stamp = std::chrono::system_clock::now();
  start_time = std::chrono::steady_clock::now();
}

void TimeMeasurer::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  auto stop_time = std::chrono::steady_clock::now();
  const auto& [start_stamp, start_time] = start_stamps_times_.at(pthread_self());
  double stamp = toSeconds(start_stamp.time_since_epoch());
  double time = toSeconds(stop_time - start_time);
  add(std::make_pair(stamp, time));
}

TimeMeasurer::~TimeMeasurer() {
  if (print_results_on_destruction_) {
    std::lock_guard<std::mutex> lock(mutex_);
    setPrintSummary(
      [number_of_thread = start_stamps_times_.size()](const std::string& name,
          const std::vector<std::pair<double, double>>& observations) {
        printSummary(name, observations, number_of_thread);
      }
    );
  }
}

}
