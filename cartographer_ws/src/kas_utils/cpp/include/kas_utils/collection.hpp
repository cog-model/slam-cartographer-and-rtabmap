#pragma once

#include "kas_utils/utils.h"

#include <vector>
#include <string>
#include <mutex>
#include <functional>
#include <cstdlib>
#include <fstream>
#include <ctime>
#include <iostream>
#include <string.h>
#include <filesystem>

namespace kas_utils {

template <typename T>
class Collection {
public:
  template <typename U, typename V, typename C>
  Collection(const std::string& name,
      U&& printSummary = nullptr,
      V&& headerToOut = nullptr, C&& observationToOut = nullptr) :
    name_(name),
    abbreviation_("COL"),
    printSummary_(std::forward<U>(printSummary)),
    headerToOut_(std::forward<V>(headerToOut)),
    observationToOut_(std::forward<C>(observationToOut)),
    construction_time_(std::time(nullptr)) {};
  ~Collection();

  template <typename U>
  void add(U&& observation) {
    std::lock_guard<std::mutex> lock(mutex_);
    observations_.emplace_back(std::forward<U>(observation));
  }

  template <typename U>
  void setPrintSummary(U&& printSummary) {
    std::lock_guard<std::mutex> lock(mutex_);
    printSummary_ = std::forward<U>(printSummary);
  }

  template <typename U>
  void setHeaderToOut(U&& headerToOut) {
    std::lock_guard<std::mutex> lock(mutex_);
    headerToOut_ = std::forward<U>(headerToOut);
  }

  template <typename U>
  void setObservationToOut(U&& observationToOut) {
    std::lock_guard<std::mutex> lock(mutex_);
    observationToOut_ = std::forward<U>(observationToOut);
  }

protected:
  template <typename U, typename V, typename C>
  Collection(const std::string& name, const std::string& abbreviation,
      U&& printSummary = nullptr,
      V&& headerToOut = nullptr, C&& observationToOut = nullptr) :
    name_(name),
    abbreviation_(abbreviation),
    printSummary_(std::forward<U>(printSummary)),
    headerToOut_(std::forward<V>(headerToOut)),
    observationToOut_(std::forward<C>(observationToOut)),
    construction_time_(std::time(nullptr)) {};

private:
  std::string getOutLogFile();

private:
  std::mutex mutex_;
  std::string name_;
  std::string abbreviation_;
  std::time_t construction_time_;

  std::function<void(const std::string&, const std::vector<T>&)> printSummary_;
  std::function<void(std::ostream&)> headerToOut_;
  std::function<void(std::ostream&, const T&)> observationToOut_;

  std::vector<T> observations_;
};

template <typename T>
Collection<T>::~Collection() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (printSummary_) {
    printSummary_(name_, observations_);
  }

  if (observationToOut_) {
    std::string out_log_file = getOutLogFile();
    if (out_log_file.size()) {
      std::ofstream output(out_log_file.c_str(), std::ios::out);
      if (output.is_open()) {
        bool first_line = true;
        if (headerToOut_) {
          headerToOut_(output);
          first_line = false;
        }
        for (const T& observation : observations_) {
          if (!first_line) {
            output << '\n';
          }
          observationToOut_(output, observation);
          first_line = false;
        }
        output.close();
      } else {
        std::cout << "Cound not open file " << out_log_file <<
            " to save logs from time measurer '"  << name_ << "'.\n";
      }
    }
  }
}

template <typename T>
std::string Collection<T>::getOutLogFile() {
  std::string out_log_file;
  std::string out_log_file_env_name = name_ + "_" + abbreviation_ + "_LOG_FILE";
  const char* out_log_file_c = std::getenv(out_log_file_env_name.c_str());
  if (out_log_file_c != nullptr && strlen(out_log_file_c) > 0) {
    out_log_file = out_log_file_c;
  } else {
    std::string out_log_folder_env_name = name_ + "_" + abbreviation_ + "_LOG_FOLDER";
    const char* out_log_folder_c = std::getenv(out_log_folder_env_name.c_str());
    if (out_log_folder_c != nullptr && strlen(out_log_folder_c) > 0) {
      bool success = true;
      if (!std::filesystem::is_directory(out_log_folder_c)) {
        if (std::filesystem::exists(out_log_folder_c)) {
          success = false;
        } else {
          success = std::filesystem::create_directories(out_log_folder_c);
        }
      }
      if (success) {
        char time_str_c[sizeof("yyyy-mm-dd_hh.mm.ss")];
        std::strftime(time_str_c, sizeof(time_str_c), "%Y-%m-%d_%H.%M.%S", std::localtime(&construction_time_));
        out_log_file = std::string(out_log_folder_c) + "/" + time_str_c + "_" + name_ + ".txt";
      } else {
        std::cout << "Cloud not create directory " << out_log_folder_c << " for logs.\n";
      }
    }
  }
  return out_log_file;
}
  
}
