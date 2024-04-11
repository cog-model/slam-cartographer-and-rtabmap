/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/common/configuration_file_resolver.h"

#include <fstream>
#include <iostream>
#include <streambuf>

#include "cartographer/common/config.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

ConfigurationFileResolver::ConfigurationFileResolver(
    const std::vector<std::string>& configuration_files_directories) {
  configuration_files_directories_.push_back("");
  configuration_files_directories_.insert(configuration_files_directories_.end(),
      configuration_files_directories.begin(), configuration_files_directories.end());
  configuration_files_directories_.push_back(kConfigurationFilesDirectory);

  for (auto it = configuration_files_directories_.begin() + 1;
        it != configuration_files_directories_.end(); ++it) {
    if (it->empty()) {
      *it = "./";
    } else {
      *it += '/';
    }
  }
}

std::string ConfigurationFileResolver::GetFullPathOrDie(
    const std::string& file) {
  for (const auto& path : configuration_files_directories_) {
    const std::string filename = path + file;
    std::ifstream stream(filename.c_str());
    if (stream.good()) {
      LOG(INFO) << "Found '" << filename << "' for '" << file << "'.";
      return filename;
    }
    if (file[0] == '/') {
      break;
    }
  }
  LOG(FATAL) << "File '" << file << "' was not found.";
}

std::string ConfigurationFileResolver::GetFileContentOrDie(
    const std::string& file) {
  CHECK(!file.empty()) << "File name cannot be empty." << file;
  const std::string filename = GetFullPathOrDie(file);
  std::ifstream stream(filename.c_str());
  return std::string((std::istreambuf_iterator<char>(stream)),
                     std::istreambuf_iterator<char>());
}

}  // namespace common
}  // namespace cartographer
