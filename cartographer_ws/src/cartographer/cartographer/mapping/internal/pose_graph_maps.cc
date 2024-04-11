#include "cartographer/mapping/internal/pose_graph_maps.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

bool PoseGraphMaps::ContainsTrajectory(int trajectory_id) const {
  return trajectory_id_to_map_.count(trajectory_id);
}

bool PoseGraphMaps::HasMap(const std::string& map_name) const {
  return map_name_to_id_.count(map_name);
}

void PoseGraphMaps::AddTrajectory(const std::string& map_name, int trajectory_id) {
  CHECK(!ContainsTrajectory(trajectory_id));
  int map_id;
  if (map_name_to_id_.count(map_name)) {
    map_id = map_name_to_id_.at(map_name);
    CHECK(map_id_to_name_.count(map_id));
  } else {
    map_id = map_name_to_id_.size();
    CHECK(map_id_to_name_.count(map_id) == 0);
    map_id_to_name_[map_id] = map_name;
    map_name_to_id_[map_name] = map_id;
  }
  map_id_to_trajectories_[map_id].insert(trajectory_id);
  trajectory_id_to_map_[trajectory_id] = map_id;
}

void PoseGraphMaps::DeleteTrajectory(int trajectory_id) {
  CHECK(ContainsTrajectory(trajectory_id));
  int map_id = trajectory_id_to_map_.at(trajectory_id);
  trajectory_id_to_map_.erase(trajectory_id);
  map_id_to_trajectories_.at(map_id).erase(trajectory_id);
}

void PoseGraphMaps::MoveTrajectory(
    int trajectory_id, const std::string& new_map_name) {
  DeleteTrajectory(trajectory_id);
  AddTrajectory(new_map_name, trajectory_id);
}

void PoseGraphMaps::RenameMap(
    const std::string& old_map_name, const std::string& new_map_name) {
  CHECK(HasMap(old_map_name));
  int map_id = map_name_to_id_.at(old_map_name);
  map_name_to_id_.erase(old_map_name);
  map_name_to_id_[new_map_name] = map_id;
  map_id_to_name_.at(map_id) = new_map_name;
}

bool PoseGraphMaps::TrajectoriesBelongToTheSameMap(
    int trajectory_a, int trajectory_b) const {
  CHECK(ContainsTrajectory(trajectory_a));
  CHECK(ContainsTrajectory(trajectory_b));
  return trajectory_id_to_map_.at(trajectory_a) ==
      trajectory_id_to_map_.at(trajectory_b);
}

std::string PoseGraphMaps::GetMapName(int trajectory_id) const {
  CHECK(ContainsTrajectory(trajectory_id));
  int map_id = trajectory_id_to_map_.at(trajectory_id);
  return map_id_to_name_.at(map_id);
}

void PoseGraphMaps::UpdateData(const std::map<std::string, std::set<int>>& data) {
  for (const auto& [map_name, trajectory_ids] : data) {
    for (int trajectory_id : trajectory_ids) {
      if (ContainsTrajectory(trajectory_id)) {
        MoveTrajectory(trajectory_id, map_name);
      } else {
        AddTrajectory(map_name, trajectory_id);
      }
    }
  }
}

std::map<std::string, std::set<int>> PoseGraphMaps::GetData() const {
  std::map<std::string, std::set<int>> data;
  for (const auto& [map_id, trajectory_ids] : map_id_to_trajectories_) {
    data[map_id_to_name_.at(map_id)] = trajectory_ids;
  }
  return data;
}

}
}
