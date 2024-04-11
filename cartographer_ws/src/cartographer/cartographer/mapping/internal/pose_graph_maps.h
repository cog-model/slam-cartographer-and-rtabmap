#pragma once

#include "cartographer/mapping/proto/pose_graph.pb.h"

#include <string>
#include <map>
#include <set>

namespace cartographer {
namespace mapping {

class PoseGraphMaps {
public:
  bool ContainsTrajectory(int trajectory_id) const;
  void AddTrajectory(const std::string& map_name, int trajectory_id);
  void DeleteTrajectory(int trajectory_id);
  void MoveTrajectory(int trajectory_id, const std::string& new_map_name);
  void RenameMap(const std::string& old_map_name, const std::string& new_map_name);
  bool TrajectoriesBelongToTheSameMap(int trajectory_a, int trajectory_b) const;

  std::string GetMapName(int trajectory_id) const;

  void UpdateData(const std::map<std::string, std::set<int>>& data);
  std::map<std::string, std::set<int>> GetData() const;

private:
  bool HasMap(const std::string& map_name) const;

private:
  std::map<int, std::set<int>> map_id_to_trajectories_;
  std::map<int, int> trajectory_id_to_map_;

  std::map<int, std::string> map_id_to_name_;
  std::map<std::string, int> map_name_to_id_;
};

}
}
