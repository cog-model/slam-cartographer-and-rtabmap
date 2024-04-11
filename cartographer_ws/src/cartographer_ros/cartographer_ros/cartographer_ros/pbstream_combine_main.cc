#include <iostream>
#include <string>
#include <set>
#include <vector>

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/null_points_processor.h"
#include "cartographer/io/pcd_writing_points_processor.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"

#include "cartographer_ros/node_options.h"

DEFINE_string(input_1, "", "pbstream file to process");
DEFINE_string(input_2, "", "pbstream file to process");
DEFINE_bool(load_frozen_state_1, false, "Load the first pbstream state as frozen (non-optimized) trajectories.");
DEFINE_string(config, "", "config file to build pose graph");
DEFINE_string(output, "", "output pbstream file");

namespace cartographer_ros {
namespace {

typedef cartographer::mapping::MapById<cartographer::mapping::NodeId, cartographer::mapping::TrajectoryNode> NodePoses;
typedef cartographer::mapping::MapById<cartographer::mapping::SubmapId, cartographer::mapping::PoseGraphInterface::SubmapPose> SubmapPoses;

void pbstream_combine(const std::string& pbstream_filename_1,
                      const std::string& pbstream_filename_2,
                      bool load_frozen_state_1,
                      const std::string& config_filename,
                      const std::string& output_pbstream_filename) {
  // create map builder
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) = LoadOptions(config_filename);
  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  
  // load maps
  cartographer::io::ProtoStreamReader map_reader_1(pbstream_filename_1);
  std::vector<cartographer::mapping::proto::TrajectoryRosOptions> trajectory_ros_options_1;
  std::map<int, int> trajectory_remapping_1 =
      map_builder->LoadState(&map_reader_1, load_frozen_state_1, &trajectory_ros_options_1);
  CHECK(trajectory_remapping_1.size() == 1);

  cartographer::io::ProtoStreamReader map_reader_2(pbstream_filename_2);
  std::vector<cartographer::mapping::proto::TrajectoryRosOptions> trajectory_ros_options_2;
  std::map<int, int> trajectory_remapping_2 =
      map_builder->LoadState(&map_reader_2, false, &trajectory_ros_options_2);
  CHECK(trajectory_remapping_2.size() == 1);

  // find global constraint
  cartographer::mapping::PoseGraph3D* pose_graph_3d =
      dynamic_cast<cartographer::mapping::PoseGraph3D*>(map_builder->pose_graph());
  CHECK(pose_graph_3d);
  const NodePoses& nodes = pose_graph_3d->GetTrajectoryNodes();
  const SubmapPoses& submaps = pose_graph_3d->GetAllSubmapPoses();
  for (const auto& submap_id_data : submaps.trajectory(0)) {
    for (const auto& node_id_data : nodes.trajectory(1)) {
      cartographer::mapping::NodeId node_id = node_id_data.id;
      cartographer::mapping::SubmapId submap_id = submap_id_data.id;
      pose_graph_3d->AddWorkItem([pose_graph_3d, node_id, submap_id](){
        pose_graph_3d->ComputeConstraint(node_id, submap_id, false, true);
        return cartographer::mapping::WorkItem::Result::kDoNotRunOptimization;
      });
    }
    float progress = 1.0f * submap_id_data.id.submap_index / std::prev(submaps.trajectory(0).end())->id.submap_index;
    std::cout << progress * 100.0f << " %\n";
  }
  pose_graph_3d->AddWorkItem([pose_graph_3d](){
    pose_graph_3d->NotifyEndOfNode();
    return cartographer::mapping::WorkItem::Result::kRunOptimization;
  });
  pose_graph_3d->IncNumTrajectoryNodes();
  pose_graph_3d->WaitForAllComputations();

  for (const auto& submap_id_data : submaps.trajectory(0)) {
    for (const auto& node_id_data : nodes.trajectory(1)) {
      cartographer::mapping::NodeId node_id = node_id_data.id;
      cartographer::mapping::SubmapId submap_id = submap_id_data.id;
      pose_graph_3d->AddWorkItem([pose_graph_3d, node_id, submap_id](){
        pose_graph_3d->ComputeConstraint(node_id, submap_id, true, false);
        return cartographer::mapping::WorkItem::Result::kDoNotRunOptimization;
      });
    }
    float progress = 1.0f * submap_id_data.id.submap_index / std::prev(submaps.trajectory(0).end())->id.submap_index;
    std::cout << progress * 100.0f << " %\n";
  }
  pose_graph_3d->AddWorkItem([pose_graph_3d](){
    pose_graph_3d->NotifyEndOfNode();
    return cartographer::mapping::WorkItem::Result::kRunOptimization;
  });
  pose_graph_3d->IncNumTrajectoryNodes();
  pose_graph_3d->WaitForAllComputations();

  pose_graph_3d->RunFinalOptimization();
  map_builder->SerializeStateToFile(true, output_pbstream_filename);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char* argv[]) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(!FLAGS_input_1.empty()) << "-input_1 pbstream is missing.";
  CHECK(!FLAGS_input_2.empty()) << "-input_2 pbstream is missing.";
  CHECK(!FLAGS_config.empty()) << "-config file is missing.";
  CHECK(!FLAGS_output.empty()) << "-output is missing.";

  cartographer_ros::pbstream_combine(FLAGS_input_1, FLAGS_input_2, FLAGS_load_frozen_state_1,
      FLAGS_config, FLAGS_output);
  return 0;
}
