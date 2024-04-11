#include <iostream>
#include <string>
#include <set>
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

DEFINE_string(input, "", "pbstream file to process");
DEFINE_string(output, "", "output pcd file");
DEFINE_bool(use_local_poses, false, "use local (odometry) poses");
DEFINE_string(point_cloud_name, "low_resolution", "which point cloud to use: filtered_gravity_aligned, high_resolution or low_resolution");

namespace cartographer_ros {
namespace {

#define NODE_DATA 4

typedef cartographer::mapping::MapById<cartographer::mapping::NodeId, cartographer::transform::Rigid3f> NodePoses;

NodePoses get_global_node_poses(const cartographer::mapping::proto::PoseGraph& pose_graph) {
  NodePoses global_node_poses;
  for (const auto trajectory : pose_graph.trajectory()) {
    for (const auto& node : trajectory.node()) {
      cartographer::mapping::NodeId node_id(trajectory.trajectory_id(), node.node_index());
      cartographer::transform::Rigid3f node_pose = cartographer::transform::ToRigid3(node.pose()).cast<float>();
      global_node_poses.Insert(node_id, node_pose);
    }
  }
  return global_node_poses;
}

void pbstream_map_to_pcd(const std::string& pbstream_filename,
                         const std::string& output_pcd_filename,
                         const bool& use_local_poses,
                         const std::string& point_cloud_name) {
  std::set<std::string> avaliable_point_cloud_names = {"filtered_gravity_aligned", "high_resolution", "low_resolution"};
  CHECK(avaliable_point_cloud_names.find(point_cloud_name) != avaliable_point_cloud_names.end());

  // open map
  cartographer::io::ProtoStreamReader map_reader(pbstream_filename);
  cartographer::io::ProtoStreamDeserializer map_deserializer(&map_reader);

  // get global (optimized) node poses
  const NodePoses& global_node_poses = get_global_node_poses(map_deserializer.pose_graph());

  // prepare pcd_writer object to write points to file
  auto file_writer = std::make_unique<cartographer::io::StreamFileWriter>(output_pcd_filename);
  cartographer::io::NullPointsProcessor end_of_pipeline;
  cartographer::io::PcdWritingPointsProcessor pcd_writer(std::move(file_writer), &end_of_pipeline);

  // iterate over map data
  cartographer::mapping::proto::SerializedData data;
  bool read_ok = map_deserializer.ReadNextSerializedData(&data);
  while (read_ok) {
    if (data.data_case() == NODE_DATA) {
      auto node_data = data.node().node_data();
      cartographer::sensor::proto::CompressedPointCloud proto_point_cloud;
      if (point_cloud_name == "filtered_gravity_aligned") {
        proto_point_cloud = node_data.filtered_gravity_aligned_point_cloud();
      } else if (point_cloud_name == "high_resolution") {
        proto_point_cloud = node_data.high_resolution_point_cloud();
      } else if (point_cloud_name == "low_resolution") {
        proto_point_cloud = node_data.low_resolution_point_cloud();
      }
      cartographer::sensor::CompressedPointCloud compressed_point_cloud(proto_point_cloud);
      cartographer::sensor::PointCloud point_cloud(compressed_point_cloud.Decompress());
      cartographer::transform::Rigid3f node_pose;
      if (use_local_poses) {
        node_pose = cartographer::transform::ToRigid3(node_data.local_pose()).cast<float>();
      } else {
        cartographer::mapping::NodeId node_id(data.node().node_id().trajectory_id(), data.node().node_id().node_index());
        node_pose = global_node_poses.at(node_id);
      }
      point_cloud = cartographer::sensor::TransformPointCloud(point_cloud, node_pose);
      auto points_batch = std::make_unique<cartographer::io::PointsBatch>();
      points_batch->points = point_cloud.points();
      pcd_writer.Process(std::move(points_batch));
    }
    read_ok = map_deserializer.ReadNextSerializedData(&data);
  }
  pcd_writer.Flush();
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char* argv[]) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(!FLAGS_input.empty()) << "-input pbstream is missing.";
  CHECK(!FLAGS_output.empty()) << "-output is missing.";

  cartographer_ros::pbstream_map_to_pcd(FLAGS_input, FLAGS_output, FLAGS_use_local_poses, FLAGS_point_cloud_name);
  return 0;
}
