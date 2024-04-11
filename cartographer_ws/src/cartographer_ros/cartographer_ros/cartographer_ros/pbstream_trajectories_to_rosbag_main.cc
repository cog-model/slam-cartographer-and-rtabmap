#include <iostream>
#include <string>
#include <vector>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/transform/transform.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "rosbag/bag.h"
#include "geometry_msgs/TransformStamped.h"

DEFINE_string(input, "", "pbstream file to process");
DEFINE_string(output, "", "output bag file");
DEFINE_string(global_parent_frame, "map", "frame id to use as parent frame for global poses");
DEFINE_string(local_parent_frame, "odom", "frame id to use as parent frame for local poses");
DEFINE_string(tracking_frame, "", "frame id to use as child frame");

namespace cartographer_ros {
namespace {

#define NODE_DATA 4

geometry_msgs::TransformStamped to_transform_stamped(
    int64_t timestamp_uts, const std::string& parent_frame,
    const std::string& child_frame,
    const cartographer::transform::proto::Rigid3d& parent_T_child) {
  static int64_t seq = 0;
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.seq = ++seq;
  transform_stamped.header.frame_id = parent_frame;
  transform_stamped.header.stamp = cartographer_ros::ToRos(
      ::cartographer::common::FromUniversal(timestamp_uts));
  transform_stamped.child_frame_id = child_frame;
  transform_stamped.transform = cartographer_ros::ToGeometryMsgTransform(
      ::cartographer::transform::ToRigid3(parent_T_child));
  return transform_stamped;
}

void pbstream_trajectories_to_rosbag(const std::string& pbstream_filename,
                                  const std::string& output_bag_filename,
                                  const std::string& global_parent_frame,
                                  const std::string& local_parent_frame,
                                  const std::string& tracking_frame) {
  // open map
  cartographer::io::ProtoStreamReader map_reader(pbstream_filename);
  cartographer::io::ProtoStreamDeserializer map_deserializer(&map_reader);

  // open rosbag
  rosbag::Bag bag(output_bag_filename, rosbag::bagmode::Write);

  // write global (optimized) poses
  const cartographer::mapping::proto::PoseGraph& pose_graph = map_deserializer.pose_graph();
  for (const auto trajectory : pose_graph.trajectory()) {
    for (const auto& node : trajectory.node()) {
      std::string topic = std::string("/global_trajectory_") + std::to_string(trajectory.trajectory_id());
      const auto& transform_stamped = to_transform_stamped(node.timestamp(), global_parent_frame, tracking_frame, node.pose());
      bag.write(topic, transform_stamped.header.stamp, transform_stamped);
    }
  }

  // write local poses
  cartographer::mapping::proto::SerializedData data;
  bool read_ok = map_deserializer.ReadNextSerializedData(&data);
  while (read_ok) {
    if (data.data_case() == NODE_DATA) {
      auto node_data = data.node().node_data();
      std::string topic = std::string("/local_trajectory_") + std::to_string(data.node().node_id().trajectory_id());
      const auto& transform_stamped = to_transform_stamped(node_data.timestamp(), local_parent_frame, tracking_frame, node_data.local_pose());
      bag.write(topic, transform_stamped.header.stamp, transform_stamped);
    }
    read_ok = map_deserializer.ReadNextSerializedData(&data);
  }

  // close rosbag
  bag.close();
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char* argv[]) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(!FLAGS_input.empty()) << "-input pbstream is missing.";
  CHECK(!FLAGS_output.empty()) << "-output is missing.";
  CHECK(!FLAGS_tracking_frame.empty()) << "-tracking_frame is missing.";

  cartographer_ros::pbstream_trajectories_to_rosbag(FLAGS_input, FLAGS_output,
                                                    FLAGS_global_parent_frame,
                                                    FLAGS_local_parent_frame,
                                                    FLAGS_tracking_frame);
  return 0;
}
