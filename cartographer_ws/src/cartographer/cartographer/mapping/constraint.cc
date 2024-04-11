#include "cartographer/mapping/constraint.h"

#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

Constraint::Tag FromProto(
    const proto::Constraint::Tag& proto) {
  switch (proto) {
    case proto::Constraint::INTRA_SUBMAP:
      return Constraint::Tag::INTRA_SUBMAP;
    case proto::Constraint::INTER_SUBMAP:
      return Constraint::Tag::INTER_SUBMAP;
    case ::google::protobuf::kint32max:
    case ::google::protobuf::kint32min: {
    }
  }
  LOG(FATAL) << "Unsupported tag.";
}

std::vector<Constraint> FromProto(
    const ::google::protobuf::RepeatedPtrField<proto::Constraint>&
        constraint_protos) {
  std::vector<Constraint> constraints;
  bool warned_about_constraints_without_scores = false;
  for (const proto::Constraint& constraint_proto : constraint_protos) {
    const mapping::SubmapId submap_id{
        constraint_proto.submap_id().trajectory_id(),
        constraint_proto.submap_id().submap_index()};
    const mapping::NodeId node_id{
        constraint_proto.node_id().trajectory_id(),
        constraint_proto.node_id().node_index()};
    const Constraint::Pose pose{
        transform::ToRigid3(constraint_proto.relative_pose()),
        constraint_proto.translation_weight(),
        constraint_proto.rotation_weight()};
    const Constraint::Tag tag = FromProto(constraint_proto.tag());
    const float score = constraint_proto.score();
    if (tag == Constraint::Tag::INTER_SUBMAP && score == 0.0f &&
        !warned_about_constraints_without_scores) {
      LOG(WARNING) << "There are constraints with zero score in the map.";
      warned_about_constraints_without_scores = true;
    }
    constraints.push_back(Constraint{submap_id, node_id, pose, tag, score});
  }
  return constraints;
}

proto::Constraint::Tag ToProto(const Constraint::Tag& tag) {
  switch (tag) {
    case Constraint::Tag::INTRA_SUBMAP:
      return proto::Constraint::INTRA_SUBMAP;
    case Constraint::Tag::INTER_SUBMAP:
      return proto::Constraint::INTER_SUBMAP;
  }
  LOG(FATAL) << "Unsupported tag.";
}

proto::Constraint ToProto(const Constraint& constraint) {
  proto::Constraint constraint_proto;
  *constraint_proto.mutable_relative_pose() =
      transform::ToProto(constraint.pose.zbar_ij);
  constraint_proto.set_translation_weight(constraint.pose.translation_weight);
  constraint_proto.set_rotation_weight(constraint.pose.rotation_weight);
  constraint_proto.mutable_submap_id()->set_trajectory_id(
      constraint.submap_id.trajectory_id);
  constraint_proto.mutable_submap_id()->set_submap_index(
      constraint.submap_id.submap_index);
  constraint_proto.mutable_node_id()->set_trajectory_id(
      constraint.node_id.trajectory_id);
  constraint_proto.mutable_node_id()->set_node_index(
      constraint.node_id.node_index);
  constraint_proto.set_tag(ToProto(constraint.tag));
  constraint_proto.set_score(constraint.score);
  return constraint_proto;
}

}
}
