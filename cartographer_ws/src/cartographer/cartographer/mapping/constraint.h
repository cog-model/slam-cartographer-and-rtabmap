#pragma once

#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

struct Constraint {
  struct Pose {
    transform::Rigid3d zbar_ij;
    double translation_weight;
    double rotation_weight;
  };

  SubmapId submap_id;  // 'i' in the paper.
  NodeId node_id;      // 'j' in the paper.

  // Pose of the node 'j' relative to submap 'i'.
  Pose pose;

  enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
  float score;  // meaningful only for INTER_SUBMAP constraints
};

std::vector<Constraint> FromProto(
    const ::google::protobuf::RepeatedPtrField<proto::Constraint>&
        constraint_protos);
proto::Constraint ToProto(const Constraint& constraint);

}
}
