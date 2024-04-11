/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_IO_SERIALIZATION_FORMAT_MIGRATION_H_
#define CARTOGRAPHER_IO_SERIALIZATION_FORMAT_MIGRATION_H_

#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/proto/serialization.pb.h"

namespace cartographer {
namespace io {

void MigrateStreamVersion(
    cartographer::io::ProtoStreamReaderInterface* const input,
    cartographer::io::ProtoStreamWriterInterface* const output,
    bool include_unfinished_submaps);

mapping::MapById<mapping::SubmapId, mapping::proto::Submap> AddHistogramsToSubmaps(
    const mapping::MapById<mapping::SubmapId, mapping::proto::Submap>& submap_id_to_submaps,
    const mapping::MapById<mapping::NodeId, mapping::proto::Node>& node_id_to_nodes,
    const mapping::proto::PoseGraph& pose_graph_proto);

mapping::MapById<mapping::NodeId, mapping::proto::Node> AddTravelledDistanceToNodes(
    const mapping::MapById<mapping::NodeId, mapping::proto::Node>& node_id_to_node);

mapping::MapById<mapping::NodeId, mapping::proto::Node> AddAccumRotationToNodes(
    const mapping::MapById<mapping::NodeId, mapping::proto::Node>& node_id_to_node);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_SERIALIZATION_FORMAT_MIGRATION_H_
