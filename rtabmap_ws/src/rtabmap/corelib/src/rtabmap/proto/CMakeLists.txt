SET(PROTO_FILES
    Time.proto
    Color.proto
    Transform.proto
    MapLimits.proto
    LocalMap.proto
    OccupancyGridMap.proto

    LaserScan.proto
    SensorData.proto
    Trajectory.proto
    RawData.proto
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
SET(PROTO_SRCS ${PROTO_SRCS} PARENT_SCOPE)
SET(PROTO_HDRS ${PROTO_HDRS} PARENT_SCOPE)
ADD_CUSTOM_TARGET(generated_protobuf_code ALL DEPENDS ${PROTO_SRCS} ${PROTO_HDRS})
