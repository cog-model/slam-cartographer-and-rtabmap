#include <rtabmap/core/MapLimits.h>

namespace rtabmap {

proto::MapLimitsI toProto(const MapLimitsI& limits)
{
    proto::MapLimitsI proto;
    proto.set_min_x(limits.minX());
    proto.set_min_y(limits.minY());
    proto.set_max_x(limits.maxX());
    proto.set_max_y(limits.maxY());
    return proto;
}

MapLimitsI fromProto(const proto::MapLimitsI& proto)
{
    MapLimitsI limits(
        proto.min_x(),
        proto.min_y(),
        proto.max_x(),
        proto.max_y());
    return limits;
}

}