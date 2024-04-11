#include <rtabmap/core/SensorData.h>

namespace rtabmap {

proto::SensorData toProto(const SensorData& sensorData)
{
    proto::SensorData proto;
    UASSERT_MSG(sensorData.numImages() == 0, "Do not support images.");
    if (sensorData.hasLaserScan())
    {
        *proto.mutable_laser_scan() = toProto(sensorData.laserScan());
    }
    return proto;
}

SensorData fromProto(const proto::SensorData& proto)
{
    SensorData sensorData;
    if (proto.has_laser_scan())
    {
        sensorData.setLaserScan(fromProto(proto.laser_scan()));
    }
    return sensorData;
}

}