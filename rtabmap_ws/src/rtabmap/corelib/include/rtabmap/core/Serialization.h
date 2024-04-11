#pragma once

#include <optional>
#include <string>
#include <fstream>
#include <cinttypes>

#include <google/protobuf/message.h>
#include <rtabmap/proto/OccupancyGridMap.pb.h>
#include <rtabmap/proto/RawData.pb.h>

namespace rtabmap {

class MapSerialization
{
public:
    MapSerialization(const std::string& fileName);

    void write(const google::protobuf::Message& proto);
    void close();

private:
    std::ofstream output_;
};

class MapDeserialization
{
public:
    MapDeserialization(const std::string& fileName);

    const proto::OccupancyGridMap::MetaData& metaData();
    std::optional<proto::OccupancyGridMap::Node> readNode();
    void close();

private:
    void readMetaData();
    std::string readString();

private:
    std::ifstream input_;
    proto::OccupancyGridMap::MetaData metaData_;
};

class RawDataSerialization
{
public:
    RawDataSerialization(const std::string& fileName);

    void write(const google::protobuf::Message& proto);
    void close();

private:
    std::ofstream output_;
};

class RawDataDeserialization
{
public:
    RawDataDeserialization(const std::string& fileName);

    std::optional<proto::RawData> readRawData();
    void close();

private:
    std::string readString();

private:
    std::ifstream input_;
};

}
