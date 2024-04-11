#include <rtabmap/core/Serialization.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/ULogger.h>

#include <algorithm>

namespace rtabmap {

const std::string mapExtension = ".ocp";
const std::string rawDataExtension = ".rd";
static constexpr uint64_t magicNumber = 0x8eed8e9c3d064b79;

MapSerialization::MapSerialization(const std::string& fileName)
{
    UASSERT(fileName.size() >= mapExtension.size() &&
        std::equal(mapExtension.rbegin(), mapExtension.rend(),
            fileName.rbegin()));
    output_.open(fileName, std::ios::out | std::ios::binary);
    UASSERT(output_.is_open());
    output_.write((const char*)&magicNumber, sizeof(magicNumber));
}

void MapSerialization::write(const google::protobuf::Message& proto)
{
    std::string uncompressed;
    proto.SerializeToString(&uncompressed);
    std::string compressed = compressString(uncompressed);
    size_t size = compressed.size();
    output_.write((const char*)&size, sizeof(size));
    output_.write(compressed.data(), size);
}

void MapSerialization::close()
{
    output_.close();
    UASSERT(!output_.fail());
}

MapDeserialization::MapDeserialization(const std::string& fileName)
{
    UASSERT(fileName.size() >= mapExtension.size() &&
        std::equal(mapExtension.rbegin(), mapExtension.rend(),
            fileName.rbegin()));
    input_.open(fileName, std::ios::in | std::ios::binary);
    UASSERT(input_.is_open());
    uint64_t checkMagicNumber;
    input_.read((char*)&checkMagicNumber, sizeof(checkMagicNumber));
    UASSERT(input_.gcount() == sizeof(checkMagicNumber));
    UASSERT(checkMagicNumber == magicNumber);
    readMetaData();
}

std::string MapDeserialization::readString()
{
    size_t size;
    input_.read((char*)&size, sizeof(size));
    UASSERT(input_.gcount() == sizeof(size));
    std::string compressed(size, '\0');
    input_.read(compressed.data(), size);
    UASSERT(input_.gcount() == size);
    return decompressString(compressed);
}

void MapDeserialization::readMetaData()
{
    metaData_.ParseFromString(readString());
}

const proto::OccupancyGridMap::MetaData& MapDeserialization::metaData()
{
    return metaData_;
}

std::optional<proto::OccupancyGridMap::Node> MapDeserialization::readNode()
{
    if (input_.peek() == EOF)
    {
        return std::nullopt;
    }
    proto::OccupancyGridMap::Node node;
    node.ParseFromString(readString());
    return node;
}

void MapDeserialization::close()
{
    input_.close();
    UASSERT(!input_.fail());
}

RawDataSerialization::RawDataSerialization(const std::string& fileName)
{
    UASSERT(fileName.size() >= rawDataExtension.size() &&
        std::equal(rawDataExtension.rbegin(), rawDataExtension.rend(),
            fileName.rbegin()));
    output_.open(fileName, std::ios::out | std::ios::binary);
    UASSERT(output_.is_open());
    output_.write((const char*)&magicNumber, sizeof(magicNumber));
}

void RawDataSerialization::write(const google::protobuf::Message& proto)
{
    std::string uncompressed;
    proto.SerializeToString(&uncompressed);
    size_t size = uncompressed.size();
    output_.write((const char*)&size, sizeof(size));
    output_.write(uncompressed.data(), size);
}

void RawDataSerialization::close()
{
    output_.close();
    UASSERT(!output_.fail());
}

RawDataDeserialization::RawDataDeserialization(const std::string& fileName)
{
    UASSERT(fileName.size() >= rawDataExtension.size() &&
        std::equal(rawDataExtension.rbegin(), rawDataExtension.rend(),
            fileName.rbegin()));
    input_.open(fileName, std::ios::in | std::ios::binary);
    UASSERT(input_.is_open());
    uint64_t checkMagicNumber;
    input_.read((char*)&checkMagicNumber, sizeof(checkMagicNumber));
    UASSERT(input_.gcount() == sizeof(checkMagicNumber));
    UASSERT(checkMagicNumber == magicNumber);
}

std::string RawDataDeserialization::readString()
{
    size_t size;
    input_.read((char*)&size, sizeof(size));
    UASSERT(input_.gcount() == sizeof(size));
    std::string uncompressed(size, '\0');
    input_.read(uncompressed.data(), size);
    UASSERT(input_.gcount() == size);
    return uncompressed;
}

std::optional<proto::RawData> RawDataDeserialization::readRawData()
{
    if (input_.peek() == EOF)
    {
        return std::nullopt;
    }
    proto::RawData rawData;
    rawData.ParseFromString(readString());
    return rawData;
}

void RawDataDeserialization::close()
{
    input_.close();
    UASSERT(!input_.fail());
}

}
