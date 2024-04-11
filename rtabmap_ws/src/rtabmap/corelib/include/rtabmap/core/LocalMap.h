#pragma once

#include <rtabmap/core/Time.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Color.h>
#include <rtabmap/core/MapLimits.h>
#include <rtabmap/core/Grid.h>

#include <vector>
#include <memory>
#include <optional>
#include <utility>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <rtabmap/proto/Time.pb.h>
#include <rtabmap/proto/Color.pb.h>
#include <rtabmap/proto/Transform.pb.h>
#include <rtabmap/proto/LocalMap.pb.h>

namespace rtabmap {

class LocalMap
{
public:
    struct ColoredGrid
    {
        float cellSize = 0.0f;
        MapLimitsI limits;
        cv::Mat grid;  // CV_8S
        cv::Mat colors;  // CV_32S

        static constexpr std::int8_t unknownCellValue = -1;
        static constexpr std::int8_t emptyCellValue = 0;
        static constexpr std::int8_t occupiedCellValue = 100;
    };

    struct Properties
    {
        float sensorBlindRange2dSqr = 0.0f;
        Transform toSensor = Transform::getIdentity();

        // used to correct poses passed to updatePoses() function
        Transform fromUpdatedPose = Transform::getIdentity();

        Time time = Time(0, 0);
    };

public:
    LocalMap();
    LocalMap(const Properties& properties);
    LocalMap(const ColoredGrid& coloredGrid,
        float maxRange2dSqr, bool duplicatePoints);
    LocalMap(const ColoredGrid& coloredGrid,
        float maxRange2dSqr, bool duplicatePoints,
        const Properties& properties);

    ~LocalMap() = default;

    void fromColoredGrid(const ColoredGrid& coloredGrid,
        float maxRange2dSqr, bool duplicatePoints);
    ColoredGrid toColoredGrid() const;

    template<typename T>
    void setProperties(T&& properties) { properties_ = std::forward<T>(properties); }
    const Properties& properties() const { return properties_; }

    bool isObstacle(int i) const { return i < numObstacles_; }

    int numObstacles() const { return numObstacles_; }
    int numEmpty() const { return numEmpty_; }
    const Eigen::Matrix3Xf& points() const { return points_; }
    const std::vector<Color>& colors() const { return colors_; }
    bool pointsDuplicated() const { return pointsDuplicated_; }

    void setSensorBlindRange2dSqr(float sensorBlindRange2dSqr)
        { properties_.sensorBlindRange2dSqr = sensorBlindRange2dSqr; }
    float sensorBlindRange2dSqr() const { return properties_.sensorBlindRange2dSqr; }
    template<typename T>
    void setToSensor(T&& toSensor) { properties_.toSensor = std::forward<T>(toSensor); }
    const Transform& toSensor() const { return properties_.toSensor; }

    template<typename T>
    void setFromUpdatedPose(T&& fromUpdatedPose)
        { properties_.fromUpdatedPose = std::forward<T>(fromUpdatedPose); }
    const Transform& fromUpdatedPose() const { return properties_.fromUpdatedPose; }

    void setTime(const Time& time) { properties_.time = time; }
    const Time& time() const { return properties_.time; }

private:
    int numObstacles_;
    int numEmpty_;
    Eigen::Matrix3Xf points_;  // z = 0
    std::vector<Color> colors_;
    bool pointsDuplicated_;

    // information about ColoredGrid that produced LocalMap
    float cellSize_;
    MapLimitsI limits_;

    Properties properties_;
};

proto::LocalMap::ColoredGrid toProto(const LocalMap::ColoredGrid& coloredGrid);
LocalMap::ColoredGrid fromProto(const proto::LocalMap::ColoredGrid& proto);

proto::LocalMap toProto(const LocalMap& localMap);
std::shared_ptr<LocalMap> fromProto(const proto::LocalMap& proto);

}