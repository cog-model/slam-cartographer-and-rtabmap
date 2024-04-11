#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/OccupancyGridBuilder.h>
#include <rtabmap/core/TemporaryOccupancyGridBuilder.h>
#include <rtabmap/core/ObstacleDilation.h>
#include <rtabmap/core/ObjectTracking.h>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <memory>
#include <optional>
#include <climits>

namespace rtabmap {

class OccupancyGridMap
{
public:
    struct Parameters
    {
        float cellSize = 0.1f;
        bool enableObjectTracking = false;

        LocalMapBuilder::Parameters localMapBuilderParameters;
        std::vector<ObstacleDilation::Parameters> obstacleDilationsParameters =
            std::vector<ObstacleDilation::Parameters>(1);
        OccupancyGridBuilder::Parameters occupancyGridBuilderParameters;
        TemporaryOccupancyGridBuilder::Parameters
            temporaryOccupancyGridBuilderParameters;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["CellSize"])
            {
                parameters.cellSize = node["CellSize"].as<float>();
            }
            if (node["EnableObjectTracking"])
            {
                parameters.enableObjectTracking =
                    node["EnableObjectTracking"].as<bool>();
            }
            if (node["LocalMapBuilder"])
            {
                parameters.localMapBuilderParameters =
                    LocalMapBuilder::Parameters::createParameters(
                        node["LocalMapBuilder"]);
            }
            if (node["ObstacleDilation"])
            {
                UASSERT(node["ObstacleDilation"].IsMap() ||
                    node["ObstacleDilation"].IsSequence());
                parameters.obstacleDilationsParameters.clear();
                if (node["ObstacleDilation"].IsMap())
                {
                    parameters.obstacleDilationsParameters.push_back(
                        ObstacleDilation::Parameters::createParameters(
                            node["ObstacleDilation"]));
                }
                if (node["ObstacleDilation"].IsSequence())
                {
                    for (const YAML::Node& obstacleDilationNode : node["ObstacleDilation"])
                    {
                        parameters.obstacleDilationsParameters.push_back(
                            ObstacleDilation::Parameters::createParameters(
                                obstacleDilationNode));
                    }
                }
            }
            if (node["OccupancyGridBuilder"])
            {
                parameters.occupancyGridBuilderParameters =
                    OccupancyGridBuilder::Parameters::createParameters(
                        node["OccupancyGridBuilder"]);
            }
            if (node["TemporaryOccupancyGridBuilder"])
            {
                parameters.temporaryOccupancyGridBuilderParameters =
                    TemporaryOccupancyGridBuilder::Parameters::createParameters(
                        node["TemporaryOccupancyGridBuilder"]);
            }
            return parameters;
        }
    };

public:
    OccupancyGridMap(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    std::shared_ptr<LocalMap> createLocalMap(const SensorData& sensorData,
        const Time& time, const Transform& fromUpdatedPose) const;

    int addLocalMap(const std::shared_ptr<const LocalMap>& localMap);
    int addLocalMap(const Transform& pose,
        const std::shared_ptr<const LocalMap>& localMap);
    bool addTemporaryLocalMap(const Transform& pose,
        const std::shared_ptr<const LocalMap>& localMap);

    int addSensorData(const SensorData& sensorData, const Time& time,
        const Transform& fromUpdatedPose);
    int addSensorData(const SensorData& sensorData, const Time& time,
        const Transform& pose, const Transform& fromUpdatedPose);
    bool addTemporarySensorData(const SensorData& sensorData, const Time& time,
        const Transform& pose, const Transform& fromUpdatedPose);

    void removeNodes(const std::vector<int>& nodeIdsToRemove);

    void transformMap(const Transform& transform);

    void updatePoses(const std::map<int, Transform>& updatedPoses,
        const std::deque<Transform>& updatedTemporaryPoses,
        int lastNodeIdToIncludeInCachedMap = -1);

    OccupancyGrid getOccupancyGrid(int index) const;
    OccupancyGrid getProbOccupancyGrid(int index) const;
    ColorGrid getColorGrid(int index) const;

    float cellSize() const { return cellSize_; }
    std::pair<float, float> getGridOrigin(int index) const;
    int maxTemporaryLocalMaps(int index) const
        { return temporaryOccupancyGridBuilders_[index]->maxTemporaryLocalMaps(); }
    const std::map<int, Node>& nodes(int index) const
        { return occupancyGridBuilders_[index]->nodes(); }
    const std::deque<Node>& temporaryNodes(int index) const
        { return temporaryOccupancyGridBuilders_[index]->nodes(); }
    const std::map<int, const std::shared_ptr<const LocalMap>>& localMapsWithoutDilation() const
        { return localMapsWithoutDilation_; }
    const cv::Mat& lastDilatedSemantic() const
        { return localMapBuilder_->lastDilatedSemantic(); }
    int numBuilders() const { return numBuilders_; }
    bool objectTrackingEnabled() const { return objectTracking_ != nullptr; }
    const std::vector<ObjectTracking::TrackedObject>& trackedObjects() const
        { UASSERT(objectTracking_); return objectTracking_->trackedObjects(); }
    const std::list<ObjectTracking::MOT16TrackedObject>& mot16TrackedObjectsCache() const
        { UASSERT(objectTracking_); return objectTracking_->mot16TrackedObjectsCache(); }

    void resetAll();
    void resetTemporary();

private:
    float cellSize_;
    bool enableObjectTracking_;

    std::unique_ptr<LocalMapBuilder> localMapBuilder_;

    int numBuilders_;
    std::vector<std::unique_ptr<ObstacleDilation>> obstacleDilations_;
    std::vector<std::unique_ptr<OccupancyGridBuilder>> occupancyGridBuilders_;
    std::vector<std::unique_ptr<TemporaryOccupancyGridBuilder>>
        temporaryOccupancyGridBuilders_;

    std::unique_ptr<ObjectTracking> objectTracking_;

    std::map<int, const std::shared_ptr<const LocalMap>> localMapsWithoutDilation_;
};

}
